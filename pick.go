package handeyetest

import (
	"context"
	"fmt"
	"math"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/referenceframe"
	"go.viam.com/rdk/services/motion"
	"go.viam.com/rdk/spatialmath"
)

type pickResult struct {
	Success                   bool
	IsHolding                 bool
	DetectedPosition          r3.Vector
	DetectionFrame            string
	ObjectPositionWorldFrame  r3.Vector
	GripperPositionWorldFrame r3.Vector
	ApproachOffsetMm          r3.Vector
	WorldFrameOffsetMm        r3.Vector
	StepsCompleted            []string
}

func (r *pickResult) toMap() map[string]interface{} {
	return map[string]interface{}{
		"success":    r.Success,
		"is_holding": r.IsHolding,
		"detected_position": map[string]interface{}{
			"x_mm": r.DetectedPosition.X, "y_mm": r.DetectedPosition.Y,
			"z_mm": r.DetectedPosition.Z, "frame": r.DetectionFrame,
		},
		"object_position_world_frame": map[string]interface{}{
			"x_mm": r.ObjectPositionWorldFrame.X, "y_mm": r.ObjectPositionWorldFrame.Y,
			"z_mm": r.ObjectPositionWorldFrame.Z, "frame": "world",
		},
		"gripper_position_world_frame": map[string]interface{}{
			"x_mm": r.GripperPositionWorldFrame.X, "y_mm": r.GripperPositionWorldFrame.Y,
			"z_mm": r.GripperPositionWorldFrame.Z, "frame": "world",
		},
		"approach_offset_mm": map[string]interface{}{
			"x": r.ApproachOffsetMm.X, "y": r.ApproachOffsetMm.Y, "z": r.ApproachOffsetMm.Z,
			"total": vecNorm(r.ApproachOffsetMm),
		},
		"world_frame_offset_mm": map[string]interface{}{
			"x": r.WorldFrameOffsetMm.X, "y": r.WorldFrameOffsetMm.Y, "z": r.WorldFrameOffsetMm.Z,
			"total": vecNorm(r.WorldFrameOffsetMm),
		},
		"steps_completed": r.StepsCompleted,
	}
}

func vecNorm(v r3.Vector) float64 {
	return math.Sqrt(v.X*v.X + v.Y*v.Y + v.Z*v.Z)
}

func (s *handEyeTest) executePick(ctx context.Context, obj DetectedObject) (map[string]interface{}, error) {
	s.mu.Lock()
	s.currentStatus = "picking"
	s.mu.Unlock()

	detectionFrame := s.cfg.DetectionFrame
	if detectionFrame == "" {
		detectionFrame = s.cfg.Camera
	}
	isWorldFrame := detectionFrame == "world"

	result := &pickResult{
		DetectedPosition: obj.Center,
		DetectionFrame:   detectionFrame,
	}

	s.logger.Infof("Starting pick sequence for object at %s-frame position: (%.1f, %.1f, %.1f)mm",
		detectionFrame, obj.Center.X, obj.Center.Y, obj.Center.Z)

	// Step 1: Open gripper
	s.logger.Infof("Opening gripper...")
	if err := s.gripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open gripper: %w", err)
	}
	result.StepsCompleted = append(result.StepsCompleted, "open_gripper")

	// Step 2: Compute approach pose in detection frame
	var approachPoint r3.Vector
	if isWorldFrame {
		// World frame: Z is up, approach is above the object
		approachPoint = r3.Vector{
			X: obj.Center.X,
			Y: obj.Center.Y,
			Z: obj.Center.Z + s.cfg.ApproachOffsetMm,
		}
	} else {
		// Camera frame: Z is depth (away from camera), approach is closer to camera
		approachPoint = r3.Vector{
			X: obj.Center.X,
			Y: obj.Center.Y,
			Z: obj.Center.Z - s.cfg.ApproachOffsetMm,
		}
	}

	// Get current gripper orientation in the detection frame for the approach destination.
	var approachOrientation spatialmath.Orientation
	if isWorldFrame {
		gripperPose, err := s.motion.GetPose(ctx, s.cfg.Gripper, "world", nil, nil)
		if err != nil {
			s.logger.Warnf("Could not get gripper world pose for orientation, using default: %v", err)
			approachOrientation = &spatialmath.OrientationVectorDegrees{OX: 0, OY: 1, OZ: 0, Theta: 180}
		} else {
			approachOrientation = gripperPose.Pose().Orientation()
		}
	} else {
		approachOrientation = &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 0}
	}

	approachPose := spatialmath.NewPose(approachPoint, approachOrientation)
	approachDest := referenceframe.NewPoseInFrame(detectionFrame, approachPose)

	// Step 3: Move to approach position using motion planning (obstacle-aware)
	s.logger.Infof("Moving to approach position (%.0fmm above object) via motion planning...", s.cfg.ApproachOffsetMm)
	success, err := s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.cfg.Gripper,
		Destination:   approachDest,
	})
	if err != nil {
		return nil, fmt.Errorf("failed to move to approach position: %w", err)
	}
	if !success {
		return nil, fmt.Errorf("motion planner could not find path to approach position")
	}
	result.StepsCompleted = append(result.StepsCompleted, "approach")

	// Step 4: Re-detect from approach position for offset measurement
	s.logger.Infof("Re-detecting object from approach position...")
	redetectedObjects, err := detectObjects(ctx, s.camera, s.cfg)
	if err != nil {
		s.logger.Warnf("Re-detection failed (non-fatal): %v", err)
	} else if len(redetectedObjects) > 0 {
		redetected := redetectedObjects[0]
		result.ApproachOffsetMm = r3.Vector{
			X: redetected.Center.X - obj.Center.X,
			Y: redetected.Center.Y - obj.Center.Y,
			Z: redetected.Center.Z - obj.Center.Z,
		}
		s.logger.Infof("Approach offset: (%.1f, %.1f, %.1f)mm, total: %.1fmm",
			result.ApproachOffsetMm.X, result.ApproachOffsetMm.Y, result.ApproachOffsetMm.Z,
			vecNorm(result.ApproachOffsetMm))
	}
	result.StepsCompleted = append(result.StepsCompleted, "re_detect")

	// Step 5: Move to grasp position using direct Cartesian move via arm driver.
	// This is a short straight-line move down from the approach position — no motion planning needed.
	currentPose, err := s.arm.EndPosition(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to get arm position: %w", err)
	}
	graspDelta := s.cfg.ApproachOffsetMm - s.cfg.GraspDepthOffsetMm
	graspPoint := r3.Vector{
		X: currentPose.Point().X,
		Y: currentPose.Point().Y,
		Z: currentPose.Point().Z - graspDelta,
	}
	graspPose := spatialmath.NewPose(graspPoint, currentPose.Orientation())

	s.logger.Infof("Moving to grasp position (%.0fmm below approach, direct Cartesian move)...", graspDelta)
	if err := s.arm.MoveToPosition(ctx, graspPose, nil); err != nil {
		return nil, fmt.Errorf("failed to move to grasp position: %w", err)
	}
	result.StepsCompleted = append(result.StepsCompleted, "grasp_position")

	// Step 6: World-frame comparison
	gripperWorldPose, err := s.motion.GetPose(ctx, s.cfg.Gripper, "world", nil, nil)
	if err != nil {
		s.logger.Warnf("Could not get gripper world pose (non-fatal): %v", err)
	} else {
		gripperPos := gripperWorldPose.Pose().Point()
		result.GripperPositionWorldFrame = gripperPos

		if isWorldFrame {
			// Detection was in world frame — object position is already in world frame
			result.ObjectPositionWorldFrame = obj.Center
		} else {
			// Detection was in camera frame — transform to world
			cameraWorldPose, err := s.motion.GetPose(ctx, s.cfg.Camera, "world", nil, nil)
			if err != nil {
				s.logger.Warnf("Could not get camera world pose (non-fatal): %v", err)
			} else {
				cameraPose := cameraWorldPose.Pose()
				objectInWorld := spatialmath.Compose(cameraPose, spatialmath.NewPoseFromPoint(obj.Center))
				result.ObjectPositionWorldFrame = objectInWorld.Point()
			}
		}

		if result.ObjectPositionWorldFrame != (r3.Vector{}) {
			result.WorldFrameOffsetMm = r3.Vector{
				X: gripperPos.X - result.ObjectPositionWorldFrame.X,
				Y: gripperPos.Y - result.ObjectPositionWorldFrame.Y,
				Z: gripperPos.Z - result.ObjectPositionWorldFrame.Z,
			}
			s.logger.Infof("World-frame offset: (%.1f, %.1f, %.1f)mm, total: %.1fmm",
				result.WorldFrameOffsetMm.X, result.WorldFrameOffsetMm.Y, result.WorldFrameOffsetMm.Z,
				vecNorm(result.WorldFrameOffsetMm))
		}
	}

	// Step 7: Grab
	s.logger.Infof("Closing gripper...")
	grabbed, err := s.gripper.Grab(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to grab: %w", err)
	}
	s.logger.Infof("Grab reported: %v", grabbed)
	result.StepsCompleted = append(result.StepsCompleted, "grab")

	// Step 8: Lift using direct Cartesian move — short straight-line move up
	s.logger.Infof("Lifting %.0fmm (direct Cartesian move)...", s.cfg.LiftHeightMm)
	currentPose, err = s.arm.EndPosition(ctx, nil)
	if err != nil {
		s.logger.Warnf("Failed to get arm position for lift (non-fatal): %v", err)
	} else {
		liftPoint := r3.Vector{
			X: currentPose.Point().X,
			Y: currentPose.Point().Y,
			Z: currentPose.Point().Z + s.cfg.LiftHeightMm,
		}
		liftPose := spatialmath.NewPose(liftPoint, currentPose.Orientation())
		if err := s.arm.MoveToPosition(ctx, liftPose, nil); err != nil {
			s.logger.Warnf("Lift move failed (non-fatal): %v", err)
		}
	}
	result.StepsCompleted = append(result.StepsCompleted, "lift")

	// Step 9: Verify
	s.logger.Infof("Verifying hold...")
	holdingStatus, err := s.gripper.IsHoldingSomething(ctx, nil)
	if err != nil {
		s.logger.Warnf("IsHoldingSomething check failed (non-fatal): %v", err)
	} else {
		result.IsHolding = holdingStatus.IsHoldingSomething
	}
	result.StepsCompleted = append(result.StepsCompleted, "verify")

	result.Success = result.IsHolding
	if result.Success {
		s.logger.Infof("RESULT: PASS - calibration validated, object picked successfully")
	} else {
		s.logger.Infof("RESULT: FAIL - gripper did not hold object")
	}

	return result.toMap(), nil
}
