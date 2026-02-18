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
	Success                     bool
	IsHolding                   bool
	DetectedPositionCameraFrame r3.Vector
	ObjectPositionWorldFrame    r3.Vector
	GripperPositionWorldFrame   r3.Vector
	ApproachOffsetMm            r3.Vector
	WorldFrameOffsetMm          r3.Vector
	StepsCompleted              []string
}

func (r *pickResult) toMap(cameraName string) map[string]interface{} {
	return map[string]interface{}{
		"success":    r.Success,
		"is_holding": r.IsHolding,
		"detected_position_camera_frame": map[string]interface{}{
			"x_mm": r.DetectedPositionCameraFrame.X, "y_mm": r.DetectedPositionCameraFrame.Y,
			"z_mm": r.DetectedPositionCameraFrame.Z, "frame": cameraName,
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

	result := &pickResult{
		DetectedPositionCameraFrame: obj.Center,
	}
	cameraName := s.cfg.Camera

	s.logger.Infof("Starting pick sequence for object at camera-frame position: (%.1f, %.1f, %.1f)mm",
		obj.Center.X, obj.Center.Y, obj.Center.Z)

	// Step 1: Open gripper
	s.logger.Infof("Opening gripper...")
	if err := s.gripper.Open(ctx, nil); err != nil {
		return nil, fmt.Errorf("failed to open gripper: %w", err)
	}
	result.StepsCompleted = append(result.StepsCompleted, "open_gripper")

	// Step 2: Get world-frame reference for the object position
	// Use motion.GetPose to get gripper's world-frame position, and then we'll compare after moving.
	gripperWorldPoseBefore, err := s.motion.GetPose(ctx, s.cfg.Gripper, "world", nil, nil)
	if err != nil {
		s.logger.Warnf("Could not get gripper world pose (non-fatal): %v", err)
	}
	_ = gripperWorldPoseBefore // for reference

	// Step 3: Compute approach pose in camera frame (offset above the object)
	approachPose := spatialmath.NewPose(
		r3.Vector{X: obj.Center.X, Y: obj.Center.Y, Z: obj.Center.Z - s.cfg.ApproachOffsetMm},
		&spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 0},
	)
	approachDest := referenceframe.NewPoseInFrame(cameraName, approachPose)

	// Step 4: Move to approach position
	s.logger.Infof("Moving to approach position (%.0fmm above object)...", s.cfg.ApproachOffsetMm)
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

	// Step 5: Re-detect object from approach position for offset measurement
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

	// Step 6: Move to grasp position
	graspPose := spatialmath.NewPose(
		r3.Vector{
			X: obj.Center.X,
			Y: obj.Center.Y,
			Z: obj.Center.Z + s.cfg.GraspDepthOffsetMm,
		},
		&spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 0},
	)
	graspDest := referenceframe.NewPoseInFrame(cameraName, graspPose)

	s.logger.Infof("Moving to grasp position...")
	success, err = s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.cfg.Gripper,
		Destination:   graspDest,
	})
	if err != nil {
		return nil, fmt.Errorf("failed to move to grasp position: %w", err)
	}
	if !success {
		return nil, fmt.Errorf("motion planner could not find path to grasp position")
	}
	result.StepsCompleted = append(result.StepsCompleted, "grasp_position")

	// Step 7: World-frame comparison
	gripperWorldPose, err := s.motion.GetPose(ctx, s.cfg.Gripper, "world", nil, nil)
	if err != nil {
		s.logger.Warnf("Could not get gripper world pose after grasp move (non-fatal): %v", err)
	} else {
		gripperPos := gripperWorldPose.Pose().Point()
		result.GripperPositionWorldFrame = gripperPos

		// Also get the object's world-frame position for comparison
		objectWorldPose, err := s.motion.GetPose(ctx, s.cfg.Camera, "world", nil, nil)
		if err != nil {
			s.logger.Warnf("Could not get camera world pose (non-fatal): %v", err)
		} else {
			// The object is at obj.Center in camera frame. Transform to world using camera's world pose.
			cameraPose := objectWorldPose.Pose()
			objectInWorld := spatialmath.Compose(cameraPose, spatialmath.NewPoseFromPoint(obj.Center))
			result.ObjectPositionWorldFrame = objectInWorld.Point()

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

	// Step 8: Grab
	s.logger.Infof("Closing gripper...")
	grabbed, err := s.gripper.Grab(ctx, nil)
	if err != nil {
		return nil, fmt.Errorf("failed to grab: %w", err)
	}
	s.logger.Infof("Grab reported: %v", grabbed)
	result.StepsCompleted = append(result.StepsCompleted, "grab")

	// Step 9: Lift
	s.logger.Infof("Lifting %.0fmm...", s.cfg.LiftHeightMm)
	liftPose := spatialmath.NewPose(
		r3.Vector{
			X: obj.Center.X,
			Y: obj.Center.Y,
			Z: obj.Center.Z + s.cfg.GraspDepthOffsetMm - s.cfg.LiftHeightMm,
		},
		&spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 0},
	)
	liftDest := referenceframe.NewPoseInFrame(cameraName, liftPose)
	_, err = s.motion.Move(ctx, motion.MoveReq{
		ComponentName: s.cfg.Gripper,
		Destination:   liftDest,
	})
	if err != nil {
		s.logger.Warnf("Lift move failed (non-fatal): %v", err)
	}
	result.StepsCompleted = append(result.StepsCompleted, "lift")

	// Step 10: Verify
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

	return result.toMap(cameraName), nil
}
