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

func (s *handEyeTest) handleMoveTo(ctx context.Context, target r3.Vector, stepSize float64) (map[string]interface{}, error) {
	s.mu.Lock()
	s.currentStatus = "moving"
	s.mu.Unlock()
	defer func() {
		s.mu.Lock()
		s.currentStatus = "idle"
		s.mu.Unlock()
	}()

	const maxSteps = 200

	var steps int
	for steps = 0; steps < maxSteps; steps++ {
		gripperPose, err := s.motion.GetPose(ctx, s.cfg.Gripper, "world", nil, nil)
		if err != nil {
			return nil, fmt.Errorf("failed to get gripper pose: %w", err)
		}
		currentPos := gripperPose.Pose().Point()
		currentOri := gripperPose.Pose().Orientation()

		diff := r3.Vector{X: target.X - currentPos.X, Y: target.Y - currentPos.Y, Z: target.Z - currentPos.Z}
		dist := math.Sqrt(diff.X*diff.X + diff.Y*diff.Y + diff.Z*diff.Z)

		s.logger.Infof("Step %d: current=(%.1f, %.1f, %.1f), distance to target=%.1fmm",
			steps, currentPos.X, currentPos.Y, currentPos.Z, dist)

		if dist <= 1.0 {
			s.logger.Infof("Reached target (within 1mm)")
			break
		}

		var nextPoint r3.Vector
		if dist <= stepSize {
			nextPoint = target
		} else {
			direction := r3.Vector{X: diff.X / dist, Y: diff.Y / dist, Z: diff.Z / dist}
			nextPoint = r3.Vector{
				X: currentPos.X + direction.X*stepSize,
				Y: currentPos.Y + direction.Y*stepSize,
				Z: currentPos.Z + direction.Z*stepSize,
			}
		}

		dest := referenceframe.NewPoseInFrame("world", spatialmath.NewPose(nextPoint, currentOri))
		s.logger.Infof("Step %d: moving to (%.1f, %.1f, %.1f)...", steps, nextPoint.X, nextPoint.Y, nextPoint.Z)

		success, err := s.motion.Move(ctx, motion.MoveReq{
			ComponentName: s.cfg.Gripper,
			Destination:   dest,
		})
		if err != nil {
			return nil, fmt.Errorf("step %d move failed: %w", steps, err)
		}
		if !success {
			return nil, fmt.Errorf("step %d: motion planner could not find path", steps)
		}
	}

	if steps >= maxSteps {
		return nil, fmt.Errorf("did not reach target after %d steps", maxSteps)
	}

	finalPose, err := s.motion.GetPose(ctx, s.cfg.Gripper, "world", nil, nil)
	var finalPos r3.Vector
	if err == nil {
		finalPos = finalPose.Pose().Point()
	}

	return map[string]interface{}{
		"success": true,
		"steps":   steps,
		"final_position": map[string]interface{}{
			"x_mm": finalPos.X, "y_mm": finalPos.Y, "z_mm": finalPos.Z,
		},
		"target": map[string]interface{}{
			"x_mm": target.X, "y_mm": target.Y, "z_mm": target.Z,
		},
	}, nil
}
