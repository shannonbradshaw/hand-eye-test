package handeyetest

import (
	"context"
	"fmt"
	"sync"

	"go.viam.com/rdk/components/arm"
	"go.viam.com/rdk/components/camera"
	"go.viam.com/rdk/components/gripper"
	"go.viam.com/rdk/logging"
	"go.viam.com/rdk/resource"
	"go.viam.com/rdk/services/motion"
)

type handEyeTest struct {
	resource.AlwaysRebuild

	name   resource.Name
	logger logging.Logger
	cfg    *Config

	arm     arm.Arm
	camera  camera.Camera
	gripper gripper.Gripper
	motion  motion.Service

	cancelCtx  context.Context
	cancelFunc func()

	mu            sync.Mutex
	lastDetection []DetectedObject
	currentStatus string
	lastResult    map[string]interface{}
}

func newHandEyeTest(ctx context.Context, deps resource.Dependencies, rawConf resource.Config, logger logging.Logger) (resource.Resource, error) {
	conf, err := resource.NativeConfig[*Config](rawConf)
	if err != nil {
		return nil, err
	}
	return NewHandEyeTest(ctx, deps, rawConf.ResourceName(), conf, logger)
}

func NewHandEyeTest(ctx context.Context, deps resource.Dependencies, name resource.Name, cfg *Config, logger logging.Logger) (resource.Resource, error) {
	a, err := arm.FromDependencies(deps, cfg.Arm)
	if err != nil {
		return nil, fmt.Errorf("getting arm %q: %w", cfg.Arm, err)
	}

	cam, err := camera.FromDependencies(deps, cfg.Camera)
	if err != nil {
		return nil, fmt.Errorf("getting camera %q: %w", cfg.Camera, err)
	}

	grip, err := gripper.FromDependencies(deps, cfg.Gripper)
	if err != nil {
		return nil, fmt.Errorf("getting gripper %q: %w", cfg.Gripper, err)
	}

	motionSvc, err := motion.FromDependencies(deps, "builtin")
	if err != nil {
		return nil, fmt.Errorf("getting motion service: %w", err)
	}

	cancelCtx, cancelFunc := context.WithCancel(context.Background())

	s := &handEyeTest{
		name:          name,
		logger:        logger,
		cfg:           cfg,
		arm:           a,
		camera:        cam,
		gripper:       grip,
		motion:        motionSvc,
		cancelCtx:     cancelCtx,
		cancelFunc:    cancelFunc,
		currentStatus: "idle",
	}
	return s, nil
}

func (s *handEyeTest) Name() resource.Name {
	return s.name
}

func (s *handEyeTest) DoCommand(ctx context.Context, cmd map[string]interface{}) (map[string]interface{}, error) {
	command, ok := cmd["command"].(string)
	if !ok {
		return nil, fmt.Errorf("missing or invalid 'command' field")
	}

	switch command {
	case "detect":
		return s.handleDetect(ctx)
	case "pick":
		objectIndex := 0
		if idx, ok := cmd["object_index"].(float64); ok {
			objectIndex = int(idx)
		}
		return s.handlePick(ctx, objectIndex)
	case "pick_detected":
		objectIndex := 0
		if idx, ok := cmd["object_index"].(float64); ok {
			objectIndex = int(idx)
		}
		return s.handlePickDetected(ctx, objectIndex)
	case "status":
		return s.handleStatus()
	default:
		return nil, fmt.Errorf("unknown command: %s", command)
	}
}

func (s *handEyeTest) handleDetect(ctx context.Context) (map[string]interface{}, error) {
	s.mu.Lock()
	s.currentStatus = "detecting"
	s.mu.Unlock()

	objects, err := detectObjects(ctx, s.camera, s.cfg)
	if err != nil {
		s.mu.Lock()
		s.currentStatus = "idle"
		s.mu.Unlock()
		return nil, fmt.Errorf("detection failed: %w", err)
	}

	s.mu.Lock()
	s.lastDetection = objects
	s.currentStatus = "idle"
	s.mu.Unlock()

	objList := make([]interface{}, len(objects))
	for i, obj := range objects {
		objList[i] = map[string]interface{}{
			"index":       i,
			"point_count": obj.PointCount,
			"center_x_mm": obj.Center.X,
			"center_y_mm": obj.Center.Y,
			"center_z_mm": obj.Center.Z,
		}
	}

	return map[string]interface{}{
		"objects": objList,
		"count":   len(objects),
	}, nil
}

func (s *handEyeTest) handlePick(ctx context.Context, objectIndex int) (map[string]interface{}, error) {
	s.mu.Lock()
	s.currentStatus = "detecting"
	s.mu.Unlock()

	objects, err := detectObjects(ctx, s.camera, s.cfg)
	if err != nil {
		s.mu.Lock()
		s.currentStatus = "idle"
		s.mu.Unlock()
		return nil, fmt.Errorf("detection failed: %w", err)
	}

	s.mu.Lock()
	s.lastDetection = objects
	s.mu.Unlock()

	if objectIndex >= len(objects) {
		s.mu.Lock()
		s.currentStatus = "idle"
		s.mu.Unlock()
		return nil, fmt.Errorf("object_index %d out of range (detected %d objects)", objectIndex, len(objects))
	}

	result, err := s.executePick(ctx, objects[objectIndex])
	s.mu.Lock()
	s.currentStatus = "idle"
	s.lastResult = result
	s.mu.Unlock()
	return result, err
}

func (s *handEyeTest) handlePickDetected(ctx context.Context, objectIndex int) (map[string]interface{}, error) {
	s.mu.Lock()
	objects := s.lastDetection
	s.mu.Unlock()

	if objects == nil {
		return nil, fmt.Errorf("no previous detection; run 'detect' first")
	}
	if objectIndex >= len(objects) {
		return nil, fmt.Errorf("object_index %d out of range (detected %d objects)", objectIndex, len(objects))
	}

	result, err := s.executePick(ctx, objects[objectIndex])
	s.mu.Lock()
	s.currentStatus = "idle"
	s.lastResult = result
	s.mu.Unlock()
	return result, err
}

func (s *handEyeTest) handleStatus() (map[string]interface{}, error) {
	s.mu.Lock()
	defer s.mu.Unlock()

	result := map[string]interface{}{
		"status": s.currentStatus,
	}
	if s.lastResult != nil {
		result["last_result"] = s.lastResult
	}
	if s.lastDetection != nil {
		result["detected_objects"] = len(s.lastDetection)
	}
	return result, nil
}

func (s *handEyeTest) Close(context.Context) error {
	s.cancelFunc()
	return nil
}
