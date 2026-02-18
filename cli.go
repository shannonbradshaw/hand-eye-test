package handeyetest

import (
	"context"
	"encoding/json"
	"flag"
	"fmt"
	"os"

	"github.com/erh/vmodutils"
	"go.viam.com/rdk/logging"
	generic "go.viam.com/rdk/services/generic"
)

// RunCLI runs the CLI mode, connecting to a remote machine and executing a command.
func RunCLI(subcommand string, args []string) {
	err := runCLI(subcommand, args)
	if err != nil {
		fmt.Fprintf(os.Stderr, "Error: %v\n", err)
		os.Exit(1)
	}
}

func runCLI(subcommand string, args []string) error {
	ctx := context.Background()
	logger := logging.NewLogger("hand-eye-test")

	fs := flag.NewFlagSet(subcommand, flag.ExitOnError)
	host := fs.String("host", "", "machine host address (required)")
	debug := fs.Bool("debug", false, "enable debug logging")
	objectIndex := fs.Int("object", 0, "index of object to pick")

	// Config overrides
	armName := fs.String("arm", "arm", "arm component name")
	cameraName := fs.String("camera", "camera", "camera component name")
	gripperName := fs.String("gripper", "gripper", "gripper component name")
	approachOffset := fs.Float64("approach-offset", 100, "mm above object for approach pose")
	graspOffset := fs.Float64("grasp-offset", 0, "mm adjustment for grasp depth")
	liftHeight := fs.Float64("lift-height", 50, "mm to lift after grasping")

	if err := fs.Parse(args); err != nil {
		return err
	}

	if *debug {
		logger.SetLevel(logging.DEBUG)
	}

	if *host == "" {
		return fmt.Errorf("--host is required")
	}

	cfg := Config{
		Arm:                *armName,
		Camera:             *cameraName,
		Gripper:            *gripperName,
		ApproachOffsetMm:   *approachOffset,
		GraspDepthOffsetMm: *graspOffset,
		LiftHeightMm:       *liftHeight,
		Segmentation: SegmentationConfig{
			MinPtsInPlane:      1500,
			MaxDistFromPlane:   5.0,
			GroundNormal:       []float64{0, 0, 1},
			AngleTolerance:     20,
			MinPtsInSegment:    100,
			ClusteringRadiusMm: 5.0,
			MeanKFiltering:     50,
		},
	}

	logger.Infof("Connecting to %s...", *host)
	machine, err := vmodutils.ConnectToHostFromCLIToken(ctx, *host, logger)
	if err != nil {
		return fmt.Errorf("failed to connect: %w", err)
	}
	defer machine.Close(ctx)

	deps, err := vmodutils.MachineToDependencies(machine)
	if err != nil {
		return fmt.Errorf("failed to get dependencies: %w", err)
	}

	svc, err := NewHandEyeTest(ctx, deps, generic.Named("hand-eye-test"), &cfg, logger)
	if err != nil {
		return fmt.Errorf("failed to create service: %w", err)
	}
	defer svc.Close(ctx)

	var cmdMap map[string]interface{}
	switch subcommand {
	case "detect":
		cmdMap = map[string]interface{}{"command": "detect"}
	case "pick":
		cmdMap = map[string]interface{}{"command": "pick", "object_index": float64(*objectIndex)}
	case "status":
		cmdMap = map[string]interface{}{"command": "status"}
	default:
		return fmt.Errorf("unknown subcommand: %s", subcommand)
	}

	result, err := svc.DoCommand(ctx, cmdMap)
	if err != nil {
		return err
	}

	output, err := json.MarshalIndent(result, "", "  ")
	if err != nil {
		return fmt.Errorf("failed to marshal result: %w", err)
	}
	fmt.Println(string(output))

	return nil
}
