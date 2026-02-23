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

// addConnectionFlags adds flags common to all subcommands.
func addConnectionFlags(fs *flag.FlagSet) (*string, *bool) {
	host := fs.String("host", "", "machine host address (required)")
	debug := fs.Bool("debug", false, "enable debug logging")
	return host, debug
}

// addComponentFlags adds flags for overriding component names.
func addComponentFlags(fs *flag.FlagSet) (arm, camera, gripper *string) {
	arm = fs.String("arm", "arm", "arm component name")
	camera = fs.String("camera", "camera", "camera component name")
	gripper = fs.String("gripper", "gripper", "gripper component name")
	return
}

// segmentationFlags holds pointers to all segmentation-related flag values.
type segmentationFlags struct {
	detectionFrame   *string
	minPtsInPlane    *int
	maxDistFromPlane *float64
	angleTolerance   *float64
	minPtsInSegment  *int
	clusteringRadius *float64
	meanKFiltering   *int
	maxDepth         *float64
	maxPointCount    *int
}

// addSegmentationFlags adds flags for tuning point cloud segmentation.
func addSegmentationFlags(fs *flag.FlagSet) segmentationFlags {
	return segmentationFlags{
		detectionFrame:   fs.String("detection-frame", "", "frame of detected coordinates (default: camera name, use 'world' for crop cameras)"),
		minPtsInPlane:    fs.Int("min-plane-pts", 1500, "min points to identify the ground plane"),
		maxDistFromPlane: fs.Float64("max-dist-from-plane", 5.0, "max distance from plane to be considered part of it (mm)"),
		angleTolerance:   fs.Float64("angle-tolerance", 20, "angle tolerance for ground normal matching (degrees)"),
		minPtsInSegment:  fs.Int("min-pts", 100, "min points for an object cluster"),
		clusteringRadius: fs.Float64("clustering-radius", 5.0, "radius for clustering (mm)"),
		meanKFiltering:   fs.Int("mean-k", 50, "mean-k for noise filtering"),
		maxDepth:         fs.Float64("max-depth", 0, "max depth in mm (0 = no limit)"),
		maxPointCount:    fs.Int("max-pts", 0, "max points per object cluster (0 = no limit)"),
	}
}

func (sf segmentationFlags) toConfig() SegmentationConfig {
	return SegmentationConfig{
		MinPtsInPlane:      *sf.minPtsInPlane,
		MaxDistFromPlane:   *sf.maxDistFromPlane,
		GroundNormal:       []float64{0, 0, 1},
		AngleTolerance:     *sf.angleTolerance,
		MinPtsInSegment:    *sf.minPtsInSegment,
		ClusteringRadiusMm: *sf.clusteringRadius,
		MeanKFiltering:     *sf.meanKFiltering,
		MaxDepthMm:         *sf.maxDepth,
		MaxPointCount:      *sf.maxPointCount,
	}
}

func runCLI(subcommand string, args []string) error {
	ctx := context.Background()
	logger := logging.NewLogger("hand-eye-test")

	var host *string
	var debug *bool
	var armName, cameraName, gripperName *string
	var cfg Config
	var cmdMap map[string]interface{}

	switch subcommand {
	case "detect":
		fs := flag.NewFlagSet("detect", flag.ExitOnError)
		fs.Usage = func() {
			fmt.Fprintf(os.Stderr, `Capture a point cloud from the camera and detect objects using plane segmentation
and clustering. Returns a list of detected objects with their positions and point counts.

Usage:
  hand-eye-test detect --host <address> [flags]

Example:
  hand-eye-test detect --host my-robot.viam.cloud
  hand-eye-test detect --host my-robot.viam.cloud --camera wrist-cam --min-pts 200

Flags:
`)
			fs.PrintDefaults()
		}
		host, debug = addConnectionFlags(fs)
		armName, cameraName, gripperName = addComponentFlags(fs)
		seg := addSegmentationFlags(fs)
		if err := fs.Parse(args); err != nil {
			return err
		}
		cfg = Config{
			Arm: *armName, Camera: *cameraName, Gripper: *gripperName,
			DetectionFrame: *seg.detectionFrame,
			Segmentation:   seg.toConfig(),
		}
		cmdMap = map[string]interface{}{"command": "detect"}

	case "pick":
		fs := flag.NewFlagSet("pick", flag.ExitOnError)
		fs.Usage = func() {
			fmt.Fprintf(os.Stderr, `Detect objects, then execute a full pick sequence on one of them. The sequence is:
  1. Open gripper
  2. Move to approach position (above the object, via motion planning)
  3. Re-detect object from approach position (measures approach offset)
  4. Move to grasp position (straight down via arm driver)
  5. Compare gripper world-frame position to detected object position
  6. Close gripper (grab)
  7. Lift
  8. Verify gripper is holding something

Reports calibration accuracy as approach offset and world-frame offset in mm.

Usage:
  hand-eye-test pick --host <address> [flags]

Example:
  hand-eye-test pick --host my-robot.viam.cloud
  hand-eye-test pick --host my-robot.viam.cloud --object 1 --approach-offset 80

Flags:
`)
			fs.PrintDefaults()
		}
		host, debug = addConnectionFlags(fs)
		armName, cameraName, gripperName = addComponentFlags(fs)
		seg := addSegmentationFlags(fs)
		objectIndex := fs.Int("object", 0, "index of detected object to pick (0 = closest/largest)")
		approachOffset := fs.Float64("approach-offset", 100, "mm above object for approach pose")
		graspOffset := fs.Float64("grasp-offset", 0, "mm adjustment for grasp depth (positive = deeper)")
		liftHeight := fs.Float64("lift-height", 50, "mm to lift after grasping")
		if err := fs.Parse(args); err != nil {
			return err
		}
		cfg = Config{
			Arm: *armName, Camera: *cameraName, Gripper: *gripperName,
			DetectionFrame:     *seg.detectionFrame,
			ApproachOffsetMm:   *approachOffset,
			GraspDepthOffsetMm: *graspOffset,
			LiftHeightMm:       *liftHeight,
			Segmentation:       seg.toConfig(),
		}
		cmdMap = map[string]interface{}{"command": "pick", "object_index": float64(*objectIndex)}

	case "move-to":
		fs := flag.NewFlagSet("move-to", flag.ExitOnError)
		fs.Usage = func() {
			fmt.Fprintf(os.Stderr, `Incrementally move the gripper to a target position in the world frame. Each step
moves the gripper closer by --step-size mm using the motion service (obstacle-aware).
Useful for testing reachability, collision geometry, and frame system accuracy.

Usage:
  hand-eye-test move-to --host <address> --x <mm> --y <mm> --z <mm> [flags]

Example:
  hand-eye-test move-to --host my-robot.viam.cloud --x 413 --y 731 --z 45
  hand-eye-test move-to --host my-robot.viam.cloud --x 300 --y 350 --z 300 --step-size 5
  hand-eye-test move-to --host my-robot.viam.cloud --x 680 --y 160 --z 30 --arm right-arm --gripper right-gripper

Flags:
`)
			fs.PrintDefaults()
		}
		host, debug = addConnectionFlags(fs)
		armName, cameraName, gripperName = addComponentFlags(fs)
		targetX := fs.Float64("x", 0, "target X position in world frame (mm)")
		targetY := fs.Float64("y", 0, "target Y position in world frame (mm)")
		targetZ := fs.Float64("z", 0, "target Z position in world frame (mm)")
		moveStepSize := fs.Float64("step-size", 20, "step size per move increment (mm)")
		if err := fs.Parse(args); err != nil {
			return err
		}
		cfg = Config{
			Arm: *armName, Camera: *cameraName, Gripper: *gripperName,
		}
		cmdMap = map[string]interface{}{
			"command":   "move_to",
			"x":         *targetX,
			"y":         *targetY,
			"z":         *targetZ,
			"step_size": *moveStepSize,
		}

	case "status":
		fs := flag.NewFlagSet("status", flag.ExitOnError)
		fs.Usage = func() {
			fmt.Fprintf(os.Stderr, `Return the current service status (idle, detecting, picking, moving) and the
result of the last operation.

Usage:
  hand-eye-test status --host <address> [flags]

Example:
  hand-eye-test status --host my-robot.viam.cloud

Flags:
`)
			fs.PrintDefaults()
		}
		host, debug = addConnectionFlags(fs)
		armName, cameraName, gripperName = addComponentFlags(fs)
		if err := fs.Parse(args); err != nil {
			return err
		}
		cfg = Config{
			Arm: *armName, Camera: *cameraName, Gripper: *gripperName,
		}
		cmdMap = map[string]interface{}{"command": "status"}

	default:
		return fmt.Errorf("unknown subcommand: %s", subcommand)
	}

	if *debug {
		logger.SetLevel(logging.DEBUG)
	}

	if *host == "" {
		return fmt.Errorf("--host is required")
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
