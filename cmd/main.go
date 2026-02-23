package main

import (
	"fmt"
	"os"

	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"

	handeyetest "handeyetest"
)

func printUsage() {
	fmt.Fprintf(os.Stderr, `hand-eye-test - validate hand-eye calibration by moving an arm to detected objects

Usage:
  hand-eye-test <command> [flags]

Commands:
  detect    Capture a point cloud and detect objects via plane segmentation + clustering.
            Returns object positions in the camera (or detection) frame.

  pick      Detect objects, then execute a full pick sequence on one of them:
            open gripper -> approach -> re-detect -> grasp -> grab -> lift -> verify.
            Reports calibration accuracy (approach offset and world-frame offset in mm).

  move-to   Incrementally move the gripper to a world-frame coordinate using the
            motion service. Useful for testing reachability and collision geometry.

  status    Return the current service status and last result.

Run 'hand-eye-test <command> --help' for flag details on a specific command.

Authentication:
  Requires a valid Viam CLI token. Run 'viam login' first.
`)
}

func main() {
	// If first arg is a known CLI subcommand, run in CLI mode.
	// Otherwise, run as a Viam module (viam-server passes a socket path as arg).
	if len(os.Args) > 1 {
		switch os.Args[1] {
		case "detect", "pick", "move-to", "status":
			handeyetest.RunCLI(os.Args[1], os.Args[2:])
			return
		case "--help", "-help", "-h", "help":
			printUsage()
			return
		}
	}

	module.ModularMain(
		resource.APIModel{API: generic.API, Model: handeyetest.Model},
	)
}
