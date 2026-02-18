package main

import (
	"os"

	"go.viam.com/rdk/module"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"

	handeyetest "handeyetest"
)

func main() {
	// If first arg is a known CLI subcommand, run in CLI mode.
	// Otherwise, run as a Viam module (viam-server passes a socket path as arg).
	if len(os.Args) > 1 {
		switch os.Args[1] {
		case "detect", "pick", "status":
			handeyetest.RunCLI(os.Args[1], os.Args[2:])
			return
		}
	}

	module.ModularMain(
		resource.APIModel{API: generic.API, Model: handeyetest.Model},
	)
}
