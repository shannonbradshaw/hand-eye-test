# Hand-Eye Calibration Test Module

## Context

You have a robot arm with an Intel RealSense depth camera mounted at the wrist (eye-in-hand). The hand-eye calibration is already done and configured in Viam's frame system (camera pose relative to the end-effector). You need a Viam module to validate the calibration by detecting an object via point cloud segmentation and commanding the arm to pick it up. You also want a CLI client to control it from your laptop without needing to be on the robot.

## Architecture

**Single binary, two modes** — inspired by viam-chess and the hello world tutorial part 3:

- **Module mode** (default) - when viam-server invokes the binary, it passes a socket path as the argument. `module.ModularMain()` picks this up and runs the generic service inside viam-server.
- **CLI mode** - when you run the binary with a subcommand (`detect`, `pick`), it connects to a remote machine using `vmodutils`, creates the same service with remote dependencies, and calls DoCommand.

The same `NewHandEyeTest()` constructor works for both contexts because `vmodutils.MachineToDependencies()` makes remote resources look like local dependencies.

```
Module mode (on robot)                CLI mode (from laptop)
┌────────────────────┐                ┌──────────────────────┐
│ viam-server        │                │ hand-eye-test detect │
│  └─ hand-eye-test  │   gRPC         │  vmodutils.Connect() │
│     generic service│ ◄────────────► │  MachineToDeps()     │
│     DoCommand()    │                │  NewHandEyeTest()    │
│     - arm          │                │  DoCommand("detect") │
│     - camera       │                │                      │
│     - gripper      │                └──────────────────────┘
│     - motion svc   │
└────────────────────┘

# Module mode (viam-server invokes with socket path):
hand-eye-test /tmp/viam-module-xxxx.sock

# CLI mode (you invoke with subcommand):
hand-eye-test detect --host "robot.viam.cloud"
hand-eye-test pick --host "robot.viam.cloud" --object 0
```

## Dependencies (configured on the machine)

| Dependency | Type | Purpose |
|---|---|---|
| arm | `rdk:component:arm` | Robot arm to move |
| camera | `rdk:component:camera` | RealSense depth camera (point clouds) |
| gripper | `rdk:component:gripper` | End-effector gripper for pick |
| motion service | `rdk:service:motion` | Frame-aware motion planning (handles calibration transforms) |

The frame system config on the machine defines the camera's pose relative to the end-effector (the calibration result). The motion service uses this automatically when we express targets in the camera frame.

## DoCommand Interface

### `detect`
Capture point cloud from camera, run plane segmentation + clustering, return detected objects.

```json
{"command": "detect"}
```

Response:
```json
{
  "objects": [
    {"index": 0, "point_count": 342, "center_x_mm": 120.5, "center_y_mm": -45.2, "center_z_mm": 310.0}
  ],
  "count": 1
}
```

### `pick`
Full test cycle: detect, open gripper, approach, measure offset, grasp, measure world-frame error, grab, lift, verify.

```json
{"command": "pick", "object_index": 0}
```

Response:
```json
{
  "success": true,
  "is_holding": true,
  "detected_position_camera_frame": {"x_mm": 120.5, "y_mm": -45.2, "z_mm": 310.0, "frame": "wrist-cam"},
  "object_position_world_frame": {"x_mm": 450.2, "y_mm": 120.8, "z_mm": 15.0, "frame": "world"},
  "gripper_position_world_frame": {"x_mm": 450.5, "y_mm": 121.1, "z_mm": 15.2, "frame": "world"},
  "approach_offset_mm": {"x": 0.8, "y": -0.3, "z": 1.2, "total": 1.5},
  "world_frame_offset_mm": {"x": 0.3, "y": 0.3, "z": 0.2, "total": 0.5},
  "steps_completed": ["detect", "open_gripper", "approach", "re_detect", "grasp_position", "grab", "lift", "verify"]
}
```

### `pick_detected`
Pick a previously detected object (from the last `detect` call). Allows inspecting detection before triggering the pick.

```json
{"command": "pick_detected", "object_index": 0}
```

### `status`
Returns current state (idle, detecting, picking, etc.) and last result.

## Config

```json
{
  "arm": "my-arm",
  "camera": "wrist-cam",
  "gripper": "my-gripper",
  "approach_offset_mm": 100,
  "grasp_depth_offset_mm": 0,
  "lift_height_mm": 50,
  "segmentation": {
    "min_pts_in_plane": 1500,
    "max_dist_from_plane_mm": 5.0,
    "ground_normal": [0, 0, 1],
    "angle_tolerance_deg": 20,
    "min_pts_in_segment": 100,
    "clustering_radius_mm": 5.0,
    "mean_k_filtering": 50
  }
}
```

## Pick Sequence (core logic in `pick.go`)

1. **Open gripper** - `gripper.Open()`
2. **Detect object** - `camera.NextPointCloud()`, then `segmentation.SegmentPlaneWRTGround()` to remove the surface, then cluster remaining points to find objects. Record object position in camera frame.
3. **Get world-frame reference** - `motionService.GetPose()` to get gripper position in world frame. Also use `GetPose()` to transform detected object position to world frame. Record both for later comparison.
4. **Compute grasp pose** - Object center is in camera frame. Create approach pose (offset above object) and grasp pose as `referenceframe.NewPoseInFrame(cameraName, pose)`
5. **Move to approach** - `motionService.Move()` with destination in camera frame. The motion service resolves the frame transform (camera -> EE -> arm) using the calibration in the frame system config
6. **Measure offset at approach** - After reaching approach position, re-detect the object from this new vantage point. Compare the expected object position (where we think it is) vs. the newly observed position (where the camera now sees it). Report the offset in mm -- this measures how much the calibration drifted the estimate.
7. **Move to grasp** - `motionService.Move()` to the grasp pose in camera frame
8. **World-frame comparison** - Use `motionService.GetPose()` to get the gripper's actual world-frame position. Compare to the detected object's world-frame position from step 3. Report XYZ offset in mm.
9. **Grab** - `gripper.Grab()` returns whether something was grabbed
10. **Lift** - Move arm upward by `lift_height_mm`
11. **Verify** - `gripper.IsHoldingSomething()` confirms the pick
12. **Report** - Print comprehensive results:
    - **Pass/fail**: Did the gripper grab something?
    - **Approach offset**: mm error between expected and re-detected object position at approach
    - **World-frame offset**: mm error between gripper position and object's world position
    - **All positions**: Detected position (camera frame), planned grasp (camera frame), actual gripper position (world frame), object position (world frame)

## Directory Structure

```
hand-eye-test/
├── main.go                   # Single entry point: dispatches to module or CLI mode
├── meta.json                 # Module manifest (entrypoint: bin/hand-eye-test)
├── config.go                 # Config struct + Validate() (returns required deps)
├── hand_eye_test.go          # Service: NewHandEyeTest(), DoCommand() dispatcher
├── pick.go                   # Pick sequence logic
├── detect.go                 # Point cloud segmentation wrapper
├── cli.go                    # CLI mode: flag parsing, remote connection, subcommand dispatch
├── go.mod
├── go.sum
└── Makefile                  # Builds bin/hand-eye-test (single binary)
```

## Implementation Steps

### Step 1: Scaffold the project
- `go.mod` with dependencies: `go.viam.com/rdk`, `github.com/erh/vmodutils`
- `meta.json` for module registration (entrypoint: `bin/hand-eye-test`)
- `Makefile` with targets: `build` (single binary -> `bin/hand-eye-test`), `module.tar.gz`

### Step 2: Config and registration (`config.go`)
- Define `Config` struct with arm, camera, gripper names + segmentation params
- `Validate()` returns `[arm, camera, gripper]` as required dependencies (motion service is accessed via robot/deps)
- Register in `init()`: `resource.RegisterService(generic.API, Model, registration)`
- Model: e.g. `resource.NewModel("shannon", "hand-eye-test", "calibration-tester")`

### Step 3: Service constructor (`hand_eye_test.go`)
- `NewHandEyeTest(ctx, deps, name, cfg, logger)` - same constructor for module and CLI
- Extract deps: `arm.FromDependencies(deps, cfg.Arm)`, `camera.FromDependencies(deps, cfg.Camera)`, `gripper.FromDependencies(deps, cfg.Gripper)`
- Get motion service: `motion.FromDependencies(deps, "builtin")`
- Struct holds all clients + mutex-protected state (last detection, current status)

### Step 4: DoCommand dispatcher (`hand_eye_test.go`)
- Extract `cmd["command"]` string
- Switch on: `detect`, `pick`, `pick_detected`, `status`
- Route to handler methods
- Return results as `map[string]interface{}`

### Step 5: Object detection (`detect.go`)
- `detectObjects(ctx, cam, cfg)` function
- Fetch point cloud: `cam.NextPointCloud(ctx, nil)`
- Remove dominant plane: `segmentation.SegmentPlaneWRTGround()` with configured normal vector and thresholds
- Cluster remaining points using radius clustering from `rdk/vision/segmentation/`
- For each cluster, compute center position (mean of points) -- this is in camera frame
- Return `[]DetectedObject` with center positions and point counts

### Step 6: Pick sequence (`pick.go`)
- `executePick(ctx, clients, detectedObject, cfg)` function
- Open gripper
- Compute approach pose in camera frame:
  ```go
  approachPose := spatialmath.NewPose(
      r3.Vector{X: center.X, Y: center.Y, Z: center.Z - approachOffset},
      &spatialmath.OrientationVectorDegrees{OZ: 1, Theta: 0},
  )
  dest := referenceframe.NewPoseInFrame(cameraName, approachPose)
  ```
- `motionService.Move(ctx, motion.MoveReq{ComponentName: gripperName, Destination: dest})`
- Move to grasp pose, grab, lift, verify
- Return step-by-step results

### Step 7: Single entry point (`main.go`)
```go
func main() {
    // If first arg is a known CLI subcommand, run in CLI mode
    if len(os.Args) > 1 {
        switch os.Args[1] {
        case "detect", "pick", "status":
            runCLI(os.Args[1], os.Args[2:])
            return
        }
    }
    // Otherwise run as Viam module (viam-server passes socket path as arg)
    module.ModularMain(
        resource.APIModel{API: generic.API, Model: handeyetest.Model},
    )
}
```

### Step 8: CLI mode (`cli.go`)
- `runCLI(subcommand, args)` function
- Parse flags from args: `--host` (required), `--object` (for pick), plus config overrides
- Connect: `vmodutils.ConnectToHostFromCLIToken(ctx, host, logger)`
- Convert: `vmodutils.MachineToDependencies(machine)`
- Construct: `NewHandEyeTest(ctx, deps, name, &cfg, logger)`
- Build command map from subcommand and flags
- Execute: `svc.DoCommand(ctx, cmdMap)`
- Print results to stdout

### Step 9: Build and test
- `make build` -> `bin/hand-eye-test` (single binary)
- Install module on machine via Viam app (entrypoint: `bin/hand-eye-test`)
- From laptop: `viam login` then `./bin/hand-eye-test detect --host "robot.xyz.viam.cloud"`

## Key Viam APIs Used

| Package | Functions |
|---|---|
| `go.viam.com/rdk/robot/client` | `client.New()` |
| `go.viam.com/rdk/components/arm` | `FromDependencies()`, `EndPosition()` |
| `go.viam.com/rdk/components/camera` | `FromDependencies()`, `NextPointCloud()` |
| `go.viam.com/rdk/components/gripper` | `FromDependencies()`, `Open()`, `Grab()`, `IsHoldingSomething()` |
| `go.viam.com/rdk/services/motion` | `FromDependencies()`, `Move()` |
| `go.viam.com/rdk/vision/segmentation` | `SegmentPlaneWRTGround()`, radius clustering |
| `go.viam.com/rdk/referenceframe` | `NewPoseInFrame()` |
| `go.viam.com/rdk/spatialmath` | `NewPose()`, `Compose()` |
| `go.viam.com/rdk/pointcloud` | `PointCloud` interface |
| `go.viam.com/rdk/module` | `ModularMain()` |
| `go.viam.com/rdk/resource` | `RegisterService()`, `NativeConfig()` |
| `github.com/erh/vmodutils` | `ConnectToHostFromCLIToken()`, `MachineToDependencies()` |

## Code References

| What | Path |
|---|---|
| Module + CLI pattern | `viam-chess/cmd/module/main.go`, `viam-chess/cmd/cli/main.go` (our single binary merges both) |
| Service registration | `inspection-module-starter/module.go` |
| DoCommand dispatch | `kettle-cycle-test-demo/module.go` |
| vmodutils connectivity | `vmodutils/machine.go` (ConnectToHostFromCLIToken, MachineToDependencies) |
| Plane segmentation | `rdk/vision/segmentation/plane_segmentation.go` |
| Radius clustering | `rdk/vision/segmentation/radius_clustering.go` |
| Segmentation module example | `obstacles-pointcloud/obstacles-pointcloud.go` |
| Pose math | `rdk/spatialmath/pose.go` |
| Motion service | `rdk/services/motion/motion.go` |
| Point cloud gRPC | `rdk/components/camera/client.go` (NextPointCloud) |

## Verification

1. **Build**: `make build`
2. **Install module** on a test machine (via Viam app or local config) with arm, RealSense (frame system configured for hand-eye calibration), gripper, motion service
3. **Test detect** from CLI: `./bin/hand-eye-test detect --host "robot.viam.cloud"` -- should show detected objects with positions
4. **Test pick** from CLI: `./bin/hand-eye-test pick --host "robot.viam.cloud" --object 0` -- arm should move to object, grasp, lift
5. **Calibration quality**:
   - **Pass/fail**: Did the gripper actually grab the object?
   - **Approach offset**: After reaching the approach position, the module re-detects the object and reports how far the new detection is from the original. Large offset = calibration error (object isn't where the calibrated transform predicted).
   - **World-frame offset**: Compares the gripper's actual world-frame pose (from `GetPose`) to the object's world-frame pose. Reports XYZ mm offset and total Euclidean distance. This is the most direct measure of calibration accuracy.
   - A good calibration should show offsets under a few mm. Larger offsets indicate systematic calibration error.
6. **Module-side test**: Can also trigger commands via Viam app's DoCommand panel on the generic service
