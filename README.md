# hand-eye-test

Validate hand-eye calibration by detecting objects via point cloud segmentation
and commanding a robot arm to pick them up. Reports calibration accuracy as
millimeter offsets between where the camera thinks an object is and where the
gripper actually goes.

Runs as a **Viam module** (on the robot) or as a **CLI tool** (from your laptop).
Same binary, same code path.

## Prerequisites

A machine configured in the Viam app with:

- An **arm** component (any model — real or `fake`)
- A **depth camera** mounted on or near the arm (must produce point clouds)
- A **gripper** component
- The **builtin motion service** (automatically available)
- A **frame system** that relates the camera to the arm's end effector
  (this is the calibration you're validating)

## Build

```bash
make build        # creates bin/hand-eye-test
make module       # creates module.tar.gz for the Viam registry
```

## Usage

### CLI mode

Authenticate first with `viam login`, then run commands against a remote machine.

#### detect

Capture a point cloud and find objects via plane segmentation + clustering.

```bash
./bin/hand-eye-test detect --host my-robot.viam.cloud
```

Returns JSON:

```json
{
  "objects": [
    {"index": 0, "point_count": 842, "center_x_mm": 12.3, "center_y_mm": -5.1, "center_z_mm": 310.7}
  ],
  "count": 1
}
```

#### pick

Full pick sequence: detect, open gripper, move to approach position, re-detect,
descend to grasp, grab, lift, verify.

```bash
./bin/hand-eye-test pick --host my-robot.viam.cloud --object 0
```

Returns calibration measurements:

```json
{
  "success": true,
  "is_holding": true,
  "detected_position": {"x_mm": 12.3, "y_mm": -5.1, "z_mm": 310.7, "frame": "wrist-cam"},
  "object_position_world_frame": {"x_mm": 413.2, "y_mm": 731.0, "z_mm": 45.3},
  "gripper_position_world_frame": {"x_mm": 414.1, "y_mm": 730.5, "z_mm": 44.8},
  "approach_offset_mm": {"x": 0.8, "y": -0.3, "z": 1.2, "total": 1.5},
  "world_frame_offset_mm": {"x": 0.9, "y": -0.5, "z": -0.5, "total": 1.2},
  "steps_completed": ["open_gripper", "approach", "re_detect", "grasp_move", "world_frame_check", "grab", "lift", "verify"]
}
```

Key metrics:
- **approach_offset_mm**: difference between first detection and re-detection
  after moving to the approach position. Large values mean the object shifts
  in camera coordinates as the arm moves — indicates a calibration problem.
- **world_frame_offset_mm**: difference between where the gripper actually is
  (in world frame) and where the camera said the object was (transformed to
  world frame). This is the most direct measure of calibration accuracy.
  Good calibration: < 5 mm total.

#### move-to

Incrementally move the gripper to a target position in world frame. Each step
uses the motion service for obstacle-aware planning.

```bash
./bin/hand-eye-test move-to --host my-robot.viam.cloud --x 400 --y 100 --z 50
```

#### status

Return current service state and last result.

```bash
./bin/hand-eye-test status --host my-robot.viam.cloud
```

### CLI flags

**Common flags** (all commands):

| Flag | Default | Description |
|------|---------|-------------|
| `--host` | (required) | Machine address |
| `--debug` | false | Enable debug logging |
| `--arm` | `arm` | Arm component name |
| `--camera` | `camera` | Camera component name |
| `--gripper` | `gripper` | Gripper component name |
| `--detection-frame` | camera name | Frame for detected coordinates |

**Segmentation tuning** (detect, pick):

| Flag | Default | Description |
|------|---------|-------------|
| `--min-plane-pts` | 1500 | Min points to identify the ground plane |
| `--max-dist-from-plane` | 5.0 | Max distance (mm) from plane to be part of it |
| `--angle-tolerance` | 20 | Angle tolerance (degrees) for ground normal matching |
| `--min-pts` | 100 | Min points per object cluster |
| `--clustering-radius` | 5.0 | Radius (mm) for clustering |
| `--mean-k` | 50 | Mean-k for statistical noise filtering |
| `--max-depth` | 0 | Max depth (mm); 0 = no limit |
| `--max-pts` | 0 | Max points per cluster; 0 = no limit |

**Pick flags**:

| Flag | Default | Description |
|------|---------|-------------|
| `--object` | 0 | Index of detected object to pick |
| `--approach-offset` | 100 | Distance (mm) above object for approach pose |
| `--grasp-offset` | 0 | Depth adjustment (mm) for grasp position |
| `--lift-height` | 50 | Distance (mm) to lift after grasping |

**Move-to flags**:

| Flag | Default | Description |
|------|---------|-------------|
| `--x` | 0 | Target X position (mm) in world frame |
| `--y` | 0 | Target Y position (mm) in world frame |
| `--z` | 0 | Target Z position (mm) in world frame |
| `--step-size` | 20 | Distance (mm) per incremental move |

### Module mode

Add the module to your machine config as a generic service:

```json
{
  "services": [
    {
      "type": "generic",
      "name": "hand-eye-test",
      "model": "shannon:hand-eye-test:calibration-tester",
      "attributes": {
        "arm": "my-arm",
        "camera": "wrist-cam",
        "gripper": "my-gripper",
        "detection_frame": "wrist-cam",
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
    }
  ]
}
```

Trigger commands via DoCommand from the Viam app or any SDK client:

```json
{"command": "detect"}
{"command": "pick", "object_index": 0}
{"command": "pick_detected", "object_index": 0}
{"command": "move_to", "x": 400, "y": 100, "z": 50, "step_size": 20}
{"command": "status"}
```

`pick_detected` skips detection and picks from the last `detect` result.

## How the pick sequence works

1. **Open gripper** — ensure gripper is ready
2. **Detect** — capture point cloud, remove ground plane, cluster remaining
   points, compute center of target object
3. **Approach** — move to a position offset above the object using the motion
   service (obstacle-aware, uses frame system transforms)
4. **Re-detect** — capture another point cloud from the approach position and
   measure how much the detection shifted (approach offset)
5. **Descend to grasp** — move directly to the object using `arm.MoveToPosition`
6. **World-frame check** — query the gripper's actual pose in world frame and
   compare to where the camera said the object was (world-frame offset)
7. **Grab** — close the gripper
8. **Lift** — move upward by `lift_height_mm`
9. **Verify** — call `gripper.IsHoldingSomething()`

## How detection works

The detection pipeline uses point cloud segmentation:

1. Capture a point cloud from the depth camera
2. **Plane segmentation**: identify the ground/table plane using the configured
   ground normal vector and remove it
3. **Radius clustering**: group remaining points into object clusters
4. **Filtering**: remove clusters that are too small, too far, or have too many
   points (configurable thresholds)
5. Return object centers as mean positions of each cluster

## Configuration reference

| Attribute | Type | Default | Description |
|-----------|------|---------|-------------|
| `arm` | string | — | Arm component name (required) |
| `camera` | string | — | Camera component name (required) |
| `gripper` | string | — | Gripper component name (required) |
| `detection_frame` | string | camera name | Frame for detected coordinates |
| `approach_offset_mm` | float | 100 | Distance above object for approach |
| `grasp_depth_offset_mm` | float | 0 | Depth adjustment for grasp |
| `lift_height_mm` | float | 50 | Lift distance after grasping |
| `segmentation.min_pts_in_plane` | int | 1500 | Min points for ground plane |
| `segmentation.max_dist_from_plane_mm` | float | 5.0 | Plane thickness |
| `segmentation.ground_normal` | [3]float | [0,0,1] | Expected ground normal |
| `segmentation.angle_tolerance_deg` | float | 20 | Normal matching tolerance |
| `segmentation.min_pts_in_segment` | int | 100 | Min points per object |
| `segmentation.clustering_radius_mm` | float | 5.0 | Clustering radius |
| `segmentation.mean_k_filtering` | int | 50 | Noise filter parameter |
| `segmentation.max_depth_mm` | float | 0 | Max depth filter (0 = off) |
| `segmentation.max_point_count` | int | 0 | Max points per cluster (0 = off) |
