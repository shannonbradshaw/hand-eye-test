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

Authenticate first with `viam login`, then run commands against a remote machine.

### detect

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

### move-to

Incrementally move the gripper to a target position in world frame. Each step
uses the motion service for obstacle-aware planning.

```bash
./bin/hand-eye-test move-to --host my-robot.viam.cloud --x 400 --y 100 --z 50
```

## CLI flags

**Common flags** (all commands):

| Flag | Default | Description |
|------|---------|-------------|
| `--host` | (required) | Machine address |
| `--debug` | false | Enable debug logging |
| `--arm` | `arm` | Arm component name |
| `--camera` | `camera` | Camera component name |
| `--gripper` | `gripper` | Gripper component name |
| `--detection-frame` | camera name | Frame for detected coordinates |

**Segmentation tuning** (detect):

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

**Move-to flags**:

| Flag | Default | Description |
|------|---------|-------------|
| `--x` | 0 | Target X position (mm) in world frame |
| `--y` | 0 | Target Y position (mm) in world frame |
| `--z` | 0 | Target Z position (mm) in world frame |
| `--step-size` | 20 | Distance (mm) per incremental move |
