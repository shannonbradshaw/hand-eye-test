package handeyetest

import (
	"fmt"

	"github.com/golang/geo/r3"
	"go.viam.com/rdk/resource"
	generic "go.viam.com/rdk/services/generic"
)

var Model = resource.NewModel("shannon", "hand-eye-test", "calibration-tester")

func init() {
	resource.RegisterService(generic.API, Model,
		resource.Registration[resource.Resource, *Config]{
			Constructor: newHandEyeTest,
		},
	)
}

type SegmentationConfig struct {
	MinPtsInPlane      int       `json:"min_pts_in_plane"`
	MaxDistFromPlane   float64   `json:"max_dist_from_plane_mm"`
	GroundNormal       []float64 `json:"ground_normal"`
	AngleTolerance     float64   `json:"angle_tolerance_deg"`
	MinPtsInSegment    int       `json:"min_pts_in_segment"`
	ClusteringRadiusMm float64   `json:"clustering_radius_mm"`
	MeanKFiltering     int       `json:"mean_k_filtering"`
}

func (sc *SegmentationConfig) groundNormalVec() r3.Vector {
	if len(sc.GroundNormal) == 3 {
		return r3.Vector{X: sc.GroundNormal[0], Y: sc.GroundNormal[1], Z: sc.GroundNormal[2]}
	}
	return r3.Vector{X: 0, Y: 0, Z: 1}
}

type Config struct {
	Arm                string             `json:"arm"`
	Camera             string             `json:"camera"`
	Gripper            string             `json:"gripper"`
	ApproachOffsetMm   float64            `json:"approach_offset_mm"`
	GraspDepthOffsetMm float64            `json:"grasp_depth_offset_mm"`
	LiftHeightMm       float64            `json:"lift_height_mm"`
	Segmentation       SegmentationConfig `json:"segmentation"`
}

func (cfg *Config) Validate(path string) ([]string, []string, error) {
	if cfg.Arm == "" {
		return nil, nil, fmt.Errorf("%s: arm is required", path)
	}
	if cfg.Camera == "" {
		return nil, nil, fmt.Errorf("%s: camera is required", path)
	}
	if cfg.Gripper == "" {
		return nil, nil, fmt.Errorf("%s: gripper is required", path)
	}
	if cfg.ApproachOffsetMm == 0 {
		cfg.ApproachOffsetMm = 100
	}
	if cfg.LiftHeightMm == 0 {
		cfg.LiftHeightMm = 50
	}
	if cfg.Segmentation.MinPtsInPlane == 0 {
		cfg.Segmentation.MinPtsInPlane = 1500
	}
	if cfg.Segmentation.MaxDistFromPlane == 0 {
		cfg.Segmentation.MaxDistFromPlane = 5.0
	}
	if cfg.Segmentation.AngleTolerance == 0 {
		cfg.Segmentation.AngleTolerance = 20
	}
	if cfg.Segmentation.MinPtsInSegment == 0 {
		cfg.Segmentation.MinPtsInSegment = 100
	}
	if cfg.Segmentation.ClusteringRadiusMm == 0 {
		cfg.Segmentation.ClusteringRadiusMm = 5.0
	}
	if cfg.Segmentation.MeanKFiltering == 0 {
		cfg.Segmentation.MeanKFiltering = 50
	}
	deps := []string{cfg.Arm, cfg.Camera, cfg.Gripper}
	return deps, nil, nil
}
