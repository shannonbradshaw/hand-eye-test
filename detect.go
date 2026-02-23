package handeyetest

import (
	"context"
	"fmt"

	"github.com/golang/geo/r3"

	"go.viam.com/rdk/components/camera"
	pc "go.viam.com/rdk/pointcloud"
	"go.viam.com/rdk/vision/segmentation"
)

// DetectedObject represents an object found in the point cloud.
type DetectedObject struct {
	Center     r3.Vector
	PointCount int
}

// detectObjects captures a point cloud from the camera and runs plane segmentation
// followed by radius clustering to find objects. The returned centers are in the camera frame.
func detectObjects(ctx context.Context, cam camera.Camera, cfg *Config) ([]DetectedObject, error) {
	segCfg := &segmentation.RadiusClusteringConfig{
		MinPtsInPlane:      cfg.Segmentation.MinPtsInPlane,
		MaxDistFromPlane:   cfg.Segmentation.MaxDistFromPlane,
		NormalVec:          cfg.Segmentation.groundNormalVec(),
		AngleTolerance:     cfg.Segmentation.AngleTolerance,
		MinPtsInSegment:    cfg.Segmentation.MinPtsInSegment,
		ClusteringRadiusMm: cfg.Segmentation.ClusteringRadiusMm,
		MeanKFiltering:     cfg.Segmentation.MeanKFiltering,
	}

	if err := segCfg.CheckValid(); err != nil {
		return nil, fmt.Errorf("invalid segmentation config: %w", err)
	}

	objects, err := segCfg.RadiusClustering(ctx, cam)
	if err != nil {
		return nil, fmt.Errorf("segmentation failed: %w", err)
	}

	if len(objects) == 0 {
		return nil, nil
	}

	var detected []DetectedObject
	for _, obj := range objects {
		center := computeCenter(obj)
		if cfg.Segmentation.MaxDepthMm > 0 && center.Z > cfg.Segmentation.MaxDepthMm {
			continue
		}
		if cfg.Segmentation.MaxPointCount > 0 && obj.Size() > cfg.Segmentation.MaxPointCount {
			continue
		}
		detected = append(detected, DetectedObject{
			Center:     center,
			PointCount: obj.Size(),
		})
	}

	return detected, nil
}

// computeCenter computes the mean position of all points in a point cloud.
func computeCenter(cloud pc.PointCloud) r3.Vector {
	var sum r3.Vector
	count := 0
	cloud.Iterate(0, 0, func(p r3.Vector, _ pc.Data) bool {
		sum = sum.Add(p)
		count++
		return true
	})
	if count == 0 {
		return r3.Vector{}
	}
	return r3.Vector{
		X: sum.X / float64(count),
		Y: sum.Y / float64(count),
		Z: sum.Z / float64(count),
	}
}
