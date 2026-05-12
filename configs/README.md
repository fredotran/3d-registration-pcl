# Example Configurations

This directory contains example parameter configuration files for different registration pipelines.

## Available Configurations

### sift_pipeline.txt
- **Pipeline**: SIFT-based registration
- **Features**: Uses SIFT (Scale-Invariant Feature Transform) keypoints
- **Best for**: Scenes with distinctive scale-invariant features
- **Parameters**: Optimized for general-purpose SIFT detection

### harris_pipeline.txt
- **Pipeline**: Harris 3D-based registration
- **Features**: Uses Harris 3D corner detector
- **Best for**: Scenes with sharp corners and edges
- **Parameters**: Optimized for corner detection

### allpoints_pipeline.txt
- **Pipeline**: All-points registration
- **Features**: Uses all points in the cloud (no keypoint detection)
- **Best for**: Small point clouds or when computational resources are limited
- **Parameters**: Optimized for full point cloud alignment

## Using Configuration Files

Copy an example configuration and modify it for your needs:

```bash
cp configs/sift_pipeline.txt my_experiment.txt
# Edit my_experiment.txt with your parameters
./main_registration my_experiment.txt
```

## Parameter Descriptions

### Common Parameters
- `pipelineType`: Registration pipeline (sift, harris, allpoints)
- `typeTransformation`: Transformation type (rotation, translation, etc.)
- `surface_model_data_file`: Path to reference point cloud file
- `x_uncertainty`, `y_uncertainty`: Positional uncertainty (meters)
- `sourceWidth`, `sourceHeight`: Source cloud dimensions (meters)
- `x_angle_min/max`, `y_angle_min/max`, `z_angle_min/max`: Rotation angle ranges (degrees)

### Normal Estimation
- `normals_search_radius`: Search radius for normal estimation (meters)

### FPFH Descriptors
- `fpfh_search_radius`: Search radius for FPFH computation (meters)

### SIFT Detector
- `sift_min_scale_source/target`: Minimum scale for SIFT
- `sift_num_octaves_source/target`: Number of octaves
- `sift_num_scales_per_octave_source/target`: Scales per octave
- `sift_min_contrast_source/target`: Minimum contrast threshold

### Harris 3D Detector
- `harris_search_radius`: Search radius for Harris detection (meters)
- `harris_threshold`: Harris response threshold

### SAC-IA Alignment
- `sacia_min_sample_distance`: Minimum sample distance (meters)
- `sacia_max_correspondence_distance`: Maximum correspondence distance (meters)
- `sacia_num_iterations`: Number of RANSAC iterations
- `sacia_num_samples`: Number of samples per iteration

## Parameter Tuning Guidelines

### For Better Accuracy
- Increase `sacia_num_iterations` (slower but more accurate)
- Decrease `sacia_max_correspondence_distance` (stricter matching)
- Adjust search radii based on point cloud density

### For Faster Processing
- Decrease `sacia_num_iterations`
- Use `allpoints` pipeline for small clouds
- Increase search radii to reduce computation

### For Different Scene Types
- **Indoor scenes**: Smaller search radii, higher thresholds
- **Outdoor scenes**: Larger search radii, lower thresholds
- **Structured scenes**: Use Harris pipeline
- **Natural scenes**: Use SIFT pipeline

## Tips

1. Start with example configurations and tune gradually
2. Keep a log of parameter changes and results
3. Use the grid search experiments for systematic parameter exploration
4. Visualize intermediate results to understand pipeline behavior
5. Test on similar scenes before applying to new data
