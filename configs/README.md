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

### scenario_a.txt / scenario_b.txt
- **Pipeline**: Two-epoch scenario evaluation (Sabel et al., ICAS 2022, Table 2)
- **Features**: PCUAV cropped from a separate UAV survey cloud and displaced by random horizontal translation, heading, pitch/roll and scale errors, then registered against a PCref crop from the reference survey
- **Best for**: Reproducing the GNSS-denied localisation experiments of the paper
- **Run with**: `./scenario_registration ../configs/scenario_a.txt 100`

### benchmark_nibio.txt
- **Pipeline**: Two-epoch scenario evaluation on real public data (FOR-instance NIBIO, Zenodo record 8287792, CC BY 4.0)
- **Features**: `uav_data_file` is the NIBIO plots 17/18/22/23 UAV laser scan (Riegl VUX, boreal forest, Norway, ~55x52 m) nested inside a reference block merged from plots 16-19 and 21-23 (~56x101 m) — a real PCUAV-inside-PCref setup in UTM 32N coordinates (converted to a local frame with a common offset by the fetch script)
- **Data setup**: `pip install remotezip laspy numpy`, then `python3 scripts/fetch_benchmark_data.py --voxel 0.15 --ref-plots 16 17 18 19 21 22 23 --uav-plots 17 18 22 23` — streams only the needed LAS members (~470 MB transferred) out of the 1.6 GB archive and writes `data/benchmarks/nibio_{ref,uav}.pcd` (gitignored)
- **Pipeline choice**: use `all` — the Harris/ISS/SIFT detectors are tuned for sparse ALS and return far too few keypoints on dense UAV canopy
- **Caveat**: single acquisition epoch, so this benchmarks registration quality on real forest geometry/density, not cross-epoch change; the PCref extent must fit inside the UAV cloud footprint, so plot selection controls the maximum `sourceWidth` + translation range
- **Runtime**: ~10-15 min per iteration on the merged block (normals + FPFH on ~3.2M points + SAC-IA)
- **Run with**: `./scenario_registration ../configs/benchmark_nibio.txt 3`

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
- `uav_data_file` (optional): Path to the UAV survey point cloud used to simulate PCUAV. When set, two-epoch scenario mode is enabled and the source is cropped from this cloud at the same georeferenced anchor as the reference crop. When omitted, the source is cropped from inside the reference cloud (legacy single-cloud mode)
- `x_uncertainty`, `y_uncertainty`: Positional uncertainty (meters)
- `sourceWidth`, `sourceHeight`: Source cloud dimensions (meters)
- `x_angle_min/max`, `y_angle_min/max`, `z_angle_min/max`: Rotation angle ranges (degrees)
- `x/y/z_translation_min/max` (optional, default 0): A priori translation error ranges (meters)
- `scale_min/max` (optional, default 1.0): A priori scale error range (factor, e.g. 0.98-1.02 for ±2%)

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
