# 3D Point Cloud Registration using PCL

C++ implementation of 3D point cloud registration experiments with the Point
Cloud Library (PCL). Developed for evaluating vision-based localisation of UAVs
in GNSS-denied environments: a UAV point cloud (PCUAV) is registered against a
georeferenced reference cloud (PCref) under a priori pose errors, as described
in Sabel et al., "Vision-based localisation for autonomous aerial navigation in
GNSS-denied situations" (ICAS 2022).

## Features

- Feature-based registration: SIFT, Harris 3D and ISS keypoints with FPFH or
  SHOT descriptors and SAC-IA initial alignment
- Optional ICP refinement and correspondence rejection
- Experiment framework: reproducible, seeded crops and displacements with
  per-run CSV output
- Two-epoch scenario evaluation reproducing the displacement scenarios of the
  paper (translation, heading/pitch/roll and scale errors)
- Reads PCD, PLY, OBJ and VTK point clouds; converts between them
- Unit tests (Google Test) and a small benchmark harness

## Requirements

- CMake 3.10+, a C++17 compiler
- PCL 1.7+ (developed against 1.14)
- Google Test (optional, for the test suite)
- OpenMP (optional, used when available)

## Build

```bash
cmake -B build
cmake --build build
```

Executables are written to `build/`. All parameter files use paths relative to
the build directory, so run the programs from there:

```bash
cd build
```

Useful CMake options: `-DBUILD_TESTS=OFF`, `-DBUILD_EXAMPLES=OFF`,
`-DBUILD_TOOLS=OFF`, `-DBUILD_BENCHMARKS=ON`, `-DUSE_OPENMP=OFF`.

## Running the experiments

### Scenario evaluation (two-epoch mode)

Reproduces the a priori error scenarios of the paper: a `sourceWidth` ×
`sourceHeight` PCUAV footprint is cropped from a UAV survey cloud at a random
location, displaced by random translation, rotation and scale errors, then
registered against a PCref crop of the reference cloud around the same
location. PCref is sized so the displaced PCUAV is always contained.

```bash
./scenario_registration ../configs/scenario_a.txt 100
./scenario_registration ../configs/scenario_b.txt 100
```

Scenario A: ±30 m horizontal error per axis, ±5° heading, ±0.5° pitch/roll,
±2% scale. Scenario B: ±5 m horizontal, ±25° heading, ±5° pitch/roll, ±2%
scale. See `configs/README.md` for details, and point `uav_data_file` at a
second-epoch cloud for real experiments.

### Main registration loops

```bash
./main_registration <iterations> [all|sift|harris|iss]
```

Runs `fullRegistration` for each iteration using the parameter files in
`data/` (`harris_600m-40deg-FOV_rotation-Z.txt`, etc.) and appends results to
`results/*.csv`. Each iteration draws a new reference cutout, source crop and
random displacement.

### Other executables

| Executable | Purpose |
|-----------|---------|
| `main_registration` | Registration loops over the `data/` parameter files |
| `scenario_registration` | Parameter-file-driven scenario evaluation |
| `experiment2`, `experiment2_ds` | Older multi-pipeline experiment loops |
| `verification_test1`, `verification_test2` | Register two sample clouds, show the result in a PCL visualizer |
| `exp_sacia_param_grid_search`, `fred_exp_sacia_param_grid_search` | SAC-IA parameter sweeps |
| `point_cloud_converter` | `point_cloud_converter input.pcd output.ply [binary]` |
| `visualization` | `visualization <source.pcd> <target.pcd>` — side-by-side viewer |
| `benchmark_registration` | Timing harness (needs `BUILD_BENCHMARKS=ON`) |

`run_demo.sh` runs `verification_test1` from the build directory (starts Xvfb
automatically when no display is available).

## Parameter files

Key/value pairs, one per line. The required keys accept both `camelCase` and
the legacy `snake_case` spelling. Example:

```ini
pipelineType=harris
typeTransformation=rotation
surface_model_data_file=../data/reference_LM.pcd
uav_data_file=                # optional; enables two-epoch scenario mode
x_uncertainty=12.7
y_uncertainty=12.7
sourceWidth=180
sourceHeight=120
x_angle_min=0
x_angle_max=0
y_angle_min=0
y_angle_max=0
z_angle_min=-180
z_angle_max=180
# optional displacement ranges (default: 0 translation, scale 1.0)
x_translation_min=-30
x_translation_max=30
y_translation_min=-30
y_translation_max=30
z_translation_min=0
z_translation_max=0
scale_min=0.98
scale_max=1.02
```

- `pipelineType`: `all` (all points), `sift`, `harris`, `iss`, `shot`
- `uav_data_file`: when set, the source crop is taken from this cloud and the
  reference crop from `surface_model_data_file` at the same georeferenced
  anchor. When empty, the source is cropped from inside the reference cloud
- `x/y_uncertainty`: margin (m) used to size the reference cutout; in scenario
  mode only used as fallback when no translation range is given
- Rotation/translation ranges and `scale_min/max` bound the random a priori
  displacement applied to the source crop
- Example files: `configs/*.txt`, `data/*.txt`

Pipeline-level settings (search radii, SAC-IA iterations, ICP, ISS, voxel
downsampling, correspondence rejection) are set in
`getPipelineDefaultSettings()` and can be overridden through the `Settings`
object; see `include/parameters.hpp` for the setting names.

## Registration pipelines

All pipelines share the same structure: normals → keypoints (or all points) →
descriptors → SAC-IA initial alignment → optional correspondence rejection →
optional ICP refinement. The detectors/descriptors are:

- `all`: all points, FPFH (default `sacia` settings work best for small/medium clouds)
- `sift`: SIFT keypoints + FPFH
- `harris`: Harris 3D keypoints + FPFH
- `iss`: ISS keypoints + FPFH
- `shot`: all points, SHOT descriptors

## Metrics and output

Each run appends one row to a CSV in `results/` containing, among others:

- `err_MTRE`: mean target registration error — mean Euclidean distance between
  the registered displaced cloud and the ground-truth crop
- `err_MTRE_horizontal`: XY-only variant (the horizontal localisation error)
- `err_RMSE`, `err_inlier_ratio`, `err_precision`, `err_recall`, `err_f1_score`
- `err_bias_x/y/z`: systematic registration bias
- applied displacement: `angle_x/y/z`, `trans_x/y/z`, `scale`
- experiment parameters, dataset names and seeds for reproducibility

## Project structure

```
include/   parameters.hpp, registration.hpp, tools.hpp (public API + pipelines)
src/       library code (registration, tools, file_io, randomization) + drivers
configs/   example parameter files (incl. scenario_a/b.txt)
data/      sample point clouds and parameter files
tests/     Google Test suite (ctest)
benchmarks/timing harness
results/   CSV output
```

## Testing

```bash
cmake -B build -DBUILD_TESTS=ON
cmake --build build
cd build && ctest
```

## Docker

```bash
docker-compose build
docker-compose up -d
docker-compose exec pcl-registration bash
cd build && ../run_demo.sh
```

See `DOCKER.md` for details.

## Data formats

`loadingCloud()` supports `.pcd`, `.ply`, `.obj` and `.vtk`. For `.las`/`.laz`
or other formats, convert with CloudCompare or `pdal` to PCD first.
