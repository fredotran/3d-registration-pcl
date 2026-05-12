# 3D Point Cloud Registration using PCL

A C++ implementation of 3D point cloud registration algorithms for large-scale point clouds using the Point Cloud Library (PCL). The project provides multiple registration pipelines including SIFT-based, Harris 3D-based, and all-points SAC-IA (Sample Consensus Initial Alignment) approaches.

## Requirements

* [Ubuntu (20.04 or higher)](https://ubuntu.com/download/desktop) or similar Linux distribution
* [CMake (3.10 or higher)](https://cmake.org/runningcmake/)
* [Point Cloud Library (PCL) 1.7 or higher](https://pointclouds.org/downloads/)
* C++17 compatible compiler

## Building the Project

### Native Build

```bash
# Clone the repository
cd /path/to/3d-registration-pcl

# Create build directory and configure
cmake -B build

# Build the project
cmake --build build

# Or alternatively:
cd build
make
```

### Docker Build

The project includes a Dockerfile for easy containerized builds. See [DOCKER.md](DOCKER.md) for detailed Docker instructions.

Quick start with Docker:

```bash
# Build the Docker image
docker-compose build

# Run the container
docker-compose up -d

# Enter the container
docker-compose exec pcl-registration bash

# Run the demo
cd build
../run_demo.sh
```

## Project Structure

```
3d-registration-pcl/
├── include/              # Header files
│   ├── parameters.hpp   # Type definitions and parameter constants
│   ├── Settings.hpp     # Settings management class
│   ├── file_io.hpp      # File I/O and CSV result saving
│   ├── registration.hpp # Registration pipelines (SIFT, Harris, SAC-IA)
│   ├── tools.hpp        # Transformation helpers and pipeline orchestration
│   ├── visualization_tools.hpp # PCL visualization utilities
│   ├── randomization.hpp # Random number generation utilities
│   └── param_parsing.hpp # Parameter parsing from text files
├── src/                 # Implementation files and executables
│   ├── parameters.cpp   # Default settings implementation
│   ├── visualization.cpp # Point cloud visualization executable
│   ├── cpp_version.cpp  # C++ version check utility
│   ├── main_registration.cpp # Main registration executable
│   ├── experiment2.cpp  # Experiment 2 executable
│   ├── experiment2_ds.cpp # Downsampled experiment 2
│   ├── verification_test1.cpp # Verification test 1
│   ├── verification_test2.cpp # Verification test 2
│   ├── fred_exp_sacia_param_grid_search.cpp # SAC-IA parameter grid search
│   └── exp_sacia_param_grid_search.cpp # Additional SAC-IA experiments
├── tests/               # Unit tests
│   ├── CMakeLists.txt   # Test build configuration
│   ├── test_main.cpp    # Test main entry point
│   ├── test_settings.cpp # Settings class tests
│   └── test_file_io.cpp  # File I/O tests
├── data/                # Point cloud data and parameter files
├── results/             # CSV output files with registration results
├── CMakeLists.txt       # CMake build configuration
├── Dockerfile           # Docker container configuration
├── docker-compose.yml   # Docker Compose configuration
├── run_demo.sh          # Demo execution script
├── .clang-format        # Code formatting configuration
├── DOCKER.md            # Detailed Docker usage documentation
└── README.md            # This file
```

## Architecture

The project follows a modular architecture with clear separation of concerns:

### Core Components

1. **Registration Pipelines** (`registration.hpp`)
   - **Detector classes**: SIFT, Harris 3D, and BRISK keypoint detectors
   - **Descriptor class**: FPFH (Fast Point Feature Histograms) computation
   - **SearchingMethods class**: SAC-IA (Sample Consensus Initial Alignment) implementation
   - **Pipeline functions**: `siftPipeline()`, `harrisPipeline()`, `pipelineAllPoints()`

2. **Transformation Tools** (`tools.hpp`)
   - Point cloud loading and saving
   - Custom rotation and translation transformations
   - Point cloud cropping and reference/source cloud generation
   - Full pipeline orchestration with experiment setup

3. **Parameter Management** (`parameters.hpp`, `Settings.hpp`, `param_parsing.hpp`)
   - Type definitions and PCL includes
   - Parameter constants and default values
   - Settings class for runtime parameter management
   - Parameter file parsing from text files

4. **File I/O** (`file_io.hpp`)
   - CSV result saving with automatic header generation
   - Parameter file reading with validation
   - File existence checking and extension handling

5. **Utilities**
   - **Visualization tools** (`visualization_tools.hpp`): PCL visualizer utilities
   - **Randomization** (`randomization.hpp`): Seeded random number generation for reproducible experiments

### Data Flow

```
Surface Model Point Cloud
         ↓
    Crop to Reference Cloud
         ↓
    Crop to Source Cloud
         ↓
    Apply Random Transformation
         ↓
    Compute Normals
         ↓
    Detect Keypoints (SIFT/Harris/None)
         ↓
    Compute FPFH Descriptors
         ↓
    SAC-IA Alignment
         ↓
    Apply Final Transformation
         ↓
    Compute Error Metrics (MTRE, Bias)
         ↓
    Save Results to CSV
```

### Design Patterns

- **Template-based pipeline**: Generic pipeline template handles different detector types
- **Strategy pattern**: Different detectors (SIFT, Harris) can be swapped
- **Factory pattern**: Pipeline functions create appropriate output structures
- **Builder pattern**: Settings class builds configuration from parameters

### Key Design Decisions

1. **Modular detectors**: Each detector is encapsulated in its own class for easy extension
2. **Template-based descriptors**: FPFH computation is templated for different keypoint types
3. **Separated concerns**: Registration logic is separate from file I/O and visualization
4. **Reproducible experiments**: Seeded randomization ensures consistent results
5. **Configuration-driven**: All parameters can be specified in external text files

### Error Handling

- File operations throw exceptions on failure
- Parameter parsing validates input and provides descriptive error messages
- Point cloud operations handle NaN values gracefully
- Empty data checks prevent undefined behavior

## Testing

The project includes unit tests using Google Test. To build and run tests:

```bash
# Build with tests enabled (default)
cmake -B build -DBUILD_TESTS=ON
cmake --build build

# Run tests
cd build
ctest
```

To disable tests:
```bash
cmake -B build -DBUILD_TESTS=OFF
```

## Available Executables

The project builds several executables for different registration experiments:

- `main_registration` - Main registration pipeline with SIFT and Harris detectors
- `experiment2` - Experimental registration pipeline
- `experiment2_ds` - Downsampled variant of experiment2
- `verification_test1` - First verification test
- `verification_test2` - Second verification test
- `fred_exp_sacia_param_grid_search` - SAC-IA parameter grid search experiments
- `exp_sacia_param_grid_search` - Additional SAC-IA parameter experiments
- `visualization` - Point cloud visualization tool
- `cpp_version` - C++ version check utility
- `point_cloud_converter` - Convert between different point cloud formats (PCD, PLY, OBJ, VTK)
- `benchmark_registration` - Performance benchmarking suite (requires BUILD_BENCHMARKS=ON)

## Usage

### Basic Usage

Most executables follow the pattern:

```bash
cd build
./executable_name [number_of_iterations]
```

For example:

```bash
# Run main registration with 10 iterations
./main_registration 10
```

### Parameter Files

Parameter files are simple text files with `key=value` format stored in the `data/` directory. Example format:

```
pipelineType=sift
typeTransformation=rotation
surface_model_data_file=../data/reference_LM.pcd
x_uncertainty=5.0
y_uncertainty=5.0
sourceWidth=100.0
sourceHeight=100.0
x_angle_min=-10.0
x_angle_max=10.0
y_angle_min=-10.0
y_angle_max=10.0
z_angle_min=-10.0
z_angle_max=10.0
```

### Data Format

The framework supports multiple point cloud formats:

- **PCD** (Point Cloud Data) - Native PCL format
- **PLY** (Polygon File Format)
- **OBJ** (Wavefront OBJ)
- **VTK** (Visualization Toolkit)

#### Converting Between Formats

A utility tool is provided for converting between different point cloud formats:

```bash
cd build
./point_cloud_converter input.ply output.pcd
./point_cloud_converter input.pcd output.ply binary
```

#### Using CloudCompare

If you have data in other formats (.las, .laz, .xyz), you can convert them using [CloudCompare](https://www.danielgm.net/cc/):

1. Load your file in CloudCompare
2. Click File > Save
3. Choose "Point Cloud Library format" (.pcd)
4. Save the file

Place your point cloud files in the `data/` directory.

## Registration Pipelines

The project implements three main registration approaches:

### 1. SIFT-based Pipeline
- Uses SIFT (Scale-Invariant Feature Transform) keypoints for feature detection
- Computes FPFH (Fast Point Feature Histograms) descriptors
- Performs alignment using SAC-IA

### 2. Harris 3D Pipeline
- Uses Harris 3D corner detector for keypoint extraction
- Computes FPFH descriptors on detected keypoints
- Performs alignment using SAC-IA

### 3. All-Points Pipeline
- Uses all points in the cloud (no keypoint detection)
- Computes FPFH descriptors on entire point cloud
- Performs alignment using SAC-IA

## Output Format

Registration results are saved as CSV files in the `results/` directory. Each CSV file contains:

- Pipeline type used
- Mean Target Registration Error (MTRE)
- Registration error bias (x, y, z components)
- Applied rotation angles (x, y, z)
- Source dimensions and uncertainty parameters
- Dataset filename
- Seed values used for randomization
- All pipeline parameters (search radii, thresholds, etc.)

## Default Parameters

The following default parameters are used by `getPipelineDefaultSettings()`:

- Normal search radius: 2.0
- FPFH search radius: 2.0
- Harris search radius: 2.0
- Harris threshold: 1e-6
- SIFT min scale (source/target): 0.1
- SIFT number of octaves (source/target): 6
- SIFT scales per octave (source/target): 4
- SIFT min contrast (source/target): 0.001
- SAC-IA min sample distance: 3.0
- SAC-IA max correspondence distance: 4.0
- SAC-IA iterations: 400
- SAC-IA samples per iteration: 3
- Visualizer parameter: 0.0
- BRISK threshold: 0.0

These can be overridden by providing custom parameter files.

## Tools and Components

Key components located in the `include/` directory:

- **visualization_tools.hpp** - PCL visualization utilities for displaying point clouds
- **registration.hpp** - Core registration algorithms and pipeline implementations
- **parameters.hpp** - Type definitions, PCL includes, and parameter constants
- **tools.hpp** - Transformation helpers, pipeline orchestration, and error metrics
- **file_io.hpp** - File I/O operations and CSV result saving
- **randomization.hpp** - Random number generation for experiments
- **param_parsing.hpp** - Parameter file parsing utilities
- **Settings.hpp** - Simple settings management class using map<string, double>

## Error Metrics

The project computes two main error metrics:

1. **Mean Target Registration Error (MTRE)**: Average Euclidean distance between corresponding points in the original and transformed source clouds.

2. **Registration Error Bias**: Systematic error components in x, y, and z directions between the original and transformed source clouds.

## More Information

For more details on using PCL in your C++ projects, see the [PCL tutorial](https://pcl.readthedocs.io/projects/tutorials/en/latest/using_pcl_pcl_config.html#using-pcl-pcl-config).
