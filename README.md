# 3D Point Cloud Registration using PCL

A C++ implementation of 3D point cloud registration algorithms for large-scale point clouds using the Point Cloud Library (PCL).

## ✨ Features

- **Multiple Registration Pipelines**: SIFT-based, Harris 3D, and all-points SAC-IA approaches
- **Flexible Architecture**: Modular design with swappable detectors and descriptors
- **Multiple Data Formats**: Support for PCD, PLY, OBJ, and VTK point cloud formats
- **Reproducible Experiments**: Seeded randomization for consistent results
- **Comprehensive Testing**: Unit tests with Google Test framework
- **Docker Support**: Easy containerized builds and execution
- **Performance Benchmarking**: Built-in benchmarking suite for algorithm evaluation

## 🚀 Quick Start

### Prerequisites

- Ubuntu 20.04+ (or similar Linux distribution)
- CMake 3.10+
- Point Cloud Library (PCL) 1.7+
- C++17 compatible compiler

### Build & Run (Native)

```bash
# Clone and build
git clone <repository-url>
cd 3d-registration-pcl
cmake -B build && cmake --build build

# Run main registration with 10 iterations
cd build
./main_registration 10
```

### Build & Run (Docker)

```bash
# Build and run in Docker
docker-compose build
docker-compose up -d
docker-compose exec pcl-registration bash

# Inside container
cd build && ../run_demo.sh
```

See [DOCKER.md](DOCKER.md) for detailed Docker instructions.

## 📁 Project Structure

```
3d-registration-pcl/
├── include/              # Header files
│   ├── registration.hpp  # Core registration pipelines
│   ├── tools.hpp         # Transformation helpers & orchestration
│   ├── parameters.hpp    # Type definitions & constants
│   ├── Settings.hpp      # Settings management
│   ├── file_io.hpp       # File I/O & CSV saving
│   ├── visualization_tools.hpp # PCL visualization
│   ├── randomization.hpp # Random number generation
│   └── param_parsing.hpp # Parameter file parsing
├── src/                  # Implementation & executables
├── tests/                # Unit tests (Google Test)
├── data/                 # Point cloud data & parameters
├── results/              # CSV output files
├── CMakeLists.txt        # Build configuration
├── Dockerfile            # Docker setup
├── docker-compose.yml    # Docker Compose config
├── run_demo.sh           # Demo script
└── README.md             # This file
```

## 🏗️ Architecture Overview

### Core Components

| Component | Purpose |
|-----------|---------|
| **Registration Pipelines** | SIFT, Harris 3D, and all-points detectors with SAC-IA alignment |
| **Transformation Tools** | Point cloud loading, transformations, and pipeline orchestration |
| **Parameter Management** | Configuration handling and parameter file parsing |
| **File I/O** | CSV result saving and parameter file reading |
| **Visualization** | PCL-based point cloud visualization utilities |
| **Randomization** | Seeded random number generation for reproducible experiments |

### Data Processing Pipeline

```
Input Point Cloud
    ↓
Crop to Reference/Source
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
Compute Error Metrics (MTRE, Bias)
    ↓
Save Results to CSV
```

### Design Patterns

- **Template-based pipelines** for different detector types
- **Strategy pattern** for swappable detectors
- **Factory pattern** for pipeline creation
- **Builder pattern** for configuration management

## 🧪 Testing

Build and run unit tests:

```bash
# Build with tests (default)
cmake -B build -DBUILD_TESTS=ON
cmake --build build

# Run tests
cd build && ctest

# Disable tests if needed
cmake -B build -DBUILD_TESTS=OFF
```

## 📊 Available Executables

| Executable | Purpose |
|-----------|---------|
| `main_registration` | Main pipeline with SIFT and Harris detectors |
| `experiment2` | Experimental registration pipeline |
| `experiment2_ds` | Downsampled variant of experiment2 |
| `verification_test1` | Verification test 1 |
| `verification_test2` | Verification test 2 |
| `fred_exp_sacia_param_grid_search` | SAC-IA parameter grid search |
| `exp_sacia_param_grid_search` | Additional SAC-IA experiments |
| `visualization` | Point cloud visualization tool |
| `cpp_version` | C++ version check utility |
| `point_cloud_converter` | Convert between point cloud formats |
| `benchmark_registration` | Performance benchmarking (requires BUILD_BENCHMARKS=ON) |

## 💻 Usage

### Basic Execution

Most executables follow this pattern:

```bash
cd build
./executable_name [number_of_iterations]

# Example: Run main registration with 10 iterations
./main_registration 10
```

### Configuration Files

Create parameter files in the `data/` directory with `key=value` format:

```ini
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

### Data Formats

Supported point cloud formats: **PCD**, **PLY**, **OBJ**, **VTK**

#### Convert Between Formats

```bash
cd build
./point_cloud_converter input.ply output.pcd
./point_cloud_converter input.pcd output.ply binary
```

#### Using CloudCompare for Other Formats

For `.las`, `.laz`, `.xyz` files:
1. Load file in [CloudCompare](https://www.danielgm.net/cc/)
2. File → Save → Choose "Point Cloud Library format" (.pcd)
3. Place converted file in `data/` directory

## 🔄 Registration Pipelines

The project implements three main registration approaches:

### 1. SIFT-based Pipeline
Uses Scale-Invariant Feature Transform keypoints for feature detection, FPFH descriptors, and SAC-IA alignment.

### 2. Harris 3D Pipeline
Uses Harris 3D corner detector for keypoint extraction, FPFH descriptors, and SAC-IA alignment.

### 3. All-Points Pipeline
Uses all points in the cloud (no keypoint detection), FPFH descriptors on entire cloud, and SAC-IA alignment.

## 📈 Output & Results

### CSV Output Format

Results are saved in the `results/` directory with:
- Pipeline type used
- Mean Target Registration Error (MTRE)
- Registration error bias (x, y, z components)
- Applied rotation angles
- Source dimensions and uncertainty parameters
- Dataset filename and seed values
- All pipeline parameters

### Error Metrics

1. **Mean Target Registration Error (MTRE)**: Average Euclidean distance between corresponding points in original and transformed source clouds.

2. **Registration Error Bias**: Systematic error components in x, y, and z directions.

## ⚙️ Default Parameters

Key default parameters (can be overridden via configuration files):

| Parameter | Value |
|-----------|-------|
| Normal search radius | 2.0 |
| FPFH search radius | 2.0 |
| Harris search radius | 2.0 |
| Harris threshold | 1e-6 |
| SIFT min scale | 0.1 |
| SIFT octaves | 6 |
| SIFT scales per octave | 4 |
| SIFT min contrast | 0.001 |
| SAC-IA min sample distance | 3.0 |
| SAC-IA max correspondence distance | 4.0 |
| SAC-IA iterations | 400 |
| SAC-IA samples per iteration | 3 |

## 📚 More Information

- [PCL Documentation](https://pcl.readthedocs.io/projects/tutorials/en/latest/using_pcl_pcl_config.html#using-pcl-pcl-config)
- [Docker Setup](DOCKER.md)
