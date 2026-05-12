# Changelog

All notable changes to the 3D Registration PCL project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- GitHub Actions CI/CD pipeline with multi-config builds (Debug/Release, GCC/Clang)
- Automated testing integration with CTest
- Code quality checks (clang-format, cppcheck) in CI pipeline
- Docker build verification in CI workflow
- cppcheck configuration for static analysis
- .editorconfig for consistent editor settings across team
- Doxygen configuration for API documentation generation
- CONTRIBUTING.md with comprehensive development guidelines
- CHANGELOG.md for version tracking
- Comprehensive test infrastructure with Google Test
- Example configuration files for common use cases
- Benchmark/performance testing framework
- Script for running all experiments
- Support for additional point cloud formats (PLY, OBJ)

### Changed
- Improved error handling in core registration functions
- Enhanced documentation with architecture diagrams
- Better code organization and structure

### Fixed
- Fixed memory leaks in point cloud processing
- Improved handling of edge cases in transformation functions

### Security
- Added input validation for parameter files
- Improved error handling for malformed data

## [1.0.0] - 2021-07-08

### Added
- Initial release of 3D Point Cloud Registration using PCL
- SIFT-based registration pipeline
- Harris 3D-based registration pipeline
- All-points SAC-IA registration pipeline
- Parameter file parsing system
- CSV result saving functionality
- Point cloud visualization tools
- Randomization utilities for experiments
- Verification test executables
- Parameter grid search experiments
- Docker support with Dockerfile and docker-compose.yml
- Basic test infrastructure with Google Test
- Code formatting with .clang-format
- Comprehensive README and Docker documentation

### Features
- Multiple registration pipelines (SIFT, Harris 3D, All-Points)
- FPFH descriptor computation
- SAC-IA (Sample Consensus Initial Alignment)
- Error metrics computation (MTRE, registration error bias)
- Experiment framework with parameter variations
- Visualization tools for point clouds
- Support for .pcd point cloud format

## [0.9.0] - 2021-06-XX

### Added
- Initial development version
- Basic registration functionality
- Experimental pipelines

---

## Version Format

The project uses semantic versioning: `MAJOR.MINOR.PATCH`

- **MAJOR**: Incompatible API changes
- **MINOR**: Backwards-compatible functionality additions
- **PATCH**: Backwards-compatible bug fixes

## Release Process

1. Update version in CMakeLists.txt
2. Update CHANGELOG.md with release notes
3. Create git tag: `git tag -a v1.0.0 -m "Release version 1.0.0"`
4. Push tag: `git push origin v1.0.0`
5. Create GitHub release with changelog
