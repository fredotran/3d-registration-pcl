#!/bin/bash

# Script to run all experiments in the 3D Registration PCL project
# This script runs all available executables with appropriate parameters

set -e  # Exit on error

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Print colored output
print_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if build directory exists
if [ ! -d "build" ]; then
    print_error "Build directory not found. Please build the project first:"
    echo "  cmake -B build"
    echo "  cmake --build build"
    exit 1
fi

# Check if executables exist
check_executable() {
    if [ ! -f "build/$1" ]; then
        print_warning "Executable $1 not found. Skipping..."
        return 1
    fi
    return 0
}

print_info "========================================"
print_info "   Running All Experiments"
print_info "========================================"

# Create results directory if it doesn't exist
mkdir -p results

# Run verification tests
print_info "Running verification tests..."
if check_executable "verification_test1"; then
    print_info "Running verification_test1..."
    cd build
    ./verification_test1 || print_warning "verification_test1 failed"
    cd ..
fi

if check_executable "verification_test2"; then
    print_info "Running verification_test2..."
    cd build
    ./verification_test2 || print_warning "verification_test2 failed"
    cd ..
fi

# Run parameter grid search experiments
print_info "Running parameter grid search experiments..."
if check_executable "fred_exp_sacia_param_grid_search"; then
    print_info "Running fred_exp_sacia_param_grid_search..."
    cd build
    ./fred_exp_sacia_param_grid_search || print_warning "fred_exp_sacia_param_grid_search failed"
    cd ..
fi

if check_executable "exp_sacia_param_grid_search"; then
    print_info "Running exp_sacia_param_grid_search..."
    cd build
    ./exp_sacia_param_grid_search || print_warning "exp_sacia_param_grid_search failed"
    cd ..
fi

# Run experiment 2
print_info "Running experiment 2..."
if check_executable "experiment2"; then
    print_info "Running experiment2..."
    cd build
    ./experiment2 || print_warning "experiment2 failed"
    cd ..
fi

if check_executable "experiment2_ds"; then
    print_info "Running experiment2_ds (downsampled)..."
    cd build
    ./experiment2_ds || print_warning "experiment2_ds failed"
    cd ..
fi

# Run main registration with example configurations
print_info "Running main registration with example configurations..."

if [ -d "configs" ]; then
    for config in configs/*.txt; do
        if [ -f "$config" ]; then
            config_name=$(basename "$config")
            print_info "Running main_registration with $config_name..."
            cd build
            ../main_registration "$config" || print_warning "main_registration with $config_name failed"
            cd ..
        fi
    done
else
    print_warning "configs directory not found. Skipping example configurations."
fi

# Run unit tests
print_info "Running unit tests..."
cd build
if [ -f "tests/pcl_registration_tests" ]; then
    print_info "Running unit tests with CTest..."
    ctest --output-on-failure || print_warning "Some unit tests failed"
else
    print_warning "Test executable not found. Skipping unit tests."
fi
cd ..

# Run benchmarks if available
print_info "Running benchmarks..."
if check_executable "benchmark_registration"; then
    print_info "Running benchmark_registration..."
    cd build
    ./benchmark_registration || print_warning "benchmark_registration failed"
    cd ..
else
    print_info "Benchmarks not built. To build benchmarks, run:"
    echo "  cmake -B build -DBUILD_BENCHMARKS=ON"
    echo "  cmake --build build"
fi

print_info "========================================"
print_info "   All experiments completed"
print_info "========================================"
print_info "Results saved in the 'results/' directory"
print_info ""
print_info "To view the results, check the CSV files in results/"
