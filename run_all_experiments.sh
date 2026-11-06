#!/bin/bash

# Script to run all experiments in the 3D Registration PCL project
# This script runs all available executables with appropriate parameters
# Run from the project root directory

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

# Run a demo that opens a PCL visualizer. The visualizer spins until closed,
# so enforce a timeout; start Xvfb when no display is available.
run_visual_demo() {
    local exe=$1
    local timeout_sec=${2:-30}
    local start_xvfb=0

    if [ -z "$DISPLAY" ]; then
        print_info "No DISPLAY detected, starting Xvfb for headless visualization"
        export DISPLAY=:99
        Xvfb :99 -screen 0 1024x768x24 &
        XVFB_PID=$!
        start_xvfb=1
        sleep 1
    fi

    local exit_code=0
    (cd build && timeout "$timeout_sec" ./"$exe") || exit_code=$?

    if [ $start_xvfb -eq 1 ]; then
        kill $XVFB_PID 2>/dev/null || true
    fi

    if [ $exit_code -eq 124 ]; then
        print_info "$exe timed out after ${timeout_sec}s (expected - visualizer runs indefinitely)"
    elif [ $exit_code -ne 0 ]; then
        print_warning "$exe failed with exit code $exit_code"
    fi
}

print_info "========================================"
print_info "   Running All Experiments"
print_info "========================================"

# Create results directory if it doesn't exist
mkdir -p results

# Run verification tests (these open an interactive PCL visualizer)
print_info "Running verification tests..."
if check_executable "verification_test1"; then
    print_info "Running verification_test1..."
    run_visual_demo "verification_test1"
fi

if check_executable "verification_test2"; then
    print_info "Running verification_test2..."
    run_visual_demo "verification_test2"
fi

# Run parameter grid search experiments
print_info "Running parameter grid search experiments..."
if check_executable "fred_exp_sacia_param_grid_search"; then
    print_info "Running fred_exp_sacia_param_grid_search..."
    (cd build && ./fred_exp_sacia_param_grid_search) || print_warning "fred_exp_sacia_param_grid_search failed"
fi

if check_executable "exp_sacia_param_grid_search"; then
    print_info "Running exp_sacia_param_grid_search..."
    (cd build && ./exp_sacia_param_grid_search) || print_warning "exp_sacia_param_grid_search failed"
fi

# Run experiment 2 (argument = number of iterations)
print_info "Running experiment 2..."
if check_executable "experiment2"; then
    print_info "Running experiment2..."
    (cd build && ./experiment2 1) || print_warning "experiment2 failed"
fi

if check_executable "experiment2_ds"; then
    print_info "Running experiment2_ds (downsampled)..."
    (cd build && ./experiment2_ds 1) || print_warning "experiment2_ds failed"
fi

# Run main registration (argument = iterations, detector)
if check_executable "main_registration"; then
    print_info "Running main_registration..."
    (cd build && ./main_registration 1) || print_warning "main_registration failed"
fi

# Run each example configuration through the scenario runner
print_info "Running configurations in configs/..."
if [ -d "configs" ] && check_executable "scenario_registration"; then
    for config in configs/*.txt; do
        if [ -f "$config" ]; then
            config_name=$(basename "$config")
            print_info "Running scenario_registration with $config_name..."
            (cd build && ./scenario_registration "../$config" 1) || print_warning "scenario_registration with $config_name failed"
        fi
    done
else
    print_warning "configs directory not found. Skipping example configurations."
fi

# Run unit tests
print_info "Running unit tests..."
if [ -f "build/tests/pcl_registration_tests" ]; then
    print_info "Running unit tests with CTest..."
    (cd build && ctest --output-on-failure) || print_warning "Some unit tests failed"
else
    print_warning "Test executable not found. Skipping unit tests."
fi

# Run benchmarks if available
print_info "Running benchmarks..."
if check_executable "benchmark_registration"; then
    print_info "Running benchmark_registration..."
    (cd build && ./benchmark_registration) || print_warning "benchmark_registration failed"
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
