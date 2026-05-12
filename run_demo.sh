#!/bin/bash

# Demo script for 3D Point Cloud Registration using PCL
# This script demonstrates the registration pipeline using verification test data

set -e  # Exit on error

echo "=========================================="
echo "3D Point Cloud Registration Demo"
echo "=========================================="
echo ""

# Check if we're running in the build directory
if [ ! -f "verification_test1" ]; then
    echo "Error: Please run this script from the build directory"
    echo "Usage: cd build && ../run_demo.sh"
    exit 1
fi

echo "Step 1: Running verification test 1..."
echo "This will register source_verification1.pcd with target_verification1.pcd"
echo ""

# Run verification test 1
./verification_test1

echo ""
echo "=========================================="
echo "Demo completed successfully!"
echo "=========================================="
echo ""
echo "Results:"
echo "- Check the visualization windows for point cloud alignment"
echo "- Registration results are saved in the results/ directory"
echo ""
echo "To run other demos:"
echo "- ./verification_test2  : Test with custom transformation"
echo "- ./main_registration 10 : Run main registration with 10 iterations"
echo "- ./cpp_version        : Check C++ version"
echo ""
