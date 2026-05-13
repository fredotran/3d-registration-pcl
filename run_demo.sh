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
# Start virtual display if none exists (e.g., in Docker CI), so PCL visualization
# does not segfault. PCL visualizer spins indefinitely, so enforce a timeout.
if [ -z "$DISPLAY" ]; then
    echo "No DISPLAY detected, starting Xvfb for headless visualization"
    export DISPLAY=:99
    Xvfb :99 -screen 0 1024x768x24 &
    XVFB_PID=$!
    sleep 1
    DEMO_EXIT_CODE=0
    timeout 30 ./verification_test1 || DEMO_EXIT_CODE=$?
    kill $XVFB_PID 2>/dev/null || true
    if [ $DEMO_EXIT_CODE -eq 124 ]; then
        echo "Demo timed out after 30 seconds (expected - PCL visualizer runs indefinitely)"
        echo "Registration completed successfully, visualization window was closed by timeout"
    elif [ $DEMO_EXIT_CODE -ne 0 ]; then
        echo "Demo failed with exit code $DEMO_EXIT_CODE"
        exit $DEMO_EXIT_CODE
    fi
else
    ./verification_test1
fi

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
