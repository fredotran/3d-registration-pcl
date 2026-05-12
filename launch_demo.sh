#!/bin/bash

# Launch script for 3D Point Cloud Registration Docker Demo
# This script builds and runs the Docker container with the demo

set -e

echo "=========================================="
echo "3D Point Cloud Registration - Docker Demo"
echo "=========================================="
echo ""

# Check if Docker is installed
if ! command -v docker &> /dev/null; then
    echo "Error: Docker is not installed"
    echo "Please install Docker from: https://docs.docker.com/get-docker/"
    exit 1
fi

# Check if Docker Compose is installed
if ! command -v docker-compose &> /dev/null; then
    echo "Error: Docker Compose is not installed"
    echo "Please install Docker Compose from: https://docs.docker.com/compose/install/"
    exit 1
fi

# Check if we're in the right directory
if [ ! -f "Dockerfile" ] || [ ! -f "docker-compose.yml" ]; then
    echo "Error: Please run this script from the project root directory"
    exit 1
fi

echo "Step 1: Building Docker image..."
echo "This may take 10-20 minutes on first run..."
echo ""

docker-compose build

echo ""
echo "Step 2: Starting Docker container..."
echo ""

docker-compose up -d

echo ""
echo "Step 3: Running the demo inside container..."
echo ""

docker-compose exec -T pcl-registration bash -c "cd build && ./verification_test1"

echo ""
echo "=========================================="
echo "Demo completed!"
echo "=========================================="
echo ""
echo "To interact with the container manually:"
echo "  docker-compose exec pcl-registration bash"
echo ""
echo "To stop the container:"
echo "  docker-compose down"
echo ""