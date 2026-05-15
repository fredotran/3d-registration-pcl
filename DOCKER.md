# Docker Guide for 3D Point Cloud Registration

This guide provides detailed instructions for building and running the 3D Point Cloud Registration project using Docker.

## Prerequisites

- Docker (version 20.10 or higher)
- Docker Compose (version 1.29 or higher)

## Quick Start

### Using Docker Compose (Recommended)

1. **Build the Docker image:**
```bash
docker-compose build
```

2. **Run the container:**
```bash
docker-compose up -d
```

3. **Enter the container:**
```bash
docker-compose exec pcl-registration bash
```

4. **Run the demo:**
```bash
cd build
../run_demo.sh
```

### Using Docker Directly

1. **Build the Docker image:**
```bash
docker build -t pcl-registration .
```

2. **Run the container:**
```bash
docker run -it --rm \
  -v $(pwd)/data:/app/data \
  -v $(pwd)/results:/app/results \
  pcl-registration
```

3. **Run executables inside the container:**
```bash
cd build
./verification_test1
```

## Docker Configuration

### Dockerfile

The Dockerfile is based on Ubuntu 24.04 and includes:

- **Build tools:** GCC, CMake, Make
- **PCL dependencies:** Eigen3, FLANN, Boost, VTK, Qhull
- **PCL library:** libpcl-dev from Ubuntu package manager
- **Testing:** Google Test (libgtest-dev)

The project is built automatically during the Docker image creation using the following CMake options:
- `CMAKE_BUILD_TYPE=Release`
- `BUILD_TESTS=ON`
- `BUILD_EXAMPLES=ON`
- `BUILD_TOOLS=ON`

### Docker Compose

The docker-compose.yml file provides:

- **Volume mounts:**
  - Project directory: `/app`
  - Data directory: `/app/data`
  - Results directory: `/app/results`
- **Working directory:** `/app`
- **Container name:** `pcl-registration-demo`

### X11 Forwarding (Optional)

For visualization support, uncomment the X11 forwarding section in `docker-compose.yml`:

```yaml
environment:
  - DISPLAY=${DISPLAY}
volumes_from:
  - x11
network_mode: host
```

Then uncomment the x11 service as well.

**Note:** X11 forwarding requires additional host setup and may not work on all systems.

## Common Docker Commands

### Build Commands

```bash
# Build using Docker Compose
docker-compose build

# Build with no cache
docker-compose build --no-cache

# Build using Docker directly
docker build -t pcl-registration .
```

### Run Commands

```bash
# Start container in detached mode
docker-compose up -d

# Start container and attach to logs
docker-compose up

# Run container interactively
docker-compose run --rm pcl-registration bash

# Stop container
docker-compose down

# Stop container and remove volumes
docker-compose down -v
```

### Container Management

```bash
# View running containers
docker-compose ps

# View container logs
docker-compose logs -f

# Execute command in running container
docker-compose exec pcl-registration <command>

# Enter running container
docker-compose exec pcl-registration bash
```

### Volume Management

```bash
# List volumes
docker volume ls

# Remove unused volumes
docker volume prune
```

## Development Workflow

### Making Changes to Source Code

1. Edit files on the host machine (changes are reflected in the container due to volume mounts)
2. Rebuild inside the container:
```bash
docker-compose exec pcl-registration bash
cd build
make
```

Or rebuild the entire image:
```bash
docker-compose build
docker-compose up -d
```

### Running Tests

```bash
docker-compose exec pcl-registration bash
cd build
ctest
```

### Running Specific Executables

```bash
# Enter container
docker-compose exec pcl-registration bash

# Run verification test
cd build
./verification_test1

# Run main registration with 10 iterations
./main_registration 10

# Run C++ version check
./cpp_version
```

## Troubleshooting

### Build Issues

**Problem:** Build fails with PCL not found
```
Solution: Ensure the Dockerfile has the correct PCL dependencies installed.
The current Dockerfile uses libpcl-dev from Ubuntu 24.04 repositories.
```

**Problem:** Build fails with CMake errors
```
Solution: Check CMake version (minimum 3.10 required) and ensure all dependencies are installed.
```

### Runtime Issues

**Problem:** Cannot find data files
```
Solution: Ensure the data directory is properly mounted. Check docker-compose.yml volume mounts.
```

**Problem:** Permission denied when writing to results directory
```
Solution: Ensure the results directory has write permissions on the host:
chmod +w results/
```

**Problem:** Visualization windows don't appear
```
Solution: X11 forwarding requires proper setup. For headless operation, use executables
that don't require visualization (e.g., verification_test1 with visualizer disabled).
```

### Container Issues

**Problem:** Container won't start
```
Solution: Check logs with docker-compose logs pcl-registration
```

**Problem:** Container exits immediately
```
Solution: The container is configured to run /bin/bash by default. If it exits immediately,
check for errors in the build process or entrypoint script.
```

## Performance Considerations

- **Build time:** Initial build may take 10-20 minutes depending on your system
- **Image size:** The Docker image is approximately 2-3 GB due to PCL dependencies
- **Runtime:** Registration performance depends on point cloud size and pipeline parameters
- **Memory:** Ensure Docker has access to sufficient memory (recommended: 4GB+)

## Advanced Usage

### Custom Build Options

To build with custom CMake options, modify the Dockerfile:

```dockerfile
RUN cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=ON \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_TOOLS=ON \
    -DWARNINGS_AS_ERRORS=OFF
```

### Multi-stage Build

For smaller production images, consider a multi-stage build:

```dockerfile
# Build stage
FROM ubuntu:24.04 as builder
# ... build steps ...

# Runtime stage
FROM ubuntu:24.04
COPY --from=builder /app/build /app/build
# ... runtime dependencies only ...
```

### GPU Support

For GPU-accelerated operations, install nvidia-docker and use:

```bash
docker run --gpus all ...
```

## Cleaning Up

### Remove Docker Image

```bash
docker rmi pcl-registration
```

### Remove Docker Containers

```bash
docker-compose down
```

### Remove All Docker Resources

```bash
docker-compose down -v
docker system prune -a
```

## Support

For issues specific to:
- **Docker:** Check Docker documentation at https://docs.docker.com
- **PCL:** Check PCL documentation at https://pointclouds.org/documentation/
- **This project:** Check the main README.md and open an issue on the repository
