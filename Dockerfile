# Dockerfile for 3D Point Cloud Registration using PCL
FROM ubuntu:24.04

# Avoid interactive prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Set working directory
WORKDIR /app

# Install system dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    wget \
    libeigen3-dev \
    libflann-dev \
    libboost-all-dev \
    libvtk9-dev \
    libqhull-dev \
    libx11-dev \
    libxi-dev \
    libxrandr-dev \
    libglew-dev \
    libglu1-mesa-dev \
    libopenni2-dev \
    mpi-default-dev \
    pkg-config \
    && rm -rf /var/lib/apt/lists/*

# Install PCL from apt
RUN apt-get update && apt-get install -y \
    libpcl-dev \
    pcl-tools \
    && rm -rf /var/lib/apt/lists/*

# Install Xvfb for headless visualization support
RUN apt-get update && apt-get install -y \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# Install Google Test for testing
RUN apt-get update && apt-get install -y \
    libgtest-dev \
    && rm -rf /var/lib/apt/lists/*

# Copy project files
COPY . /app/

# Create build directory (clean any cached build from host)
RUN rm -rf /app/build && mkdir -p /app/build

# Build the project
WORKDIR /app/build
RUN cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_TESTS=ON \
    -DBUILD_EXAMPLES=ON \
    -DBUILD_TOOLS=ON

RUN make -j$(nproc)

# Set working directory back to app for running demos
WORKDIR /app

# Default command
CMD ["/bin/bash"]
