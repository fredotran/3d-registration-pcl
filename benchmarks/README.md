# Benchmark Suite

This directory contains performance benchmarks for the 3D Registration PCL project.

## Building Benchmarks

Benchmarks are not built by default. To enable them:

```bash
cmake -B build -DBUILD_BENCHMARKS=ON
cmake --build build
```

## Running Benchmarks

After building, run the benchmark executable:

```bash
cd build
./benchmark_registration
```

## Benchmark Framework

The `benchmark.hpp` header provides a simple benchmarking framework:

### Basic Usage

```cpp
#include "benchmark.hpp"

// Run a function multiple times and get statistics
Benchmark::run("My Benchmark", []() {
    // Code to benchmark
}, iterations);

// Measure a single function call
double time = Benchmark::measure([]() {
    // Code to measure
});

// Use timer class for manual timing
Benchmark::Timer timer;
// ... do work ...
double elapsed = timer.elapsed();
```

### Example Output

```
=== Benchmark: My Benchmark ===
Iterations: 10
Mean:       0.1234 s
Min:        0.1200 s
Max:        0.1300 s
Std Dev:    0.0023 s
Total:      1.2340 s
```

## Available Benchmarks

### benchmark_registration.cpp

Comprehensive benchmarks for:

- **Point Cloud Operations**: Creation, transformation, and manipulation
- **Error Metrics**: MTRE and registration error bias computation
- **File I/O**: PCD file loading and saving

## Adding New Benchmarks

To add a new benchmark:

1. Create a new `.cpp` file in the `benchmarks/` directory
2. Include `benchmark.hpp` and necessary headers
3. Use the `Benchmark::run()` function to measure performance
4. Add the executable to `CMakeLists.txt`:

```cmake
if(BUILD_BENCHMARKS)
    add_executable(my_benchmark benchmarks/my_benchmark.cpp)
    target_link_libraries(my_benchmark pcl_registration_lib ${PCL_LIBRARIES})
    target_include_directories(my_benchmark PRIVATE
        ${CMAKE_SOURCE_DIR}/include
        ${CMAKE_SOURCE_DIR}/benchmarks
    )
endif()
```

## Interpreting Results

- **Mean**: Average execution time across all iterations
- **Min/Max**: Best and worst case execution times
- **Std Dev**: Standard deviation (measure of consistency)
- **Total**: Cumulative time for all iterations

Lower times indicate better performance. Consistent results (low standard deviation) are desirable for reproducible performance.

## Tips for Accurate Benchmarking

1. Run benchmarks multiple times to account for system variability
2. Ensure the system is idle during benchmarking
3. Use consistent hardware and software configurations
4. Consider warm-up runs to account for cache effects
5. Run benchmarks in Release mode for realistic performance measurements
