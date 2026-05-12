#pragma once

#include <chrono>
#include <iostream>
#include <string>
#include <vector>
#include <functional>
#include <iomanip>

/**
 * Simple benchmark framework for measuring execution time
 */
class Benchmark {
public:
    /**
     * Run a benchmark function multiple times and return statistics
     * @param name Name of the benchmark
     * @param func Function to benchmark
     * @param iterations Number of iterations to run
     */
    static void run(const std::string& name, std::function<void()> func, int iterations = 10) {
        std::vector<double> times;
        times.reserve(iterations);

        // Warm-up run
        func();

        // Timed runs
        for (int i = 0; i < iterations; ++i) {
            auto start = std::chrono::high_resolution_clock::now();
            func();
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            times.push_back(elapsed.count());
        }

        // Calculate statistics
        double min = *std::min_element(times.begin(), times.end());
        double max = *std::max_element(times.begin(), times.end());
        double sum = std::accumulate(times.begin(), times.end(), 0.0);
        double mean = sum / iterations;

        // Calculate standard deviation
        double variance = 0.0;
        for (double time : times) {
            variance += (time - mean) * (time - mean);
        }
        variance /= iterations;
        double std_dev = std::sqrt(variance);

        // Print results
        std::cout << "\n=== Benchmark: " << name << " ===" << std::endl;
        std::cout << "Iterations: " << iterations << std::endl;
        std::cout << "Mean:       " << std::fixed << std::setprecision(4) << mean << " s" << std::endl;
        std::cout << "Min:        " << std::fixed << std::setprecision(4) << min << " s" << std::endl;
        std::cout << "Max:        " << std::fixed << std::setprecision(4) << max << " s" << std::endl;
        std::cout << "Std Dev:    " << std::fixed << std::setprecision(4) << std_dev << " s" << std::endl;
        std::cout << "Total:      " << std::fixed << std::setprecision(4) << sum << " s" << std::endl;
    }

    /**
     * Measure a single function call and return the time in seconds
     * @param func Function to measure
     * @return Execution time in seconds
     */
    static double measure(std::function<void()> func) {
        auto start = std::chrono::high_resolution_clock::now();
        func();
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start;
        return elapsed.count();
    }

    /**
     * Simple timer class for measuring elapsed time
     */
    class Timer {
    public:
        Timer() : start_time(std::chrono::high_resolution_clock::now()) {}

        /**
         * Reset the timer
         */
        void reset() {
            start_time = std::chrono::high_resolution_clock::now();
        }

        /**
         * Get elapsed time in seconds
         * @return Elapsed time in seconds
         */
        double elapsed() const {
            auto end_time = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end_time - start_time;
            return elapsed.count();
        }

        /**
         * Get elapsed time in milliseconds
         * @return Elapsed time in milliseconds
         */
        double elapsed_ms() const {
            return elapsed() * 1000.0;
        }

    private:
        std::chrono::high_resolution_clock::time_point start_time;
    };
};
