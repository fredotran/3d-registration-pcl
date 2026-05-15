#include <iostream>
#include <string>

#include "point_cloud_formats.hpp"

/**
 * Point cloud format converter utility
 * Usage: ./point_cloud_converter input_file output_file [binary]
 *
 * Examples:
 *   ./point_cloud_converter input.ply output.pcd
 *   ./point_cloud_converter input.pcd output.ply binary
 */

void printUsage(const std::string& programName) {
    std::cout << "Point Cloud Format Converter" << std::endl;
    std::cout << "Usage: " << programName << " <input_file> <output_file> [binary]" << std::endl;
    std::cout << std::endl;
    std::cout << "Arguments:" << std::endl;
    std::cout << "  input_file   : Path to input point cloud file" << std::endl;
    std::cout << "  output_file  : Path to output point cloud file" << std::endl;
    std::cout << "  binary       : Optional flag to save in binary format (for PCD/PLY)"
              << std::endl;
    std::cout << std::endl;
    std::cout << "Supported formats: PCD, PLY, OBJ, VTK" << std::endl;
    std::cout << std::endl;
    std::cout << "Examples:" << std::endl;
    std::cout << "  " << programName << " input.ply output.pcd" << std::endl;
    std::cout << "  " << programName << " input.pcd output.ply binary" << std::endl;
}

int main(int argc, char* argv[]) {
    if (argc < 3 || argc > 4) {
        printUsage(argv[0]);
        return 1;
    }

    std::string inputFile = argv[1];
    std::string outputFile = argv[2];
    bool binary = false;

    if (argc == 4) {
        std::string binaryFlag = argv[3];
        if (binaryFlag == "binary" || binaryFlag == "true" || binaryFlag == "1") {
            binary = true;
        }
    }

    try {
        std::cout << "========================================" << std::endl;
        std::cout << "   Point Cloud Format Converter" << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << "Input file:  " << inputFile << std::endl;
        std::cout << "Output file: " << outputFile << std::endl;
        std::cout << "Binary mode: " << (binary ? "Yes" : "No") << std::endl;
        std::cout << "========================================" << std::endl;
        std::cout << std::endl;

        convertPointCloudFormat(inputFile, outputFile, binary);

        std::cout << std::endl;
        std::cout << "Conversion completed successfully!" << std::endl;
        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
