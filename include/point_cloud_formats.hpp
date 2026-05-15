#pragma once

#include <pcl/io/obj_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "parameters.hpp"

/**
 * Point cloud format support for multiple file formats
 * Supports: PCD, PLY, OBJ, VTK
 */

enum class PointCloudFormat { PCD, PLY, OBJ, VTK, UNKNOWN };

/**
 * Detect point cloud format from file extension
 * @param filename Path to the point cloud file
 * @return Detected format
 */
inline PointCloudFormat detectFormat(const std::string& filename) {
    size_t lastdot = filename.find_last_of(".");
    if (lastdot == std::string::npos) {
        return PointCloudFormat::UNKNOWN;
    }

    std::string ext = filename.substr(lastdot + 1);
    std::transform(ext.begin(), ext.end(), ext.begin(), ::tolower);

    if (ext == "pcd") {
        return PointCloudFormat::PCD;
    } else if (ext == "ply") {
        return PointCloudFormat::PLY;
    } else if (ext == "obj") {
        return PointCloudFormat::OBJ;
    } else if (ext == "vtk") {
        return PointCloudFormat::VTK;
    }

    return PointCloudFormat::UNKNOWN;
}

/**
 * Get format name as string
 * @param format Point cloud format
 * @return Format name
 */
inline std::string formatToString(PointCloudFormat format) {
    switch (format) {
        case PointCloudFormat::PCD:
            return "PCD";
        case PointCloudFormat::PLY:
            return "PLY";
        case PointCloudFormat::OBJ:
            return "OBJ";
        case PointCloudFormat::VTK:
            return "VTK";
        case PointCloudFormat::UNKNOWN:
            return "UNKNOWN";
    }
    return "UNKNOWN";
}

/**
 * Load point cloud from file (auto-detect format)
 * @param filename Path to the point cloud file
 * @return Loaded point cloud
 * @throws std::runtime_error if file cannot be loaded
 */
inline PointCloudPtr loadPointCloud(const std::string& filename) {
    auto cloud = PointCloudPtr(new PointCloud);
    PointCloudFormat format = detectFormat(filename);

    int result = -1;

    switch (format) {
        case PointCloudFormat::PCD:
            result = pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud);
            break;
        case PointCloudFormat::PLY:
            result = pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud);
            break;
        case PointCloudFormat::OBJ:
            // OBJ format may require different handling
            result = pcl::io::loadOBJFile(filename, *cloud);
            break;
        case PointCloudFormat::VTK:
            throw std::runtime_error(
                "VTK format loading is not directly supported for point clouds");
        case PointCloudFormat::UNKNOWN:
            throw std::runtime_error("Unknown point cloud format: " + filename);
    }

    if (result == -1) {
        throw std::runtime_error("Failed to load point cloud file: " + filename);
    }

    // Remove NaN values
    std::vector<int> nanValues;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, nanValues);

    std::cout << "Loaded point cloud from " << filename << " (" << formatToString(format)
              << " format): " << cloud->size() << " points." << std::endl;

    return cloud;
}

/**
 * Load point cloud from file with specific format
 * @param filename Path to the point cloud file
 * @param format Specific format to use
 * @return Loaded point cloud
 * @throws std::runtime_error if file cannot be loaded
 */
inline PointCloudPtr loadPointCloud(const std::string& filename, PointCloudFormat format) {
    auto cloud = PointCloudPtr(new PointCloud);

    int result = -1;

    switch (format) {
        case PointCloudFormat::PCD:
            result = pcl::io::loadPCDFile<pcl::PointXYZ>(filename, *cloud);
            break;
        case PointCloudFormat::PLY:
            result = pcl::io::loadPLYFile<pcl::PointXYZ>(filename, *cloud);
            break;
        case PointCloudFormat::OBJ:
            result = pcl::io::loadOBJFile(filename, *cloud);
            break;
        case PointCloudFormat::VTK:
            throw std::runtime_error(
                "VTK format loading is not directly supported for point clouds");
        case PointCloudFormat::UNKNOWN:
            throw std::runtime_error("Cannot load with UNKNOWN format");
    }

    if (result == -1) {
        throw std::runtime_error("Failed to load point cloud file: " + filename);
    }

    // Remove NaN values
    std::vector<int> nanValues;
    pcl::removeNaNFromPointCloud(*cloud, *cloud, nanValues);

    std::cout << "Loaded point cloud from " << filename << " (" << formatToString(format)
              << " format): " << cloud->size() << " points." << std::endl;

    return cloud;
}

/**
 * Save point cloud to file (auto-detect format from extension)
 * @param cloud Point cloud to save
 * @param filename Output file path
 * @param binary Save in binary format (for supported formats)
 * @throws std::runtime_error if file cannot be saved
 */
inline void savePointCloud(PointCloudPtr cloud, const std::string& filename, bool binary = false) {
    if (!cloud || cloud->empty()) {
        throw std::runtime_error("Cannot save empty point cloud");
    }

    PointCloudFormat format = detectFormat(filename);

    int result = -1;

    switch (format) {
        case PointCloudFormat::PCD:
            if (binary) {
                result = pcl::io::savePCDFileBinary<pcl::PointXYZ>(filename, *cloud);
            } else {
                result = pcl::io::savePCDFileASCII<pcl::PointXYZ>(filename, *cloud);
            }
            break;
        case PointCloudFormat::PLY:
            if (binary) {
                result = pcl::io::savePLYFileBinary<pcl::PointXYZ>(filename, *cloud);
            } else {
                result = pcl::io::savePLYFileASCII<pcl::PointXYZ>(filename, *cloud);
            }
            break;
        case PointCloudFormat::OBJ:
            throw std::runtime_error(
                "OBJ format saving is not directly supported for point clouds");
        case PointCloudFormat::VTK:
            throw std::runtime_error(
                "VTK format saving is not directly supported for point clouds");
        case PointCloudFormat::UNKNOWN:
            throw std::runtime_error("Unknown point cloud format for saving: " + filename);
    }

    if (result == -1) {
        throw std::runtime_error("Failed to save point cloud file: " + filename);
    }

    std::cout << "Saved point cloud to " << filename << " (" << formatToString(format)
              << " format): " << cloud->size() << " points." << std::endl;
}

/**
 * Convert point cloud from one format to another
 * @param inputFilename Input file path
 * @param outputFilename Output file path
 * @param binary Save in binary format (for supported formats)
 */
inline void convertPointCloudFormat(const std::string& inputFilename,
                                    const std::string& outputFilename, bool binary = false) {
    std::cout << "Converting " << inputFilename << " to " << outputFilename << std::endl;

    auto cloud = loadPointCloud(inputFilename);
    savePointCloud(cloud, outputFilename, binary);

    std::cout << "Conversion completed successfully." << std::endl;
}

/**
 * Get supported formats list
 * @return Vector of supported format names
 */
inline std::vector<std::string> getSupportedFormats() {
    return {"PCD", "PLY", "OBJ", "VTK"};
}
