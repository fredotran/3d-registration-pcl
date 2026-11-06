#include "file_io.hpp"

#include <unistd.h>

#include <cstdio>
#include <ctime>

std::string removeExtension(const std::string& filename) {
    size_t lastdot = filename.find_last_of(".");
    size_t lastsep = filename.find_last_of("/\\");
    // Ignore dots that are part of a directory name (e.g. "..")
    if (lastdot == std::string::npos || (lastsep != std::string::npos && lastdot < lastsep))
        return filename;
    return filename.substr(0, lastdot);
}

bool isFileExists(const std::string& filename) {
    std::ifstream ifile(filename);
    return static_cast<bool>(ifile);
}

void saveResults(const std::string& savedFilename, const StringMap& settingsMap) {
    if (settingsMap.empty()) {
        std::cerr << "Warning: Attempting to save empty settings map to " << savedFilename
                  << std::endl;
        return;
    }

    std::string filename = savedFilename + ".csv";
    // Use PID + timestamp for unique temp file to avoid concurrent writer collisions
    std::string tmpFilename =
        filename + ".tmp." + std::to_string(getpid()) + "." + std::to_string(std::time(nullptr));
    bool fileExists = isFileExists(filename);

    // If appending, copy existing content to temp file first
    if (fileExists) {
        std::ifstream src(filename, std::ios::binary);
        std::ofstream dst(tmpFilename, std::ios::binary);
        if (!src.is_open() || !dst.is_open()) {
            throw std::runtime_error("Failed to create temp file for: " + filename);
        }
        dst << src.rdbuf();
        src.close();
        dst.close();
    }

    std::ofstream outfile;
    outfile.open(tmpFilename, std::ios_base::app);

    if (!outfile.is_open()) {
        throw std::runtime_error("Failed to open temp file for writing: " + tmpFilename);
    }

    std::set<std::string> keys;
    for (const auto& item : settingsMap) {
        keys.insert(item.first);
    }

    bool firstItem = true;
    if (!fileExists) {
        for (const std::string& key : keys) {
            if (!firstItem) {
                outfile << ",";
            }
            firstItem = false;
            outfile << key;
        }
        outfile << std::endl;
    }

    firstItem = true;
    for (const std::string& key : keys) {
        const std::string& value = settingsMap.at(key);
        if (!firstItem) {
            outfile << ",";
        }
        firstItem = false;
        outfile << value;
    }

    outfile << std::endl;
    outfile.close();

    // Atomic rename
    if (std::rename(tmpFilename.c_str(), filename.c_str()) != 0) {
        throw std::runtime_error("Failed to rename temp file to: " + filename);
    }

    std::cout << "\nResults saved to " << savedFilename << std::endl;
}

namespace {

// Look up a parameter value, accepting an optional legacy (snake_case) alias
const std::string& lookupParam(const std::map<std::string, std::string>& parameters,
                               const std::string& key, const std::string& alias = "") {
    auto it = parameters.find(key);
    if (it != parameters.end())
        return it->second;
    if (!alias.empty()) {
        it = parameters.find(alias);
        if (it != parameters.end())
            return it->second;
    }
    throw std::out_of_range(key);
}

double parseDoubleParam(const std::map<std::string, std::string>& parameters,
                        const std::string& key, const std::string& alias = "") {
    return std::stod(lookupParam(parameters, key, alias));
}

double parseOptionalDoubleParam(const std::map<std::string, std::string>& parameters,
                                const std::string& key, double defaultValue,
                                const std::string& alias = "") {
    auto it = parameters.find(key);
    if (it == parameters.end() && !alias.empty())
        it = parameters.find(alias);
    if (it == parameters.end() || it->second.empty())
        return defaultValue;
    return std::stod(it->second);
}

std::string parseOptionalStringParam(const std::map<std::string, std::string>& parameters,
                                     const std::string& key, const std::string& alias = "") {
    auto it = parameters.find(key);
    if (it == parameters.end() && !alias.empty())
        it = parameters.find(alias);
    return (it != parameters.end()) ? it->second : "";
}

}  // namespace

TupleParameters parametersArray(const std::string& parametersFilename,
                                const std::map<std::string, std::string>& parameters) {
    try {
        ExperimentParameters params;
        params.pipelineType = lookupParam(parameters, "pipelineType", "pipeline_type");
        params.typeTransformation =
            lookupParam(parameters, "typeTransformation", "type_transformation");
        params.referenceDataFilename =
            lookupParam(parameters, "surface_model_data_file", "surface_model_data_file");
        params.uavDataFilename =
            parseOptionalStringParam(parameters, "uav_data_file", "uavDataFilename");

        params.x_uncertainty = parseDoubleParam(parameters, "x_uncertainty", "xUncertainty");
        params.y_uncertainty = parseDoubleParam(parameters, "y_uncertainty", "yUncertainty");
        params.sourceWidth = parseDoubleParam(parameters, "sourceWidth", "source_width");
        params.sourceHeight = parseDoubleParam(parameters, "sourceHeight", "source_height");

        params.x_angle_min = parseDoubleParam(parameters, "x_angle_min");
        params.x_angle_max = parseDoubleParam(parameters, "x_angle_max");
        params.y_angle_min = parseDoubleParam(parameters, "y_angle_min");
        params.y_angle_max = parseDoubleParam(parameters, "y_angle_max");
        params.z_angle_min = parseDoubleParam(parameters, "z_angle_min");
        params.z_angle_max = parseDoubleParam(parameters, "z_angle_max");

        params.x_translation_min = parseOptionalDoubleParam(parameters, "x_translation_min", 0.0);
        params.x_translation_max = parseOptionalDoubleParam(parameters, "x_translation_max", 0.0);
        params.y_translation_min = parseOptionalDoubleParam(parameters, "y_translation_min", 0.0);
        params.y_translation_max = parseOptionalDoubleParam(parameters, "y_translation_max", 0.0);
        params.z_translation_min = parseOptionalDoubleParam(parameters, "z_translation_min", 0.0);
        params.z_translation_max = parseOptionalDoubleParam(parameters, "z_translation_max", 0.0);
        params.scale_min = parseOptionalDoubleParam(parameters, "scale_min", 1.0);
        params.scale_max = parseOptionalDoubleParam(parameters, "scale_max", 1.0);

        return params;
    } catch (const std::out_of_range& e) {
        throw std::invalid_argument("Missing parameter in file: " + parametersFilename + ". " +
                                    e.what());
    } catch (const std::invalid_argument& e) {
        throw std::invalid_argument("Invalid numeric parameter in file: " + parametersFilename +
                                    ". " + e.what());
    }
}

std::map<std::string, std::string> readParameters(const std::string& inputFilename) {
    std::ifstream inFile(inputFilename);
    if (!inFile.is_open()) {
        throw std::runtime_error("Failed to open parameter file: " + inputFilename);
    }

    std::map<std::string, std::string> params;
    std::string line;

    while (std::getline(inFile, line)) {
        size_t start = line.find_first_not_of(" \t\r\n");
        if (start == std::string::npos)
            continue;

        size_t eqPos = line.find('=');
        if (eqPos == std::string::npos)
            continue;

        std::string key = line.substr(0, eqPos);
        std::string value = line.substr(eqPos + 1);

        size_t keyStart = key.find_first_not_of(" \t\r\n");
        size_t keyEnd = key.find_last_not_of(" \t\r\n");
        if (keyStart != std::string::npos && keyEnd != std::string::npos) {
            key = key.substr(keyStart, keyEnd - keyStart + 1);
        }

        size_t valStart = value.find_first_not_of(" \t\r\n");
        size_t valEnd = value.find_last_not_of(" \t\r\n");
        if (valStart != std::string::npos && valEnd != std::string::npos) {
            value = value.substr(valStart, valEnd - valStart + 1);
        } else {
            value = "";
        }

        params[key] = value;
    }

    if (params.empty()) {
        throw std::runtime_error("No parameters found in file: " + inputFilename);
    }

    return params;
}
