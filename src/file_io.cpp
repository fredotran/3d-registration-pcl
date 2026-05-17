#include "file_io.hpp"

#include <cstdio>
#include <ctime>
#include <unistd.h>

std::string removeExtension(const std::string& filename) {
    size_t lastdot = filename.find_last_of(".");
    if (lastdot == std::string::npos)
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
    std::string tmpFilename = filename + ".tmp." + std::to_string(getpid()) + "." +
                              std::to_string(std::time(nullptr));
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

TupleParameters parametersArray(const std::string& parametersFilename,
                                       const std::map<std::string, std::string>& parameters) {
    try {
        std::string pipelineType = parameters.at("pipelineType");
        std::string typeTransformation = parameters.at("typeTransformation");
        std::string referenceDataFilename = parameters.at("surface_model_data_file");

        double x_uncertainty = std::stod(parameters.at("x_uncertainty"));
        double y_uncertainty = std::stod(parameters.at("y_uncertainty"));
        double sourceWidth = std::stod(parameters.at("sourceWidth"));
        double sourceHeight = std::stod(parameters.at("sourceHeight"));
        double x_angle_min = std::stod(parameters.at("x_angle_min"));
        double x_angle_max = std::stod(parameters.at("x_angle_max"));
        double y_angle_min = std::stod(parameters.at("y_angle_min"));
        double y_angle_max = std::stod(parameters.at("y_angle_max"));
        double z_angle_min = std::stod(parameters.at("z_angle_min"));
        double z_angle_max = std::stod(parameters.at("z_angle_max"));

        return {pipelineType,  typeTransformation, referenceDataFilename, x_uncertainty,
                y_uncertainty, sourceWidth,        sourceHeight,          x_angle_min,
                x_angle_max,   y_angle_min,        y_angle_max,           z_angle_min,
                z_angle_max};
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
        if (start == std::string::npos) continue;

        size_t eqPos = line.find('=');
        if (eqPos == std::string::npos) continue;

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
