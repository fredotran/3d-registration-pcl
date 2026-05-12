#pragma once

#include <string>
#include <fstream>
#include <set>
#include <iostream>

#include "parameters.hpp"

///////////////////////////////
//////// PROTOTYPES ///////////
///////////////////////////////

bool isFileExists(const std::string &filename);

// saving tools
std::string removeExtension(const std::string &filename);
void saveResults(const std::string &savedFilename, const StringMap &settingsMap);

// Parameters tools
TupleParameters parametersArray(const std::string &parametersFilename,
                                 const std::vector<std::string> &parameters);
std::vector<std::string> readParameters(const std::string &inputFilename);

///////////////////////////////
////////// METHODS ///////////
///////////////////////////////

/* Removing extension from a file */
inline std::string removeExtension(const std::string &filename)
{
    size_t lastdot = filename.find_last_of(".");
    if (lastdot == std::string::npos)
        return filename;
    return filename.substr(0, lastdot);
}

// Returns true if file exists, false otherwise.
inline bool isFileExists(const std::string &filename) {
    std::ifstream ifile(filename);
    return static_cast<bool>(ifile);
}

/* Saving results into csv-formatted text file. Parameters in map written in alphabetic order.
Header is automatically added the first time the file is created. */
inline void saveResults(const std::string &savedFilename, const StringMap &settingsMap)
{
    if (settingsMap.empty()) {
        std::cerr << "Warning: Attempting to save empty settings map to " << savedFilename << std::endl;
        return;
    }

    std::string filename = savedFilename + ".csv";
    // make sure header is set up when save file is first created
    bool fileExists = isFileExists(filename);

    std::ofstream outfile;
    outfile.open(filename, std::ios_base::app);

    if (!outfile.is_open()) {
        throw std::runtime_error("Failed to open file for writing: " + filename);
    }

    // the set will automatically keep the keys sorted alphabetically
    std::set<std::string> keys;
    for (const auto &item : settingsMap) {
        keys.insert(item.first);
    }

    // Write header on first line if file is created now
    bool firstItem = true;
    if (!fileExists) {
        for (const std::string &key : keys) {
            if (!firstItem) {
                outfile << ",";
            }
            firstItem = false;
            outfile << key;
        }
        outfile << std::endl;
    }

    // Add comma-separated values in alphabetic order
    firstItem = true;
    for (const std::string &key : keys) {
        const std::string &value = settingsMap.at(key);
        if (!firstItem) {
            outfile << ",";
        }
        firstItem = false;
        outfile << value;
    }

    outfile << std::endl;
    std::cout << "\nResults saved to " << savedFilename << std::endl;
    outfile.close();
}

/* Transforming the data from the text file to the corresponding types*/
inline TupleParameters parametersArray(const std::string &parametersFilename,
                                        const std::vector<std::string> &parameters)
{
    constexpr size_t expected_params = 13;
    if (parameters.size() != expected_params) {
        throw std::invalid_argument("Invalid number of parameters in file: " + parametersFilename +
                                   ". Expected " + std::to_string(expected_params) +
                                   ", got " + std::to_string(parameters.size()));
    }

    std::string pipelineType = parameters[0];
    std::string typeTransformation = parameters[1];
    std::string referenceDataFilename = parameters[2];

    try {
        double x_uncertainty = std::stod(parameters[3]);
        double y_uncertainty = std::stod(parameters[4]);
        double sourceWidth = std::stod(parameters[5]);
        double sourceHeight = std::stod(parameters[6]);
        double x_angle_min = std::stod(parameters[7]);
        double x_angle_max = std::stod(parameters[8]);
        double y_angle_min = std::stod(parameters[9]);
        double y_angle_max = std::stod(parameters[10]);
        double z_angle_min = std::stod(parameters[11]);
        double z_angle_max = std::stod(parameters[12]);

        return {pipelineType, typeTransformation, referenceDataFilename,
                x_uncertainty, y_uncertainty, sourceWidth, sourceHeight,
                x_angle_min, x_angle_max, y_angle_min, y_angle_max, z_angle_min, z_angle_max};
    } catch (const std::invalid_argument& e) {
        throw std::invalid_argument("Invalid numeric parameter in file: " + parametersFilename + ". " + e.what());
    } catch (const std::out_of_range& e) {
        throw std::invalid_argument("Parameter out of range in file: " + parametersFilename + ". " + e.what());
    }
}

/* Reading the parameters text file */
inline std::vector<std::string> readParameters(const std::string &inputFilename)
{
    std::ifstream inFile(inputFilename);
    if (!inFile.is_open()) {
        throw std::runtime_error("Failed to open parameter file: " + inputFilename);
    }

    std::vector<std::string> tokens;
    std::string titles, parameters;

    while (std::getline(inFile, titles, '=') && std::getline(inFile, parameters)) {
        // Trim whitespace from parameters
        size_t start = parameters.find_first_not_of(" \t\r\n");
        size_t end = parameters.find_last_not_of(" \t\r\n");

        if (start != std::string::npos && end != std::string::npos) {
            tokens.push_back(parameters.substr(start, end - start + 1));
        } else if (!parameters.empty()) {
            tokens.push_back(parameters);
        }
    }

    if (tokens.empty()) {
        throw std::runtime_error("No parameters found in file: " + inputFilename);
    }

    return tokens;
}