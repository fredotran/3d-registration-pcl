#pragma once

#include <fstream>
#include <iostream>
#include <map>
#include <set>
#include <string>

#include "parameters.hpp"

bool isFileExists(const std::string& filename);

std::string removeExtension(const std::string& filename);
void saveResults(const std::string& savedFilename, const StringMap& settingsMap);

TupleParameters parametersArray(const std::string& parametersFilename,
                                const std::map<std::string, std::string>& parameters);
std::map<std::string, std::string> readParameters(const std::string& inputFilename);
