#pragma once

#include <map>
#include <stdexcept>
#include <string>

#include "parameters.hpp"

// parameter parsing
std::string getParameterStringValue(const std::map<std::string, std::string>& map,
                                    const std::string& paramName);
double getParameterDoubleValue(const std::map<std::string, double>& map,
                               const std::string& paramName, const DoubleMap& defaultValues);
double getParameterValue(const std::map<std::string, double>& map, const std::string& paramName);

inline std::string getParameterStringValue(const std::map<std::string, std::string>& map,
                                           const std::string& paramName) {
    auto it = map.find(paramName);
    if (it != map.end()) {
        return it->second;
    }
    return "";
}

inline double getParameterDoubleValue(const std::map<std::string, double>& map,
                                      const std::string& paramName,
                                      const DoubleMap& defaultValues) {
    auto it = map.find(paramName);
    if (it != map.end()) {
        return it->second;
    }
    auto defaultIt = defaultValues.find(paramName);
    if (defaultIt != defaultValues.end()) {
        return defaultIt->second;
    }
    throw std::invalid_argument("Missing setting: " + paramName);
}

inline double getParameterValue(const std::map<std::string, double>& map,
                                const std::string& paramName) {
    auto it = map.find(paramName);
    if (it != map.end()) {
        return it->second;
    }
    throw std::invalid_argument("Missing setting: " + paramName);
}