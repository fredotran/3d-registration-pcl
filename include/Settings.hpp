#pragma once

#include <string>
#include <map>
#include <iostream>
#include <stdexcept>

class Settings {
private:
    std::map<std::string, double> settingsMap;

public:
    void setValue(const std::string& name, double value) {
        settingsMap[name] = value;
    }

    bool exists(const std::string& name) const {
        return settingsMap.find(name) != settingsMap.end();
    }

    double getValue(const std::string& name) const {
        auto it = settingsMap.find(name);
        if (it != settingsMap.end()) {
            return it->second;
        }
        throw std::invalid_argument("Setting does not exist: " + name);
    }

    const std::map<std::string, double>& getAllSettings() const {
        return settingsMap;
    }

    void print() const {
        for (const auto& pair : settingsMap) {
            std::cout << "Name = " << pair.first << ", value = " << pair.second << std::endl;
        }
    }
};