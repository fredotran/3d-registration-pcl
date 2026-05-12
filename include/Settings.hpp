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

    void setValue(std::string&& name, double value) {
        settingsMap[std::move(name)] = value;
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
        for (const auto& [name, value] : settingsMap) {
            std::cout << "Name = " << name << ", value = " << value << std::endl;
        }
    }
};