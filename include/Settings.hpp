#include <string>
#include <list>
#include <iostream>

class Setting {
    private:
        std::string name;
        double value;

    public:
        double getValue() {
            return value;
        }

        std::string getName(){
            return name;
        }

        bool isName(std::string nameToCompare) {
            int comp = name.compare(nameToCompare);
            if (comp == 0)
                return true;
            else
                return false;
        }

        void setValue(double valueToSet) {
            std::cout << "Setting = " << name << " to new value = " << valueToSet << std::endl;
            value = valueToSet;
        }

        void print() {
            std::cout << "Name = " << name << ", value = " << value << std::endl;
        }

    Setting(std::string nameToSet, double valueToSet) {
        name = nameToSet;
        value = valueToSet;
    }

};

class Settings {
    private:
        std::list<Setting> settingsList;
        
    public:
        void setValue(std::string name, double value) {
            // add new setting if there are no previous settings
            if (settingsList.size() == 0) {
                std::cout << "No previous settings stored in the list, adding new setting " << name << " with value " << value << std::endl;
                Setting setting(name, value);
                settingsList.push_back(setting);
                return;
            }
            // update setting value if already existing
            std::list<Setting>::iterator it = settingsList.begin();
            while (it != settingsList.end()) {
                if (it->isName(name)) {
                    std::cout << "Overwriting existing setting " << name << " with value " << value << std::endl;
                    it->setValue(value);
                    return;
                }
                it++;
            }
            // add new settings if not already existing
            std::cout << "Adding setting " << name << " with value " << value << std::endl;
                    
            Setting newSetting(name, value);
            settingsList.push_back(newSetting);
        }

        bool exists(std::string name) {
            for (auto setting : settingsList) {
                if (setting.isName(name)) {
                    return true;
                }
            }
            return false;
        }

        // returns value or throws exception if setting does not exist
        double getValue(std::string name) {
            for (auto setting : settingsList) {
                if (setting.isName(name)) {
                    return setting.getValue();
                }
            }
            throw std::invalid_argument("Setting does not exist: " + name);
        }

        std::map<std::string, double> getAllSettings() {
            std::map<std::string, double> map;
            for (auto setting : settingsList) {
                map[setting.getName()] = setting.getValue();
            }
            return map;
        }

        void print() {
            for (auto setting : settingsList) {

                setting.print();
            }
        }
};
