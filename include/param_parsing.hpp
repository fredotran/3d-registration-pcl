#include <string>
#include <map>
#include <cmath> // for NaN numbers
#include <stdexcept>

// parameter parsing
std::string getParameterStringValue(std::map<std::string, std::string> map, std::string paramName);
double getParameterDoubleValue(std::map<std::string, double> map, std::string paramName, DoubleMap defaultValues);


/* Returns the string valued parameter from the provided map, given the parameter name. 
Returns empty string if parameter was not found in map. */
std::string getParameterStringValue(std::map<std::string, std::string> map, std::string paramName)
{
    for (auto item : map)
    {   std::string key = item.first;
        if (key.compare(paramName) == 0)
            return item.second;
    }
    return "";
}

/* Returns the double valued parameter from the provided map, given the parameter name. 
Returns provided default value if parameter was not found in map. */
double getParameterDoubleValue(std::map<std::string, double> map, std::string paramName, DoubleMap defaultValues)
{
    for (auto item : map)
    {   std::string key = item.first;
        if (key.compare(paramName) == 0)
            return item.second;
    }
    return defaultValues[paramName];
}

/* Returns the double valued parameter from the provided map, given the parameter name. 
Throws error if parameter was not found in map. */
double getParameterValue(std::map<std::string, double> map, std::string paramName)
{
    // Create an iterator of map
    std::map<std::string, double>::iterator it;
    // Find the element with the key
    it = map.find(paramName);
    // Check if element exists in map or not
    if (it != map.end())
    {
        // Access the Value from iterator
        double value = it->second;
        return value;
    }

    throw std::invalid_argument("Missing setting: " + paramName);
}
