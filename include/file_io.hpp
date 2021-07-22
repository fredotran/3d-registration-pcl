// Functions for reading/writing files and parsing their content

////////////////////////////////
////////// PROTOTYPES //////////
////////////////////////////////

bool isFileExists(std::string &filename);

// saving tools
std::string removeExtension(const std::string &filename);
void saveResults(std::string &savedFilename, StringMap settingsMap);

// Parameters tools
tupleParameters parametersArray(std::string parametersFilename,
                                std::vector<std::string> parameters);
std::vector<std::string> readParameters(std::string &inputFilename);

////////////////////////////////
//////////// METHODS ///////////
////////////////////////////////

/* Removing extension from a file */
std::string removeExtension(const std::string &filename)
{
        size_t lastdot = filename.find_last_of(".");
        int len = filename.length();
        int extLen = len - lastdot;
        if (lastdot == std::string::npos || extLen > 4)
                return filename;
        return filename.substr(0, lastdot);
}

// Returns true if file exits, false otherwise.
bool isFileExists(std::string &filename) {
        /* try to open file to read */
        std::ifstream ifile;
        ifile.open(filename);
        if(ifile) {
                return true;
        }
        std::cout << "File not found: " << filename << std::endl;
        return false;
}

/* Saving results into csv-formatted text file. Parameters in map written in alphabetic order.
Header is automatically added the first time the file is created. */
void saveResults(std::string &savedFilename, std::map<std::string, std::string> settingsMap)
{

        std::string filename = savedFilename + ".csv";
        // make sure header is set up when save file is first created
        bool fileExists = isFileExists(filename);        

        std::ofstream outfile;
        outfile.open(filename, std::ios_base::app);

        // the set will autonamtically keep the keys sorted alphabetically
        std::set<std::string> keys;
        for (auto item : settingsMap) {
                keys.insert(item.first);
        }

        // Write header on first line if file is created now
        bool firstItem = true;
        if (!fileExists) {
                for (std::string key : keys) {
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
        for (std::string key : keys) {
                std::string value = settingsMap[key];
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
tupleParameters parametersArray(std::string parametersFilename,
                                std::vector<std::string> parameters)
{
        std::string pipelineType = parameters[0];
        std::string typeTransformation = parameters[1];
        std::string referenceDataFilename = parameters[2];
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
}

/* Reading the parameters text file */
std::vector<std::string> readParameters(std::string &inputFilename)
{
        ifstream inFile;
        inFile.open(inputFilename);
        std::vector<std::string> tokens;
        std::string titles = "", parameters = "";

        while (!inFile.eof())
        {
                getline(inFile, titles, '=');      //grtting string upto =
                getline(inFile, parameters, '\n'); //getting string after =
                tokens.push_back(parameters);
        }
        inFile.close();

        return tokens;
}
