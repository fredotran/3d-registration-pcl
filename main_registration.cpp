#include "./include/tools.hpp"

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        throw std::runtime_error("Required arguments: folder_name file_name");
    }
    std::string folderName = argv[1];
    std::string parametersFileName = argv[2];
    std::string fullParametersFilename = folderName + parametersFileName; // including path

    double seedRef(1.0), seedSource(1.0), seedCustomRotation(1.0);
    fullRegistration(fullParametersFilename, parametersFileName, seedRef, seedSource, seedCustomRotation);

    return 0;
}