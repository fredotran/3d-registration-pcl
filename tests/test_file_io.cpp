#include <gtest/gtest.h>
#include <fstream>
#include "file_io.hpp"

TEST(FileIOTest, RemoveExtension) {
    EXPECT_EQ(removeExtension("test.txt"), "test");
    EXPECT_EQ(removeExtension("data.pcd"), "data");
    EXPECT_EQ(removeExtension("no_extension"), "no_extension");
    EXPECT_EQ(removeExtension("multiple.dots.txt"), "multiple.dots");
}

TEST(FileIOTest, IsFileExists) {
    // Test with non-existent file
    EXPECT_FALSE(isFileExists("nonexistent_file.txt"));
    
    // Test with temporary file
    std::ofstream tempFile("temp_test_file.txt");
    tempFile << "test content";
    tempFile.close();
    EXPECT_TRUE(isFileExists("temp_test_file.txt"));
    
    // Clean up
    std::remove("temp_test_file.txt");
}

TEST(FileIOTest, SaveResults) {
    StringMap testData;
    testData["param1"] = "value1";
    testData["param2"] = "value2";
    testData["param3"] = "value3";
    
    std::string testFilename = "test_results";
    saveResults(testFilename, testData);
    
    // Check if file was created
    EXPECT_TRUE(isFileExists(testFilename + ".csv"));
    
    // Clean up
    std::remove((testFilename + ".csv").c_str());
}

TEST(FileIOTest, ReadParameters) {
    // Create a temporary parameter file
    std::ofstream paramFile("test_params.txt");
    paramFile << "param1=value1\n";
    paramFile << "param2=value2\n";
    paramFile << "param3=value3\n";
    paramFile.close();
    
    auto params = readParameters("test_params.txt");
    EXPECT_EQ(params.size(), 3);
    EXPECT_EQ(params[0], "value1");
    EXPECT_EQ(params[1], "value2");
    EXPECT_EQ(params[2], "value3");
    
    // Clean up
    std::remove("test_params.txt");
}
