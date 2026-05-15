#include <gtest/gtest.h>

#include "Settings.hpp"

TEST(SettingsTest, SetValueAndGetValue) {
    Settings settings;
    settings.setValue("test_param", 42.0);
    EXPECT_EQ(settings.getValue("test_param"), 42.0);
}

TEST(SettingsTest, GetValueNonExistent) {
    Settings settings;
    EXPECT_THROW(settings.getValue("nonexistent"), std::invalid_argument);
}

TEST(SettingsTest, Exists) {
    Settings settings;
    settings.setValue("param1", 1.0);
    EXPECT_TRUE(settings.exists("param1"));
    EXPECT_FALSE(settings.exists("param2"));
}

TEST(SettingsTest, UpdateExistingValue) {
    Settings settings;
    settings.setValue("param", 10.0);
    settings.setValue("param", 20.0);
    EXPECT_EQ(settings.getValue("param"), 20.0);
}

TEST(SettingsTest, GetAllSettings) {
    Settings settings;
    settings.setValue("param1", 1.0);
    settings.setValue("param2", 2.0);

    const auto& allSettings = settings.getAllSettings();
    EXPECT_EQ(allSettings.size(), 2);
    EXPECT_EQ(allSettings.at("param1"), 1.0);
    EXPECT_EQ(allSettings.at("param2"), 2.0);
}
