#pragma once

#include <iostream>
#include <cmath>
#include <array>
#include <string>
#include <unordered_map>

#include "network.h"

// Utility class containing different utility functions
namespace Util{
    double toRadians(double degree);
    double dist(std::unordered_map<std::string, std::array<double, 3>>& chargerMap,
                std::string s1, std::string s2);
    bool checkValid(std::string initCharger, std::string goalCharger, std::unordered_map<std::string, std::array<double, 3>>& chargerMap);
    std::unordered_map<std::string, std::array<double, 3>> getChargerMap();
}