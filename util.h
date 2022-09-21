#pragma once

#include <iostream>
#include <cmath>
#include <array>
#include <unordered_map>
#include "network.h"

// Utility class containing different utility functions
namespace Util{
    double toRadians(double degree);
    double dist(std::array<double, 3> s1, std::array<double, 3> s2);
    std::unordered_map<std::string, std::array<double, 3>> getChargerMap(std::array<row, 303>& network);
}