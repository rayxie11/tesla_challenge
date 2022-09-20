#pragma once

#include <iostream>
#include <eigen3/Eigen/Core>
#include <cmath>
#include <array>
#include "network.h"

// Utility class containing different utility functions
namespace Util{
    double toRadians(double degree);
    double dist(std::array<double, 3> s1, std::array<double, 3> s2);
    Eigen::ArrayXd distFromStation(Eigen::MatrixXd& posMat, row s);
}