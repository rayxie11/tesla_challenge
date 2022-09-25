/*
 * Songchun (Ray) Xie
 * Tesla Coding Challenge Solution
 * path.h
 * This file is the header file for Path class
*/

#pragma once
#include <string>
#include <queue>
#include <vector>
#include <array>
#include <unordered_map>
#include <utility>

#include "structs.h"
#include "util.h"

class Path{
public:
    // Constructors
    Path(std::unordered_map<std::string, std::array<double, 3>>& chargerMap,
         std::string initCharger, tsl::car initCar);
    Path();
    Path(const Path& rhs);

    // Member functions
    double checkMaxCharge(double proposedCharge, int idx);
    double getTotTime();
    bool chargeCar(double chargeRequired);
    void addNewCharger(std::string charger, tsl::car car);
    std::string getOutputStr();
    cha::waypoint getCurWayPoint();

private:
    // Storage params
    double totTime;
    std::priority_queue<std::pair<double, int>> chargerPQ;
    std::vector<cha::waypoint> path;
    std::unordered_map<std::string, std::array<double, 3>> chargerMap;
};