#pragma once
#include <iostream>
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
    
    // Copy constructor
    Path(const Path& rhs);

    // Member functions
    double checkMaxCharge(double proposedCharge, int idx);
    bool chargeCar(double chargeRequired);
    bool verify();
    void addNewCharger(std::string charger, tsl::car car);
    std::string getOutputStr();
    cha::waypoint getCurWayPoint();

    // Storage params
    double totTime;
    std::priority_queue<std::pair<double, int>> chargerPQ;

private:
    // Storage params
    std::vector<cha::waypoint> path;
    std::unordered_map<std::string, std::array<double, 3>> chargerMap;
};