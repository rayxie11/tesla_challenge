#pragma once
#include <iostream>
#include <string>
#include <queue>
#include <vector>
#include <array>
#include <unordered_map>

#include "structs.h"

class Path{
public:
    // Constructors
    Path(std::unordered_map<std::string, std::array<double, 3>>& chargerMap,
         std::string initCharger, tsl::car initCar);
    Path();
    
    // Copy constructor
    Path(const Path& rhs);

    // Member functions
    bool updateBestCharger(bool pop);
    std::string getOutputStr();
    bool chargeCar(double chargeRequired);
    void addCharger(std::string charger, tsl::car car);
    cha::waypoint getCurWayPoint();

    // Storage params
    cha::waypoint bestCharger;

private:
    // Storage params
    std::priority_queue<cha::waypoint> chargerQ;
    std::vector<cha::waypoint> path;
    std::unordered_map<std::string, std::array<double, 3>> chargerMap;
};