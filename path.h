#pragma once
#include <iostream>
#include <string>
#include <queue>
#include <vector>
#include <array>
#include <unordered_map>

#include <structs.h>

class Path{
public:
    // Constructor
    Path(std::unordered_map<std::string, std::array<double, 3>>& chargerMap,
         std::string initCharger, tsl::car initCar);
    
    // Copy constructor
    Path(Path& rhs);

    // Member functions
    bool updateBestCharger(bool pop);
    std::string getOutputStr();
    bool chargeCar(double chargeRequired);
    void addCharger(std::string charger, tsl::car car);
    cha::chargerCar getCurChargerCar();

    // Storage params
    cha::chargerCar bestCharger;

private:
    // Storage params
    std::priority_queue<cha::chargerCar> chargerQ;
    std::vector<cha::chargerCar> path;
    std::unordered_map<std::string, std::array<double, 3>> chargerMap;
};