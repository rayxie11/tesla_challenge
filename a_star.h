#pragma once
#include <iostream>
#include <string>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <utility>
#include <queue>

#include <network.h>
#include <util.h>
#include <float.h>
#include <structs.h>
#include <path.h>

class Astar{
public:
    // Constructor
    Astar(std::array<row, 303>& network, 
          std::unordered_map<std::string, std::array<double, 3>>& chargerMap,
          std::string initCharger, std::string goalCharger,
          tsl::car initCar);

    // Member functions
    bool solve();
    std::string showPath();

private:
    // Params passed in from constructor
    std::string initCharger;
    std::string goalCharger;
    std::unordered_map<std::string, std::array<double, 3>> chargerMap;
    tsl::car car;

    // Astar solver params
    std::unordered_set<std::string> closedSet;
    std::unordered_set<std::string> openSet;
    std::priority_queue<cha::toChargerCost> openPQueue;

    //std::unordered_map<std::string, tsl::car> chargerCar;
    std::unordered_map<std::string, Path> chargerPath;

    std::unordered_map<std::string, std::string> cameFrom;
    std::unordered_map<std::string, double> costToArrive;
    std::unordered_map<std::string, double> estCostThrough;
    std::unordered_map<std::string, double> chargeTime;

    // Astar solution params
    double time;
    std::string outputStr;

    // Member functions
    //std::string findBestEstCostThrough();
    double dist(std::string s1, std::string s2);
    double cost(std::string s1, std::string s2);
    void reconstructPath(Path path);
    std::vector<cha::chargerCar> findNeighbors(std::string s, tsl::car curCar);
};