#pragma once
#include <iostream>
#include <string>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <vector>
#include <utility>
#include <queue>
#include <float.h>

#include "network.h"
#include "structs.h"
#include "path.h"
#include "util.h"

extern std::array<row, 303> network;

class Astar{
public:
    // Constructor
    Astar(std::unordered_map<std::string, std::array<double, 3>>& chargerMap,
          std::string initCharger, std::string goalCharger,
          double v, double fullBatt);

    // Member functions
    bool solve();
    std::string showPath();

private:
    // Params passed in from constructor
    double v;
    double fullBatt;
    std::string initCharger;
    std::string goalCharger;
    std::unordered_map<std::string, std::array<double, 3>> chargerMap;

    // Astar solver params
    std::unordered_set<std::string> closedSet;
    std::unordered_set<std::string> openSet;
    std::priority_queue<cha::toChargerCost> openPQueue;
    std::unordered_map<std::string, Path> chargerPath;
    std::unordered_map<std::string, double> costToArrive;
    std::unordered_map<std::string, double> estCostThrough;

    // Astar solution path output string
    std::string outputStr;

    // Member functions
    double dist(std::string s1, std::string s2);
    double time(std::string s1, std::string s2);
    double cost(std::string s1, std::string s2);
    void reconstructPath(Path path);
    std::vector<std::string> findNeighbors(std::string s, tsl::car curCar);
};