#pragma once
#include <string>
#include <iostream>
#include <array>
#include <unordered_map>
#include <unordered_set>
#include <eigen3/Eigen/Core>

#include <network.h>
#include <util.h>
#include <float.h>

class Astar{
public:
    // Constructors
    Astar(std::array<row, 303>& network, 
          std::unordered_map<std::string, std::array<double, 3>>& stationMap,
          Eigen::MatrixXd& posMat,
          std::string initCharger, std::string goalCharger,
          double carV, double carBatt);
    
    // Member functions
    bool solve();

private:
    // Params passed from constructor
    std::string initCharger;
    std::string goalCharger;
    Eigen::MatrixXd posMat;
    std::unordered_map<std::string, std::array<double, 3>> stationMap;

    // Astar solving params
    std::unordered_set<std::string> closedSet;
    std::unordered_set<std::string> openSet;
    std::unordered_map<std::string, std::string> cameFrom;
    std::unordered_map<std::string, double> costToArrive;
    std::unordered_map<std::string, double> estCostThrough;
    std::unordered_map<std::string, double> chargeTime;

    // Astar solution storage params 
    std::array<std::string, 303> waypointChargers;
    std::array<double, 303> chargingTimes;
    std::string outputStr;

    // Member functions
    std::string findBestEstCostThrough();
    double cost(std::string s1, std::string s2);
    void reconstructPath();
    std::unordered_set<std::string> findNeighbors(std::string s);

    
};