#include <eigen3/Eigen/Core>
#include <unordered_map>
#include <string>

#include "network.h"
#include "util.h"
#include "a_star.h"

extern std::array<row, 303> network;

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Error: requires initial and final supercharger names" << std::endl;        
        return -1;
    }
    
    std::string initial_charger_name = argv[1];
    std::string goal_charger_name = argv[2];
    
    double carBatt = 320.0;    // Initial car battery life
    double carV = 105.0;       // Car velocity

    // stationMap key: charger name, value: charger location, charging rate
    std::unordered_map<std::string, std::array<double, 3>> stationMap; 
    // posMat contains longitudes and altitudes for all chargers
    Eigen::MatrixXd posMat(303,2); 

    for (int i = 0; i < network.size(); i++){
        posMat(i,0) = network[i].lat;
        posMat(i,1) = network[i].lon;
        std::array<double, 3> params = {network[i].lat,network[i].lon,network[i].rate};
        stationMap[network[i].name] = params;
    }
    
    // Check if intial and goal charger is present in network;
    if (!stationMap.count(initial_charger_name) || !stationMap.count(goal_charger_name)){
        std::cout << "Error: either initial or goal charger is not in network" << std::endl;
        return -1;
    }

    Astar solution(network,stationMap,posMat,initial_charger_name,goal_charger_name,carV,carBatt);
    bool solve = solution.solve();


    //std::cout << "what" << std::endl;
    //std::cout << posMat << std::endl;
    //Eigen::ArrayXd res = Util::distFromStation(posMat,network[0]);
    //std::cout << res << std::endl;

    /*
    row s1 = network[0];
    row s2 = network[1];
    double distance = Util::dist(s1,s2);
    std::cout << distance << std::endl;
    //std::cout << initial_charger_name << std::endl;
    */
    return 0;
}