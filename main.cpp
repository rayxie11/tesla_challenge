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

    // Set intial car battery life and velocity
    double velocity = 105.0;
    double batteryLife = 320.0;

    // Generate chargerMap
    std::unordered_map<std::string, std::array<double, 3>> chargerMap = Util::getChargerMap(network); 
    
    // Check if intial and goal charger is present in network;
    if (!chargerMap.count(initial_charger_name) || !chargerMap.count(goal_charger_name)){
        std::cout << "Error: either initial or goal supercharger is not in network" << std::endl;
        return -1;
    }

    Astar solution(network,chargerMap,initial_charger_name,goal_charger_name,velocity,batteryLife);
    bool solve = solution.solve();

    std::cout << solve << std::endl;

    std::string res = solution.showPath();
    std::cout << res << std::endl;


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