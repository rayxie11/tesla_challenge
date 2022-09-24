#include <string>

#include "network.h"
#include "util.h"
#include "structs.h"
#include "a_star.h"

extern std::array<row, 303> network;

int main(int argc, char** argv)
{
    if (argc != 3)
    {
        std::cout << "Error: requires initial and final supercharger names" << std::endl;        
        return -1;
    }

    // Setting initial and goal charger names    
    std::string initial_charger_name = argv[1];
    std::string goal_charger_name = argv[2];

    // Set car velocity and top battery life
    double v = 105.0;
    double battLife = 320.0;

    // Generate chargerMap
    std::unordered_map<std::string, std::array<double, 3>> chargerMap = Util::getChargerMap(); 
    
    // Check if intial and goal charger is present in network;
    if (Util::checkValid(initial_charger_name, goal_charger_name, chargerMap)){
        std::cout << "Error: either initial or goal supercharger is not in network" << std::endl;
        return -1;
    }

    // Create Astar object and try to solve
    Astar solution(chargerMap, initial_charger_name, goal_charger_name, v, battLife);
    bool solve = solution.solve();

    // Check is solution is found
    if (solve){
        std::string solPath = solution.showPath();
        std::cout << solPath << std::endl;
    } else {
        std::cout << "No path found from " << initial_charger_name << " to " << goal_charger_name << "." << std::endl;
        return -1;
    }
    
    return 0;
}