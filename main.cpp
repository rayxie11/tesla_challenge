#include "network.h"
#include "util.h"

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
    
    row s1 = network[0];
    row s2 = network[1];
    double distance = Util::dist(s1,s2);
    std::cout << distance << std::endl;
    //std::cout << initial_charger_name << std::endl;
    return 0;
}