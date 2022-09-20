#include <a_star.h>

// Default constructor
Astar::Astar(std::array<row, 303>& network, 
             std::unordered_map<std::string, std::array<double, 3>>& stationMap,
             Eigen::MatrixXd& posMat,
             std::string initCharger, std::string goalCharger,
             double carV, double carBatt)
             {
                this->initCharger = initCharger;
                this->goalCharger = goalCharger;
                this->posMat = posMat;
                this->stationMap = stationMap;
                
                this->openSet.insert(this->initCharger);
                this->costToArrive[this->initCharger] = 0.0;
                this->chargeTime[this->initCharger] = 0.0;
                this->estCostThrough[this->initCharger] = Util::dist(this->stationMap[this->initCharger],
                                                                     this->stationMap[this->goalCharger]);
             }

// Find the min estimated cost through current charger to goal charger
std::string Astar::findBestEstCostThrough(){
    double minCost = DBL_MAX;
    std::string res;
    for (std::string charger:openSet){
        double cost = estCostThrough[charger];
        if (minCost > cost){
            minCost = cost;
            res = charger;
        }
    }
    return res;
}

// Build the output string for found solution path
void Astar::reconstructPath(){
    outputStr += goalCharger;
    std::string curCharger = goalCharger;
    while (curCharger != initCharger){
        std::string prevCharger = cameFrom[curCharger];
        double chargingTime = chargeTime[prevCharger];
        outputStr = prevCharger+std::to_string(chargingTime)+outputStr;
        curCharger = prevCharger;
    }
}

// Find valid chargers that the car can reach right now
std::unordered_set<std::string> Astar::findNeighbors(std::string s){
    
}

// Astar solver function
bool Astar::solve(){
    while (openSet.size() > 0){
        std::string curCharger = Astar::findBestEstCostThrough();
        if (curCharger == goalCharger){
            Astar::reconstructPath();
            return true;
        }
        openSet.erase(curCharger);
        closedSet.insert(curCharger);

    }
    return false;
}