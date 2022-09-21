#include <a_star.h>

// Astar constructor
Astar::Astar(std::array<row, 303>& network, 
             std::unordered_map<std::string, std::array<double, 3>>& chargerMap,
             std::string initCharger, std::string goalCharger,
             double v, double batt)
             {
                this->initCharger = initCharger;
                this->goalCharger = goalCharger;
                this->chargerMap = chargerMap;
                this->car.v = v;
                this->car.batt = batt;
                this->time = 0.0;

                this->openSet.insert(this->initCharger);
                this->chargerCar[this->initCharger] = this->car;
                this->costToArrive[this->initCharger] = 0.0;
                this->chargeTime[this->initCharger] = 0.0;
                this->estCostThrough[this->initCharger] = Util::dist(this->chargerMap[this->initCharger],
                                                                     this->chargerMap[this->goalCharger]);
             }


// Find the min estimated cost through current charger to goal charger
std::string Astar::findBestEstCostThrough(){
    double minCost = DBL_MAX;
    std::string res = "";

    // Loop through openSet to find charger with minimum estimated cost through
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

    // Find the previous charger until it reaches the inital charger
    while (curCharger != initCharger){
        std::string prevCharger = cameFrom[curCharger];
        double chargingTime = chargeTime[prevCharger];
        if (prevCharger == initCharger){
            outputStr = prevCharger+", "+outputStr;
        } else {
            outputStr = prevCharger+", "+std::to_string(chargingTime)+", "+outputStr;
        }
        curCharger = prevCharger;
    }
}

// Find valid chargers that the car can reach right now
std::vector<std::pair<std::string, tsl::car>> Astar::findNeighbors(std::string s, tsl::car curCar){
    std::vector<std::pair<std::string, tsl::car>> res;
    std::array<double, 3> curCharger = chargerMap[s];

    // Loop through all chargers to see if reachable by curCar
    for (row elem:network){
        // Continue if staying put (trivial)
        if (elem.name == s){
            continue;
        }
        std::array<double, 3> chargerParam = {elem.lat,elem.lon,elem.rate};
        double distance = Util::dist(chargerParam,curCharger);
        if (distance <= curCar.batt){
            tsl::car newCar;
            newCar.v = curCar.v;
            newCar.batt = curCar.batt-distance;
            std::pair<std::string, tsl::car> newPair(elem.name,newCar);
            res.push_back(newPair);
        }
    }
    return res;
}


// Calculate the cost of travelling between stations
double Astar::cost(std::string s1, std::string s2){
    std::array<double, 3> s1Param = chargerMap[s1];
    std::array<double, 3> s2Param = chargerMap[s2];
    return Util::dist(s1Param,s2Param);
}


// Astar solver function
bool Astar::solve(){
    // Continuously explore neighboring chargers until none is left
    while (openSet.size() > 0){

        //std::cout << openSet.size() << std::endl;

        // Find charger and car condition with the minimum estimated cost through
        std::string curCharger = Astar::findBestEstCostThrough();
        tsl::car curCar = chargerCar[curCharger];

        //std::cout << "this is curCharger: " << curCharger << std::endl;

        // Reconstruct path if goalCharger is reached
        if (curCharger == goalCharger){
            Astar::reconstructPath();
            return true;
        }

        // Remove curCharger from openSet and add to closedSet
        openSet.erase(curCharger);
        closedSet.insert(curCharger);

        // Find curCar's next possible chargers from curCharger
        std::vector<std::pair<std::string, tsl::car>> neighbors = findNeighbors(curCharger,curCar);
        
        //std::cout << "after erase: " << openSet.size() << std::endl;
        /*
        for (std::pair<std::string, tsl::car> neighbor:neighbors){
            std::cout << neighbor.first << std::endl;
        }
        */
        
        //std::cout << "new neighbors.size: " << neighbors.size() << std::endl;

        // Loop through all next possible chargers
        for (std::pair<std::string, tsl::car> neighbor:neighbors){
            // If in closedSet (visited), continue to prevent cycles
            if (closedSet.count(neighbor.first)){
                continue;
            }
            // Compute cost from initial to neighbor charger
            double newCost = costToArrive[curCharger]+cost(neighbor.first,curCharger);

            double battLeft = neighbor.second.batt;
            
            neighbor.second.batt = 320.0;
            
            // If not in openSet (not visited), add to openSet
            if (!openSet.count(neighbor.first)){
                openSet.insert(neighbor.first);
                // Continue if cost from initial to neighbor charge is greater than stored cost
            } else if (newCost > costToArrive[neighbor.first]){
                continue;
            }
            cameFrom[neighbor.first] = curCharger;
            costToArrive[neighbor.first] = newCost;
            estCostThrough[neighbor.first] = newCost+cost(neighbor.first,goalCharger);
            chargerCar[neighbor.first] = neighbor.second;

            std::array<double, 3> neighParam = chargerMap[neighbor.first];
            chargeTime[neighbor.first] = (320.0-battLeft)/neighParam[2];
        }
        
        //std::cout << " " << std::endl;
        //std::cout << "this is after insert: " << openSet.size() << std::endl;

        //break;
    }
    return false;
}


// Return path string including charging time
std::string Astar::showPath(){
    return outputStr;
}