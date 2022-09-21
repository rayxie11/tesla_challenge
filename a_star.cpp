#include <a_star.h>

// Astar object constructor
Astar::Astar(std::array<row, 303>& network, 
             std::unordered_map<std::string, std::array<double, 3>>& chargerMap,
             std::string initCharger, std::string goalCharger,
             tsl::car initCar)
             {
                // Set initial and goal charger names
                this->initCharger = initCharger;
                this->goalCharger = goalCharger;

                // Set chargerMap
                this->chargerMap = chargerMap;

                // Set initial car
                this->car = initCar;
                
                // Calculate initial to goal charger estimated cost
                double initGoalCost = Util::dist(this->chargerMap[this->initCharger],
                                                 this->chargerMap[this->goalCharger]);
                cha::toChargerCost initChargerEstCost(this->initCharger, initGoalCost);

                // openPQueue stores chargers to be visited in PQueue
                this->openPQueue.push(initChargerEstCost);

                // openSet stores chargers to be visited
                this->openSet.insert(this->initCharger);

                // chargerCar stores charger and car condition
                this->chargerCar[this->initCharger] = this->car;

                // costToArrive stores cost to arrive at charger
                this->costToArrive[this->initCharger] = 0.0;

                // chargetime stores time charing at charger
                this->chargeTime[this->initCharger] = 0.0;

                // estCostThrough stores estimated cost through charger to goal charger
                this->estCostThrough[this->initCharger] = initGoalCost;
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
            tsl::car newCar(curCar.v,curCar.batt-distance);
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
    while (!openPQueue.empty() > 0){
        // Find charger and car condition with the minimum estimated cost through
        cha::toChargerCost curChargerCost = openPQueue.top();
        std::string curCharger = curChargerCost.name;
        double curCost = curChargerCost.val;
        tsl::car curCar = chargerCar[curCharger];

        // Reconstruct path if goalCharger is reached
        if (curCharger == goalCharger){
            Astar::reconstructPath();
            return true;
        }

        // Remove curCharger from openSet, openPQueue and add to closedSet
        openSet.erase(curCharger);
        openPQueue.pop();
        closedSet.insert(curCharger);

        // Find curCar's next possible chargers from curCharger
        std::vector<std::pair<std::string, tsl::car>> neighbors = findNeighbors(curCharger,curCar);
        
        // Loop through all next possible chargers
        for (std::pair<std::string, tsl::car> neighbor:neighbors){
            // If in closedSet (visited), continue to prevent cycles
            if (closedSet.count(neighbor.first)){
                continue;
            }
            // Compute cost from initial to neighbor charger
            double newCost = costToArrive[curCharger]+cost(neighbor.first,curCharger);

            cha::toChargerCost neighborChargerCost(neighbor.first,newCost);

            double battLeft = neighbor.second.batt;
            
            neighbor.second.batt = 320.0;
            
            // If not in openSet (not visited), add to openSet and openPQueue
            if (!openSet.count(neighbor.first)){
                openSet.insert(neighbor.first);
                openPQueue.push(neighborChargerCost);
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
    }
    return false;
}


// Return path string including charging time
std::string Astar::showPath(){
    return outputStr;
}