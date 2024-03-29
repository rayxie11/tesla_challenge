/*
 * Songchun (Ray) Xie
 * Tesla Coding Challenge Solution 
 * a_star.ccp
 * This file implements the Astar class specified in a_star.h
*/

#include "a_star.h"

extern std::array<row, 303> network;

// Astar object constructor
Astar::Astar(std::unordered_map<std::string, std::array<double, 3>>& chargerMap,
             std::string initCharger, std::string goalCharger,
             double v, double fullBatt)
             {
                // Set initial and goal charger names
                this->initCharger = initCharger;
                this->goalCharger = goalCharger;

                // Set vehicle velocity and max battery life
                this->v = v;
                this->fullBatt = fullBatt;

                // Initialize car with full battery
                tsl::car initCar(this->fullBatt);

                // Set chargerMap
                this->chargerMap = chargerMap;
                
                // Calculate initial to goal charger estimated cost
                double initEstCost = cost(this->initCharger, this->goalCharger);
                cha::toChargerCost initChargerEstCost(this->initCharger, initEstCost);

                // openPQueue stores chargers to be visited in PQueue
                this->openPQueue.push(initChargerEstCost);

                // openSet stores chargers to be visited
                this->openSet.insert(this->initCharger);

                // chargerPath stores charger and the path from initial to current charger
                Path initPath(chargerMap, initCharger, initCar);
                this->chargerPath[initCharger] = initPath;

                // costToArrive stores cost to arrive at charger
                this->costToArrive[this->initCharger] = 0.0;
                
                // estCostThrough stores estimated cost through charger to goal charger
                this->estCostThrough[this->initCharger] = initEstCost;
             }


// Build the output string for found solution path
void Astar::reconstructPath(Path path){
    outputStr = initCharger+", "+path.getOutputStr()+goalCharger;
}


// Find valid chargers that the car can reach right now
std::vector<std::string> Astar::findNeighbors(std::string s, tsl::car curCar){
    std::vector<std::string> res;

    // Loop through all chargers to see if reachable by curCar
    for (row elem:network){
        // Continue if staying put (trivial)
        if (elem.name == s){
            continue;
        }
        double distance = Util::dist(chargerMap, elem.name, s);
        if (distance <= curCar.batt){
            res.push_back(elem.name);
        }
    }
    return res;
}


// Calculate the travelling distance between chargers
double Astar::dist(std::string s1, std::string s2){
    return Util::dist(chargerMap, s1, s2);
}


// Calculate the travelling time between chargers
double Astar::time(std::string s1, std::string s2){
    return dist(s1, s2)/v;
}


// Calculate the cost of travelling between chargers
double Astar::cost(std::string s1, std::string s2){
    // Faster chargers have smaller costs
    return time(s1, s2)-0.01*chargerMap[s1][2];
}


// Astar solver function
bool Astar::solve(){
    // Continuously explore neighboring chargers until none is left
    while (!openPQueue.empty() > 0){
        // Find charger and car condition with the minimum estimated cost through
        cha::toChargerCost curChargerCost = openPQueue.top();
        std::string curCharger = curChargerCost.name;
        double curCost = curChargerCost.cost;

        // Get the path that leads to curCharger
        Path curPath = chargerPath[curCharger];
        // Get current car configuration
        tsl::car curCar = curPath.getCurWayPoint().car;

        // Reconstruct path if goalCharger is reached
        if (curCharger == goalCharger){
            reconstructPath(curPath);
            return true;
        }

        // Remove curCharger from openSet, openPQueue and add to closedSet
        openSet.erase(curCharger);
        openPQueue.pop();
        closedSet.insert(curCharger);

        // Find curCar's next possible chargers from curCharger
        std::vector<std::string> neighbors = findNeighbors(curCharger, curCar);

        // If no neighbors, not enough battery life, search again with full battery life
        bool needToCharge = false;
        if (neighbors.size() == 0){
            tsl::car fullCar(fullBatt);
            neighbors = findNeighbors(curCharger, fullCar);
            needToCharge = true;
        }

        // Loop through all next possible chargers
        for (std::string neighbor:neighbors){
            // If in closedSet, continue to prevent cycles
            if (closedSet.count(neighbor)){
                continue;
            }

            // Make a copy of the current path
            Path newPath = curPath;

            // New car condition at neighbor, battery life initialize to 0
            tsl::car newCar(0.0);

            // If car needs to charge, go through current path to charge
            if (needToCharge){
                double chargeNeeded = dist(curCharger, neighbor)-curCar.batt;
                bool isCharged = newPath.chargeCar(chargeNeeded);
                // If not charged properly, neighbor unreachable, give up this search
                if (!isCharged){
                    continue;
                } 
            } else {
                // If no need to charge, compute battery life left after travelling to this neighbor
                double battLeft = curCar.batt-dist(curCharger, neighbor);
                newCar.batt = battLeft;
            }

            // Update path up until this neighbor
            newPath.addNewCharger(neighbor, newCar);
            
            // Compute cost from initial to neighbor charger
            double newCost = newPath.getTotTime();
            cha::toChargerCost neighborChargerCost(neighbor, newCost);
            
            // If not in openSet, add to openSet and openPQueue
            if (!openSet.count(neighbor)){
                openSet.insert(neighbor);
                openPQueue.push(neighborChargerCost);
                // Continue if cost from initial to neighbor charge is greater than stored cost
            } else if (newCost > costToArrive[neighbor]){
                continue;
            }

            // Store params
            costToArrive[neighbor] = newCost;
            estCostThrough[neighbor] = newCost+cost(neighbor, goalCharger);
            chargerPath[neighbor] = newPath;
        }
    }
    return false;
}


// Return path string including charging time
std::string Astar::showPath(){
    return outputStr;
}