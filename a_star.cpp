#include "a_star.h"

extern std::array<row, 303> network;

// Astar object constructor
Astar::Astar(std::unordered_map<std::string, std::array<double, 3>>& chargerMap,
             std::string initCharger, std::string goalCharger,
             tsl::car initCar)
             {
                // Set initial and goal charger names
                this->initCharger = initCharger;
                this->goalCharger = goalCharger;

                // Set chargerMap
                this->chargerMap = chargerMap;
                
                // Calculate initial to goal charger estimated cost
                double initEstCost = dist(this->initCharger, this->goalCharger);
                cha::toChargerCost initChargerEstCost(this->initCharger, initEstCost);

                // openPQueue stores chargers to be visited in PQueue
                this->openPQueue.push(initChargerEstCost);

                // openSet stores chargers to be visited
                this->openSet.insert(this->initCharger);
                
                /*
                // chargerCar stores charger and car condition
                this->chargerCar[this->initCharger] = this->car;
                */
                
                // chargerPath stores charger and the path from initial to current charger
                Path initPath(chargerMap, initCharger, initCar);
                this->chargerPath[initCharger] = initPath;

                // costToArrive stores cost to arrive at charger
                this->costToArrive[this->initCharger] = 0.0;

                /*
                // chargetime stores time charing at charger
                this->chargeTime[this->initCharger] = 0.0;
                */
                
                // estCostThrough stores estimated cost through charger to goal charger
                this->estCostThrough[this->initCharger] = initEstCost;
             }


// Build the output string for found solution path
void Astar::reconstructPath(Path path){
    outputStr = path.getOutputStr()+goalCharger;
    /*
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
    */
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


// Calculate the cost of travelling from s1 to s2
double Astar::cost(std::string s1, std::string s2){
    double distance = dist(s1,s2);

    // Faster charging station have smaller costs
    double cost = distance-0.5*chargerMap[s2][2];
    return cost;
}


// Astar solver function
bool Astar::solve(){
    // Continuously explore neighboring chargers until none is left
    while (!openPQueue.empty() > 0){

        //std::cout << "front loop openPQueue size: " << openPQueue.size() << std::endl;


        // Find charger and car condition with the minimum estimated cost through
        cha::toChargerCost curChargerCost = openPQueue.top();
        std::string curCharger = curChargerCost.name;
        double curCost = curChargerCost.val;

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

        // If no neighbors, not enough battery life, search with full battery life
        bool needToCharge = false;
        if (neighbors.size() == 0){
            tsl::car fullCar(320.0);
            neighbors = findNeighbors(curCharger, fullCar);
            needToCharge = true;
        }

        //std::cout << "# of neighbors: " << neighbors.size() << std::endl;
        
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
                //std::cout << "need charging" << std::endl;
                double chargeNeeded = dist(curCharger, neighbor)-curCar.batt;
                bool isCharged = newPath.chargeCar(chargeNeeded);
                // If not charged, neighbor unreachable, give up this search
                if (!isCharged){

                    //std::cout << "charging unavailable" << std::endl;

                    continue;
                } 
            }

            // Compute cost from initial to neighbor charger
            double newCost = costToArrive[curCharger]+cost(neighbor, curCharger);
            cha::toChargerCost neighborChargerCost(neighbor, newCost);

            // If no need to charge, compute battery life left after travelling to this neighbor
            if (!needToCharge){
                double battLeft = curCar.batt-dist(curCharger, neighbor);
                newCar.batt = battLeft;
            }

            // Update path up until this neighbor
            newPath.addCharger(neighbor, newCar);
            
            //std::cout << "openSet contains neighbor: " << openSet.count(neighbor) << std::endl;
            //std::cout << "new cost: " << newCost << std::endl;

            // If not in openSet, add to openSet and openPQueue
            if (!openSet.count(neighbor)){
                //std::cout << "added neighbor" << std::endl;
                openSet.insert(neighbor);
                openPQueue.push(neighborChargerCost);
                // Continue if cost from initial to neighbor charge is greater than stored cost
            } else if (newCost > costToArrive[neighbor]){
                continue;
            }

            // Store params
            cameFrom[neighbor] = curCharger;
            costToArrive[neighbor] = newCost;
            estCostThrough[neighbor] = newCost+dist(neighbor, goalCharger);
            chargerPath[neighbor] = newPath;
            /*
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
            //chargerCar[neighbor.first] = neighbor.second;

            std::array<double, 3> neighParam = chargerMap[neighbor.first];
            //chargeTime[neighbor.first] = (320.0-battLeft)/neighParam[2];
            */
        }

        //std::cout << "end loop openPQueue size: " << openPQueue.size() << std::endl;
        std::cout << " " << std::endl;
    }
    return false;
}


// Return path string including charging time
std::string Astar::showPath(){
    return outputStr;
}