/*
 * Songchun (Ray) Xie
 * Tesla Coding Challenge Solution
 * path.cpp
 * This file implements Path class specified in path.h
*/

#include "path.h"

// Path object constructor
Path::Path(std::unordered_map<std::string, std::array<double, 3>>& chargerMap,
           std::string initCharger, tsl::car initCar)
           {
            // Set chargerMap
            this->chargerMap = chargerMap;

            // Get initial charger speed
            double initSpeed = chargerMap[initCharger][2];
            cha::waypoint initWayPoint(initCharger,initSpeed,initCar);

            // Push initCharger into chargerQ
            std::pair<double, int> speedIdx = {initSpeed,0};
            this->chargerPQ.push(speedIdx);

            // Push initCharger into path
            this->path.push_back(initWayPoint);

            // Total running time on this path
            this->totTime = 0.0;
           }


// Path object copy constructor
Path::Path(const Path& rhs){
    this->chargerMap = rhs.chargerMap;
    this->chargerPQ = rhs.chargerPQ;
    this->path = rhs.path;
    this->totTime = rhs.totTime;
}


// Path object default constructor
Path::Path(){}


// Return the path solution string excluding initial and goal chargers
std::string Path::getOutputStr(){
    std::string res = "";
    for (int i = 1; i < path.size()-1; i++){
        cha::waypoint waypoint = path[i];
        res += waypoint.name+", "+std::to_string(waypoint.chargeTime)+", ";
    }
    return res;
}


// Go from the best charger and return the max amount that can be charged in current path
double Path::checkMaxCharge(double proposedCharge, int idx){
    // Record original parameters
    int tracker = idx;
    int orgIdx = idx;
    double maxCharge = proposedCharge;

    // Find the lowest allowed charge from the best charger
    while (idx < path.size()){
        double curMaxCharge = path[idx].car.topBatt-path[idx].car.batt;
        if (curMaxCharge < maxCharge){
            // Update the charge and index in path
            maxCharge = curMaxCharge;
            tracker = idx;
        }
        idx ++;
    }

    // If maxCharge is 0, pop the current bestCharger
    if (maxCharge == 0.0){
        chargerPQ.pop();
    }
    return maxCharge;
}


// Charge the car at bestCharger
/* This is the most confusing part for me so here's more details
 * on the logic implemented here. chargerPQ stores all the chargers 
 * in a priority queue (faster charging speed, greater priority). 
 * When the car needs to be charged, we want to backtrack to the 
 * fastest charger and see how much we can charge there (limited 
 * by the battery left in car at that charger). Another limiting
 * factor is the available charge left in the car for the following
 * chargers after the fastest charger. For example, if I can charge
 * 20 km at the current fastest charger but can only charge 10 km
 * at a charger trailing the current fastest charger, then I can
 * only charge a max of 10 km at the current fastest charger. Pop 
 * from chargerPQ if the actual charge is 0. We continue this 
 * process until either the required charge turns 0 or chargerPQ
 * is empty (charge unsuccessful, give up this path). 
*/
bool Path::chargeCar(double chargeRequired){
    // Charge until required charge is 0 or chargerPQ is empty
    while (!chargerPQ.empty() && chargeRequired > 0){
        // Get best charger index in path
        int idx = chargerPQ.top().second;
        // Calculate the actual charge that can be charged
        double actualCharge = checkMaxCharge(chargeRequired, idx);

        // If actualCharge = 0, continue
        if (actualCharge == 0.0){
            continue;
        }

        // Calculate the charging time
        double t = actualCharge/path[idx].speed;

        // Update path waypoint
        path[idx].chargeTime += t;
        path[idx].car.batt += actualCharge;
        
        // Add charging time to path total time
        totTime += t;

        // Add charged amount to all path waypoints
        while (idx < path.size()-1){
            idx ++;
            path[idx].car.batt += actualCharge;
        }

        chargeRequired -= actualCharge;
    }
    // If still charge left, charging unsuccessful
    if (chargeRequired > 0.0){
        return false;
    }

    return true;
}


// Add a new charger and update the new bestCharger for path
void Path::addNewCharger(std::string charger, tsl::car car){
    // Generate new waypoint
    double chargeSpeed = chargerMap[charger][2];
    cha::waypoint newWayPoint(charger, chargeSpeed, car);

    // Push into chargerPQ
    chargerPQ.push({chargeSpeed, path.size()});

    // Push into path vector
    path.push_back(newWayPoint);

    // Add travelling time to new waypoint to total time
    totTime += Util::dist(chargerMap, getCurWayPoint().name, charger)/car.v;
}

            
// Get the lastest chargerCar in path
cha::waypoint Path::getCurWayPoint(){
    int idx = path.size()-1;
    return path[idx];
}


// Get total travelling time
double Path::getTotTime(){
    return totTime;
}