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
            //this->totTime = 0.0;
           }


// Path object copy constructor
Path::Path(const Path& rhs){
    this->chargerMap = rhs.chargerMap;
    this->chargerPQ = rhs.chargerPQ;
    this->path = rhs.path;
}


// Path object default constructor
Path::Path(){}


// Return the path solution string
std::string Path::getOutputStr(){
    /*
    std::string res = "";
    //for (int i = 1; i < path.size()-1; i++){
    for (int i = 0; i < path.size(); i++){
        cha::waypoint waypoint = path[i];
        res += waypoint.name+", "+std::to_string(waypoint.chargeTime)+", "+std::to_string(waypoint.car.batt)+", ";
    }
    return res;
    */

    std::string res = "";
    for (int i = 1; i < path.size()-1; i++){
        cha::waypoint waypoint = path[i];
        res += waypoint.name+", "+std::to_string(waypoint.chargeTime)+", ";
    }
    return res;
}

/*
// Update path and chargerPQ for maxCharge >= chargeRequired
void Path::updateMaxGeqReq(){
    // Update path
    path[bestCharger.idx]= bestCharger;

    // Update chargerPQ

    chargerPQ.pop();
    //chargerPQ.push(bestCharger);
    bestCharger = chargerPQ.top();
}


// Update path and chargerPQ for maxCharge < chargeRequired
bool Path::updateMaxLesReq(){
    // If chargerPQ has less than one element, no best charger
    if (chargerPQ.size() < 1){
        return false;
    }

    // Update path
    path[bestCharger.idx] = bestCharger;

    // Update chargerPQ
    chargerPQ.pop();
    bestCharger = chargerPQ.top();
    return true;
}
*/

bool Path::verify(){
    double batt = 320.0;
    for (int i = 1; i < path.size(); i++){
        double distance = Util::dist(chargerMap,path[i-1].name,path[i].name);
        batt -= distance;
        batt += path[i].chargeTime*path[i].speed;
        std::cout << path[i].name << "; batt: " << path[i].car.batt << "; chargetime: " << path[i].chargeTime << "; distance: " << distance << std::endl;
        if (batt < 0 || batt > 320){
            //std::cout << getOutputStr() << std::endl;
            std::cout << path[i-1].name << "<->" << path[i].name << "; " << batt << std::endl;
            std::cout << "bad charge! check!!!!!!!!!!!!!!!!!!!!!" << std::endl;
            //return false;
        }
    }
    return true;
}


// Go through all chargers and return the max amount that can be charged
double Path::checkMaxCharge(double proposedCharge, int idx){
    //int tracker = idx;
    //int orgIdx = idx;
    double maxCharge = proposedCharge;
    while (idx < path.size()){
        double curMaxCharge = path[idx].car.topBatt-path[idx].car.batt;
        if (curMaxCharge < maxCharge){
            maxCharge = curMaxCharge;
            //tracker = idx;
        }
        idx ++;
    }
    /*
    if (tracker != orgIdx){
        std::pair<double, int> topPair = chargerPQ.top();
        while (topPair.second != idx){
            chargerPQ.pop();
            topPair = chargerPQ.top();
        }
    }*/
    return maxCharge;
}



// Charge the car at bestCharger
bool Path::chargeCar(double chargeRequired){
    
    while (!chargerPQ.empty() && chargeRequired > 0){
        int idx = chargerPQ.top().second;

        std::cout << path[idx].name << ":" << std::endl;
        
        /*
        std::cout << "before 0: " << getOutputStr() << std::endl;
        if (path[idx].car.batt >= 320){
            chargerPQ.pop();
            continue;
        }*/

        double actualCharge = checkMaxCharge(chargeRequired, idx);
        path[idx].chargeTime += actualCharge/path[idx].speed;
        path[idx].car.batt += actualCharge;

        
        if (actualCharge < chargeRequired){
            chargerPQ.pop();
        }
        /*
        if (path[idx].car.batt >= path[idx].car.topBatt){
            chargerPQ.pop();
        }*/

        while (idx < path.size()-1){
            idx ++;
            path[idx].car.batt += actualCharge;
        }

        chargeRequired -= actualCharge;
        /*
        double maxCharge = 320-path[idx].car.batt;

        //std::cout << "max charge: " << maxCharge << "charge speed: " << path[i]. << std::endl;

        if (maxCharge > chargeRequired){
            double actualCharge = checkMaxCharge(maxCharge, idx);


            path[idx].chargeTime += chargeRequired/path[idx].speed;
            path[idx].car.batt += chargeRequired;
        } else {
            path[idx].chargeTime += maxCharge/path[idx].speed;
            path[idx].car.batt += maxCharge;
            chargerPQ.pop();
        }
        std::cout << "before 1: " << getOutputStr() << std::endl;
        while (idx < path.size()-1){
            idx ++;
            path[idx].car.batt += std::min(maxCharge, chargeRequired);
            if (path[idx].car.batt > 320){
                //path[idx].car.batt = 320;
                return false;
            }
        }
        std::cout << "after: " << getOutputStr() << std::endl;
        
        chargeRequired -= maxCharge;
        */
    }
    
    //std::cout << bestCharger.name << " time: " << bestCharger.chargeTime << ", batt: " << bestCharger.car.batt << std::endl;

    // If reached maximum charge, charge at the next bestCharger until no charging is needed

    // If still charge left, charging unsuccessful
    if (chargeRequired > 0.0){
        return false;
    }
    std::cout << " " << std::endl;
    std::cout << "----check----" << std::endl;
    bool check = verify();
    std::cout << "----check----" << std::endl;
    
    if (!check){
        return false;
    }
    return true;
}


// Add a new charger and update the new bestCharger for path
void Path::addNewCharger(std::string charger, tsl::car car){
    double chargeSpeed = chargerMap[charger][2];
    cha::waypoint newWayPoint(charger, chargeSpeed, car);

    // Push into chargerPQ
    chargerPQ.push({chargeSpeed, path.size()});

    // Push into path vector
    path.push_back(newWayPoint);

    // Update best charger
    //bestCharger = chargerPQ.top();

    // Add travelling distance from current charger to new charger
    //totTime += Util::dist(chargerMap, getCurWayPoint().name, charger)/car.v;
}

            
// Get the lastest chargerCar in path
cha::waypoint Path::getCurWayPoint(){
    int idx = path.size()-1;
    return path[idx];
}