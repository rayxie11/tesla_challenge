#include "path.h"

// Path object constructor
Path::Path(std::unordered_map<std::string, std::array<double, 3>>& chargerMap,
           std::string initCharger, tsl::car initCar)
           {
            // Set chargerMap
            this->chargerMap = chargerMap;

            // Get initial charger speed
            double initSpeed = chargerMap[initCharger][2];
            cha::waypoint initWayPoint(initCharger,initSpeed,initCar,0);

            /*
            // Best charger is the charger within the path that has the fastest charging speed
            // Initialize with initCharger
            this->bestCharger = initWayPoint;
            */

            // Push initCharger into chargerQ
            this->chargerPQ.push(initWayPoint);

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
    //this->bestCharger = rhs.bestCharger;
}


// Path object default constructor
Path::Path(){}


// Return the path solution string
std::string Path::getOutputStr(){
    std::string res = "";
    //for (int i = 1; i < path.size()-1; i++){
    for (int i = 0; i < path.size(); i++){
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
        std::cout << path[i].name << "; batt: " << batt << "; chargetime: " << path[i].chargeTime<< std::endl;
        batt -= Util::dist(chargerMap,path[i-1].name,path[i].name);
        batt += path[i].chargeTime*path[i].speed;
        if (batt < 0 || batt > 320){
            //std::cout << getOutputStr() << std::endl;
            std::cout << path[i-1].name << "<->" << path[i].name << "; " << batt << std::endl;
            std::cout << "bad charge! check!" << std::endl;
            //return false;
        }
    }
    return true;
}


// Charge the car at bestCharger
bool Path::chargeCar(double chargeRequired){

    std::cout << "before charge: "<< getOutputStr() << std::endl;
    while (chargeRequired >= 0.0 && !chargerPQ.empty()){
        cha::waypoint bestCharger = chargerPQ.top();
        std::cout << "best charger: " << bestCharger.name << std::endl;
        
        double maxCharge = bestCharger.car.topBatt-bestCharger.car.batt;
        if (maxCharge > chargeRequired){
            //bestCharger.chargeTime += chargeRequired/bestCharger.speed;
            //bestCharger.car.batt += chargeRequired;
            bestCharger.chargeTime += chargeRequired/bestCharger.speed;
            bestCharger.car.batt += chargeRequired;
            chargeRequired -= maxCharge;
            path[bestCharger.idx] = bestCharger;
            chargerPQ.pop();
            chargerPQ.push(bestCharger);
            
        } else {
            //bestCharger.chargeTime += maxCharge/bestCharger.speed;
            //bestCharger.car.batt += maxCharge;
            chargeRequired -= maxCharge;
            bestCharger.chargeTime += maxCharge/bestCharger.speed;
            bestCharger.car.batt += maxCharge;
            path[bestCharger.idx] = bestCharger;
            //bestCharger.chargeTime += maxCharge/bestCharger.speed;
            //bestCharger.car.batt += maxCharge;

            //
            chargerPQ.pop();
        }
    }

    std::cout << "post charge: " << getOutputStr() << std::endl;
    
    //std::cout << bestCharger.name << " time: " << bestCharger.chargeTime << ", batt: " << bestCharger.car.batt << std::endl;

    // If reached maximum charge, charge at the next bestCharger until no charging is needed
    /*
    while (chargeRequired > 0.0){
        if (chargerPQ.size() <= 1){
            return false;
        }
        std::cout << bestCharger.name << std::endl;
        // Get max charge at bestCharge
        double maxCharge = bestCharger.car.topBatt-bestCharger.car.batt;
        std::cout << "max charge: " << maxCharge << ", required: " << chargeRequired << std::endl;
        if (maxCharge > chargeRequired){
            //double t = chargeRequired/bestCharger.speed;
            //bestCharger.chargeTime += t;
            bestCharger.chargeTime += chargeRequired/bestCharger.speed;
            bestCharger.car.batt += chargeRequired;
            // Add charging time to total time
            //totTime += t;
            // Update chargerPQ
            updateMaxGeqReq();
            //path[bestCharger.idx] = bestCharger;
            //chargerPQ.pop();
            //bestCharger = chargerPQ.top();
            chargeRequired = -1.0;
        } else {
            //double t = maxCharge/bestCharger.speed;
            //bestCharger.chargeTime += t;
            bestCharger.chargeTime += maxCharge/bestCharger.speed;
            bestCharger.car.batt += maxCharge;
            chargeRequired -= maxCharge;
            path[bestCharger.idx] = bestCharger;
            chargerPQ.pop();
            bestCharger = chargerPQ.top();
            
            // Add charging time to total time
            //totTime += t;
            // Update chargerPQ;
            //bool canCharge = updateMaxLesReq();
            //if (!canCharge){
            //    return false;
            //}
        } 
    } */

    // If still charge left, charging unsuccessful
    if (chargeRequired > 0.0){
        return false;
    }
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
    cha::waypoint newWayPoint(charger, chargeSpeed, car, path.size());

    // Push into chargerPQ
    chargerPQ.push(newWayPoint);

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