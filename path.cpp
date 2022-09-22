#include <path.h>

// Path object constructor
Path::Path(std::unordered_map<std::string, std::array<double, 3>>& chargerMap,
           std::string initCharger, tsl::car initCar)
           {
            // Set chargerMap
            this->chargerMap = chargerMap;

            // Get initial charger speed
            double initSpeed = chargerMap[initCharger][2];
            cha::chargerCar init(initCharger,initSpeed,0.0,initCar);

            // Best charger is the charger within the path that has the fastest charging speed
            // Initialize with initCharger
            this->bestCharger = init;

            // Push initCharger into chargerQ
            this->chargerQ.push(init);

            // Push initCharger into path
            this->path.push_back(init);
           }


// Path object copy constructor
Path::Path(Path& rhs){
    this->chargerMap = rhs.chargerMap;
    this->chargerQ = rhs.chargerQ;
    this->path = rhs.path;
    this->bestCharger = rhs.bestCharger;
}


// Return the path solution string
std::string Path::getOutputStr(){
    std::string res = "";
    for (cha::chargerCar waypoint:path){
        res += waypoint.name+", "+std::to_string(waypoint.chargeTime)+", ";
    }
    return res;
}


// Update the best charger
bool Path::updateBestCharger(bool pop){
    // If charger PQueue is empty, charging is unavailable
    if (chargerQ.size() == 0){
        return false;
    }

    // If charger PQueue has one element and needs to be popped, charging is unavailable
    if (chargerQ.size() == 1 && pop){
        return false;
    }

    // Two update methods: get the new bestCharger by popping current one or not
    if (pop){
        chargerQ.pop();
        bestCharger = chargerQ.top();
    } else {
        bestCharger = chargerQ.top();
    }
    return true;
}


// Charge the car at bestCharger
bool Path::chargeCar(double chargeRequired){
    // If reached maximum charge, charge at the next bestCharger until no charging is needed
    while (chargeRequired >= 0.0){
        // Get max charge at bestCharge
        double maxCharge = bestCharger.car.topBatt-bestCharger.car.batt;
        if (maxCharge >= chargeRequired){
            bestCharger.chargeTime += chargeRequired/bestCharger.speed;
            bestCharger.car.batt += chargeRequired;
            chargeRequired = -1.0;
        } else {
            bestCharger.chargeTime += maxCharge/bestCharger.speed;
            bestCharger.car.batt += maxCharge;
            chargeRequired -= maxCharge;
            bool canCharge = updateBestCharger(true);
            if (!canCharge){
                return false;
            }
        }
    }

    // If still charge left, charging unsuccessful
    if (chargeRequired > 0.0){
        return false;
    }
    return true;
}


// Add a new charger and update the new bestCharger for path
void Path::addCharger(std::string charger, tsl::car car){
    double chargeSpeed = chargerMap[charger][2];
    cha::chargerCar newCharger(charger,chargeSpeed,0.0,car);
    chargerQ.push(newCharger);
    path.push_back(newCharger);
    updateBestCharger(false);
}

            
// Get the lastest chargerCar in path
cha::chargerCar Path::getCurChargerCar(){
    int idx = path.size()-1;
    return path[idx];
}