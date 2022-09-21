#pragma once

#include <string>

namespace tsl{
    struct car
    {   
        double v;                   // Velocity in km/h
        double batt;                // Battery life in km
        double topBattLife = 320.0; // Top battery life in km
        car(double v, double batt): v(v),batt(batt){}
        car(): v(0.0),batt(0.0){};
    };
}

namespace cha{
    struct charger
    {
        std::string name;  // Name of charger
        double val;        // Cost of arriving this charger
        charger(std::string name, double val):name(name),val(val){}
        bool operator< (const charger& b) const{
            return val>b.val;
        }
    };
}
