#pragma once

#include <string>

namespace tsl{
    struct car
    {   
        double v = 105.0;           // Velocity in km/h
        double batt;                // Battery life in km
        double topBatt = 320.0;     // Top battery life in km
        car(double batt): batt(batt){}
        car(): batt(0.0){};
    };
}

namespace cha{
    struct toChargerCost
    {
        std::string name;  // Name of charger
        double val;        // Cost of arriving this charger
        toChargerCost(std::string name, double val):name(name),val(val){}
        bool operator< (const toChargerCost& b) const{
            return val>b.val;
        }
    };

    struct waypoint
    {
        std::string name;        // Name of charger
        double speed;            // Charging speed
        double chargeTime = 0.0; // Charging time at charger
        tsl::car car;            // Car condition at charger
        waypoint(std::string name, double speed, tsl::car car):name(name),speed(speed),car(car){}
        waypoint():name(""),speed(0.0),car(car){}
        bool operator< (const waypoint& b) const{
            return speed>b.speed;
        }
    };
}
