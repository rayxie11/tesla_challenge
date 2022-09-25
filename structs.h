/*
 * Songchun (Ray) Xie
 * Tesla Coding Challenge Solution
 * structs.h
 * This file specifies and implements structs: tsl::car, cha::toChargerCost, and cha::waypoint
*/

#pragma once
#include <string>

namespace tsl{
    struct car
    {   
        double v = 105.0;           // Velocity in km/h
        double batt;                // Battery life in km
        double topBatt = 320.0;     // Top battery life in km

        // Constructors
        car(double batt): batt(batt){}
        car(): batt(0.0){};
    };
}


namespace cha{
    struct toChargerCost
    {
        std::string name;   // Name of charger
        double cost;        // Cost of arriving this charger

        // Constructors
        toChargerCost(std::string name, double val):name(name),cost(val){}
        toChargerCost(): name(""),cost(0.0){}
        
        // < operator for PQueue
        bool operator< (const toChargerCost& b) const{
            return cost>b.cost;
        }
    };

    struct waypoint
    {
        std::string name;        // Name of charger
        double speed;            // Charging speed
        double chargeTime = 0.0; // Charging time at charger
        tsl::car car;            // Car condition at charger

        // Constructors
        waypoint(std::string name, double speed, tsl::car car):name(name),speed(speed),car(car){}
        waypoint():name(""),speed(0.0),car(car){}
    };
}