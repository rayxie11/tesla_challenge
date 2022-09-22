#pragma once

#include <string>

#include "structs.h"

class chargerCar{
public:
    // Constructors
    chargerCar(std::string name, double speed, tsl::car car);
    chargerCar();

    // Parameters
    std::string name;
    double speed;
    tsl::car car;

    // Member function
    void charge(double chargeRequired);
    bool operator< (const chargerCar& b);
};