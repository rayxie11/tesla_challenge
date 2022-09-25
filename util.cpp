/*
 * Songchun (Ray) Xie
 * Tesla Coding Challenge Solution
 * util.cpp
 * This file implements all the utility functions specified in util.h
*/

#include "util.h"

// Earth radius constant
static const double EARTH_RADIUS = 6356.752;

// Charget network
extern std::array<row, 303> network;

// Changes the angle from degree to radian
double Util::toRadians(double degree){
    double deg = M_PI/180.0;
    return deg*degree;
}

// Returns the distance between two charging stations
double Util::dist(std::unordered_map<std::string, std::array<double, 3>>& chargerMap,
                  std::string s1, std::string s2)
                  {
                    std::array<double, 3> param1 = chargerMap[s1];
                    std::array<double, 3> param2 = chargerMap[s2];
                    double lat1 = Util::toRadians(param1[0]);
                    double lon1 = Util::toRadians(param1[1]);
                    double lat2 = Util::toRadians(param2[0]);
                    double lon2 = Util::toRadians(param2[1]);

                    double diffLat = lat2-lat1;
                    double diffLon = lon2-lon1;
                    double mid = pow(sin(diffLat/2.0),2.0)+
                                 cos(lat1)*cos(lat2)*
                                 pow(sin(diffLon/2.0),2.0);
                    double res = 2.0*asin(sqrt(mid))*EARTH_RADIUS;
                    return res;
                  } 

// Returns a charger map key: charger name, value: charger location, charging rate
std::unordered_map<std::string, std::array<double, 3>> Util::getChargerMap(){
    std::unordered_map<std::string, std::array<double, 3>> chargerMap;
    for (int i = 0; i < network.size(); i++){
        std::array<double, 3> params = {network[i].lat,network[i].lon,network[i].rate};
        chargerMap[network[i].name] = params;
    }
    return chargerMap;
}

// Checks whether initial and goal chargers are in chargerMap
bool Util::checkValid(std::string initCharger, std::string goalCharger, 
                      std::unordered_map<std::string, std::array<double, 3>>& chargerMap)
                      {
                        return !chargerMap.count(initCharger) || !chargerMap.count(goalCharger);
                      }