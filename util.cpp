#include "util.h"

// Earth radius constant
static const double EARTH_RADIUS = 6356.752;

// Changes the angle from degree to radian
double Util::toRadians(double degree){
    double deg = M_PI/180.0;
    return deg*degree;
}

// Returns the distance between two charging stations
double Util::dist(std::array<double, 3> s1, std::array<double, 3> s2){
    double lat1 = Util::toRadians(s1[0]);
    double lon1 = Util::toRadians(s1[1]);
    double lat2 = Util::toRadians(s2[0]);
    double lon2 = Util::toRadians(s2[1]);

    double diffLat = lat2-lat1;
    double diffLon = lon2-lon1;
    double mid = pow(sin(diffLat/2.0),2.0)+
                     cos(lat1)*cos(lat2)*
                     pow(sin(diffLon/2.0),2.0);
    double res = 2.0*asin(sqrt(mid))*EARTH_RADIUS;
    return res;
}

// Returns a charger map key: charger name, value: charger location, charging rate
std::unordered_map<std::string, std::array<double, 3>> Util::getChargerMap(std::array<row, 303>& network){
    std::unordered_map<std::string, std::array<double, 3>> chargerMap;
    for (int i = 0; i < network.size(); i++){
        std::array<double, 3> params = {network[i].lat,network[i].lon,network[i].rate};
        chargerMap[network[i].name] = params;
    }
    return chargerMap;
}