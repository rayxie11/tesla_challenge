#include "util.h"

#define _USE_MATH_DEFINES
#include <cmath>

// Earth radius constant
static const double EARTH_RADIUS = 6356.752;


// Changes the angle from degree to radian
double Util::toRadians(double degree){
    double deg = M_PI/180.0;
    return deg*degree;
}

// Returns the distance between two charging stations
double Util::dist(row s1, row s2){
    double lat1 = Util::toRadians(s1.lat);
    double lon1 = Util::toRadians(s1.lon);
    double lat2 = Util::toRadians(s2.lat);
    double lon2 = Util::toRadians(s2.lon);

    double diffLat = lat2-lat1;
    double diffLon = lon2-lon1;

    double ans = pow(sin(diffLat/2.0),2.0)+
                     cos(lat1)*cos(lat2)*
                     pow(sin(diffLon/2.0),2.0);
    
    return 2*asin(sqrt(ans))*EARTH_RADIUS;
}