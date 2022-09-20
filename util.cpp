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

// Returns the distance of one charging station to all the others
Eigen::ArrayXd Util::distFromStation(Eigen::MatrixXd& posMat, row s){
    double deg = M_PI/180;

    Eigen::MatrixXd sPos(1,2);
    sPos << s.lat, s.lon;
    Eigen::MatrixXd sMat = deg*sPos.replicate(303,1);
    posMat = deg*posMat;
    Eigen::ArrayXd lat1 = posMat.col(0).array();
    Eigen::ArrayXd lat2 = sMat.col(0).array();

    Eigen::MatrixXd diffMat = sMat-posMat;
    Eigen::ArrayXd diffLat = diffMat.col(0).array();
    Eigen::ArrayXd diffLon = diffMat.col(1).array();
    Eigen::ArrayXd mid = pow(sin(diffLat/2.0),2.0)+
                         cos(lat1)*cos(lat2)*
                         pow(sin(diffLon/2.0),2.0);
    Eigen::ArrayXd res = 2.0*asin(sqrt(mid))*EARTH_RADIUS;
    return res;
}