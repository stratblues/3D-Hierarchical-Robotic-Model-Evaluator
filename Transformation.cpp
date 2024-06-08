#include "Transformation.h"
#include <cmath>

Eigen::Matrix4d Transformation::rotateAroundZ(double theta) {
    double r = theta * M_PI / 180.0;
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m(0, 0) = std::cos(r);
    m(0, 1) = -std::sin(r);
    m(1, 0) = std::sin(r);
    m(1, 1) = std::cos(r);
    return m;
}

Eigen::Matrix4d Transformation::rotateAroundY(double theta) {
    double r = theta * M_PI / 180.0;
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m(0, 0) = std::cos(r);
    m(0, 2) = std::sin(r);
    m(2, 0) = -std::sin(r);
    m(2, 2) = std::cos(r);
    return m;
}

Eigen::Matrix4d Transformation::translationZ(double ln) {
    Eigen::Matrix4d m = Eigen::Matrix4d::Identity();
    m(2, 3) = ln;
    return m;
}
