#ifndef TRANSFORMATION_H
#define TRANSFORMATION_H

#include <eigen3/Eigen/Dense>

class Transformation {
public:
    static Eigen::Matrix4d rotateAroundZ(double theta);
    static Eigen::Matrix4d rotateAroundY(double theta);
    static Eigen::Matrix4d translationZ(double ln);
};

#endif // TRANSFORMATION_H
