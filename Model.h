#ifndef MODEL_H
#define MODEL_H

#include "TreeNode.h"
#include <vector>
#include <string>

class Model {
public:
    Model();
    void evaluateCommandLine(int argc, char* argv[], double& thetaOne, double& thetaTwo, double& thetaThree, double& lengthOne, double& lengthTwo, double& lengthThree);
    void run(int argc, char* argv[]);
    static std::vector<Eigen::Vector4d> calculateCuboidVertices(const Eigen::Vector4d& lowerLeft, const Eigen::Vector4d& upperRight);
    static std::vector<Eigen::Vector4d> transformVertices(const std::vector<Eigen::Vector4d>& verts, const Eigen::Matrix4d& transform);
    static void printToOpenInventorFormat(const std::unique_ptr<TreeNode>& node);
    static void printSpheresToOpenInventorFormat(const Eigen::Matrix4d& node);
};

#endif // MODEL_H
