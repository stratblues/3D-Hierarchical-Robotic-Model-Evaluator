#ifndef TREENODE_H
#define TREENODE_H

#include <eigen3/Eigen/Dense>
#include <vector>
#include <memory>

class TreeNode {
public:
    Eigen::Matrix4d transformationMatrix;
    std::vector<Eigen::Vector4d> vertices;
    std::unique_ptr<TreeNode> child;

    TreeNode();
    TreeNode(const Eigen::Matrix4d& m, const std::vector<Eigen::Vector4d>& verts);
    void addChild(std::unique_ptr<TreeNode> node);
};

#endif // TREENODE_H

