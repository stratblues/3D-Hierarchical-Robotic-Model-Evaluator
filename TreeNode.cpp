#include "TreeNode.h"

TreeNode::TreeNode() : transformationMatrix(Eigen::Matrix4d::Identity()), child(nullptr) {}

TreeNode::TreeNode(const Eigen::Matrix4d& m, const std::vector<Eigen::Vector4d>& verts)
    : transformationMatrix(m), vertices(verts), child(nullptr) {}

void TreeNode::addChild(std::unique_ptr<TreeNode> node) {
    if (!child) {
        child = std::move(node);
    }
    else {
        TreeNode* temp = child.get();
        while (temp->child) {
            temp = temp->child.get();
        }
        temp->child = std::move(node);
    }
}
