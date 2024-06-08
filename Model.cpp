#include <memory>
#include <iostream>
#include "Model.h"
#include "Transformation.h"
#include "TreeNode.h"

Model::Model() {}

void Model::evaluateCommandLine(int argc, char* argv[], double& thetaOne, double& thetaTwo, double& thetaThree, double& lengthOne, double& lengthTwo, double& lengthThree) {
    for (int i = 0; i < argc; ++i) {
        if (std::string(argv[i]) == "-t" && i + 1 < argc) {
            thetaOne = std::stod(argv[i + 1]);
            i++;
        }
        else if (std::string(argv[i]) == "-u" && i + 1 < argc) {
            thetaTwo = std::stod(argv[i + 1]);
            i++;
        }
        else if (std::string(argv[i]) == "-v" && i + 1 < argc) {
            thetaThree = std::stod(argv[i + 1]);
            i++;
        }
        else if (std::string(argv[i]) == "-l" && i + 1 < argc) {
            lengthOne = std::stod(argv[i + 1]);
            i++;
        }
        else if (std::string(argv[i]) == "-m" && i + 1 < argc) {
            lengthTwo = std::stod(argv[i + 1]);
            i++;
        }
        else if (std::string(argv[i]) == "-n" && i + 1 < argc) {
            lengthThree = std::stod(argv[i + 1]);
            i++;
        }
    }
}

void Model::run(int argc, char* argv[]) {
    double thetaOne = -51.00;
    double thetaTwo = 39.00;
    double thetaThree = 65.00;
    double lengthOne = 4.00;
    double lengthTwo = 3.00;
    double lengthThree = 2.5;
    Eigen::Vector4d baseLowerLeftCorner(-2.0, -2.0, 0.0, 1.0);
    Eigen::Vector4d baseUpperRightCorner(2.0, 2.0, 1.0, 1.0);
    evaluateCommandLine(argc, argv, thetaOne, thetaTwo, thetaThree, lengthOne, lengthTwo, lengthThree);

    std::vector<Eigen::Vector4d> baseVerts = calculateCuboidVertices(baseLowerLeftCorner, baseUpperRightCorner);
    auto base = std::make_unique<TreeNode>(Eigen::Matrix4d::Identity(), baseVerts);

    Eigen::Matrix4d m = Transformation::translationZ(1) * Transformation::rotateAroundZ(thetaOne);
    std::vector<Eigen::Vector4d> link1Verts = calculateCuboidVertices(Eigen::Vector4d(-0.5, -0.5, 0.0, 1.0), Eigen::Vector4d(0.5, 0.5, lengthOne, 1.0));
    std::vector<Eigen::Vector4d> link1Transformed = transformVertices(link1Verts, m);
    auto link1 = std::make_unique<TreeNode>(m, link1Transformed);
    base->addChild(std::move(link1));

    m = m * Transformation::translationZ(lengthOne) * Transformation::rotateAroundY(thetaTwo);
    std::vector<Eigen::Vector4d> link2Verts = calculateCuboidVertices(Eigen::Vector4d(-0.5, -0.5, 0.0, 1.0), Eigen::Vector4d(0.5, 0.5, lengthTwo, 1.0));
    std::vector<Eigen::Vector4d> link2Transformed = transformVertices(link2Verts, m);
    auto link2 = std::make_unique<TreeNode>(m, link2Transformed);
    base->child->addChild(std::move(link2));

    m = m * Transformation::translationZ(lengthTwo) * Transformation::rotateAroundY(thetaThree);
    std::vector<Eigen::Vector4d> link3Verts = calculateCuboidVertices(Eigen::Vector4d(-0.5, -0.5, 0.0, 1.0), Eigen::Vector4d(0.5, 0.5, lengthThree, 1.0));
    std::vector<Eigen::Vector4d> link3Transformed = transformVertices(link3Verts, m);
    auto link3 = std::make_unique<TreeNode>(m, link3Transformed);
    base->child->child->addChild(std::move(link3));

    Eigen::Matrix4d finalSphereTransform = base->child->child->child->transformationMatrix;
    finalSphereTransform *= Transformation::translationZ(lengthThree);

    std::cout << "#Inventor V2.0 ascii\n";
    printToOpenInventorFormat(base);
    printSpheresToOpenInventorFormat(finalSphereTransform);
}

std::vector<Eigen::Vector4d> Model::calculateCuboidVertices(const Eigen::Vector4d& lowerLeft, const Eigen::Vector4d& upperRight) {
    std::vector<Eigen::Vector4d> vertices(8);

    vertices[0] = upperRight;
    vertices[1] = Eigen::Vector4d(lowerLeft[0], upperRight[1], upperRight[2], 1.0);
    vertices[2] = Eigen::Vector4d(lowerLeft[0], lowerLeft[1], upperRight[2], 1.0);
    vertices[3] = Eigen::Vector4d(upperRight[0], lowerLeft[1], upperRight[2], 1.0);
    vertices[4] = Eigen::Vector4d(upperRight[0], upperRight[1], lowerLeft[2], 1.0);
    vertices[5] = Eigen::Vector4d(lowerLeft[0], upperRight[1], lowerLeft[2], 1.0);
    vertices[6] = lowerLeft;
    vertices[7] = Eigen::Vector4d(upperRight[0], lowerLeft[1], lowerLeft[2], 1.0);

    return vertices;
}

std::vector<Eigen::Vector4d> Model::transformVertices(const std::vector<Eigen::Vector4d>& verts, const Eigen::Matrix4d& transform) {
    std::vector<Eigen::Vector4d> transformedVerts;
    transformedVerts.reserve(verts.size());
    for (const auto& vert : verts) {
        transformedVerts.push_back(transform * vert);
    }
    return transformedVerts;
}

void Model::printToOpenInventorFormat(const std::unique_ptr<TreeNode>& node) {
    if (!node) return;

    std::cout << "Separator {\n";
    std::cout << "  Coordinate3 {\n";
    std::cout << "    point [\n";
    for (const auto& vertex : node->vertices) {
        std::cout << "      " << vertex[0] << " " << vertex[1] << " " << vertex[2] << ",\n";
    }
    std::cout << "    ]\n";
    std::cout << "  }\n";

    std::cout << "  IndexedLineSet {\n";
    std::cout << "    coordIndex [\n";
    std::cout << " 0, 1, 2, 0, -1, \n";
    std::cout << " 0, 2, 3, 0, -1, \n";
    std::cout << " 7, 6, 5, 7, -1, \n";
    std::cout << " 7, 5, 4, 7, -1, \n";
    std::cout << " 0, 3, 7, 0, -1, \n";
    std::cout << " 0, 7, 4, 0, -1, \n";
    std::cout << " 1, 5, 6, 1, -1, \n";
    std::cout << " 1, 6, 2, 1, -1, \n";
    std::cout << " 0, 4, 5, 0, -1, \n";
    std::cout << " 0, 5, 1, 0, -1, \n";
    std::cout << " 3, 2, 6, 3, -1, \n";
    std::cout << " 3, 6, 7, 3, -1\n";
    std::cout << "    ]\n";
    std::cout << "  }\n";
    std::cout << "}\n";
    printToOpenInventorFormat(node->child);
}

void Model::printSpheresToOpenInventorFormat(const Eigen::Matrix4d& finalSphereTransform) {
    std::cout << "   Separator{  \n";
    std::cout << " LightModel { \n";
    std::cout << " model PHONG \n";
    std::cout << " } \n";
    std::cout << " Material { \n";
    std::cout << "        diffuseColor 1.0 1.0 1.0 \n";
    std::cout << " } \n";
    std::cout << " Sphere { \n";
    std::cout << "        radius  0.20 \n";
    std::cout << "    } \n";
    std::cout << " }\n";
    std::cout << "   Separator{  \n";
    std::cout << " LightModel { \n";
    std::cout << " model PHONG \n";
    std::cout << " } \n";
    std::cout << " Material { \n";
    std::cout << "        diffuseColor 1.0 1.0 1.0 \n";
    std::cout << " } \n";
    std::cout << " Transform { \n";
    std::cout << " translation " << finalSphereTransform(0, 3) << " " << finalSphereTransform(1, 3) << " " << finalSphereTransform(2, 3) << "\n";
    std::cout << " } \n";
    std::cout << " Sphere { \n";
    std::cout << "        radius  0.20 \n";
    std::cout << "    } \n";
    std::cout << " }\n";
}
