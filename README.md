# CG_hw7 - 3D Hierarchical Model Evaluation

## Author
Tony Stanell

## Date
June 7, 2024

## Course Section
CS536

## Program Overview

This program, developed in C++, is designed for evaluating a 3D Hierarchical Model, specifically a robot defined by joint angles and link lengths.

## Development Environment

The program was built using C++ on Linux. It was developed with Visual Studio 2022, using a remote SSH connection to Drexel's Tux servers for compilation. The standard C++ compiler associated with Visual Studio was used.

## Robot Specification

The robot is defined by six parameters, three joint angles and three link lengths:

- **Joint Angles:**
  - θ1 (Rotation around the Z axis in world coordinates): `-t θ1_val`
  - θ2 (Rotation around the Y axis of Link 1's coordinate system): `-u θ2_val`
  - θ3 (Rotation around the Y axis of Link 2's coordinate system): `-v θ3_val`
- **Link Lengths:**
  - L1: `-l L1_val`
  - L2: `-m L2_val`
  - L3: `-n L3_val`

### Default Values:
- θ1 = -51°, θ2 = 39°, θ3 = 65°
- L1 = 4, L2 = 3, L3 = 2.5

### Execution Command

/CG_hw7 > out.iv


## Features

- **3D Hierarchical Model:** The model consists of four cuboids (links), connected by three joints.
- **Transformation Matrices:** Each node in the TreeNode class contains a transformation matrix and a 4D array of vertices. Vertices are transformed using the current transformation matrix to convert local coordinates into world coordinates.
- **Dynamic Configuration:** Link lengths and joint angles can be adjusted dynamically through user inputs, allowing for a parametric representation of the robot model.

## Description of Key Functions

- **transformVertices:** This function takes a transformation matrix and multiplies it with each vertex in the 4D vertices array to produce a transformed set of vertices.
- **TreeNode:** Nodes in this class represent components of the model, holding both the transformation matrix and the transformed vertices.

## Dependencies

Ensure you have a C++ compiler like g++, configured for your environment.
