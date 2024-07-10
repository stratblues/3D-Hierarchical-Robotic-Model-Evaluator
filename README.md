@main.cpp
@author Tony Stanell
@date June 7, 2024
@section CS536


The Program: 



This is a c++ program for solving the problem of evaluating a 3D Hierachical Model.


Overview:
This program was built using C++ for Linux. The development environment I used was Visual Studio 2022 with a remote SSH connection to Drexel's Tux servers for compilation. I used the standard C++ compiler associated with visual studio. 

Specification:

1. The robot is defined by 6 parameters
	3 joint angles θ1: -t θ1_val,   θ2: -u θ2_val,   θ3: -v θ3_val
	3 link lengths L1: -l L1_val,   L2 -m L2_val,   L3 -n L3_val
Default values: θ1 = -51°,   θ2 = 39°,   θ3 = 65°,   L1 = 4,    L2 = 3,    L3 = 2.5  
 
2. θ1 defines a rotation around the Z axis in world coordinates.
θ2 defines a rotation around the Y axis of Link 1's coordinate system.
θ3 defines a rotation around the Y axis of Link 2's coordinate system.

Test with the command "./CG_hw7 > out.iv"



Features:


This program generates a 3D Hierarchical Model of a robot represented by 4 cuboids, or links, with 3 joint connections between links. In order to transform the links properly into the world coordinate system from their local system we have to form and concatenate the matrices correctly. Each node in my TreeNode class holds a transformationMatrix as well as a 4D array of vertices. I transform the vertices in the transformVertices function by taking the current transformation matrix and multiplying each 4D vertices to it to create a new 4D array of transformed vertices now transformed by that Matrix. After the vertices are transformed into world coorindates through that previous computation I then insert them into a new node and to make the tree structure I add this node as a child to the previous. For the final sphere I take the last transformation matrix and multiply it by the translation of the last length of the Z component which gives me the end of the structure. 
Because this is a parametric representation the lengths of the links can be changed with user input to the console as well as the joint angles, which are both represented as parameters when computing the 3D hierarchical model.
