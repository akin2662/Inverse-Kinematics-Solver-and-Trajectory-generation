# Inverse-Kinematics-Solver-and-Trajectory-generation
**Note:** This project was a requirement for the course ENPM 662- Introduction to Robot Modeling at University of Maryland, College Park

## Project Description
The project involves inverse kinematics solver for a 6-DOF robotic manipulator using the Denavit-Hartenberg (DH) convention. It computes transformation matrices, derives the Jacobian matrix, and simulates the trajectory of the end effector.

## Dependencies
* Python 3.11 (any version above 3 should work)
* Python running IDE (I've used Pycharm)

## Libraries used
* SymPy
* NumPy
* math
* matplolib

## Instructions
1. Download the zip file and extract it
	
2. Install python and the required dependencies: 

   * From the terminal in your IDE, run the following commands:
     
      `git clone https://github.com/sympy/sympy.git`

      `git pull origin master`

      `python -m pip install -e`

     `pip install numpy matplotlib'
	
4. Run the code or use desired python IDE:

	`$python3 Ikinematics_solver.py`

## How it works

* Transformation Matrices: The program constructs transformation matrices using DH parameters.
* Jacobian Computation: It calculates the Jacobian matrix to relate joint velocities to end-effector velocities.
* Inverse Kinematics: The inverse Jacobian is used to update joint angles iteratively.
* Trajectory Visualization: The end-effector movement is plotted in 3D.

