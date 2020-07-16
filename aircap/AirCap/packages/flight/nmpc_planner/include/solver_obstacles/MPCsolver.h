#ifndef OCPSOLVER_H
#define OCPSOLVER_H

#include <Eigen/Dense>

extern "C"{
#include "solver.h"
}



class MPCsolver
{

public:
    MPCsolver();

    // Design a trajectory containing desired position, velocity, acceleration, jerk via the solution of the convex optimization problem
    Eigen::Matrix<double, 9, 17> OCPsolDesigner(double deltaT, Eigen::Matrix<double, 3, 16> obstacle_force, Eigen::Vector3d ExternalForce, Eigen::Matrix<double, 15, 1> curState, Eigen::Matrix<double, 15, 16> ReferencePath, Eigen::Matrix<double, 15, 1> termState, Eigen::Matrix<double, 18, 2> CostFunction, Eigen::Matrix<double, 15, 2> stateLimits, Eigen::Matrix<double, 3, 2> input_constraint);
    
    // Load and generate required parameters for the optimal control problem
    void load_data(double deltaT, Eigen::Matrix<double, 3, 16> obstacle_force, Eigen::Vector3d ExternalForce, Eigen::Matrix<double, 15, 1> InitialState, Eigen::Matrix<double, 15, 1> TerminalState, Eigen::Matrix<double, 15, 16> ReferencePath, Eigen::Matrix<double, 15, 2> StateConstraint, Eigen::Matrix<double, 3, 2> InputConstraint, Eigen::Matrix<double, 18, 2> CostFunction);
    
    // output the solution of decoupled optimal control problems
    // x vx, y,vy,z,vz for 16 time steps and one more field for latest control inputs.
    Eigen::Matrix<double, 9, 17> use_solution(Vars vars);

};

#endif // OCPSOLVER_H
