#ifndef OCPSOLVER_H
#define OCPSOLVER_H

#include <Eigen/Dense>

extern "C"{
#include "solver.h"
}



class OCPSolver
{

public:
    OCPSolver();

    // Design a trajectory containing desired position, velocity, acceleration, jerk via the solution of the convex optimization problem
    Eigen::Matrix3d OCPsolDesigner(double deltaT, Eigen::Vector3d ExternalForce, Eigen::Vector3d CurPosition, Eigen::Vector3d CurVelocity, Eigen::Matrix<double, 3, 16> ReferencePath, Eigen::Vector3d Waypoint,  Eigen::Vector3d desVelocity, Eigen::Matrix3d CostFunction);
    // Load and generate required parameters for the optimal control problem
    void load_data(double deltaT, Eigen::Vector3d ExternalForce, Eigen::Matrix<double, 6, 1> InitialState, Eigen::Matrix<double, 6, 1> TerminalState, Eigen::Matrix<double, 3, 16> ReferencePath, Eigen::Matrix<double, 6, 2> StateConstraint, Eigen::Matrix<double, 3, 2> InputConstraint, Eigen::Matrix3d CostFunction);
    // output the solution of decoupled optimal control problems
    Eigen::Matrix3d use_solution(Vars vars);

};

#endif // OCPSOLVER_H
