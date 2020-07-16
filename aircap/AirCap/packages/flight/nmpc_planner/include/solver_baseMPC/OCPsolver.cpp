#include "OCPsolver.h"
#include <iostream>

Vars vars;
Params params;
Workspace work;
Settings settings;

OCPSolver::OCPSolver()
{
}

Eigen::Matrix3d OCPSolver::OCPsolDesigner(double deltaT, Eigen::Vector3d ExternalForce, Eigen::Vector3d CurPosition, Eigen::Vector3d CurVelocity, Eigen::Matrix<double, 3, 16> ReferencePath, Eigen::Vector3d Waypoint, Eigen::Vector3d desVelocity, Eigen::Matrix3d CostFunction)
{

    // initialize 1st predictive state and control input
    Eigen::Matrix3d StateInput = Eigen::Matrix3d::Zero();
//    int num_iters_x, num_iters_y, num_iters_z;

    // initialize defaults parameters and variables
    set_defaults();
    setup_indexing();
    // disable output of solver progress
    settings.verbose = 0;


    // set up initial state and constraints for optimal control problem
    Eigen::Matrix<double, 6, 1> curState;
    curState << CurPosition(0), CurVelocity(0), CurPosition(1), CurVelocity(1), CurPosition(2), CurVelocity(2);
    Eigen::Matrix<double, 6, 1> termState;
    termState << Waypoint(0), desVelocity(0), Waypoint(1), desVelocity(1), Waypoint(2), desVelocity(2);

    Eigen::Matrix<double, 6, 2> state_constraint;
    state_constraint << -15, 15,
                        -4, 4,
                        -15, 15,
                        -4, 4,
                         0, 2,
                        -4, 4;

    Eigen::Matrix<double, 3, 2> input_constraint;
    input_constraint << -30, 30,
                        -30, 30,
                         0, 50;


    // generate an optimal control problem for x-axis motion
    load_data(deltaT, ExternalForce, curState, termState, ReferencePath, state_constraint, input_constraint, CostFunction);
    // solve 3d OCP using cvxgen
    solve();
    // output 1st step of nominal input and state estimate (along x-axis) as reference
    StateInput = use_solution(vars);

    return StateInput;
}

void OCPSolver::load_data(double deltaT, Eigen::Vector3d ExternalForce, Eigen::Matrix<double, 6, 1> InitialState, Eigen::Matrix<double, 6, 1> TerminalState, Eigen::Matrix<double, 3, 16> ReferencePath, Eigen::Matrix<double, 6, 2> StateConstraint, Eigen::Matrix<double, 3, 2> InputConstraint, Eigen::Matrix3d CostFunction)
{
    params.x_0[0] = InitialState(0);
    params.x_0[1] = InitialState(1);
    params.x_0[2] = InitialState(2);
    params.x_0[3] = InitialState(3);
    params.x_0[4] = InitialState(4);
    params.x_0[5] = InitialState(5);

    params.xN[0] = TerminalState(0);
    params.xN[1] = TerminalState(1);
    params.xN[2] = TerminalState(2);
    params.xN[3] = TerminalState(3);
    params.xN[4] = TerminalState(4);
    params.xN[5] = TerminalState(5);

    params.x_min[0] = StateConstraint(0,0);
    params.x_min[1] = StateConstraint(1,0);
    params.x_min[2] = StateConstraint(2,0);
    params.x_min[3] = StateConstraint(3,0);
    params.x_min[4] = StateConstraint(4,0);
    params.x_min[5] = StateConstraint(5,0);

    params.x_max[0] = StateConstraint(0,1);
    params.x_max[1] = StateConstraint(1,1);
    params.x_max[2] = StateConstraint(2,1);
    params.x_max[3] = StateConstraint(3,1);
    params.x_max[4] = StateConstraint(4,1);
    params.x_max[5] = StateConstraint(5,1);

    params.u_min[0] = InputConstraint(0,0);
    params.u_min[1] = InputConstraint(1,0);
    params.u_min[2] = InputConstraint(2,0);

    params.u_max[0] = InputConstraint(0,1);
    params.u_max[1] = InputConstraint(1,1);
    params.u_max[2] = InputConstraint(2,1);

    params.g[0] = ExternalForce(0);
    params.g[1] = ExternalForce(1);
    params.g[2] = ExternalForce(2);

    params.A[0] = 1;
    params.A[1] = deltaT;
    params.A[2] = 1;
    params.A[3] = 1;
    params.A[4] = deltaT;
    params.A[5] = 1;
    params.A[6] = 1;
    params.A[7] = deltaT;
    params.A[8] = 1;

    params.B[0] = 0.5*deltaT*deltaT;
    params.B[1] = deltaT;
    params.B[2] = 0.5*deltaT*deltaT;
    params.B[3] = deltaT;
    params.B[4] = 0.5*deltaT*deltaT;
    params.B[5] = deltaT;

    params.L_stage[0] = CostFunction(0,0);
    params.L_stage[1] = CostFunction(1,0);
    params.L_stage[2] = CostFunction(0,0);
    params.L_stage[3] = CostFunction(1,0);
    params.L_stage[4] = CostFunction(2,0);
    params.L_stage[5] = CostFunction(1,0);

    params.R[0] = CostFunction(0,1);
    params.R[1] = CostFunction(0,1);
    params.R[2] = CostFunction(1,1);

    params.L_term[0] = CostFunction(0,2);
    params.L_term[1] = CostFunction(1,2);
    params.L_term[2] = CostFunction(0,2);
    params.L_term[3] = CostFunction(1,2);
    params.L_term[4] = CostFunction(2,2);
    params.L_term[5] = CostFunction(2,1);

    params.xr_0[0] = ReferencePath(0,0);
    params.xr_0[1] = 0;
    params.xr_0[2] = ReferencePath(1,0);
    params.xr_0[3] = 0;
    params.xr_0[4] = ReferencePath(2,0);
    params.xr_0[5] = 0;

    params.xr_1[0] = ReferencePath(0,1);
    params.xr_1[1] = 0;
    params.xr_1[2] = ReferencePath(1,1);
    params.xr_1[3] = 0;
    params.xr_1[4] = ReferencePath(2,1);
    params.xr_1[5] = 0;

    params.xr_2[0] = ReferencePath(0,2);
    params.xr_2[1] = 0;
    params.xr_2[2] = ReferencePath(1,2);
    params.xr_2[3] = 0;
    params.xr_2[4] = ReferencePath(2,2);
    params.xr_2[5] = 0;

    params.xr_3[0] = ReferencePath(0,3);
    params.xr_3[1] = 0;
    params.xr_3[2] = ReferencePath(1,3);
    params.xr_3[3] = 0;
    params.xr_3[4] = ReferencePath(2,3);
    params.xr_3[5] = 0;

    params.xr_4[0] = ReferencePath(0,4);
    params.xr_4[1] = 0;
    params.xr_4[2] = ReferencePath(1,4);
    params.xr_4[3] = 0;
    params.xr_4[4] = ReferencePath(2,4);
    params.xr_4[5] = 0;

    params.xr_5[0] = ReferencePath(0,5);
    params.xr_5[1] = 0;
    params.xr_5[2] = ReferencePath(1,5);
    params.xr_5[3] = 0;
    params.xr_5[4] = ReferencePath(2,5);
    params.xr_5[5] = 0;

    params.xr_6[0] = ReferencePath(0,6);
    params.xr_6[1] = 0;
    params.xr_6[2] = ReferencePath(1,6);
    params.xr_6[3] = 0;
    params.xr_6[4] = ReferencePath(2,6);
    params.xr_6[5] = 0;

    params.xr_7[0] = ReferencePath(0,7);
    params.xr_7[1] = 0;
    params.xr_7[2] = ReferencePath(1,7);
    params.xr_7[3] = 0;
    params.xr_7[4] = ReferencePath(2,7);
    params.xr_7[5] = 0;

    params.xr_8[0] = ReferencePath(0,8);
    params.xr_8[1] = 0;
    params.xr_8[2] = ReferencePath(1,8);
    params.xr_8[3] = 0;
    params.xr_8[4] = ReferencePath(2,8);
    params.xr_8[5] = 0;

    params.xr_9[0] = ReferencePath(0,9);
    params.xr_9[1] = 0;
    params.xr_9[2] = ReferencePath(1,9);
    params.xr_9[3] = 0;
    params.xr_9[4] = ReferencePath(2,9);
    params.xr_9[5] = 0;

    params.xr_10[0] = ReferencePath(0,10);
    params.xr_10[1] = 0;
    params.xr_10[2] = ReferencePath(1,10);
    params.xr_10[3] = 0;
    params.xr_10[4] = ReferencePath(2,10);
    params.xr_10[5] = 0;

    params.xr_11[0] = ReferencePath(0,11);
    params.xr_11[1] = 0;
    params.xr_11[2] = ReferencePath(1,11);
    params.xr_11[3] = 0;
    params.xr_11[4] = ReferencePath(2,11);
    params.xr_11[5] = 0;

    params.xr_12[0] = ReferencePath(0,12);
    params.xr_12[1] = 0;
    params.xr_12[2] = ReferencePath(1,12);
    params.xr_12[3] = 0;
    params.xr_12[4] = ReferencePath(2,12);
    params.xr_12[5] = 0;

    params.xr_13[0] = ReferencePath(0,13);
    params.xr_13[1] = 0;
    params.xr_13[2] = ReferencePath(1,13);
    params.xr_13[3] = 0;
    params.xr_13[4] = ReferencePath(2,13);
    params.xr_13[5] = 0;

    params.xr_14[0] = ReferencePath(0,14);
    params.xr_14[1] = 0;
    params.xr_14[2] = ReferencePath(1,14);
    params.xr_14[3] = 0;
    params.xr_14[4] = ReferencePath(2,14);
    params.xr_14[5] = 0;

    params.xr_15[0] = ReferencePath(0,15);
    params.xr_15[1] = 0;
    params.xr_15[2] = ReferencePath(1,15);
    params.xr_15[3] = 0;
    params.xr_15[4] = ReferencePath(2,15);
    params.xr_15[5] = 0;

}


Eigen::Matrix3d OCPSolver::use_solution(Vars vars)
{
    Eigen::Matrix3d StateInput_output;
   StateInput_output << vars.x_1[0], vars.x_1[1], vars.x_1[2],
                        vars.x_1[3], vars.x_1[4], vars.x_1[5],
                        vars.u_0[0], vars.u_0[1], vars.u_0[2];
    return StateInput_output;
}

