#include "MPCsolver.h"
#include <iostream>

Vars vars;
Params params;
Workspace work;
Settings settings;

MPCsolver::MPCsolver()
{
}

Eigen::Matrix3d MPCsolver::OCPsolDesigner(double deltaT, Eigen::Vector3d ExternalForce, Eigen::Matrix<double, 15, 1> curState, Eigen::Matrix<double, 15, 16> ReferencePath, Eigen::Matrix<double, 15, 1> termState, Eigen::Matrix<double, 18, 2> CostFunction, Eigen::Matrix<double, 15, 2> stateLimits, Eigen::Matrix<double, 3, 2> input_constraint)
{

    // initialize 1st predictive state and control input
    Eigen::Matrix3d StateInput = Eigen::Matrix3d::Zero();
//    int num_iters_x, num_iters_y, num_iters_z;

    // initialize defaults parameters and variables
    set_defaults();
    setup_indexing();
    // disable output of solver progress
    settings.verbose = 0;
    //settings.max_iters = 50;
    settings.eps = 1e-1;
    settings.resid_tol = 1e-1;

    // set up initial state and constraints for optimal control problem
    //     Eigen::Matrix<double, 9, 1> curState;
    //     curState << CurPosition(0) - CurPosition(1),
    //                 CurPosition(0) - CurPosition(2),
    //                 CurPosition(1) - CurPosition(2),
    //                 CurPosition(0), 
    //                 CurVelocity(0), 
    //                 CurPosition(1), 
    //                 CurVelocity(1), 
    //                 CurPosition(2), 
    //                 CurVelocity(2);
    
    //     Eigen::Matrix<double, 15, 1> termState;
    //     termState << Waypoint(0) - Waypoint(1),
    //                  Waypoint(0) - Waypoint(2),
    //                  Waypoint(1) - Waypoint(2),
    //                  Waypoint(0), 
    //                  desVelocity(0), 
    //                  Waypoint(1), 
    //                  desVelocity(1), 
    //                  Waypoint(2), 
    //                  desVelocity(2);

    //     Eigen::Matrix<double, 2, 2> input_constraint;
    //     input_constraint << inputLimits(0,0), inputLimits(0,1),
    //                         inputLimits(1,0), inputLimits(1,1);

    // generate an optimal control problem for x-axis motion
    load_data(deltaT, ExternalForce, curState, termState, ReferencePath, stateLimits, input_constraint, CostFunction);
    // solve 3d OCP using cvxgen
    solve();
    // output 1st step of nominal input and state estimate (along x-axis) as reference
    StateInput = use_solution(vars);

    return StateInput;
}

void MPCsolver::load_data(double deltaT, Eigen::Vector3d ExternalForce, Eigen::Matrix<double, 15, 1> InitialState, Eigen::Matrix<double, 15, 1> TerminalState, Eigen::Matrix<double, 15, 16> ReferencePath, Eigen::Matrix<double, 15, 2> StateConstraint, Eigen::Matrix<double, 3, 2> InputConstraint, Eigen::Matrix<double, 18, 2> CostFunction)
{
  
  
    params.g[0] = ExternalForce(0);
    params.g[1] = ExternalForce(1);
    params.g[2] = ExternalForce(2);    
    
    //last 3 rows of 1st column of the cost function are for the input cost.. all other rows are stage cost (1st column) and terminal cost (2nd column)
    params.R[0] = CostFunction(15,0);
    params.R[1] = CostFunction(16,0);
    params.R[2] = CostFunction(17,0);
    
    params.x_0[0] = InitialState(0);
    params.x_0[1] = InitialState(1);
    params.x_0[2] = InitialState(2);
    params.x_0[3] = InitialState(3);
    params.x_0[4] = InitialState(4);
    params.x_0[5] = InitialState(5);
    params.x_0[6] = InitialState(6);
    params.x_0[7] = InitialState(7);
    params.x_0[8] = InitialState(8);
    params.x_0[9] = InitialState(9);
    params.x_0[10] = InitialState(10);
    params.x_0[11] = InitialState(11);
    params.x_0[12] = InitialState(12);
    params.x_0[13] = InitialState(13);
    params.x_0[14] = InitialState(14);

    params.xN[0] = TerminalState(0);
    params.xN[1] = TerminalState(1);
    params.xN[2] = TerminalState(2);
    params.xN[3] = TerminalState(3);
    params.xN[4] = TerminalState(4);
    params.xN[5] = TerminalState(5);
    params.xN[6] = TerminalState(6);
    params.xN[7] = TerminalState(7);
    params.xN[8] = TerminalState(8);  
    params.xN[9] = TerminalState(9);
    params.xN[10] = TerminalState(10);
    params.xN[11] = TerminalState(11);
    params.xN[12] = TerminalState(12);
    params.xN[13] = TerminalState(13);
    params.xN[14] = TerminalState(14);    

    params.x_min[0] = StateConstraint(0,0);
    params.x_min[1] = StateConstraint(1,0);
    params.x_min[2] = StateConstraint(2,0);
    params.x_min[3] = StateConstraint(3,0);
    params.x_min[4] = StateConstraint(4,0);
    params.x_min[5] = StateConstraint(5,0);
    params.x_min[6] = StateConstraint(6,0);
    params.x_min[7] = StateConstraint(7,0);
    params.x_min[8] = StateConstraint(8,0);    
    params.x_min[9] = StateConstraint(9,0);
    params.x_min[10] = StateConstraint(10,0);
    params.x_min[11] = StateConstraint(11,0);
    params.x_min[12] = StateConstraint(12,0);
    params.x_min[13] = StateConstraint(13,0);
    params.x_min[14] = StateConstraint(14,0);    

    params.x_max[0] = StateConstraint(0,1);
    params.x_max[1] = StateConstraint(1,1);
    params.x_max[2] = StateConstraint(2,1);
    params.x_max[3] = StateConstraint(3,1);
    params.x_max[4] = StateConstraint(4,1);
    params.x_max[5] = StateConstraint(5,1);
    params.x_max[6] = StateConstraint(6,1);
    params.x_max[7] = StateConstraint(7,1);
    params.x_max[8] = StateConstraint(8,1);   
    params.x_max[9] = StateConstraint(9,1);    
    params.x_max[10] = StateConstraint(10,1);
    params.x_max[11] = StateConstraint(11,1);
    params.x_max[12] = StateConstraint(12,1);
    params.x_max[13] = StateConstraint(13,1);    
    params.x_max[14] = StateConstraint(14,1); 

    params.u_min[0] = InputConstraint(0,0);
    params.u_min[1] = InputConstraint(1,0);
    params.u_min[2] = InputConstraint(2,0);    

    params.u_max[0] = InputConstraint(0,1);
    params.u_max[1] = InputConstraint(1,1);
    params.u_max[2] = InputConstraint(2,1);    

    params.A[0] = 1;
    params.A[1] = deltaT;
    params.A[2] = 1;
    params.A[3] = 1;
    params.A[4] = deltaT;
    params.A[5] = 1;
    params.A[6] = 1;
    params.A[7] = deltaT;
    params.A[8] = 1;
    
    params.A[9]  = 1;
    params.A[10] = deltaT;
    params.A[11] = 0.5*deltaT*deltaT;
    params.A[12] = 1;
    params.A[13] = deltaT;
    params.A[14] = 1;
    
    params.A[15] = 1;
    params.A[16] = deltaT;
    params.A[17] = 0.5*deltaT*deltaT;
    params.A[18] = 1;
    params.A[19] = deltaT;
    params.A[20] = 1;
    
    params.A[21] = 1;
    params.A[22] = deltaT;
    params.A[23] = 0.5*deltaT*deltaT;
    params.A[24] = 1;
    params.A[25] = deltaT;
    params.A[26] = 1;    

    params.B[0] = 0.5*deltaT*deltaT;
    params.B[1] = deltaT;
    params.B[2] = 0.5*deltaT*deltaT;
    params.B[3] = deltaT;
    params.B[4] = 0.5*deltaT*deltaT;
    params.B[5] = deltaT;    

    
    params.L_stage[0] = CostFunction(0,0);
    params.L_stage[1] = CostFunction(1,0);
    params.L_stage[2] = CostFunction(2,0);
    params.L_stage[3] = CostFunction(3,0);
    params.L_stage[4] = CostFunction(4,0);
    params.L_stage[5] = CostFunction(5,0);
    params.L_stage[6] = CostFunction(6,0);
    params.L_stage[7] = CostFunction(7,0);
    params.L_stage[8] = CostFunction(8,0);
    params.L_stage[9] = CostFunction(9,0);
    params.L_stage[10] = CostFunction(10,0);
    params.L_stage[11] = CostFunction(11,0);
    params.L_stage[12] = CostFunction(12,0);
    params.L_stage[13] = CostFunction(13,0);
    params.L_stage[14] = CostFunction(14,0);  

    //std::cout<<"hi i am compiled 3"<<std::endl;

    params.L_term[0] = CostFunction(0,1);
    params.L_term[1] = CostFunction(1,1);
    params.L_term[2] = CostFunction(2,1);
    params.L_term[3] = CostFunction(3,1);
    params.L_term[4] = CostFunction(4,1);
    params.L_term[5] = CostFunction(5,1);
    params.L_term[6] = CostFunction(6,1);
    params.L_term[7] = CostFunction(7,1);
    params.L_term[8] = CostFunction(8,1);
    params.L_term[9] = CostFunction(9,1);
    params.L_term[10] = CostFunction(10,1);
    params.L_term[11] = CostFunction(11,1);
    params.L_term[12] = CostFunction(12,1);
    params.L_term[13] = CostFunction(13,1);
    params.L_term[14] = CostFunction(14,1);  


    //     params.C[0] = 1; //these relate the augmented variables in the state to the real state variables
    //     params.C[1] = -1;
    //     params.C[2] = 1;
    //     params.C[3] = 1;
    //     params.C[4] = -1;
    //     params.C[5] = 1;
    //     params.C[6] = 1;
    //     params.C[7] = -1;
    //     params.C[8] = 1;    
    //     
    for(int i=0; i<15; i++)
      params.xr_0[i] = ReferencePath(i,0);
    
    for(int i=0; i<15; i++)
      params.xr_1[i] = ReferencePath(i,1);

    for(int i=0; i<15; i++)
      params.xr_2[i] = ReferencePath(i,2);

    for(int i=0; i<15; i++)
      params.xr_3[i] = ReferencePath(i,3);

    for(int i=0; i<15; i++)
      params.xr_4[i] = ReferencePath(i,4);

    for(int i=0; i<15; i++)
      params.xr_5[i] = ReferencePath(i,5);

    for(int i=0; i<15; i++)
      params.xr_6[i] = ReferencePath(i,6);

    for(int i=0; i<15; i++)
      params.xr_7[i] = ReferencePath(i,7);

    for(int i=0; i<15; i++)
      params.xr_8[i] = ReferencePath(i,8);

    for(int i=0; i<15; i++)
      params.xr_9[i] = ReferencePath(i,9);

    for(int i=0; i<15; i++)
      params.xr_10[i] = ReferencePath(i,10);

    for(int i=0; i<15; i++)
      params.xr_11[i] = ReferencePath(i,11);

    for(int i=0; i<15; i++)
      params.xr_12[i] = ReferencePath(i,12);

    for(int i=0; i<15; i++)
      params.xr_13[i] = ReferencePath(i,13);

    for(int i=0; i<15; i++)
      params.xr_14[i] = ReferencePath(i,14);    

    for(int i=0; i<15; i++)
      params.xr_15[i] = ReferencePath(i,15);        

}


Eigen::Matrix3d MPCsolver::use_solution(Vars vars)
{
   Eigen::Matrix3d StateInput_output;
   StateInput_output << vars.x_14[0], vars.x_14[1], vars.x_14[2],
                        vars.x_14[3], vars.x_14[4], vars.x_14[5],
                        vars.x_14[6], vars.u_0[0], vars.u_0[1];
                        
   //std::cout<<"predicted target velocity at the end of prediction horizon Vx = "<<vars.x_14[7]<<" Vy = "<<-vars.x_14[10]<<"  Vz = "<<-vars.x_14[13]<<std::endl;                     
                        
   return StateInput_output;
}

