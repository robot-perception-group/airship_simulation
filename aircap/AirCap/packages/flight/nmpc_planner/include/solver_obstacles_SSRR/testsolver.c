/* Produced by CVXGEN, 2018-04-10 14:30:47 -0400.  */
/* CVXGEN is Copyright (C) 2006-2017 Jacob Mattingley, jem@cvxgen.com. */
/* The code in this file is Copyright (C) 2006-2017 Jacob Mattingley. */
/* CVXGEN, or solvers produced by CVXGEN, cannot be used for commercial */
/* applications without prior written permission from Jacob Mattingley. */

/* Filename: testsolver.c. */
/* Description: Basic test harness for solver.c. */
#include "solver.h"
Vars vars;
Params params;
Workspace work;
Settings settings;
#define NUMTESTS 0
int main(int argc, char **argv) {
  int num_iters;
#if (NUMTESTS > 0)
  int i;
  double time;
  double time_per;
#endif
  set_defaults();
  setup_indexing();
  load_default_data();
  /* Solve problem instance for the record. */
  settings.verbose = 1;
  num_iters = solve();
#ifndef ZERO_LIBRARY_MODE
#if (NUMTESTS > 0)
  /* Now solve multiple problem instances for timing purposes. */
  settings.verbose = 0;
  tic();
  for (i = 0; i < NUMTESTS; i++) {
    solve();
  }
  time = tocq();
  printf("Timed %d solves over %.3f seconds.\n", NUMTESTS, time);
  time_per = time / NUMTESTS;
  if (time_per > 1) {
    printf("Actual time taken per solve: %.3g s.\n", time_per);
  } else if (time_per > 1e-3) {
    printf("Actual time taken per solve: %.3g ms.\n", 1e3*time_per);
  } else {
    printf("Actual time taken per solve: %.3g us.\n", 1e6*time_per);
  }
#endif
#endif
  return 0;
}
void load_default_data(void) {
  params.g[0] = 0.20319161029830202;
  params.g[1] = 0.8325912904724193;
  params.g[2] = -0.8363810443482227;
  params.f_obs_0[0] = 0.04331042079065206;
  params.f_obs_0[1] = 1.5717878173906188;
  params.f_obs_0[2] = 1.5851723557337523;
  params.R[0] = 1.1255853104638363;
  params.R[1] = 1.2072428781381868;
  params.R[2] = 1.0514672033008299;
  params.f_obs_1[0] = -0.23676062539745413;
  params.f_obs_1[1] = -1.8804951564857322;
  params.f_obs_1[2] = -0.17266710242115568;
  params.f_obs_2[0] = 0.596576190459043;
  params.f_obs_2[1] = -0.8860508694080989;
  params.f_obs_2[2] = 0.7050196079205251;
  params.f_obs_3[0] = 0.3634512696654033;
  params.f_obs_3[1] = -1.9040724704913385;
  params.f_obs_3[2] = 0.23541635196352795;
  params.f_obs_4[0] = -0.9629902123701384;
  params.f_obs_4[1] = -0.3395952119597214;
  params.f_obs_4[2] = -0.865899672914725;
  params.f_obs_5[0] = 0.7725516732519853;
  params.f_obs_5[1] = -0.23818512931704205;
  params.f_obs_5[2] = -1.372529046100147;
  params.f_obs_6[0] = 0.17859607212737894;
  params.f_obs_6[1] = 1.1212590580454682;
  params.f_obs_6[2] = -0.774545870495281;
  params.f_obs_7[0] = -1.1121684642712744;
  params.f_obs_7[1] = -0.44811496977740495;
  params.f_obs_7[2] = 1.7455345994417217;
  params.f_obs_8[0] = 1.9039816898917352;
  params.f_obs_8[1] = 0.6895347036512547;
  params.f_obs_8[2] = 1.6113364341535923;
  params.f_obs_9[0] = 1.383003485172717;
  params.f_obs_9[1] = -0.48802383468444344;
  params.f_obs_9[2] = -1.631131964513103;
  params.f_obs_10[0] = 0.6136436100941447;
  params.f_obs_10[1] = 0.2313630495538037;
  params.f_obs_10[2] = -0.5537409477496875;
  params.f_obs_11[0] = -1.0997819806406723;
  params.f_obs_11[1] = -0.3739203344950055;
  params.f_obs_11[2] = -0.12423900520332376;
  params.f_obs_12[0] = -0.923057686995755;
  params.f_obs_12[1] = -0.8328289030982696;
  params.f_obs_12[2] = -0.16925440270808823;
  params.f_obs_13[0] = 1.442135651787706;
  params.f_obs_13[1] = 0.34501161787128565;
  params.f_obs_13[2] = -0.8660485502711608;
  params.f_obs_14[0] = -0.8880899735055947;
  params.f_obs_14[1] = -0.1815116979122129;
  params.f_obs_14[2] = -1.17835862158005;
  params.f_obs_15[0] = -1.1944851558277074;
  params.f_obs_15[1] = 0.05614023926976763;
  params.f_obs_15[2] = -1.6510825248767813;
  params.xN[0] = -0.06565787059365391;
  params.xN[1] = -0.5512951504486665;
  params.xN[2] = 0.8307464872626844;
  params.xN[3] = 0.9869848924080182;
  params.xN[4] = 0.7643716874230573;
  params.xN[5] = 0.7567216550196565;
  params.Q[0] = 1.3736001241489282;
  params.Q[1] = 1.6681348047352675;
  params.Q[2] = 1.339848663956818;
  params.Q[3] = 1.572793869868875;
  params.Q[4] = 1.3258071580648745;
  params.Q[5] = 1.445145049263532;
  params.A[0] = -1.753884276680243;
  params.A[1] = -1.0292983112626475;
  params.A[2] = 1.8864104246942706;
  params.A[3] = -1.077663182579704;
  params.A[4] = 0.7659100437893209;
  params.A[5] = 0.6019074328549583;
  params.A[6] = 0.8957565577499285;
  params.A[7] = -0.09964555746227477;
  params.A[8] = 0.38665509840745127;
  params.x_0[0] = -1.7321223042686946;
  params.x_0[1] = -1.7097514487110663;
  params.x_0[2] = -1.2040958948116867;
  params.x_0[3] = -1.3925560119658358;
  params.x_0[4] = -1.5995826216742213;
  params.x_0[5] = -1.4828245415645833;
  params.B[0] = 0.21311092723061398;
  params.B[1] = -1.248740700304487;
  params.B[2] = 1.808404972124833;
  params.B[3] = 0.7264471152297065;
  params.B[4] = 0.16407869343908477;
  params.B[5] = 0.8287224032315907;
  params.x_min[0] = -0.9444533161899464;
  params.x_min[1] = 1.7069027370149112;
  params.x_min[2] = 1.3567722311998827;
  params.x_min[3] = 0.9052779937121489;
  params.x_min[4] = -0.07904017565835986;
  params.x_min[5] = 1.3684127435065871;
  params.x_max[0] = 0.979009293697437;
  params.x_max[1] = 0.6413036255984501;
  params.x_max[2] = 1.6559010680237511;
  params.x_max[3] = 0.5346622551502991;
  params.x_max[4] = -0.5362376605895625;
  params.x_max[5] = 0.2113782926017822;
  params.u_min[0] = -1.2144776931994525;
  params.u_min[1] = -1.2317108144255875;
  params.u_min[2] = 0.9026784957312834;
  params.u_max[0] = 1.1397468137245244;
  params.u_max[1] = 1.8883934547350631;
  params.u_max[2] = 1.4038856681660068;
}
