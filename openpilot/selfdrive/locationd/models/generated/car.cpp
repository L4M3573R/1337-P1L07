#include "car.h"

namespace {
#define DIM 9
#define EDIM 9
#define MEDIM 9
typedef void (*Hfun)(double *, double *, double *);

double mass;

void set_mass(double x){ mass = x;}

double rotational_inertia;

void set_rotational_inertia(double x){ rotational_inertia = x;}

double center_to_front;

void set_center_to_front(double x){ center_to_front = x;}

double center_to_rear;

void set_center_to_rear(double x){ center_to_rear = x;}

double stiffness_front;

void set_stiffness_front(double x){ stiffness_front = x;}

double stiffness_rear;

void set_stiffness_rear(double x){ stiffness_rear = x;}
const static double MAHA_THRESH_25 = 3.8414588206941227;
const static double MAHA_THRESH_24 = 5.991464547107981;
const static double MAHA_THRESH_30 = 3.8414588206941227;
const static double MAHA_THRESH_26 = 3.8414588206941227;
const static double MAHA_THRESH_27 = 3.8414588206941227;
const static double MAHA_THRESH_29 = 3.8414588206941227;
const static double MAHA_THRESH_28 = 3.8414588206941227;
const static double MAHA_THRESH_31 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7567495437456270106) {
   out_7567495437456270106[0] = delta_x[0] + nom_x[0];
   out_7567495437456270106[1] = delta_x[1] + nom_x[1];
   out_7567495437456270106[2] = delta_x[2] + nom_x[2];
   out_7567495437456270106[3] = delta_x[3] + nom_x[3];
   out_7567495437456270106[4] = delta_x[4] + nom_x[4];
   out_7567495437456270106[5] = delta_x[5] + nom_x[5];
   out_7567495437456270106[6] = delta_x[6] + nom_x[6];
   out_7567495437456270106[7] = delta_x[7] + nom_x[7];
   out_7567495437456270106[8] = delta_x[8] + nom_x[8];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_6277676899296321355) {
   out_6277676899296321355[0] = -nom_x[0] + true_x[0];
   out_6277676899296321355[1] = -nom_x[1] + true_x[1];
   out_6277676899296321355[2] = -nom_x[2] + true_x[2];
   out_6277676899296321355[3] = -nom_x[3] + true_x[3];
   out_6277676899296321355[4] = -nom_x[4] + true_x[4];
   out_6277676899296321355[5] = -nom_x[5] + true_x[5];
   out_6277676899296321355[6] = -nom_x[6] + true_x[6];
   out_6277676899296321355[7] = -nom_x[7] + true_x[7];
   out_6277676899296321355[8] = -nom_x[8] + true_x[8];
}
void H_mod_fun(double *state, double *out_7420380263460044385) {
   out_7420380263460044385[0] = 1.0;
   out_7420380263460044385[1] = 0.0;
   out_7420380263460044385[2] = 0.0;
   out_7420380263460044385[3] = 0.0;
   out_7420380263460044385[4] = 0.0;
   out_7420380263460044385[5] = 0.0;
   out_7420380263460044385[6] = 0.0;
   out_7420380263460044385[7] = 0.0;
   out_7420380263460044385[8] = 0.0;
   out_7420380263460044385[9] = 0.0;
   out_7420380263460044385[10] = 1.0;
   out_7420380263460044385[11] = 0.0;
   out_7420380263460044385[12] = 0.0;
   out_7420380263460044385[13] = 0.0;
   out_7420380263460044385[14] = 0.0;
   out_7420380263460044385[15] = 0.0;
   out_7420380263460044385[16] = 0.0;
   out_7420380263460044385[17] = 0.0;
   out_7420380263460044385[18] = 0.0;
   out_7420380263460044385[19] = 0.0;
   out_7420380263460044385[20] = 1.0;
   out_7420380263460044385[21] = 0.0;
   out_7420380263460044385[22] = 0.0;
   out_7420380263460044385[23] = 0.0;
   out_7420380263460044385[24] = 0.0;
   out_7420380263460044385[25] = 0.0;
   out_7420380263460044385[26] = 0.0;
   out_7420380263460044385[27] = 0.0;
   out_7420380263460044385[28] = 0.0;
   out_7420380263460044385[29] = 0.0;
   out_7420380263460044385[30] = 1.0;
   out_7420380263460044385[31] = 0.0;
   out_7420380263460044385[32] = 0.0;
   out_7420380263460044385[33] = 0.0;
   out_7420380263460044385[34] = 0.0;
   out_7420380263460044385[35] = 0.0;
   out_7420380263460044385[36] = 0.0;
   out_7420380263460044385[37] = 0.0;
   out_7420380263460044385[38] = 0.0;
   out_7420380263460044385[39] = 0.0;
   out_7420380263460044385[40] = 1.0;
   out_7420380263460044385[41] = 0.0;
   out_7420380263460044385[42] = 0.0;
   out_7420380263460044385[43] = 0.0;
   out_7420380263460044385[44] = 0.0;
   out_7420380263460044385[45] = 0.0;
   out_7420380263460044385[46] = 0.0;
   out_7420380263460044385[47] = 0.0;
   out_7420380263460044385[48] = 0.0;
   out_7420380263460044385[49] = 0.0;
   out_7420380263460044385[50] = 1.0;
   out_7420380263460044385[51] = 0.0;
   out_7420380263460044385[52] = 0.0;
   out_7420380263460044385[53] = 0.0;
   out_7420380263460044385[54] = 0.0;
   out_7420380263460044385[55] = 0.0;
   out_7420380263460044385[56] = 0.0;
   out_7420380263460044385[57] = 0.0;
   out_7420380263460044385[58] = 0.0;
   out_7420380263460044385[59] = 0.0;
   out_7420380263460044385[60] = 1.0;
   out_7420380263460044385[61] = 0.0;
   out_7420380263460044385[62] = 0.0;
   out_7420380263460044385[63] = 0.0;
   out_7420380263460044385[64] = 0.0;
   out_7420380263460044385[65] = 0.0;
   out_7420380263460044385[66] = 0.0;
   out_7420380263460044385[67] = 0.0;
   out_7420380263460044385[68] = 0.0;
   out_7420380263460044385[69] = 0.0;
   out_7420380263460044385[70] = 1.0;
   out_7420380263460044385[71] = 0.0;
   out_7420380263460044385[72] = 0.0;
   out_7420380263460044385[73] = 0.0;
   out_7420380263460044385[74] = 0.0;
   out_7420380263460044385[75] = 0.0;
   out_7420380263460044385[76] = 0.0;
   out_7420380263460044385[77] = 0.0;
   out_7420380263460044385[78] = 0.0;
   out_7420380263460044385[79] = 0.0;
   out_7420380263460044385[80] = 1.0;
}
void f_fun(double *state, double dt, double *out_3045596088569131513) {
   out_3045596088569131513[0] = state[0];
   out_3045596088569131513[1] = state[1];
   out_3045596088569131513[2] = state[2];
   out_3045596088569131513[3] = state[3];
   out_3045596088569131513[4] = state[4];
   out_3045596088569131513[5] = dt*((-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]))*state[6] - 9.8100000000000005*state[8] + stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*state[1]) + (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*state[4])) + state[5];
   out_3045596088569131513[6] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*state[4])) + state[6];
   out_3045596088569131513[7] = state[7];
   out_3045596088569131513[8] = state[8];
}
void F_fun(double *state, double dt, double *out_8700545601531255316) {
   out_8700545601531255316[0] = 1;
   out_8700545601531255316[1] = 0;
   out_8700545601531255316[2] = 0;
   out_8700545601531255316[3] = 0;
   out_8700545601531255316[4] = 0;
   out_8700545601531255316[5] = 0;
   out_8700545601531255316[6] = 0;
   out_8700545601531255316[7] = 0;
   out_8700545601531255316[8] = 0;
   out_8700545601531255316[9] = 0;
   out_8700545601531255316[10] = 1;
   out_8700545601531255316[11] = 0;
   out_8700545601531255316[12] = 0;
   out_8700545601531255316[13] = 0;
   out_8700545601531255316[14] = 0;
   out_8700545601531255316[15] = 0;
   out_8700545601531255316[16] = 0;
   out_8700545601531255316[17] = 0;
   out_8700545601531255316[18] = 0;
   out_8700545601531255316[19] = 0;
   out_8700545601531255316[20] = 1;
   out_8700545601531255316[21] = 0;
   out_8700545601531255316[22] = 0;
   out_8700545601531255316[23] = 0;
   out_8700545601531255316[24] = 0;
   out_8700545601531255316[25] = 0;
   out_8700545601531255316[26] = 0;
   out_8700545601531255316[27] = 0;
   out_8700545601531255316[28] = 0;
   out_8700545601531255316[29] = 0;
   out_8700545601531255316[30] = 1;
   out_8700545601531255316[31] = 0;
   out_8700545601531255316[32] = 0;
   out_8700545601531255316[33] = 0;
   out_8700545601531255316[34] = 0;
   out_8700545601531255316[35] = 0;
   out_8700545601531255316[36] = 0;
   out_8700545601531255316[37] = 0;
   out_8700545601531255316[38] = 0;
   out_8700545601531255316[39] = 0;
   out_8700545601531255316[40] = 1;
   out_8700545601531255316[41] = 0;
   out_8700545601531255316[42] = 0;
   out_8700545601531255316[43] = 0;
   out_8700545601531255316[44] = 0;
   out_8700545601531255316[45] = dt*(stiffness_front*(-state[2] - state[3] + state[7])/(mass*state[1]) + (-stiffness_front - stiffness_rear)*state[5]/(mass*state[4]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[6]/(mass*state[4]));
   out_8700545601531255316[46] = -dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(mass*pow(state[1], 2));
   out_8700545601531255316[47] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8700545601531255316[48] = -dt*stiffness_front*state[0]/(mass*state[1]);
   out_8700545601531255316[49] = dt*((-1 - (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*pow(state[4], 2)))*state[6] - (-stiffness_front*state[0] - stiffness_rear*state[0])*state[5]/(mass*pow(state[4], 2)));
   out_8700545601531255316[50] = dt*(-stiffness_front*state[0] - stiffness_rear*state[0])/(mass*state[4]) + 1;
   out_8700545601531255316[51] = dt*(-state[4] + (-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(mass*state[4]));
   out_8700545601531255316[52] = dt*stiffness_front*state[0]/(mass*state[1]);
   out_8700545601531255316[53] = -9.8100000000000005*dt;
   out_8700545601531255316[54] = dt*(center_to_front*stiffness_front*(-state[2] - state[3] + state[7])/(rotational_inertia*state[1]) + (-center_to_front*stiffness_front + center_to_rear*stiffness_rear)*state[5]/(rotational_inertia*state[4]) + (-pow(center_to_front, 2)*stiffness_front - pow(center_to_rear, 2)*stiffness_rear)*state[6]/(rotational_inertia*state[4]));
   out_8700545601531255316[55] = -center_to_front*dt*stiffness_front*(-state[2] - state[3] + state[7])*state[0]/(rotational_inertia*pow(state[1], 2));
   out_8700545601531255316[56] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8700545601531255316[57] = -center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8700545601531255316[58] = dt*(-(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])*state[5]/(rotational_inertia*pow(state[4], 2)) - (-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])*state[6]/(rotational_inertia*pow(state[4], 2)));
   out_8700545601531255316[59] = dt*(-center_to_front*stiffness_front*state[0] + center_to_rear*stiffness_rear*state[0])/(rotational_inertia*state[4]);
   out_8700545601531255316[60] = dt*(-pow(center_to_front, 2)*stiffness_front*state[0] - pow(center_to_rear, 2)*stiffness_rear*state[0])/(rotational_inertia*state[4]) + 1;
   out_8700545601531255316[61] = center_to_front*dt*stiffness_front*state[0]/(rotational_inertia*state[1]);
   out_8700545601531255316[62] = 0;
   out_8700545601531255316[63] = 0;
   out_8700545601531255316[64] = 0;
   out_8700545601531255316[65] = 0;
   out_8700545601531255316[66] = 0;
   out_8700545601531255316[67] = 0;
   out_8700545601531255316[68] = 0;
   out_8700545601531255316[69] = 0;
   out_8700545601531255316[70] = 1;
   out_8700545601531255316[71] = 0;
   out_8700545601531255316[72] = 0;
   out_8700545601531255316[73] = 0;
   out_8700545601531255316[74] = 0;
   out_8700545601531255316[75] = 0;
   out_8700545601531255316[76] = 0;
   out_8700545601531255316[77] = 0;
   out_8700545601531255316[78] = 0;
   out_8700545601531255316[79] = 0;
   out_8700545601531255316[80] = 1;
}
void h_25(double *state, double *unused, double *out_2824946100795669147) {
   out_2824946100795669147[0] = state[6];
}
void H_25(double *state, double *unused, double *out_2516688438275379545) {
   out_2516688438275379545[0] = 0;
   out_2516688438275379545[1] = 0;
   out_2516688438275379545[2] = 0;
   out_2516688438275379545[3] = 0;
   out_2516688438275379545[4] = 0;
   out_2516688438275379545[5] = 0;
   out_2516688438275379545[6] = 1;
   out_2516688438275379545[7] = 0;
   out_2516688438275379545[8] = 0;
}
void h_24(double *state, double *unused, double *out_3049085865514909722) {
   out_3049085865514909722[0] = state[4];
   out_3049085865514909722[1] = state[5];
}
void H_24(double *state, double *unused, double *out_339474014668229572) {
   out_339474014668229572[0] = 0;
   out_339474014668229572[1] = 0;
   out_339474014668229572[2] = 0;
   out_339474014668229572[3] = 0;
   out_339474014668229572[4] = 1;
   out_339474014668229572[5] = 0;
   out_339474014668229572[6] = 0;
   out_339474014668229572[7] = 0;
   out_339474014668229572[8] = 0;
   out_339474014668229572[9] = 0;
   out_339474014668229572[10] = 0;
   out_339474014668229572[11] = 0;
   out_339474014668229572[12] = 0;
   out_339474014668229572[13] = 0;
   out_339474014668229572[14] = 1;
   out_339474014668229572[15] = 0;
   out_339474014668229572[16] = 0;
   out_339474014668229572[17] = 0;
}
void h_30(double *state, double *unused, double *out_6312771183888182035) {
   out_6312771183888182035[0] = state[4];
}
void H_30(double *state, double *unused, double *out_1644520231869082) {
   out_1644520231869082[0] = 0;
   out_1644520231869082[1] = 0;
   out_1644520231869082[2] = 0;
   out_1644520231869082[3] = 0;
   out_1644520231869082[4] = 1;
   out_1644520231869082[5] = 0;
   out_1644520231869082[6] = 0;
   out_1644520231869082[7] = 0;
   out_1644520231869082[8] = 0;
}
void h_26(double *state, double *unused, double *out_6908065673986044634) {
   out_6908065673986044634[0] = state[7];
}
void H_26(double *state, double *unused, double *out_6258191757149435769) {
   out_6258191757149435769[0] = 0;
   out_6258191757149435769[1] = 0;
   out_6258191757149435769[2] = 0;
   out_6258191757149435769[3] = 0;
   out_6258191757149435769[4] = 0;
   out_6258191757149435769[5] = 0;
   out_6258191757149435769[6] = 0;
   out_6258191757149435769[7] = 1;
   out_6258191757149435769[8] = 0;
}
void h_27(double *state, double *unused, double *out_709679758613972836) {
   out_709679758613972836[0] = state[3];
}
void H_27(double *state, double *unused, double *out_2173118791568555829) {
   out_2173118791568555829[0] = 0;
   out_2173118791568555829[1] = 0;
   out_2173118791568555829[2] = 0;
   out_2173118791568555829[3] = 1;
   out_2173118791568555829[4] = 0;
   out_2173118791568555829[5] = 0;
   out_2173118791568555829[6] = 0;
   out_2173118791568555829[7] = 0;
   out_2173118791568555829[8] = 0;
}
void h_29(double *state, double *unused, double *out_6982675176999014474) {
   out_6982675176999014474[0] = state[1];
}
void H_29(double *state, double *unused, double *out_511875864546261266) {
   out_511875864546261266[0] = 0;
   out_511875864546261266[1] = 1;
   out_511875864546261266[2] = 0;
   out_511875864546261266[3] = 0;
   out_511875864546261266[4] = 0;
   out_511875864546261266[5] = 0;
   out_511875864546261266[6] = 0;
   out_511875864546261266[7] = 0;
   out_511875864546261266[8] = 0;
}
void h_28(double *state, double *unused, double *out_2400127593349653796) {
   out_2400127593349653796[0] = state[0];
}
void H_28(double *state, double *unused, double *out_4570523152523269308) {
   out_4570523152523269308[0] = 1;
   out_4570523152523269308[1] = 0;
   out_4570523152523269308[2] = 0;
   out_4570523152523269308[3] = 0;
   out_4570523152523269308[4] = 0;
   out_4570523152523269308[5] = 0;
   out_4570523152523269308[6] = 0;
   out_4570523152523269308[7] = 0;
   out_4570523152523269308[8] = 0;
}
void h_31(double *state, double *unused, double *out_1304974248711835435) {
   out_1304974248711835435[0] = state[8];
}
void H_31(double *state, double *unused, double *out_6884399859382787245) {
   out_6884399859382787245[0] = 0;
   out_6884399859382787245[1] = 0;
   out_6884399859382787245[2] = 0;
   out_6884399859382787245[3] = 0;
   out_6884399859382787245[4] = 0;
   out_6884399859382787245[5] = 0;
   out_6884399859382787245[6] = 0;
   out_6884399859382787245[7] = 0;
   out_6884399859382787245[8] = 1;
}
#include <eigen3/Eigen/Dense>
#include <iostream>

typedef Eigen::Matrix<double, DIM, DIM, Eigen::RowMajor> DDM;
typedef Eigen::Matrix<double, EDIM, EDIM, Eigen::RowMajor> EEM;
typedef Eigen::Matrix<double, DIM, EDIM, Eigen::RowMajor> DEM;

void predict(double *in_x, double *in_P, double *in_Q, double dt) {
  typedef Eigen::Matrix<double, MEDIM, MEDIM, Eigen::RowMajor> RRM;

  double nx[DIM] = {0};
  double in_F[EDIM*EDIM] = {0};

  // functions from sympy
  f_fun(in_x, dt, nx);
  F_fun(in_x, dt, in_F);


  EEM F(in_F);
  EEM P(in_P);
  EEM Q(in_Q);

  RRM F_main = F.topLeftCorner(MEDIM, MEDIM);
  P.topLeftCorner(MEDIM, MEDIM) = (F_main * P.topLeftCorner(MEDIM, MEDIM)) * F_main.transpose();
  P.topRightCorner(MEDIM, EDIM - MEDIM) = F_main * P.topRightCorner(MEDIM, EDIM - MEDIM);
  P.bottomLeftCorner(EDIM - MEDIM, MEDIM) = P.bottomLeftCorner(EDIM - MEDIM, MEDIM) * F_main.transpose();

  P = P + dt*Q;

  // copy out state
  memcpy(in_x, nx, DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
}

// note: extra_args dim only correct when null space projecting
// otherwise 1
template <int ZDIM, int EADIM, bool MAHA_TEST>
void update(double *in_x, double *in_P, Hfun h_fun, Hfun H_fun, Hfun Hea_fun, double *in_z, double *in_R, double *in_ea, double MAHA_THRESHOLD) {
  typedef Eigen::Matrix<double, ZDIM, ZDIM, Eigen::RowMajor> ZZM;
  typedef Eigen::Matrix<double, ZDIM, DIM, Eigen::RowMajor> ZDM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, EDIM, Eigen::RowMajor> XEM;
  //typedef Eigen::Matrix<double, EDIM, ZDIM, Eigen::RowMajor> EZM;
  typedef Eigen::Matrix<double, Eigen::Dynamic, 1> X1M;
  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor> XXM;

  double in_hx[ZDIM] = {0};
  double in_H[ZDIM * DIM] = {0};
  double in_H_mod[EDIM * DIM] = {0};
  double delta_x[EDIM] = {0};
  double x_new[DIM] = {0};


  // state x, P
  Eigen::Matrix<double, ZDIM, 1> z(in_z);
  EEM P(in_P);
  ZZM pre_R(in_R);

  // functions from sympy
  h_fun(in_x, in_ea, in_hx);
  H_fun(in_x, in_ea, in_H);
  ZDM pre_H(in_H);

  // get y (y = z - hx)
  Eigen::Matrix<double, ZDIM, 1> pre_y(in_hx); pre_y = z - pre_y;
  X1M y; XXM H; XXM R;
  if (Hea_fun){
    typedef Eigen::Matrix<double, ZDIM, EADIM, Eigen::RowMajor> ZAM;
    double in_Hea[ZDIM * EADIM] = {0};
    Hea_fun(in_x, in_ea, in_Hea);
    ZAM Hea(in_Hea);
    XXM A = Hea.transpose().fullPivLu().kernel();


    y = A.transpose() * pre_y;
    H = A.transpose() * pre_H;
    R = A.transpose() * pre_R * A;
  } else {
    y = pre_y;
    H = pre_H;
    R = pre_R;
  }
  // get modified H
  H_mod_fun(in_x, in_H_mod);
  DEM H_mod(in_H_mod);
  XEM H_err = H * H_mod;

  // Do mahalobis distance test
  if (MAHA_TEST){
    XXM a = (H_err * P * H_err.transpose() + R).inverse();
    double maha_dist = y.transpose() * a * y;
    if (maha_dist > MAHA_THRESHOLD){
      R = 1.0e16 * R;
    }
  }

  // Outlier resilient weighting
  double weight = 1;//(1.5)/(1 + y.squaredNorm()/R.sum());

  // kalman gains and I_KH
  XXM S = ((H_err * P) * H_err.transpose()) + R/weight;
  XEM KT = S.fullPivLu().solve(H_err * P.transpose());
  //EZM K = KT.transpose(); TODO: WHY DOES THIS NOT COMPILE?
  //EZM K = S.fullPivLu().solve(H_err * P.transpose()).transpose();
  //std::cout << "Here is the matrix rot:\n" << K << std::endl;
  EEM I_KH = Eigen::Matrix<double, EDIM, EDIM>::Identity() - (KT.transpose() * H_err);

  // update state by injecting dx
  Eigen::Matrix<double, EDIM, 1> dx(delta_x);
  dx  = (KT.transpose() * y);
  memcpy(delta_x, dx.data(), EDIM * sizeof(double));
  err_fun(in_x, delta_x, x_new);
  Eigen::Matrix<double, DIM, 1> x(x_new);

  // update cov
  P = ((I_KH * P) * I_KH.transpose()) + ((KT.transpose() * R) * KT);

  // copy out state
  memcpy(in_x, x.data(), DIM * sizeof(double));
  memcpy(in_P, P.data(), EDIM * EDIM * sizeof(double));
  memcpy(in_z, y.data(), y.rows() * sizeof(double));
}




}
extern "C" {

void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_25, H_25, NULL, in_z, in_R, in_ea, MAHA_THRESH_25);
}
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<2, 3, 0>(in_x, in_P, h_24, H_24, NULL, in_z, in_R, in_ea, MAHA_THRESH_24);
}
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_30, H_30, NULL, in_z, in_R, in_ea, MAHA_THRESH_30);
}
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_26, H_26, NULL, in_z, in_R, in_ea, MAHA_THRESH_26);
}
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_27, H_27, NULL, in_z, in_R, in_ea, MAHA_THRESH_27);
}
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_29, H_29, NULL, in_z, in_R, in_ea, MAHA_THRESH_29);
}
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_28, H_28, NULL, in_z, in_R, in_ea, MAHA_THRESH_28);
}
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_31, H_31, NULL, in_z, in_R, in_ea, MAHA_THRESH_31);
}
void car_err_fun(double *nom_x, double *delta_x, double *out_7567495437456270106) {
  err_fun(nom_x, delta_x, out_7567495437456270106);
}
void car_inv_err_fun(double *nom_x, double *true_x, double *out_6277676899296321355) {
  inv_err_fun(nom_x, true_x, out_6277676899296321355);
}
void car_H_mod_fun(double *state, double *out_7420380263460044385) {
  H_mod_fun(state, out_7420380263460044385);
}
void car_f_fun(double *state, double dt, double *out_3045596088569131513) {
  f_fun(state,  dt, out_3045596088569131513);
}
void car_F_fun(double *state, double dt, double *out_8700545601531255316) {
  F_fun(state,  dt, out_8700545601531255316);
}
void car_h_25(double *state, double *unused, double *out_2824946100795669147) {
  h_25(state, unused, out_2824946100795669147);
}
void car_H_25(double *state, double *unused, double *out_2516688438275379545) {
  H_25(state, unused, out_2516688438275379545);
}
void car_h_24(double *state, double *unused, double *out_3049085865514909722) {
  h_24(state, unused, out_3049085865514909722);
}
void car_H_24(double *state, double *unused, double *out_339474014668229572) {
  H_24(state, unused, out_339474014668229572);
}
void car_h_30(double *state, double *unused, double *out_6312771183888182035) {
  h_30(state, unused, out_6312771183888182035);
}
void car_H_30(double *state, double *unused, double *out_1644520231869082) {
  H_30(state, unused, out_1644520231869082);
}
void car_h_26(double *state, double *unused, double *out_6908065673986044634) {
  h_26(state, unused, out_6908065673986044634);
}
void car_H_26(double *state, double *unused, double *out_6258191757149435769) {
  H_26(state, unused, out_6258191757149435769);
}
void car_h_27(double *state, double *unused, double *out_709679758613972836) {
  h_27(state, unused, out_709679758613972836);
}
void car_H_27(double *state, double *unused, double *out_2173118791568555829) {
  H_27(state, unused, out_2173118791568555829);
}
void car_h_29(double *state, double *unused, double *out_6982675176999014474) {
  h_29(state, unused, out_6982675176999014474);
}
void car_H_29(double *state, double *unused, double *out_511875864546261266) {
  H_29(state, unused, out_511875864546261266);
}
void car_h_28(double *state, double *unused, double *out_2400127593349653796) {
  h_28(state, unused, out_2400127593349653796);
}
void car_H_28(double *state, double *unused, double *out_4570523152523269308) {
  H_28(state, unused, out_4570523152523269308);
}
void car_h_31(double *state, double *unused, double *out_1304974248711835435) {
  h_31(state, unused, out_1304974248711835435);
}
void car_H_31(double *state, double *unused, double *out_6884399859382787245) {
  H_31(state, unused, out_6884399859382787245);
}
void car_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
void car_set_mass(double x) {
  set_mass(x);
}
void car_set_rotational_inertia(double x) {
  set_rotational_inertia(x);
}
void car_set_center_to_front(double x) {
  set_center_to_front(x);
}
void car_set_center_to_rear(double x) {
  set_center_to_rear(x);
}
void car_set_stiffness_front(double x) {
  set_stiffness_front(x);
}
void car_set_stiffness_rear(double x) {
  set_stiffness_rear(x);
}
}

const EKF car = {
  .name = "car",
  .kinds = { 25, 24, 30, 26, 27, 29, 28, 31 },
  .feature_kinds = {  },
  .f_fun = car_f_fun,
  .F_fun = car_F_fun,
  .err_fun = car_err_fun,
  .inv_err_fun = car_inv_err_fun,
  .H_mod_fun = car_H_mod_fun,
  .predict = car_predict,
  .hs = {
    { 25, car_h_25 },
    { 24, car_h_24 },
    { 30, car_h_30 },
    { 26, car_h_26 },
    { 27, car_h_27 },
    { 29, car_h_29 },
    { 28, car_h_28 },
    { 31, car_h_31 },
  },
  .Hs = {
    { 25, car_H_25 },
    { 24, car_H_24 },
    { 30, car_H_30 },
    { 26, car_H_26 },
    { 27, car_H_27 },
    { 29, car_H_29 },
    { 28, car_H_28 },
    { 31, car_H_31 },
  },
  .updates = {
    { 25, car_update_25 },
    { 24, car_update_24 },
    { 30, car_update_30 },
    { 26, car_update_26 },
    { 27, car_update_27 },
    { 29, car_update_29 },
    { 28, car_update_28 },
    { 31, car_update_31 },
  },
  .Hes = {
  },
  .sets = {
    { "mass", car_set_mass },
    { "rotational_inertia", car_set_rotational_inertia },
    { "center_to_front", car_set_center_to_front },
    { "center_to_rear", car_set_center_to_rear },
    { "stiffness_front", car_set_stiffness_front },
    { "stiffness_rear", car_set_stiffness_rear },
  },
  .extra_routines = {
  },
};

ekf_lib_init(car)
