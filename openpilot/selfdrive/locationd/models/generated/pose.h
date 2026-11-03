#pragma once
#include "rednose/helpers/ekf.h"
extern "C" {
void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void pose_err_fun(double *nom_x, double *delta_x, double *out_4830772321857990181);
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5200017052910787602);
void pose_H_mod_fun(double *state, double *out_1712341249416054868);
void pose_f_fun(double *state, double dt, double *out_3100560613383761185);
void pose_F_fun(double *state, double dt, double *out_3350602915939425859);
void pose_h_4(double *state, double *unused, double *out_2074712223981570498);
void pose_H_4(double *state, double *unused, double *out_4135740557335903853);
void pose_h_10(double *state, double *unused, double *out_5949398724887603410);
void pose_H_10(double *state, double *unused, double *out_3347508424845693314);
void pose_h_13(double *state, double *unused, double *out_9220727019356339608);
void pose_H_13(double *state, double *unused, double *out_923466732003571052);
void pose_h_14(double *state, double *unused, double *out_8529780658774629922);
void pose_H_14(double *state, double *unused, double *out_172499700996419324);
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt);
}