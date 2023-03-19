#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void live_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_9(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_12(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_35(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_32(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_update_33(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void live_H(double *in_vec, double *out_2021545193082224962);
void live_err_fun(double *nom_x, double *delta_x, double *out_683457954489119670);
void live_inv_err_fun(double *nom_x, double *true_x, double *out_2290656940267729341);
void live_H_mod_fun(double *state, double *out_1415390971775915258);
void live_f_fun(double *state, double dt, double *out_4752873359450517443);
void live_F_fun(double *state, double dt, double *out_2579228032140110114);
void live_h_4(double *state, double *unused, double *out_2936653668917254183);
void live_H_4(double *state, double *unused, double *out_5235003673643920434);
void live_h_9(double *state, double *unused, double *out_7728031144715738627);
void live_H_9(double *state, double *unused, double *out_5476193320273511079);
void live_h_10(double *state, double *unused, double *out_3605403209797755241);
void live_H_10(double *state, double *unused, double *out_6549190697710806901);
void live_h_12(double *state, double *unused, double *out_2198153258387525618);
void live_H_12(double *state, double *unused, double *out_8192283992033669387);
void live_h_35(double *state, double *unused, double *out_6692132050259859426);
void live_H_35(double *state, double *unused, double *out_8601665731016527810);
void live_h_32(double *state, double *unused, double *out_644450187701719295);
void live_H_32(double *state, double *unused, double *out_1686801227322411454);
void live_h_13(double *state, double *unused, double *out_1048110653605419057);
void live_H_13(double *state, double *unused, double *out_774523308631420790);
void live_h_14(double *state, double *unused, double *out_7728031144715738627);
void live_H_14(double *state, double *unused, double *out_5476193320273511079);
void live_h_33(double *state, double *unused, double *out_5981138783616953205);
void live_H_33(double *state, double *unused, double *out_6694521338054166202);
void live_predict(double *in_x, double *in_P, double *in_Q, double dt);
}