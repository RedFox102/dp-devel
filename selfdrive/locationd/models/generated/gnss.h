#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7380922654444343112);
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_132778549325657714);
void gnss_H_mod_fun(double *state, double *out_5595204560979213869);
void gnss_f_fun(double *state, double dt, double *out_398141659558175432);
void gnss_F_fun(double *state, double dt, double *out_4210516850527502917);
void gnss_h_6(double *state, double *sat_pos, double *out_441394932428483495);
void gnss_H_6(double *state, double *sat_pos, double *out_1724092817002163779);
void gnss_h_20(double *state, double *sat_pos, double *out_4020082234097327616);
void gnss_H_20(double *state, double *sat_pos, double *out_5697814943515096384);
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4615274250774322780);
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8953292931421286144);
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4615274250774322780);
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8953292931421286144);
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt);
}