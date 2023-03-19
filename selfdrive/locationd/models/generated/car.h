#pragma once
#include "rednose/helpers/common_ekf.h"
extern "C" {
void car_update_25(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_24(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_30(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_26(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_27(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_29(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_28(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_update_31(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea);
void car_err_fun(double *nom_x, double *delta_x, double *out_2401352543292586602);
void car_inv_err_fun(double *nom_x, double *true_x, double *out_1912508296609185463);
void car_H_mod_fun(double *state, double *out_5578770441770784076);
void car_f_fun(double *state, double dt, double *out_1518318369999282121);
void car_F_fun(double *state, double dt, double *out_7278996818111342548);
void car_h_25(double *state, double *unused, double *out_7161842581966940983);
void car_H_25(double *state, double *unused, double *out_6873660070202111982);
void car_h_24(double *state, double *unused, double *out_4311580574243124230);
void car_H_24(double *state, double *unused, double *out_3315734471399788846);
void car_h_30(double *state, double *unused, double *out_731096912917582376);
void car_H_30(double *state, double *unused, double *out_7045387673379831436);
void car_h_26(double *state, double *unused, double *out_1558751156692731784);
void car_H_26(double *state, double *unused, double *out_7831580684633383410);
void car_h_27(double *state, double *unused, double *out_5341720509167409105);
void car_H_27(double *state, double *unused, double *out_9177762329145776963);
void car_h_29(double *state, double *unused, double *out_2421544747653263336);
void car_H_29(double *state, double *unused, double *out_7555619017694223620);
void car_h_28(double *state, double *unused, double *out_2304934457723076420);
void car_H_28(double *state, double *unused, double *out_2473220000624693046);
void car_h_31(double *state, double *unused, double *out_6886648519682435094);
void car_H_31(double *state, double *unused, double *out_7205372582400031934);
void car_predict(double *in_x, double *in_P, double *in_Q, double dt);
void car_set_mass(double x);
void car_set_rotational_inertia(double x);
void car_set_center_to_front(double x);
void car_set_center_to_rear(double x);
void car_set_stiffness_front(double x);
void car_set_stiffness_rear(double x);
}