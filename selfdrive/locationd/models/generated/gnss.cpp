#include "gnss.h"

namespace {
#define DIM 11
#define EDIM 11
#define MEDIM 11
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_6 = 3.8414588206941227;
const static double MAHA_THRESH_20 = 3.8414588206941227;
const static double MAHA_THRESH_7 = 3.8414588206941227;
const static double MAHA_THRESH_21 = 3.8414588206941227;

/******************************************************************************
 *                      Code generated with SymPy 1.11.1                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_7380922654444343112) {
   out_7380922654444343112[0] = delta_x[0] + nom_x[0];
   out_7380922654444343112[1] = delta_x[1] + nom_x[1];
   out_7380922654444343112[2] = delta_x[2] + nom_x[2];
   out_7380922654444343112[3] = delta_x[3] + nom_x[3];
   out_7380922654444343112[4] = delta_x[4] + nom_x[4];
   out_7380922654444343112[5] = delta_x[5] + nom_x[5];
   out_7380922654444343112[6] = delta_x[6] + nom_x[6];
   out_7380922654444343112[7] = delta_x[7] + nom_x[7];
   out_7380922654444343112[8] = delta_x[8] + nom_x[8];
   out_7380922654444343112[9] = delta_x[9] + nom_x[9];
   out_7380922654444343112[10] = delta_x[10] + nom_x[10];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_132778549325657714) {
   out_132778549325657714[0] = -nom_x[0] + true_x[0];
   out_132778549325657714[1] = -nom_x[1] + true_x[1];
   out_132778549325657714[2] = -nom_x[2] + true_x[2];
   out_132778549325657714[3] = -nom_x[3] + true_x[3];
   out_132778549325657714[4] = -nom_x[4] + true_x[4];
   out_132778549325657714[5] = -nom_x[5] + true_x[5];
   out_132778549325657714[6] = -nom_x[6] + true_x[6];
   out_132778549325657714[7] = -nom_x[7] + true_x[7];
   out_132778549325657714[8] = -nom_x[8] + true_x[8];
   out_132778549325657714[9] = -nom_x[9] + true_x[9];
   out_132778549325657714[10] = -nom_x[10] + true_x[10];
}
void H_mod_fun(double *state, double *out_5595204560979213869) {
   out_5595204560979213869[0] = 1.0;
   out_5595204560979213869[1] = 0;
   out_5595204560979213869[2] = 0;
   out_5595204560979213869[3] = 0;
   out_5595204560979213869[4] = 0;
   out_5595204560979213869[5] = 0;
   out_5595204560979213869[6] = 0;
   out_5595204560979213869[7] = 0;
   out_5595204560979213869[8] = 0;
   out_5595204560979213869[9] = 0;
   out_5595204560979213869[10] = 0;
   out_5595204560979213869[11] = 0;
   out_5595204560979213869[12] = 1.0;
   out_5595204560979213869[13] = 0;
   out_5595204560979213869[14] = 0;
   out_5595204560979213869[15] = 0;
   out_5595204560979213869[16] = 0;
   out_5595204560979213869[17] = 0;
   out_5595204560979213869[18] = 0;
   out_5595204560979213869[19] = 0;
   out_5595204560979213869[20] = 0;
   out_5595204560979213869[21] = 0;
   out_5595204560979213869[22] = 0;
   out_5595204560979213869[23] = 0;
   out_5595204560979213869[24] = 1.0;
   out_5595204560979213869[25] = 0;
   out_5595204560979213869[26] = 0;
   out_5595204560979213869[27] = 0;
   out_5595204560979213869[28] = 0;
   out_5595204560979213869[29] = 0;
   out_5595204560979213869[30] = 0;
   out_5595204560979213869[31] = 0;
   out_5595204560979213869[32] = 0;
   out_5595204560979213869[33] = 0;
   out_5595204560979213869[34] = 0;
   out_5595204560979213869[35] = 0;
   out_5595204560979213869[36] = 1.0;
   out_5595204560979213869[37] = 0;
   out_5595204560979213869[38] = 0;
   out_5595204560979213869[39] = 0;
   out_5595204560979213869[40] = 0;
   out_5595204560979213869[41] = 0;
   out_5595204560979213869[42] = 0;
   out_5595204560979213869[43] = 0;
   out_5595204560979213869[44] = 0;
   out_5595204560979213869[45] = 0;
   out_5595204560979213869[46] = 0;
   out_5595204560979213869[47] = 0;
   out_5595204560979213869[48] = 1.0;
   out_5595204560979213869[49] = 0;
   out_5595204560979213869[50] = 0;
   out_5595204560979213869[51] = 0;
   out_5595204560979213869[52] = 0;
   out_5595204560979213869[53] = 0;
   out_5595204560979213869[54] = 0;
   out_5595204560979213869[55] = 0;
   out_5595204560979213869[56] = 0;
   out_5595204560979213869[57] = 0;
   out_5595204560979213869[58] = 0;
   out_5595204560979213869[59] = 0;
   out_5595204560979213869[60] = 1.0;
   out_5595204560979213869[61] = 0;
   out_5595204560979213869[62] = 0;
   out_5595204560979213869[63] = 0;
   out_5595204560979213869[64] = 0;
   out_5595204560979213869[65] = 0;
   out_5595204560979213869[66] = 0;
   out_5595204560979213869[67] = 0;
   out_5595204560979213869[68] = 0;
   out_5595204560979213869[69] = 0;
   out_5595204560979213869[70] = 0;
   out_5595204560979213869[71] = 0;
   out_5595204560979213869[72] = 1.0;
   out_5595204560979213869[73] = 0;
   out_5595204560979213869[74] = 0;
   out_5595204560979213869[75] = 0;
   out_5595204560979213869[76] = 0;
   out_5595204560979213869[77] = 0;
   out_5595204560979213869[78] = 0;
   out_5595204560979213869[79] = 0;
   out_5595204560979213869[80] = 0;
   out_5595204560979213869[81] = 0;
   out_5595204560979213869[82] = 0;
   out_5595204560979213869[83] = 0;
   out_5595204560979213869[84] = 1.0;
   out_5595204560979213869[85] = 0;
   out_5595204560979213869[86] = 0;
   out_5595204560979213869[87] = 0;
   out_5595204560979213869[88] = 0;
   out_5595204560979213869[89] = 0;
   out_5595204560979213869[90] = 0;
   out_5595204560979213869[91] = 0;
   out_5595204560979213869[92] = 0;
   out_5595204560979213869[93] = 0;
   out_5595204560979213869[94] = 0;
   out_5595204560979213869[95] = 0;
   out_5595204560979213869[96] = 1.0;
   out_5595204560979213869[97] = 0;
   out_5595204560979213869[98] = 0;
   out_5595204560979213869[99] = 0;
   out_5595204560979213869[100] = 0;
   out_5595204560979213869[101] = 0;
   out_5595204560979213869[102] = 0;
   out_5595204560979213869[103] = 0;
   out_5595204560979213869[104] = 0;
   out_5595204560979213869[105] = 0;
   out_5595204560979213869[106] = 0;
   out_5595204560979213869[107] = 0;
   out_5595204560979213869[108] = 1.0;
   out_5595204560979213869[109] = 0;
   out_5595204560979213869[110] = 0;
   out_5595204560979213869[111] = 0;
   out_5595204560979213869[112] = 0;
   out_5595204560979213869[113] = 0;
   out_5595204560979213869[114] = 0;
   out_5595204560979213869[115] = 0;
   out_5595204560979213869[116] = 0;
   out_5595204560979213869[117] = 0;
   out_5595204560979213869[118] = 0;
   out_5595204560979213869[119] = 0;
   out_5595204560979213869[120] = 1.0;
}
void f_fun(double *state, double dt, double *out_398141659558175432) {
   out_398141659558175432[0] = dt*state[3] + state[0];
   out_398141659558175432[1] = dt*state[4] + state[1];
   out_398141659558175432[2] = dt*state[5] + state[2];
   out_398141659558175432[3] = state[3];
   out_398141659558175432[4] = state[4];
   out_398141659558175432[5] = state[5];
   out_398141659558175432[6] = dt*state[7] + state[6];
   out_398141659558175432[7] = dt*state[8] + state[7];
   out_398141659558175432[8] = state[8];
   out_398141659558175432[9] = state[9];
   out_398141659558175432[10] = state[10];
}
void F_fun(double *state, double dt, double *out_4210516850527502917) {
   out_4210516850527502917[0] = 1;
   out_4210516850527502917[1] = 0;
   out_4210516850527502917[2] = 0;
   out_4210516850527502917[3] = dt;
   out_4210516850527502917[4] = 0;
   out_4210516850527502917[5] = 0;
   out_4210516850527502917[6] = 0;
   out_4210516850527502917[7] = 0;
   out_4210516850527502917[8] = 0;
   out_4210516850527502917[9] = 0;
   out_4210516850527502917[10] = 0;
   out_4210516850527502917[11] = 0;
   out_4210516850527502917[12] = 1;
   out_4210516850527502917[13] = 0;
   out_4210516850527502917[14] = 0;
   out_4210516850527502917[15] = dt;
   out_4210516850527502917[16] = 0;
   out_4210516850527502917[17] = 0;
   out_4210516850527502917[18] = 0;
   out_4210516850527502917[19] = 0;
   out_4210516850527502917[20] = 0;
   out_4210516850527502917[21] = 0;
   out_4210516850527502917[22] = 0;
   out_4210516850527502917[23] = 0;
   out_4210516850527502917[24] = 1;
   out_4210516850527502917[25] = 0;
   out_4210516850527502917[26] = 0;
   out_4210516850527502917[27] = dt;
   out_4210516850527502917[28] = 0;
   out_4210516850527502917[29] = 0;
   out_4210516850527502917[30] = 0;
   out_4210516850527502917[31] = 0;
   out_4210516850527502917[32] = 0;
   out_4210516850527502917[33] = 0;
   out_4210516850527502917[34] = 0;
   out_4210516850527502917[35] = 0;
   out_4210516850527502917[36] = 1;
   out_4210516850527502917[37] = 0;
   out_4210516850527502917[38] = 0;
   out_4210516850527502917[39] = 0;
   out_4210516850527502917[40] = 0;
   out_4210516850527502917[41] = 0;
   out_4210516850527502917[42] = 0;
   out_4210516850527502917[43] = 0;
   out_4210516850527502917[44] = 0;
   out_4210516850527502917[45] = 0;
   out_4210516850527502917[46] = 0;
   out_4210516850527502917[47] = 0;
   out_4210516850527502917[48] = 1;
   out_4210516850527502917[49] = 0;
   out_4210516850527502917[50] = 0;
   out_4210516850527502917[51] = 0;
   out_4210516850527502917[52] = 0;
   out_4210516850527502917[53] = 0;
   out_4210516850527502917[54] = 0;
   out_4210516850527502917[55] = 0;
   out_4210516850527502917[56] = 0;
   out_4210516850527502917[57] = 0;
   out_4210516850527502917[58] = 0;
   out_4210516850527502917[59] = 0;
   out_4210516850527502917[60] = 1;
   out_4210516850527502917[61] = 0;
   out_4210516850527502917[62] = 0;
   out_4210516850527502917[63] = 0;
   out_4210516850527502917[64] = 0;
   out_4210516850527502917[65] = 0;
   out_4210516850527502917[66] = 0;
   out_4210516850527502917[67] = 0;
   out_4210516850527502917[68] = 0;
   out_4210516850527502917[69] = 0;
   out_4210516850527502917[70] = 0;
   out_4210516850527502917[71] = 0;
   out_4210516850527502917[72] = 1;
   out_4210516850527502917[73] = dt;
   out_4210516850527502917[74] = 0;
   out_4210516850527502917[75] = 0;
   out_4210516850527502917[76] = 0;
   out_4210516850527502917[77] = 0;
   out_4210516850527502917[78] = 0;
   out_4210516850527502917[79] = 0;
   out_4210516850527502917[80] = 0;
   out_4210516850527502917[81] = 0;
   out_4210516850527502917[82] = 0;
   out_4210516850527502917[83] = 0;
   out_4210516850527502917[84] = 1;
   out_4210516850527502917[85] = dt;
   out_4210516850527502917[86] = 0;
   out_4210516850527502917[87] = 0;
   out_4210516850527502917[88] = 0;
   out_4210516850527502917[89] = 0;
   out_4210516850527502917[90] = 0;
   out_4210516850527502917[91] = 0;
   out_4210516850527502917[92] = 0;
   out_4210516850527502917[93] = 0;
   out_4210516850527502917[94] = 0;
   out_4210516850527502917[95] = 0;
   out_4210516850527502917[96] = 1;
   out_4210516850527502917[97] = 0;
   out_4210516850527502917[98] = 0;
   out_4210516850527502917[99] = 0;
   out_4210516850527502917[100] = 0;
   out_4210516850527502917[101] = 0;
   out_4210516850527502917[102] = 0;
   out_4210516850527502917[103] = 0;
   out_4210516850527502917[104] = 0;
   out_4210516850527502917[105] = 0;
   out_4210516850527502917[106] = 0;
   out_4210516850527502917[107] = 0;
   out_4210516850527502917[108] = 1;
   out_4210516850527502917[109] = 0;
   out_4210516850527502917[110] = 0;
   out_4210516850527502917[111] = 0;
   out_4210516850527502917[112] = 0;
   out_4210516850527502917[113] = 0;
   out_4210516850527502917[114] = 0;
   out_4210516850527502917[115] = 0;
   out_4210516850527502917[116] = 0;
   out_4210516850527502917[117] = 0;
   out_4210516850527502917[118] = 0;
   out_4210516850527502917[119] = 0;
   out_4210516850527502917[120] = 1;
}
void h_6(double *state, double *sat_pos, double *out_441394932428483495) {
   out_441394932428483495[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + state[6];
}
void H_6(double *state, double *sat_pos, double *out_1724092817002163779) {
   out_1724092817002163779[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1724092817002163779[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1724092817002163779[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_1724092817002163779[3] = 0;
   out_1724092817002163779[4] = 0;
   out_1724092817002163779[5] = 0;
   out_1724092817002163779[6] = 1;
   out_1724092817002163779[7] = 0;
   out_1724092817002163779[8] = 0;
   out_1724092817002163779[9] = 0;
   out_1724092817002163779[10] = 0;
}
void h_20(double *state, double *sat_pos, double *out_4020082234097327616) {
   out_4020082234097327616[0] = sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2)) + sat_pos[3]*state[10] + state[6] + state[9];
}
void H_20(double *state, double *sat_pos, double *out_5697814943515096384) {
   out_5697814943515096384[0] = (-sat_pos[0] + state[0])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5697814943515096384[1] = (-sat_pos[1] + state[1])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5697814943515096384[2] = (-sat_pos[2] + state[2])/sqrt(pow(-sat_pos[0] + state[0], 2) + pow(-sat_pos[1] + state[1], 2) + pow(-sat_pos[2] + state[2], 2));
   out_5697814943515096384[3] = 0;
   out_5697814943515096384[4] = 0;
   out_5697814943515096384[5] = 0;
   out_5697814943515096384[6] = 1;
   out_5697814943515096384[7] = 0;
   out_5697814943515096384[8] = 0;
   out_5697814943515096384[9] = 1;
   out_5697814943515096384[10] = sat_pos[3];
}
void h_7(double *state, double *sat_pos_vel, double *out_4615274250774322780) {
   out_4615274250774322780[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_7(double *state, double *sat_pos_vel, double *out_8953292931421286144) {
   out_8953292931421286144[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8953292931421286144[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8953292931421286144[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8953292931421286144[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8953292931421286144[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8953292931421286144[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8953292931421286144[6] = 0;
   out_8953292931421286144[7] = 1;
   out_8953292931421286144[8] = 0;
   out_8953292931421286144[9] = 0;
   out_8953292931421286144[10] = 0;
}
void h_21(double *state, double *sat_pos_vel, double *out_4615274250774322780) {
   out_4615274250774322780[0] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + (sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2)) + state[7];
}
void H_21(double *state, double *sat_pos_vel, double *out_8953292931421286144) {
   out_8953292931421286144[0] = pow(sat_pos_vel[0] - state[0], 2)*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[3] - state[3])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8953292931421286144[1] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[1] - state[1])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[1] - state[1], 2)*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[4] - state[4])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8953292931421286144[2] = (sat_pos_vel[0] - state[0])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[3] - state[3])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + (sat_pos_vel[1] - state[1])*(sat_pos_vel[2] - state[2])*(sat_pos_vel[4] - state[4])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) + pow(sat_pos_vel[2] - state[2], 2)*(sat_pos_vel[5] - state[5])/pow(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2), 3.0/2.0) - (sat_pos_vel[5] - state[5])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8953292931421286144[3] = -(sat_pos_vel[0] - state[0])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8953292931421286144[4] = -(sat_pos_vel[1] - state[1])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8953292931421286144[5] = -(sat_pos_vel[2] - state[2])/sqrt(pow(sat_pos_vel[0] - state[0], 2) + pow(sat_pos_vel[1] - state[1], 2) + pow(sat_pos_vel[2] - state[2], 2));
   out_8953292931421286144[6] = 0;
   out_8953292931421286144[7] = 1;
   out_8953292931421286144[8] = 0;
   out_8953292931421286144[9] = 0;
   out_8953292931421286144[10] = 0;
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

void gnss_update_6(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_6, H_6, NULL, in_z, in_R, in_ea, MAHA_THRESH_6);
}
void gnss_update_20(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_20, H_20, NULL, in_z, in_R, in_ea, MAHA_THRESH_20);
}
void gnss_update_7(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_7, H_7, NULL, in_z, in_R, in_ea, MAHA_THRESH_7);
}
void gnss_update_21(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<1, 3, 0>(in_x, in_P, h_21, H_21, NULL, in_z, in_R, in_ea, MAHA_THRESH_21);
}
void gnss_err_fun(double *nom_x, double *delta_x, double *out_7380922654444343112) {
  err_fun(nom_x, delta_x, out_7380922654444343112);
}
void gnss_inv_err_fun(double *nom_x, double *true_x, double *out_132778549325657714) {
  inv_err_fun(nom_x, true_x, out_132778549325657714);
}
void gnss_H_mod_fun(double *state, double *out_5595204560979213869) {
  H_mod_fun(state, out_5595204560979213869);
}
void gnss_f_fun(double *state, double dt, double *out_398141659558175432) {
  f_fun(state,  dt, out_398141659558175432);
}
void gnss_F_fun(double *state, double dt, double *out_4210516850527502917) {
  F_fun(state,  dt, out_4210516850527502917);
}
void gnss_h_6(double *state, double *sat_pos, double *out_441394932428483495) {
  h_6(state, sat_pos, out_441394932428483495);
}
void gnss_H_6(double *state, double *sat_pos, double *out_1724092817002163779) {
  H_6(state, sat_pos, out_1724092817002163779);
}
void gnss_h_20(double *state, double *sat_pos, double *out_4020082234097327616) {
  h_20(state, sat_pos, out_4020082234097327616);
}
void gnss_H_20(double *state, double *sat_pos, double *out_5697814943515096384) {
  H_20(state, sat_pos, out_5697814943515096384);
}
void gnss_h_7(double *state, double *sat_pos_vel, double *out_4615274250774322780) {
  h_7(state, sat_pos_vel, out_4615274250774322780);
}
void gnss_H_7(double *state, double *sat_pos_vel, double *out_8953292931421286144) {
  H_7(state, sat_pos_vel, out_8953292931421286144);
}
void gnss_h_21(double *state, double *sat_pos_vel, double *out_4615274250774322780) {
  h_21(state, sat_pos_vel, out_4615274250774322780);
}
void gnss_H_21(double *state, double *sat_pos_vel, double *out_8953292931421286144) {
  H_21(state, sat_pos_vel, out_8953292931421286144);
}
void gnss_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF gnss = {
  .name = "gnss",
  .kinds = { 6, 20, 7, 21 },
  .feature_kinds = {  },
  .f_fun = gnss_f_fun,
  .F_fun = gnss_F_fun,
  .err_fun = gnss_err_fun,
  .inv_err_fun = gnss_inv_err_fun,
  .H_mod_fun = gnss_H_mod_fun,
  .predict = gnss_predict,
  .hs = {
    { 6, gnss_h_6 },
    { 20, gnss_h_20 },
    { 7, gnss_h_7 },
    { 21, gnss_h_21 },
  },
  .Hs = {
    { 6, gnss_H_6 },
    { 20, gnss_H_20 },
    { 7, gnss_H_7 },
    { 21, gnss_H_21 },
  },
  .updates = {
    { 6, gnss_update_6 },
    { 20, gnss_update_20 },
    { 7, gnss_update_7 },
    { 21, gnss_update_21 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_init(gnss);
