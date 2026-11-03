#include "pose.h"

namespace {
#define DIM 18
#define EDIM 18
#define MEDIM 18
typedef void (*Hfun)(double *, double *, double *);
const static double MAHA_THRESH_4 = 7.814727903251177;
const static double MAHA_THRESH_10 = 7.814727903251177;
const static double MAHA_THRESH_13 = 7.814727903251177;
const static double MAHA_THRESH_14 = 7.814727903251177;

/******************************************************************************
 *                      Code generated with SymPy 1.14.0                      *
 *                                                                            *
 *              See http://www.sympy.org/ for more information.               *
 *                                                                            *
 *                         This file is part of 'ekf'                         *
 ******************************************************************************/
void err_fun(double *nom_x, double *delta_x, double *out_4830772321857990181) {
   out_4830772321857990181[0] = delta_x[0] + nom_x[0];
   out_4830772321857990181[1] = delta_x[1] + nom_x[1];
   out_4830772321857990181[2] = delta_x[2] + nom_x[2];
   out_4830772321857990181[3] = delta_x[3] + nom_x[3];
   out_4830772321857990181[4] = delta_x[4] + nom_x[4];
   out_4830772321857990181[5] = delta_x[5] + nom_x[5];
   out_4830772321857990181[6] = delta_x[6] + nom_x[6];
   out_4830772321857990181[7] = delta_x[7] + nom_x[7];
   out_4830772321857990181[8] = delta_x[8] + nom_x[8];
   out_4830772321857990181[9] = delta_x[9] + nom_x[9];
   out_4830772321857990181[10] = delta_x[10] + nom_x[10];
   out_4830772321857990181[11] = delta_x[11] + nom_x[11];
   out_4830772321857990181[12] = delta_x[12] + nom_x[12];
   out_4830772321857990181[13] = delta_x[13] + nom_x[13];
   out_4830772321857990181[14] = delta_x[14] + nom_x[14];
   out_4830772321857990181[15] = delta_x[15] + nom_x[15];
   out_4830772321857990181[16] = delta_x[16] + nom_x[16];
   out_4830772321857990181[17] = delta_x[17] + nom_x[17];
}
void inv_err_fun(double *nom_x, double *true_x, double *out_5200017052910787602) {
   out_5200017052910787602[0] = -nom_x[0] + true_x[0];
   out_5200017052910787602[1] = -nom_x[1] + true_x[1];
   out_5200017052910787602[2] = -nom_x[2] + true_x[2];
   out_5200017052910787602[3] = -nom_x[3] + true_x[3];
   out_5200017052910787602[4] = -nom_x[4] + true_x[4];
   out_5200017052910787602[5] = -nom_x[5] + true_x[5];
   out_5200017052910787602[6] = -nom_x[6] + true_x[6];
   out_5200017052910787602[7] = -nom_x[7] + true_x[7];
   out_5200017052910787602[8] = -nom_x[8] + true_x[8];
   out_5200017052910787602[9] = -nom_x[9] + true_x[9];
   out_5200017052910787602[10] = -nom_x[10] + true_x[10];
   out_5200017052910787602[11] = -nom_x[11] + true_x[11];
   out_5200017052910787602[12] = -nom_x[12] + true_x[12];
   out_5200017052910787602[13] = -nom_x[13] + true_x[13];
   out_5200017052910787602[14] = -nom_x[14] + true_x[14];
   out_5200017052910787602[15] = -nom_x[15] + true_x[15];
   out_5200017052910787602[16] = -nom_x[16] + true_x[16];
   out_5200017052910787602[17] = -nom_x[17] + true_x[17];
}
void H_mod_fun(double *state, double *out_1712341249416054868) {
   out_1712341249416054868[0] = 1.0;
   out_1712341249416054868[1] = 0.0;
   out_1712341249416054868[2] = 0.0;
   out_1712341249416054868[3] = 0.0;
   out_1712341249416054868[4] = 0.0;
   out_1712341249416054868[5] = 0.0;
   out_1712341249416054868[6] = 0.0;
   out_1712341249416054868[7] = 0.0;
   out_1712341249416054868[8] = 0.0;
   out_1712341249416054868[9] = 0.0;
   out_1712341249416054868[10] = 0.0;
   out_1712341249416054868[11] = 0.0;
   out_1712341249416054868[12] = 0.0;
   out_1712341249416054868[13] = 0.0;
   out_1712341249416054868[14] = 0.0;
   out_1712341249416054868[15] = 0.0;
   out_1712341249416054868[16] = 0.0;
   out_1712341249416054868[17] = 0.0;
   out_1712341249416054868[18] = 0.0;
   out_1712341249416054868[19] = 1.0;
   out_1712341249416054868[20] = 0.0;
   out_1712341249416054868[21] = 0.0;
   out_1712341249416054868[22] = 0.0;
   out_1712341249416054868[23] = 0.0;
   out_1712341249416054868[24] = 0.0;
   out_1712341249416054868[25] = 0.0;
   out_1712341249416054868[26] = 0.0;
   out_1712341249416054868[27] = 0.0;
   out_1712341249416054868[28] = 0.0;
   out_1712341249416054868[29] = 0.0;
   out_1712341249416054868[30] = 0.0;
   out_1712341249416054868[31] = 0.0;
   out_1712341249416054868[32] = 0.0;
   out_1712341249416054868[33] = 0.0;
   out_1712341249416054868[34] = 0.0;
   out_1712341249416054868[35] = 0.0;
   out_1712341249416054868[36] = 0.0;
   out_1712341249416054868[37] = 0.0;
   out_1712341249416054868[38] = 1.0;
   out_1712341249416054868[39] = 0.0;
   out_1712341249416054868[40] = 0.0;
   out_1712341249416054868[41] = 0.0;
   out_1712341249416054868[42] = 0.0;
   out_1712341249416054868[43] = 0.0;
   out_1712341249416054868[44] = 0.0;
   out_1712341249416054868[45] = 0.0;
   out_1712341249416054868[46] = 0.0;
   out_1712341249416054868[47] = 0.0;
   out_1712341249416054868[48] = 0.0;
   out_1712341249416054868[49] = 0.0;
   out_1712341249416054868[50] = 0.0;
   out_1712341249416054868[51] = 0.0;
   out_1712341249416054868[52] = 0.0;
   out_1712341249416054868[53] = 0.0;
   out_1712341249416054868[54] = 0.0;
   out_1712341249416054868[55] = 0.0;
   out_1712341249416054868[56] = 0.0;
   out_1712341249416054868[57] = 1.0;
   out_1712341249416054868[58] = 0.0;
   out_1712341249416054868[59] = 0.0;
   out_1712341249416054868[60] = 0.0;
   out_1712341249416054868[61] = 0.0;
   out_1712341249416054868[62] = 0.0;
   out_1712341249416054868[63] = 0.0;
   out_1712341249416054868[64] = 0.0;
   out_1712341249416054868[65] = 0.0;
   out_1712341249416054868[66] = 0.0;
   out_1712341249416054868[67] = 0.0;
   out_1712341249416054868[68] = 0.0;
   out_1712341249416054868[69] = 0.0;
   out_1712341249416054868[70] = 0.0;
   out_1712341249416054868[71] = 0.0;
   out_1712341249416054868[72] = 0.0;
   out_1712341249416054868[73] = 0.0;
   out_1712341249416054868[74] = 0.0;
   out_1712341249416054868[75] = 0.0;
   out_1712341249416054868[76] = 1.0;
   out_1712341249416054868[77] = 0.0;
   out_1712341249416054868[78] = 0.0;
   out_1712341249416054868[79] = 0.0;
   out_1712341249416054868[80] = 0.0;
   out_1712341249416054868[81] = 0.0;
   out_1712341249416054868[82] = 0.0;
   out_1712341249416054868[83] = 0.0;
   out_1712341249416054868[84] = 0.0;
   out_1712341249416054868[85] = 0.0;
   out_1712341249416054868[86] = 0.0;
   out_1712341249416054868[87] = 0.0;
   out_1712341249416054868[88] = 0.0;
   out_1712341249416054868[89] = 0.0;
   out_1712341249416054868[90] = 0.0;
   out_1712341249416054868[91] = 0.0;
   out_1712341249416054868[92] = 0.0;
   out_1712341249416054868[93] = 0.0;
   out_1712341249416054868[94] = 0.0;
   out_1712341249416054868[95] = 1.0;
   out_1712341249416054868[96] = 0.0;
   out_1712341249416054868[97] = 0.0;
   out_1712341249416054868[98] = 0.0;
   out_1712341249416054868[99] = 0.0;
   out_1712341249416054868[100] = 0.0;
   out_1712341249416054868[101] = 0.0;
   out_1712341249416054868[102] = 0.0;
   out_1712341249416054868[103] = 0.0;
   out_1712341249416054868[104] = 0.0;
   out_1712341249416054868[105] = 0.0;
   out_1712341249416054868[106] = 0.0;
   out_1712341249416054868[107] = 0.0;
   out_1712341249416054868[108] = 0.0;
   out_1712341249416054868[109] = 0.0;
   out_1712341249416054868[110] = 0.0;
   out_1712341249416054868[111] = 0.0;
   out_1712341249416054868[112] = 0.0;
   out_1712341249416054868[113] = 0.0;
   out_1712341249416054868[114] = 1.0;
   out_1712341249416054868[115] = 0.0;
   out_1712341249416054868[116] = 0.0;
   out_1712341249416054868[117] = 0.0;
   out_1712341249416054868[118] = 0.0;
   out_1712341249416054868[119] = 0.0;
   out_1712341249416054868[120] = 0.0;
   out_1712341249416054868[121] = 0.0;
   out_1712341249416054868[122] = 0.0;
   out_1712341249416054868[123] = 0.0;
   out_1712341249416054868[124] = 0.0;
   out_1712341249416054868[125] = 0.0;
   out_1712341249416054868[126] = 0.0;
   out_1712341249416054868[127] = 0.0;
   out_1712341249416054868[128] = 0.0;
   out_1712341249416054868[129] = 0.0;
   out_1712341249416054868[130] = 0.0;
   out_1712341249416054868[131] = 0.0;
   out_1712341249416054868[132] = 0.0;
   out_1712341249416054868[133] = 1.0;
   out_1712341249416054868[134] = 0.0;
   out_1712341249416054868[135] = 0.0;
   out_1712341249416054868[136] = 0.0;
   out_1712341249416054868[137] = 0.0;
   out_1712341249416054868[138] = 0.0;
   out_1712341249416054868[139] = 0.0;
   out_1712341249416054868[140] = 0.0;
   out_1712341249416054868[141] = 0.0;
   out_1712341249416054868[142] = 0.0;
   out_1712341249416054868[143] = 0.0;
   out_1712341249416054868[144] = 0.0;
   out_1712341249416054868[145] = 0.0;
   out_1712341249416054868[146] = 0.0;
   out_1712341249416054868[147] = 0.0;
   out_1712341249416054868[148] = 0.0;
   out_1712341249416054868[149] = 0.0;
   out_1712341249416054868[150] = 0.0;
   out_1712341249416054868[151] = 0.0;
   out_1712341249416054868[152] = 1.0;
   out_1712341249416054868[153] = 0.0;
   out_1712341249416054868[154] = 0.0;
   out_1712341249416054868[155] = 0.0;
   out_1712341249416054868[156] = 0.0;
   out_1712341249416054868[157] = 0.0;
   out_1712341249416054868[158] = 0.0;
   out_1712341249416054868[159] = 0.0;
   out_1712341249416054868[160] = 0.0;
   out_1712341249416054868[161] = 0.0;
   out_1712341249416054868[162] = 0.0;
   out_1712341249416054868[163] = 0.0;
   out_1712341249416054868[164] = 0.0;
   out_1712341249416054868[165] = 0.0;
   out_1712341249416054868[166] = 0.0;
   out_1712341249416054868[167] = 0.0;
   out_1712341249416054868[168] = 0.0;
   out_1712341249416054868[169] = 0.0;
   out_1712341249416054868[170] = 0.0;
   out_1712341249416054868[171] = 1.0;
   out_1712341249416054868[172] = 0.0;
   out_1712341249416054868[173] = 0.0;
   out_1712341249416054868[174] = 0.0;
   out_1712341249416054868[175] = 0.0;
   out_1712341249416054868[176] = 0.0;
   out_1712341249416054868[177] = 0.0;
   out_1712341249416054868[178] = 0.0;
   out_1712341249416054868[179] = 0.0;
   out_1712341249416054868[180] = 0.0;
   out_1712341249416054868[181] = 0.0;
   out_1712341249416054868[182] = 0.0;
   out_1712341249416054868[183] = 0.0;
   out_1712341249416054868[184] = 0.0;
   out_1712341249416054868[185] = 0.0;
   out_1712341249416054868[186] = 0.0;
   out_1712341249416054868[187] = 0.0;
   out_1712341249416054868[188] = 0.0;
   out_1712341249416054868[189] = 0.0;
   out_1712341249416054868[190] = 1.0;
   out_1712341249416054868[191] = 0.0;
   out_1712341249416054868[192] = 0.0;
   out_1712341249416054868[193] = 0.0;
   out_1712341249416054868[194] = 0.0;
   out_1712341249416054868[195] = 0.0;
   out_1712341249416054868[196] = 0.0;
   out_1712341249416054868[197] = 0.0;
   out_1712341249416054868[198] = 0.0;
   out_1712341249416054868[199] = 0.0;
   out_1712341249416054868[200] = 0.0;
   out_1712341249416054868[201] = 0.0;
   out_1712341249416054868[202] = 0.0;
   out_1712341249416054868[203] = 0.0;
   out_1712341249416054868[204] = 0.0;
   out_1712341249416054868[205] = 0.0;
   out_1712341249416054868[206] = 0.0;
   out_1712341249416054868[207] = 0.0;
   out_1712341249416054868[208] = 0.0;
   out_1712341249416054868[209] = 1.0;
   out_1712341249416054868[210] = 0.0;
   out_1712341249416054868[211] = 0.0;
   out_1712341249416054868[212] = 0.0;
   out_1712341249416054868[213] = 0.0;
   out_1712341249416054868[214] = 0.0;
   out_1712341249416054868[215] = 0.0;
   out_1712341249416054868[216] = 0.0;
   out_1712341249416054868[217] = 0.0;
   out_1712341249416054868[218] = 0.0;
   out_1712341249416054868[219] = 0.0;
   out_1712341249416054868[220] = 0.0;
   out_1712341249416054868[221] = 0.0;
   out_1712341249416054868[222] = 0.0;
   out_1712341249416054868[223] = 0.0;
   out_1712341249416054868[224] = 0.0;
   out_1712341249416054868[225] = 0.0;
   out_1712341249416054868[226] = 0.0;
   out_1712341249416054868[227] = 0.0;
   out_1712341249416054868[228] = 1.0;
   out_1712341249416054868[229] = 0.0;
   out_1712341249416054868[230] = 0.0;
   out_1712341249416054868[231] = 0.0;
   out_1712341249416054868[232] = 0.0;
   out_1712341249416054868[233] = 0.0;
   out_1712341249416054868[234] = 0.0;
   out_1712341249416054868[235] = 0.0;
   out_1712341249416054868[236] = 0.0;
   out_1712341249416054868[237] = 0.0;
   out_1712341249416054868[238] = 0.0;
   out_1712341249416054868[239] = 0.0;
   out_1712341249416054868[240] = 0.0;
   out_1712341249416054868[241] = 0.0;
   out_1712341249416054868[242] = 0.0;
   out_1712341249416054868[243] = 0.0;
   out_1712341249416054868[244] = 0.0;
   out_1712341249416054868[245] = 0.0;
   out_1712341249416054868[246] = 0.0;
   out_1712341249416054868[247] = 1.0;
   out_1712341249416054868[248] = 0.0;
   out_1712341249416054868[249] = 0.0;
   out_1712341249416054868[250] = 0.0;
   out_1712341249416054868[251] = 0.0;
   out_1712341249416054868[252] = 0.0;
   out_1712341249416054868[253] = 0.0;
   out_1712341249416054868[254] = 0.0;
   out_1712341249416054868[255] = 0.0;
   out_1712341249416054868[256] = 0.0;
   out_1712341249416054868[257] = 0.0;
   out_1712341249416054868[258] = 0.0;
   out_1712341249416054868[259] = 0.0;
   out_1712341249416054868[260] = 0.0;
   out_1712341249416054868[261] = 0.0;
   out_1712341249416054868[262] = 0.0;
   out_1712341249416054868[263] = 0.0;
   out_1712341249416054868[264] = 0.0;
   out_1712341249416054868[265] = 0.0;
   out_1712341249416054868[266] = 1.0;
   out_1712341249416054868[267] = 0.0;
   out_1712341249416054868[268] = 0.0;
   out_1712341249416054868[269] = 0.0;
   out_1712341249416054868[270] = 0.0;
   out_1712341249416054868[271] = 0.0;
   out_1712341249416054868[272] = 0.0;
   out_1712341249416054868[273] = 0.0;
   out_1712341249416054868[274] = 0.0;
   out_1712341249416054868[275] = 0.0;
   out_1712341249416054868[276] = 0.0;
   out_1712341249416054868[277] = 0.0;
   out_1712341249416054868[278] = 0.0;
   out_1712341249416054868[279] = 0.0;
   out_1712341249416054868[280] = 0.0;
   out_1712341249416054868[281] = 0.0;
   out_1712341249416054868[282] = 0.0;
   out_1712341249416054868[283] = 0.0;
   out_1712341249416054868[284] = 0.0;
   out_1712341249416054868[285] = 1.0;
   out_1712341249416054868[286] = 0.0;
   out_1712341249416054868[287] = 0.0;
   out_1712341249416054868[288] = 0.0;
   out_1712341249416054868[289] = 0.0;
   out_1712341249416054868[290] = 0.0;
   out_1712341249416054868[291] = 0.0;
   out_1712341249416054868[292] = 0.0;
   out_1712341249416054868[293] = 0.0;
   out_1712341249416054868[294] = 0.0;
   out_1712341249416054868[295] = 0.0;
   out_1712341249416054868[296] = 0.0;
   out_1712341249416054868[297] = 0.0;
   out_1712341249416054868[298] = 0.0;
   out_1712341249416054868[299] = 0.0;
   out_1712341249416054868[300] = 0.0;
   out_1712341249416054868[301] = 0.0;
   out_1712341249416054868[302] = 0.0;
   out_1712341249416054868[303] = 0.0;
   out_1712341249416054868[304] = 1.0;
   out_1712341249416054868[305] = 0.0;
   out_1712341249416054868[306] = 0.0;
   out_1712341249416054868[307] = 0.0;
   out_1712341249416054868[308] = 0.0;
   out_1712341249416054868[309] = 0.0;
   out_1712341249416054868[310] = 0.0;
   out_1712341249416054868[311] = 0.0;
   out_1712341249416054868[312] = 0.0;
   out_1712341249416054868[313] = 0.0;
   out_1712341249416054868[314] = 0.0;
   out_1712341249416054868[315] = 0.0;
   out_1712341249416054868[316] = 0.0;
   out_1712341249416054868[317] = 0.0;
   out_1712341249416054868[318] = 0.0;
   out_1712341249416054868[319] = 0.0;
   out_1712341249416054868[320] = 0.0;
   out_1712341249416054868[321] = 0.0;
   out_1712341249416054868[322] = 0.0;
   out_1712341249416054868[323] = 1.0;
}
void f_fun(double *state, double dt, double *out_3100560613383761185) {
   out_3100560613383761185[0] = atan2((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), -(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]));
   out_3100560613383761185[1] = asin(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]));
   out_3100560613383761185[2] = atan2(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), -(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]));
   out_3100560613383761185[3] = dt*state[12] + state[3];
   out_3100560613383761185[4] = dt*state[13] + state[4];
   out_3100560613383761185[5] = dt*state[14] + state[5];
   out_3100560613383761185[6] = state[6];
   out_3100560613383761185[7] = state[7];
   out_3100560613383761185[8] = state[8];
   out_3100560613383761185[9] = state[9];
   out_3100560613383761185[10] = state[10];
   out_3100560613383761185[11] = state[11];
   out_3100560613383761185[12] = state[12];
   out_3100560613383761185[13] = state[13];
   out_3100560613383761185[14] = state[14];
   out_3100560613383761185[15] = state[15];
   out_3100560613383761185[16] = state[16];
   out_3100560613383761185[17] = state[17];
}
void F_fun(double *state, double dt, double *out_3350602915939425859) {
   out_3350602915939425859[0] = ((-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*cos(state[0])*cos(state[1]) - sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*cos(state[0])*cos(state[1]) - sin(dt*state[6])*sin(state[0])*cos(dt*state[7])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3350602915939425859[1] = ((-sin(dt*state[6])*sin(dt*state[8]) - sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*cos(state[1]) - (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*sin(state[1]) - sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(state[0]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*sin(state[1]) + (-sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) + sin(dt*state[8])*cos(dt*state[6]))*cos(state[1]) - sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(state[0]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3350602915939425859[2] = 0;
   out_3350602915939425859[3] = 0;
   out_3350602915939425859[4] = 0;
   out_3350602915939425859[5] = 0;
   out_3350602915939425859[6] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(dt*cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) - dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3350602915939425859[7] = (-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[6])*sin(dt*state[7])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[6])*sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) - dt*sin(dt*state[6])*sin(state[1])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + (-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))*(-dt*sin(dt*state[7])*cos(dt*state[6])*cos(state[0])*cos(state[1]) + dt*sin(dt*state[8])*sin(state[0])*cos(dt*state[6])*cos(dt*state[7])*cos(state[1]) - dt*sin(state[1])*cos(dt*state[6])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3350602915939425859[8] = ((dt*sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + dt*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (dt*sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]))*(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2)) + ((dt*sin(dt*state[6])*sin(dt*state[8]) + dt*sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (-dt*sin(dt*state[6])*cos(dt*state[8]) + dt*sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]))*(-(sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) + (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) - sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/(pow(-(sin(dt*state[6])*sin(dt*state[8]) + sin(dt*state[7])*cos(dt*state[6])*cos(dt*state[8]))*sin(state[1]) + (-sin(dt*state[6])*cos(dt*state[8]) + sin(dt*state[7])*sin(dt*state[8])*cos(dt*state[6]))*sin(state[0])*cos(state[1]) + cos(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2) + pow((sin(dt*state[6])*sin(dt*state[7])*sin(dt*state[8]) + cos(dt*state[6])*cos(dt*state[8]))*sin(state[0])*cos(state[1]) - (sin(dt*state[6])*sin(dt*state[7])*cos(dt*state[8]) - sin(dt*state[8])*cos(dt*state[6]))*sin(state[1]) + sin(dt*state[6])*cos(dt*state[7])*cos(state[0])*cos(state[1]), 2));
   out_3350602915939425859[9] = 0;
   out_3350602915939425859[10] = 0;
   out_3350602915939425859[11] = 0;
   out_3350602915939425859[12] = 0;
   out_3350602915939425859[13] = 0;
   out_3350602915939425859[14] = 0;
   out_3350602915939425859[15] = 0;
   out_3350602915939425859[16] = 0;
   out_3350602915939425859[17] = 0;
   out_3350602915939425859[18] = (-sin(dt*state[7])*sin(state[0])*cos(state[1]) - sin(dt*state[8])*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3350602915939425859[19] = (-sin(dt*state[7])*sin(state[1])*cos(state[0]) + sin(dt*state[8])*sin(state[0])*sin(state[1])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3350602915939425859[20] = 0;
   out_3350602915939425859[21] = 0;
   out_3350602915939425859[22] = 0;
   out_3350602915939425859[23] = 0;
   out_3350602915939425859[24] = 0;
   out_3350602915939425859[25] = (dt*sin(dt*state[7])*sin(dt*state[8])*sin(state[0])*cos(state[1]) - dt*sin(dt*state[7])*sin(state[1])*cos(dt*state[8]) + dt*cos(dt*state[7])*cos(state[0])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3350602915939425859[26] = (-dt*sin(dt*state[8])*sin(state[1])*cos(dt*state[7]) - dt*sin(state[0])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/sqrt(1 - pow(sin(dt*state[7])*cos(state[0])*cos(state[1]) - sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1]) + sin(state[1])*cos(dt*state[7])*cos(dt*state[8]), 2));
   out_3350602915939425859[27] = 0;
   out_3350602915939425859[28] = 0;
   out_3350602915939425859[29] = 0;
   out_3350602915939425859[30] = 0;
   out_3350602915939425859[31] = 0;
   out_3350602915939425859[32] = 0;
   out_3350602915939425859[33] = 0;
   out_3350602915939425859[34] = 0;
   out_3350602915939425859[35] = 0;
   out_3350602915939425859[36] = ((sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3350602915939425859[37] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-sin(dt*state[7])*sin(state[2])*cos(state[0])*cos(state[1]) + sin(dt*state[8])*sin(state[0])*sin(state[2])*cos(dt*state[7])*cos(state[1]) - sin(state[1])*sin(state[2])*cos(dt*state[7])*cos(dt*state[8]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(-sin(dt*state[7])*cos(state[0])*cos(state[1])*cos(state[2]) + sin(dt*state[8])*sin(state[0])*cos(dt*state[7])*cos(state[1])*cos(state[2]) - sin(state[1])*cos(dt*state[7])*cos(dt*state[8])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3350602915939425859[38] = ((-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (-sin(state[0])*sin(state[1])*sin(state[2]) - cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3350602915939425859[39] = 0;
   out_3350602915939425859[40] = 0;
   out_3350602915939425859[41] = 0;
   out_3350602915939425859[42] = 0;
   out_3350602915939425859[43] = (-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))*(dt*(sin(state[0])*cos(state[2]) - sin(state[1])*sin(state[2])*cos(state[0]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*sin(state[2])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + ((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))*(dt*(-sin(state[0])*sin(state[2]) - sin(state[1])*cos(state[0])*cos(state[2]))*cos(dt*state[7]) - dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[7])*sin(dt*state[8]) - dt*sin(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3350602915939425859[44] = (dt*(sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*sin(state[2])*cos(dt*state[7])*cos(state[1]))*(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2)) + (dt*(sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*cos(dt*state[7])*cos(dt*state[8]) - dt*sin(dt*state[8])*cos(dt*state[7])*cos(state[1])*cos(state[2]))*((-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) - (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) - sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]))/(pow(-(sin(state[0])*sin(state[2]) + sin(state[1])*cos(state[0])*cos(state[2]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*cos(state[2]) - sin(state[2])*cos(state[0]))*sin(dt*state[8])*cos(dt*state[7]) + cos(dt*state[7])*cos(dt*state[8])*cos(state[1])*cos(state[2]), 2) + pow(-(-sin(state[0])*cos(state[2]) + sin(state[1])*sin(state[2])*cos(state[0]))*sin(dt*state[7]) + (sin(state[0])*sin(state[1])*sin(state[2]) + cos(state[0])*cos(state[2]))*sin(dt*state[8])*cos(dt*state[7]) + sin(state[2])*cos(dt*state[7])*cos(dt*state[8])*cos(state[1]), 2));
   out_3350602915939425859[45] = 0;
   out_3350602915939425859[46] = 0;
   out_3350602915939425859[47] = 0;
   out_3350602915939425859[48] = 0;
   out_3350602915939425859[49] = 0;
   out_3350602915939425859[50] = 0;
   out_3350602915939425859[51] = 0;
   out_3350602915939425859[52] = 0;
   out_3350602915939425859[53] = 0;
   out_3350602915939425859[54] = 0;
   out_3350602915939425859[55] = 0;
   out_3350602915939425859[56] = 0;
   out_3350602915939425859[57] = 1;
   out_3350602915939425859[58] = 0;
   out_3350602915939425859[59] = 0;
   out_3350602915939425859[60] = 0;
   out_3350602915939425859[61] = 0;
   out_3350602915939425859[62] = 0;
   out_3350602915939425859[63] = 0;
   out_3350602915939425859[64] = 0;
   out_3350602915939425859[65] = 0;
   out_3350602915939425859[66] = dt;
   out_3350602915939425859[67] = 0;
   out_3350602915939425859[68] = 0;
   out_3350602915939425859[69] = 0;
   out_3350602915939425859[70] = 0;
   out_3350602915939425859[71] = 0;
   out_3350602915939425859[72] = 0;
   out_3350602915939425859[73] = 0;
   out_3350602915939425859[74] = 0;
   out_3350602915939425859[75] = 0;
   out_3350602915939425859[76] = 1;
   out_3350602915939425859[77] = 0;
   out_3350602915939425859[78] = 0;
   out_3350602915939425859[79] = 0;
   out_3350602915939425859[80] = 0;
   out_3350602915939425859[81] = 0;
   out_3350602915939425859[82] = 0;
   out_3350602915939425859[83] = 0;
   out_3350602915939425859[84] = 0;
   out_3350602915939425859[85] = dt;
   out_3350602915939425859[86] = 0;
   out_3350602915939425859[87] = 0;
   out_3350602915939425859[88] = 0;
   out_3350602915939425859[89] = 0;
   out_3350602915939425859[90] = 0;
   out_3350602915939425859[91] = 0;
   out_3350602915939425859[92] = 0;
   out_3350602915939425859[93] = 0;
   out_3350602915939425859[94] = 0;
   out_3350602915939425859[95] = 1;
   out_3350602915939425859[96] = 0;
   out_3350602915939425859[97] = 0;
   out_3350602915939425859[98] = 0;
   out_3350602915939425859[99] = 0;
   out_3350602915939425859[100] = 0;
   out_3350602915939425859[101] = 0;
   out_3350602915939425859[102] = 0;
   out_3350602915939425859[103] = 0;
   out_3350602915939425859[104] = dt;
   out_3350602915939425859[105] = 0;
   out_3350602915939425859[106] = 0;
   out_3350602915939425859[107] = 0;
   out_3350602915939425859[108] = 0;
   out_3350602915939425859[109] = 0;
   out_3350602915939425859[110] = 0;
   out_3350602915939425859[111] = 0;
   out_3350602915939425859[112] = 0;
   out_3350602915939425859[113] = 0;
   out_3350602915939425859[114] = 1;
   out_3350602915939425859[115] = 0;
   out_3350602915939425859[116] = 0;
   out_3350602915939425859[117] = 0;
   out_3350602915939425859[118] = 0;
   out_3350602915939425859[119] = 0;
   out_3350602915939425859[120] = 0;
   out_3350602915939425859[121] = 0;
   out_3350602915939425859[122] = 0;
   out_3350602915939425859[123] = 0;
   out_3350602915939425859[124] = 0;
   out_3350602915939425859[125] = 0;
   out_3350602915939425859[126] = 0;
   out_3350602915939425859[127] = 0;
   out_3350602915939425859[128] = 0;
   out_3350602915939425859[129] = 0;
   out_3350602915939425859[130] = 0;
   out_3350602915939425859[131] = 0;
   out_3350602915939425859[132] = 0;
   out_3350602915939425859[133] = 1;
   out_3350602915939425859[134] = 0;
   out_3350602915939425859[135] = 0;
   out_3350602915939425859[136] = 0;
   out_3350602915939425859[137] = 0;
   out_3350602915939425859[138] = 0;
   out_3350602915939425859[139] = 0;
   out_3350602915939425859[140] = 0;
   out_3350602915939425859[141] = 0;
   out_3350602915939425859[142] = 0;
   out_3350602915939425859[143] = 0;
   out_3350602915939425859[144] = 0;
   out_3350602915939425859[145] = 0;
   out_3350602915939425859[146] = 0;
   out_3350602915939425859[147] = 0;
   out_3350602915939425859[148] = 0;
   out_3350602915939425859[149] = 0;
   out_3350602915939425859[150] = 0;
   out_3350602915939425859[151] = 0;
   out_3350602915939425859[152] = 1;
   out_3350602915939425859[153] = 0;
   out_3350602915939425859[154] = 0;
   out_3350602915939425859[155] = 0;
   out_3350602915939425859[156] = 0;
   out_3350602915939425859[157] = 0;
   out_3350602915939425859[158] = 0;
   out_3350602915939425859[159] = 0;
   out_3350602915939425859[160] = 0;
   out_3350602915939425859[161] = 0;
   out_3350602915939425859[162] = 0;
   out_3350602915939425859[163] = 0;
   out_3350602915939425859[164] = 0;
   out_3350602915939425859[165] = 0;
   out_3350602915939425859[166] = 0;
   out_3350602915939425859[167] = 0;
   out_3350602915939425859[168] = 0;
   out_3350602915939425859[169] = 0;
   out_3350602915939425859[170] = 0;
   out_3350602915939425859[171] = 1;
   out_3350602915939425859[172] = 0;
   out_3350602915939425859[173] = 0;
   out_3350602915939425859[174] = 0;
   out_3350602915939425859[175] = 0;
   out_3350602915939425859[176] = 0;
   out_3350602915939425859[177] = 0;
   out_3350602915939425859[178] = 0;
   out_3350602915939425859[179] = 0;
   out_3350602915939425859[180] = 0;
   out_3350602915939425859[181] = 0;
   out_3350602915939425859[182] = 0;
   out_3350602915939425859[183] = 0;
   out_3350602915939425859[184] = 0;
   out_3350602915939425859[185] = 0;
   out_3350602915939425859[186] = 0;
   out_3350602915939425859[187] = 0;
   out_3350602915939425859[188] = 0;
   out_3350602915939425859[189] = 0;
   out_3350602915939425859[190] = 1;
   out_3350602915939425859[191] = 0;
   out_3350602915939425859[192] = 0;
   out_3350602915939425859[193] = 0;
   out_3350602915939425859[194] = 0;
   out_3350602915939425859[195] = 0;
   out_3350602915939425859[196] = 0;
   out_3350602915939425859[197] = 0;
   out_3350602915939425859[198] = 0;
   out_3350602915939425859[199] = 0;
   out_3350602915939425859[200] = 0;
   out_3350602915939425859[201] = 0;
   out_3350602915939425859[202] = 0;
   out_3350602915939425859[203] = 0;
   out_3350602915939425859[204] = 0;
   out_3350602915939425859[205] = 0;
   out_3350602915939425859[206] = 0;
   out_3350602915939425859[207] = 0;
   out_3350602915939425859[208] = 0;
   out_3350602915939425859[209] = 1;
   out_3350602915939425859[210] = 0;
   out_3350602915939425859[211] = 0;
   out_3350602915939425859[212] = 0;
   out_3350602915939425859[213] = 0;
   out_3350602915939425859[214] = 0;
   out_3350602915939425859[215] = 0;
   out_3350602915939425859[216] = 0;
   out_3350602915939425859[217] = 0;
   out_3350602915939425859[218] = 0;
   out_3350602915939425859[219] = 0;
   out_3350602915939425859[220] = 0;
   out_3350602915939425859[221] = 0;
   out_3350602915939425859[222] = 0;
   out_3350602915939425859[223] = 0;
   out_3350602915939425859[224] = 0;
   out_3350602915939425859[225] = 0;
   out_3350602915939425859[226] = 0;
   out_3350602915939425859[227] = 0;
   out_3350602915939425859[228] = 1;
   out_3350602915939425859[229] = 0;
   out_3350602915939425859[230] = 0;
   out_3350602915939425859[231] = 0;
   out_3350602915939425859[232] = 0;
   out_3350602915939425859[233] = 0;
   out_3350602915939425859[234] = 0;
   out_3350602915939425859[235] = 0;
   out_3350602915939425859[236] = 0;
   out_3350602915939425859[237] = 0;
   out_3350602915939425859[238] = 0;
   out_3350602915939425859[239] = 0;
   out_3350602915939425859[240] = 0;
   out_3350602915939425859[241] = 0;
   out_3350602915939425859[242] = 0;
   out_3350602915939425859[243] = 0;
   out_3350602915939425859[244] = 0;
   out_3350602915939425859[245] = 0;
   out_3350602915939425859[246] = 0;
   out_3350602915939425859[247] = 1;
   out_3350602915939425859[248] = 0;
   out_3350602915939425859[249] = 0;
   out_3350602915939425859[250] = 0;
   out_3350602915939425859[251] = 0;
   out_3350602915939425859[252] = 0;
   out_3350602915939425859[253] = 0;
   out_3350602915939425859[254] = 0;
   out_3350602915939425859[255] = 0;
   out_3350602915939425859[256] = 0;
   out_3350602915939425859[257] = 0;
   out_3350602915939425859[258] = 0;
   out_3350602915939425859[259] = 0;
   out_3350602915939425859[260] = 0;
   out_3350602915939425859[261] = 0;
   out_3350602915939425859[262] = 0;
   out_3350602915939425859[263] = 0;
   out_3350602915939425859[264] = 0;
   out_3350602915939425859[265] = 0;
   out_3350602915939425859[266] = 1;
   out_3350602915939425859[267] = 0;
   out_3350602915939425859[268] = 0;
   out_3350602915939425859[269] = 0;
   out_3350602915939425859[270] = 0;
   out_3350602915939425859[271] = 0;
   out_3350602915939425859[272] = 0;
   out_3350602915939425859[273] = 0;
   out_3350602915939425859[274] = 0;
   out_3350602915939425859[275] = 0;
   out_3350602915939425859[276] = 0;
   out_3350602915939425859[277] = 0;
   out_3350602915939425859[278] = 0;
   out_3350602915939425859[279] = 0;
   out_3350602915939425859[280] = 0;
   out_3350602915939425859[281] = 0;
   out_3350602915939425859[282] = 0;
   out_3350602915939425859[283] = 0;
   out_3350602915939425859[284] = 0;
   out_3350602915939425859[285] = 1;
   out_3350602915939425859[286] = 0;
   out_3350602915939425859[287] = 0;
   out_3350602915939425859[288] = 0;
   out_3350602915939425859[289] = 0;
   out_3350602915939425859[290] = 0;
   out_3350602915939425859[291] = 0;
   out_3350602915939425859[292] = 0;
   out_3350602915939425859[293] = 0;
   out_3350602915939425859[294] = 0;
   out_3350602915939425859[295] = 0;
   out_3350602915939425859[296] = 0;
   out_3350602915939425859[297] = 0;
   out_3350602915939425859[298] = 0;
   out_3350602915939425859[299] = 0;
   out_3350602915939425859[300] = 0;
   out_3350602915939425859[301] = 0;
   out_3350602915939425859[302] = 0;
   out_3350602915939425859[303] = 0;
   out_3350602915939425859[304] = 1;
   out_3350602915939425859[305] = 0;
   out_3350602915939425859[306] = 0;
   out_3350602915939425859[307] = 0;
   out_3350602915939425859[308] = 0;
   out_3350602915939425859[309] = 0;
   out_3350602915939425859[310] = 0;
   out_3350602915939425859[311] = 0;
   out_3350602915939425859[312] = 0;
   out_3350602915939425859[313] = 0;
   out_3350602915939425859[314] = 0;
   out_3350602915939425859[315] = 0;
   out_3350602915939425859[316] = 0;
   out_3350602915939425859[317] = 0;
   out_3350602915939425859[318] = 0;
   out_3350602915939425859[319] = 0;
   out_3350602915939425859[320] = 0;
   out_3350602915939425859[321] = 0;
   out_3350602915939425859[322] = 0;
   out_3350602915939425859[323] = 1;
}
void h_4(double *state, double *unused, double *out_2074712223981570498) {
   out_2074712223981570498[0] = state[6] + state[9];
   out_2074712223981570498[1] = state[7] + state[10];
   out_2074712223981570498[2] = state[8] + state[11];
}
void H_4(double *state, double *unused, double *out_4135740557335903853) {
   out_4135740557335903853[0] = 0;
   out_4135740557335903853[1] = 0;
   out_4135740557335903853[2] = 0;
   out_4135740557335903853[3] = 0;
   out_4135740557335903853[4] = 0;
   out_4135740557335903853[5] = 0;
   out_4135740557335903853[6] = 1;
   out_4135740557335903853[7] = 0;
   out_4135740557335903853[8] = 0;
   out_4135740557335903853[9] = 1;
   out_4135740557335903853[10] = 0;
   out_4135740557335903853[11] = 0;
   out_4135740557335903853[12] = 0;
   out_4135740557335903853[13] = 0;
   out_4135740557335903853[14] = 0;
   out_4135740557335903853[15] = 0;
   out_4135740557335903853[16] = 0;
   out_4135740557335903853[17] = 0;
   out_4135740557335903853[18] = 0;
   out_4135740557335903853[19] = 0;
   out_4135740557335903853[20] = 0;
   out_4135740557335903853[21] = 0;
   out_4135740557335903853[22] = 0;
   out_4135740557335903853[23] = 0;
   out_4135740557335903853[24] = 0;
   out_4135740557335903853[25] = 1;
   out_4135740557335903853[26] = 0;
   out_4135740557335903853[27] = 0;
   out_4135740557335903853[28] = 1;
   out_4135740557335903853[29] = 0;
   out_4135740557335903853[30] = 0;
   out_4135740557335903853[31] = 0;
   out_4135740557335903853[32] = 0;
   out_4135740557335903853[33] = 0;
   out_4135740557335903853[34] = 0;
   out_4135740557335903853[35] = 0;
   out_4135740557335903853[36] = 0;
   out_4135740557335903853[37] = 0;
   out_4135740557335903853[38] = 0;
   out_4135740557335903853[39] = 0;
   out_4135740557335903853[40] = 0;
   out_4135740557335903853[41] = 0;
   out_4135740557335903853[42] = 0;
   out_4135740557335903853[43] = 0;
   out_4135740557335903853[44] = 1;
   out_4135740557335903853[45] = 0;
   out_4135740557335903853[46] = 0;
   out_4135740557335903853[47] = 1;
   out_4135740557335903853[48] = 0;
   out_4135740557335903853[49] = 0;
   out_4135740557335903853[50] = 0;
   out_4135740557335903853[51] = 0;
   out_4135740557335903853[52] = 0;
   out_4135740557335903853[53] = 0;
}
void h_10(double *state, double *unused, double *out_5949398724887603410) {
   out_5949398724887603410[0] = 9.8100000000000005*sin(state[1]) - state[4]*state[8] + state[5]*state[7] + state[12] + state[15];
   out_5949398724887603410[1] = -9.8100000000000005*sin(state[0])*cos(state[1]) + state[3]*state[8] - state[5]*state[6] + state[13] + state[16];
   out_5949398724887603410[2] = -9.8100000000000005*cos(state[0])*cos(state[1]) - state[3]*state[7] + state[4]*state[6] + state[14] + state[17];
}
void H_10(double *state, double *unused, double *out_3347508424845693314) {
   out_3347508424845693314[0] = 0;
   out_3347508424845693314[1] = 9.8100000000000005*cos(state[1]);
   out_3347508424845693314[2] = 0;
   out_3347508424845693314[3] = 0;
   out_3347508424845693314[4] = -state[8];
   out_3347508424845693314[5] = state[7];
   out_3347508424845693314[6] = 0;
   out_3347508424845693314[7] = state[5];
   out_3347508424845693314[8] = -state[4];
   out_3347508424845693314[9] = 0;
   out_3347508424845693314[10] = 0;
   out_3347508424845693314[11] = 0;
   out_3347508424845693314[12] = 1;
   out_3347508424845693314[13] = 0;
   out_3347508424845693314[14] = 0;
   out_3347508424845693314[15] = 1;
   out_3347508424845693314[16] = 0;
   out_3347508424845693314[17] = 0;
   out_3347508424845693314[18] = -9.8100000000000005*cos(state[0])*cos(state[1]);
   out_3347508424845693314[19] = 9.8100000000000005*sin(state[0])*sin(state[1]);
   out_3347508424845693314[20] = 0;
   out_3347508424845693314[21] = state[8];
   out_3347508424845693314[22] = 0;
   out_3347508424845693314[23] = -state[6];
   out_3347508424845693314[24] = -state[5];
   out_3347508424845693314[25] = 0;
   out_3347508424845693314[26] = state[3];
   out_3347508424845693314[27] = 0;
   out_3347508424845693314[28] = 0;
   out_3347508424845693314[29] = 0;
   out_3347508424845693314[30] = 0;
   out_3347508424845693314[31] = 1;
   out_3347508424845693314[32] = 0;
   out_3347508424845693314[33] = 0;
   out_3347508424845693314[34] = 1;
   out_3347508424845693314[35] = 0;
   out_3347508424845693314[36] = 9.8100000000000005*sin(state[0])*cos(state[1]);
   out_3347508424845693314[37] = 9.8100000000000005*sin(state[1])*cos(state[0]);
   out_3347508424845693314[38] = 0;
   out_3347508424845693314[39] = -state[7];
   out_3347508424845693314[40] = state[6];
   out_3347508424845693314[41] = 0;
   out_3347508424845693314[42] = state[4];
   out_3347508424845693314[43] = -state[3];
   out_3347508424845693314[44] = 0;
   out_3347508424845693314[45] = 0;
   out_3347508424845693314[46] = 0;
   out_3347508424845693314[47] = 0;
   out_3347508424845693314[48] = 0;
   out_3347508424845693314[49] = 0;
   out_3347508424845693314[50] = 1;
   out_3347508424845693314[51] = 0;
   out_3347508424845693314[52] = 0;
   out_3347508424845693314[53] = 1;
}
void h_13(double *state, double *unused, double *out_9220727019356339608) {
   out_9220727019356339608[0] = state[3];
   out_9220727019356339608[1] = state[4];
   out_9220727019356339608[2] = state[5];
}
void H_13(double *state, double *unused, double *out_923466732003571052) {
   out_923466732003571052[0] = 0;
   out_923466732003571052[1] = 0;
   out_923466732003571052[2] = 0;
   out_923466732003571052[3] = 1;
   out_923466732003571052[4] = 0;
   out_923466732003571052[5] = 0;
   out_923466732003571052[6] = 0;
   out_923466732003571052[7] = 0;
   out_923466732003571052[8] = 0;
   out_923466732003571052[9] = 0;
   out_923466732003571052[10] = 0;
   out_923466732003571052[11] = 0;
   out_923466732003571052[12] = 0;
   out_923466732003571052[13] = 0;
   out_923466732003571052[14] = 0;
   out_923466732003571052[15] = 0;
   out_923466732003571052[16] = 0;
   out_923466732003571052[17] = 0;
   out_923466732003571052[18] = 0;
   out_923466732003571052[19] = 0;
   out_923466732003571052[20] = 0;
   out_923466732003571052[21] = 0;
   out_923466732003571052[22] = 1;
   out_923466732003571052[23] = 0;
   out_923466732003571052[24] = 0;
   out_923466732003571052[25] = 0;
   out_923466732003571052[26] = 0;
   out_923466732003571052[27] = 0;
   out_923466732003571052[28] = 0;
   out_923466732003571052[29] = 0;
   out_923466732003571052[30] = 0;
   out_923466732003571052[31] = 0;
   out_923466732003571052[32] = 0;
   out_923466732003571052[33] = 0;
   out_923466732003571052[34] = 0;
   out_923466732003571052[35] = 0;
   out_923466732003571052[36] = 0;
   out_923466732003571052[37] = 0;
   out_923466732003571052[38] = 0;
   out_923466732003571052[39] = 0;
   out_923466732003571052[40] = 0;
   out_923466732003571052[41] = 1;
   out_923466732003571052[42] = 0;
   out_923466732003571052[43] = 0;
   out_923466732003571052[44] = 0;
   out_923466732003571052[45] = 0;
   out_923466732003571052[46] = 0;
   out_923466732003571052[47] = 0;
   out_923466732003571052[48] = 0;
   out_923466732003571052[49] = 0;
   out_923466732003571052[50] = 0;
   out_923466732003571052[51] = 0;
   out_923466732003571052[52] = 0;
   out_923466732003571052[53] = 0;
}
void h_14(double *state, double *unused, double *out_8529780658774629922) {
   out_8529780658774629922[0] = state[6];
   out_8529780658774629922[1] = state[7];
   out_8529780658774629922[2] = state[8];
}
void H_14(double *state, double *unused, double *out_172499700996419324) {
   out_172499700996419324[0] = 0;
   out_172499700996419324[1] = 0;
   out_172499700996419324[2] = 0;
   out_172499700996419324[3] = 0;
   out_172499700996419324[4] = 0;
   out_172499700996419324[5] = 0;
   out_172499700996419324[6] = 1;
   out_172499700996419324[7] = 0;
   out_172499700996419324[8] = 0;
   out_172499700996419324[9] = 0;
   out_172499700996419324[10] = 0;
   out_172499700996419324[11] = 0;
   out_172499700996419324[12] = 0;
   out_172499700996419324[13] = 0;
   out_172499700996419324[14] = 0;
   out_172499700996419324[15] = 0;
   out_172499700996419324[16] = 0;
   out_172499700996419324[17] = 0;
   out_172499700996419324[18] = 0;
   out_172499700996419324[19] = 0;
   out_172499700996419324[20] = 0;
   out_172499700996419324[21] = 0;
   out_172499700996419324[22] = 0;
   out_172499700996419324[23] = 0;
   out_172499700996419324[24] = 0;
   out_172499700996419324[25] = 1;
   out_172499700996419324[26] = 0;
   out_172499700996419324[27] = 0;
   out_172499700996419324[28] = 0;
   out_172499700996419324[29] = 0;
   out_172499700996419324[30] = 0;
   out_172499700996419324[31] = 0;
   out_172499700996419324[32] = 0;
   out_172499700996419324[33] = 0;
   out_172499700996419324[34] = 0;
   out_172499700996419324[35] = 0;
   out_172499700996419324[36] = 0;
   out_172499700996419324[37] = 0;
   out_172499700996419324[38] = 0;
   out_172499700996419324[39] = 0;
   out_172499700996419324[40] = 0;
   out_172499700996419324[41] = 0;
   out_172499700996419324[42] = 0;
   out_172499700996419324[43] = 0;
   out_172499700996419324[44] = 1;
   out_172499700996419324[45] = 0;
   out_172499700996419324[46] = 0;
   out_172499700996419324[47] = 0;
   out_172499700996419324[48] = 0;
   out_172499700996419324[49] = 0;
   out_172499700996419324[50] = 0;
   out_172499700996419324[51] = 0;
   out_172499700996419324[52] = 0;
   out_172499700996419324[53] = 0;
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

void pose_update_4(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_4, H_4, NULL, in_z, in_R, in_ea, MAHA_THRESH_4);
}
void pose_update_10(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_10, H_10, NULL, in_z, in_R, in_ea, MAHA_THRESH_10);
}
void pose_update_13(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_13, H_13, NULL, in_z, in_R, in_ea, MAHA_THRESH_13);
}
void pose_update_14(double *in_x, double *in_P, double *in_z, double *in_R, double *in_ea) {
  update<3, 3, 0>(in_x, in_P, h_14, H_14, NULL, in_z, in_R, in_ea, MAHA_THRESH_14);
}
void pose_err_fun(double *nom_x, double *delta_x, double *out_4830772321857990181) {
  err_fun(nom_x, delta_x, out_4830772321857990181);
}
void pose_inv_err_fun(double *nom_x, double *true_x, double *out_5200017052910787602) {
  inv_err_fun(nom_x, true_x, out_5200017052910787602);
}
void pose_H_mod_fun(double *state, double *out_1712341249416054868) {
  H_mod_fun(state, out_1712341249416054868);
}
void pose_f_fun(double *state, double dt, double *out_3100560613383761185) {
  f_fun(state,  dt, out_3100560613383761185);
}
void pose_F_fun(double *state, double dt, double *out_3350602915939425859) {
  F_fun(state,  dt, out_3350602915939425859);
}
void pose_h_4(double *state, double *unused, double *out_2074712223981570498) {
  h_4(state, unused, out_2074712223981570498);
}
void pose_H_4(double *state, double *unused, double *out_4135740557335903853) {
  H_4(state, unused, out_4135740557335903853);
}
void pose_h_10(double *state, double *unused, double *out_5949398724887603410) {
  h_10(state, unused, out_5949398724887603410);
}
void pose_H_10(double *state, double *unused, double *out_3347508424845693314) {
  H_10(state, unused, out_3347508424845693314);
}
void pose_h_13(double *state, double *unused, double *out_9220727019356339608) {
  h_13(state, unused, out_9220727019356339608);
}
void pose_H_13(double *state, double *unused, double *out_923466732003571052) {
  H_13(state, unused, out_923466732003571052);
}
void pose_h_14(double *state, double *unused, double *out_8529780658774629922) {
  h_14(state, unused, out_8529780658774629922);
}
void pose_H_14(double *state, double *unused, double *out_172499700996419324) {
  H_14(state, unused, out_172499700996419324);
}
void pose_predict(double *in_x, double *in_P, double *in_Q, double dt) {
  predict(in_x, in_P, in_Q, dt);
}
}

const EKF pose = {
  .name = "pose",
  .kinds = { 4, 10, 13, 14 },
  .feature_kinds = {  },
  .f_fun = pose_f_fun,
  .F_fun = pose_F_fun,
  .err_fun = pose_err_fun,
  .inv_err_fun = pose_inv_err_fun,
  .H_mod_fun = pose_H_mod_fun,
  .predict = pose_predict,
  .hs = {
    { 4, pose_h_4 },
    { 10, pose_h_10 },
    { 13, pose_h_13 },
    { 14, pose_h_14 },
  },
  .Hs = {
    { 4, pose_H_4 },
    { 10, pose_H_10 },
    { 13, pose_H_13 },
    { 14, pose_H_14 },
  },
  .updates = {
    { 4, pose_update_4 },
    { 10, pose_update_10 },
    { 13, pose_update_13 },
    { 14, pose_update_14 },
  },
  .Hes = {
  },
  .sets = {
  },
  .extra_routines = {
  },
};

ekf_lib_init(pose)
