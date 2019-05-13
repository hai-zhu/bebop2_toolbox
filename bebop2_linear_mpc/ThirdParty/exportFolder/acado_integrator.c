/*
 *    This file was auto-generated using the ACADO Toolkit.
 *    
 *    While ACADO Toolkit is free software released under the terms of
 *    the GNU Lesser General Public License (LGPL), the generated code
 *    as such remains the property of the user who used ACADO Toolkit
 *    to generate this code. In particular, user dependent data of the code
 *    do not inherit the GNU LGPL license. On the other hand, parts of the
 *    generated code that are a direct copy of source code from the
 *    ACADO Toolkit or the software tools it is based on, remain, as derived
 *    work, automatically covered by the LGPL license.
 *    
 *    ACADO Toolkit is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 *    
 */


#include "acado_common.h"


void acado_rhs(const real_t* in, real_t* out)
{
const real_t* xd = in;
const real_t* u = in + 12;

/* Compute outputs: */
out[0] = (((xd[0]+((real_t)(6.6259444564401998e-02)*xd[3]))+((real_t)(9.3972469013199993e-03)*xd[8]))+((real_t)(1.4848518183130000e-03)*u[1]));
out[1] = (((xd[1]+((real_t)(6.6222649190691998e-02)*xd[4]))+((real_t)(9.3895335295179991e-03)*xd[7]))+((real_t)(1.4864685224970000e-03)*u[0]));
out[2] = (((xd[2]+((real_t)(1.0672486585754219e+00)*xd[5]))-((real_t)(1.0049332724894500e-01)*xd[6]))+((real_t)(1.9197303582300001e-03)*u[2]));
out[3] = ((((real_t)(9.8780826220014994e-01)*xd[3])+((real_t)(2.6174312862277099e-01)*xd[8]))+((real_t)(1.2882065558000000e-01)*u[1]));
out[4] = ((((real_t)(9.8670911430742803e-01)*xd[4])-((real_t)(2.6141551330886598e-01)*xd[7]))-((real_t)(1.2893425755999999e-01)*u[0]));
out[5] = ((((real_t)(8.1484431444008099e-01)*xd[5])-((real_t)(1.2754246439467801e-01)*xd[6]))+((real_t)(3.4798765237579999e-03)*u[2]));
out[6] = ((((real_t)(4.8726223064037300e-01)*xd[5])+((real_t)(6.3713514738349597e-01)*xd[6]))-((real_t)(3.5522579058221003e-02)*u[2]));
out[7] = (((real_t)(6.3750488876158695e-01)*xd[7])+((real_t)(3.6855825785903201e-01)*u[0]));
out[8] = (((real_t)(6.3840653268265402e-01)*xd[8])+((real_t)(3.6817194703901501e-01)*u[1]));
out[9] = ((xd[9]+((real_t)(5.8094630551531801e-01)*xd[10]))-((real_t)(2.3800609512830001e-03)*xd[11]));
out[10] = ((((real_t)(9.3646534939894399e-01)*xd[10])-((real_t)(4.6804990378125001e-02)*xd[11]))+((real_t)(5.1259007219671144e-04)*u[3]));
out[11] = ((((real_t)(2.3691436602558699e-01)*xd[10])+((real_t)(8.2032901524311996e-01)*xd[11]))-((real_t)(1.3647220727940000e-02)*u[3]));
}



void acado_diffs(const real_t* in, real_t* out)
{
const real_t* xd = in;

/* Compute outputs: */
out[0] = (real_t)(1.0000000000000000e+00);
out[1] = (real_t)(0.0000000000000000e+00);
out[2] = (real_t)(0.0000000000000000e+00);
out[3] = (real_t)(6.6259444564401998e-02);
out[4] = (real_t)(0.0000000000000000e+00);
out[5] = (real_t)(0.0000000000000000e+00);
out[6] = (real_t)(0.0000000000000000e+00);
out[7] = (real_t)(0.0000000000000000e+00);
out[8] = (real_t)(9.3972469013199993e-03);
out[9] = (real_t)(0.0000000000000000e+00);
out[10] = (real_t)(0.0000000000000000e+00);
out[11] = (real_t)(0.0000000000000000e+00);
out[12] = (real_t)(0.0000000000000000e+00);
out[13] = (real_t)(1.4848518183130000e-03);
out[14] = (real_t)(0.0000000000000000e+00);
out[15] = (real_t)(0.0000000000000000e+00);
out[16] = (real_t)(0.0000000000000000e+00);
out[17] = (real_t)(1.0000000000000000e+00);
out[18] = (real_t)(0.0000000000000000e+00);
out[19] = (real_t)(0.0000000000000000e+00);
out[20] = (real_t)(6.6222649190691998e-02);
out[21] = (real_t)(0.0000000000000000e+00);
out[22] = (real_t)(0.0000000000000000e+00);
out[23] = (real_t)(9.3895335295179991e-03);
out[24] = (real_t)(0.0000000000000000e+00);
out[25] = (real_t)(0.0000000000000000e+00);
out[26] = (real_t)(0.0000000000000000e+00);
out[27] = (real_t)(0.0000000000000000e+00);
out[28] = (real_t)(1.4864685224970000e-03);
out[29] = (real_t)(0.0000000000000000e+00);
out[30] = (real_t)(0.0000000000000000e+00);
out[31] = (real_t)(0.0000000000000000e+00);
out[32] = (real_t)(0.0000000000000000e+00);
out[33] = (real_t)(0.0000000000000000e+00);
out[34] = (real_t)(1.0000000000000000e+00);
out[35] = (real_t)(0.0000000000000000e+00);
out[36] = (real_t)(0.0000000000000000e+00);
out[37] = (real_t)(1.0672486585754219e+00);
out[38] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.0049332724894500e-01));
out[39] = (real_t)(0.0000000000000000e+00);
out[40] = (real_t)(0.0000000000000000e+00);
out[41] = (real_t)(0.0000000000000000e+00);
out[42] = (real_t)(0.0000000000000000e+00);
out[43] = (real_t)(0.0000000000000000e+00);
out[44] = (real_t)(0.0000000000000000e+00);
out[45] = (real_t)(0.0000000000000000e+00);
out[46] = (real_t)(1.9197303582300001e-03);
out[47] = (real_t)(0.0000000000000000e+00);
out[48] = (real_t)(0.0000000000000000e+00);
out[49] = (real_t)(0.0000000000000000e+00);
out[50] = (real_t)(0.0000000000000000e+00);
out[51] = (real_t)(9.8780826220014994e-01);
out[52] = (real_t)(0.0000000000000000e+00);
out[53] = (real_t)(0.0000000000000000e+00);
out[54] = (real_t)(0.0000000000000000e+00);
out[55] = (real_t)(0.0000000000000000e+00);
out[56] = (real_t)(2.6174312862277099e-01);
out[57] = (real_t)(0.0000000000000000e+00);
out[58] = (real_t)(0.0000000000000000e+00);
out[59] = (real_t)(0.0000000000000000e+00);
out[60] = (real_t)(0.0000000000000000e+00);
out[61] = (real_t)(1.2882065558000000e-01);
out[62] = (real_t)(0.0000000000000000e+00);
out[63] = (real_t)(0.0000000000000000e+00);
out[64] = (real_t)(0.0000000000000000e+00);
out[65] = (real_t)(0.0000000000000000e+00);
out[66] = (real_t)(0.0000000000000000e+00);
out[67] = (real_t)(0.0000000000000000e+00);
out[68] = (real_t)(9.8670911430742803e-01);
out[69] = (real_t)(0.0000000000000000e+00);
out[70] = (real_t)(0.0000000000000000e+00);
out[71] = ((real_t)(0.0000000000000000e+00)-(real_t)(2.6141551330886598e-01));
out[72] = (real_t)(0.0000000000000000e+00);
out[73] = (real_t)(0.0000000000000000e+00);
out[74] = (real_t)(0.0000000000000000e+00);
out[75] = (real_t)(0.0000000000000000e+00);
out[76] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.2893425755999999e-01));
out[77] = (real_t)(0.0000000000000000e+00);
out[78] = (real_t)(0.0000000000000000e+00);
out[79] = (real_t)(0.0000000000000000e+00);
out[80] = (real_t)(0.0000000000000000e+00);
out[81] = (real_t)(0.0000000000000000e+00);
out[82] = (real_t)(0.0000000000000000e+00);
out[83] = (real_t)(0.0000000000000000e+00);
out[84] = (real_t)(0.0000000000000000e+00);
out[85] = (real_t)(8.1484431444008099e-01);
out[86] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.2754246439467801e-01));
out[87] = (real_t)(0.0000000000000000e+00);
out[88] = (real_t)(0.0000000000000000e+00);
out[89] = (real_t)(0.0000000000000000e+00);
out[90] = (real_t)(0.0000000000000000e+00);
out[91] = (real_t)(0.0000000000000000e+00);
out[92] = (real_t)(0.0000000000000000e+00);
out[93] = (real_t)(0.0000000000000000e+00);
out[94] = (real_t)(3.4798765237579999e-03);
out[95] = (real_t)(0.0000000000000000e+00);
out[96] = (real_t)(0.0000000000000000e+00);
out[97] = (real_t)(0.0000000000000000e+00);
out[98] = (real_t)(0.0000000000000000e+00);
out[99] = (real_t)(0.0000000000000000e+00);
out[100] = (real_t)(0.0000000000000000e+00);
out[101] = (real_t)(4.8726223064037300e-01);
out[102] = (real_t)(6.3713514738349597e-01);
out[103] = (real_t)(0.0000000000000000e+00);
out[104] = (real_t)(0.0000000000000000e+00);
out[105] = (real_t)(0.0000000000000000e+00);
out[106] = (real_t)(0.0000000000000000e+00);
out[107] = (real_t)(0.0000000000000000e+00);
out[108] = (real_t)(0.0000000000000000e+00);
out[109] = (real_t)(0.0000000000000000e+00);
out[110] = ((real_t)(0.0000000000000000e+00)-(real_t)(3.5522579058221003e-02));
out[111] = (real_t)(0.0000000000000000e+00);
out[112] = (real_t)(0.0000000000000000e+00);
out[113] = (real_t)(0.0000000000000000e+00);
out[114] = (real_t)(0.0000000000000000e+00);
out[115] = (real_t)(0.0000000000000000e+00);
out[116] = (real_t)(0.0000000000000000e+00);
out[117] = (real_t)(0.0000000000000000e+00);
out[118] = (real_t)(0.0000000000000000e+00);
out[119] = (real_t)(6.3750488876158695e-01);
out[120] = (real_t)(0.0000000000000000e+00);
out[121] = (real_t)(0.0000000000000000e+00);
out[122] = (real_t)(0.0000000000000000e+00);
out[123] = (real_t)(0.0000000000000000e+00);
out[124] = (real_t)(3.6855825785903201e-01);
out[125] = (real_t)(0.0000000000000000e+00);
out[126] = (real_t)(0.0000000000000000e+00);
out[127] = (real_t)(0.0000000000000000e+00);
out[128] = (real_t)(0.0000000000000000e+00);
out[129] = (real_t)(0.0000000000000000e+00);
out[130] = (real_t)(0.0000000000000000e+00);
out[131] = (real_t)(0.0000000000000000e+00);
out[132] = (real_t)(0.0000000000000000e+00);
out[133] = (real_t)(0.0000000000000000e+00);
out[134] = (real_t)(0.0000000000000000e+00);
out[135] = (real_t)(0.0000000000000000e+00);
out[136] = (real_t)(6.3840653268265402e-01);
out[137] = (real_t)(0.0000000000000000e+00);
out[138] = (real_t)(0.0000000000000000e+00);
out[139] = (real_t)(0.0000000000000000e+00);
out[140] = (real_t)(0.0000000000000000e+00);
out[141] = (real_t)(3.6817194703901501e-01);
out[142] = (real_t)(0.0000000000000000e+00);
out[143] = (real_t)(0.0000000000000000e+00);
out[144] = (real_t)(0.0000000000000000e+00);
out[145] = (real_t)(0.0000000000000000e+00);
out[146] = (real_t)(0.0000000000000000e+00);
out[147] = (real_t)(0.0000000000000000e+00);
out[148] = (real_t)(0.0000000000000000e+00);
out[149] = (real_t)(0.0000000000000000e+00);
out[150] = (real_t)(0.0000000000000000e+00);
out[151] = (real_t)(0.0000000000000000e+00);
out[152] = (real_t)(0.0000000000000000e+00);
out[153] = (real_t)(1.0000000000000000e+00);
out[154] = (real_t)(5.8094630551531801e-01);
out[155] = ((real_t)(0.0000000000000000e+00)-(real_t)(2.3800609512830001e-03));
out[156] = (real_t)(0.0000000000000000e+00);
out[157] = (real_t)(0.0000000000000000e+00);
out[158] = (real_t)(0.0000000000000000e+00);
out[159] = (real_t)(0.0000000000000000e+00);
out[160] = (real_t)(0.0000000000000000e+00);
out[161] = (real_t)(0.0000000000000000e+00);
out[162] = (real_t)(0.0000000000000000e+00);
out[163] = (real_t)(0.0000000000000000e+00);
out[164] = (real_t)(0.0000000000000000e+00);
out[165] = (real_t)(0.0000000000000000e+00);
out[166] = (real_t)(0.0000000000000000e+00);
out[167] = (real_t)(0.0000000000000000e+00);
out[168] = (real_t)(0.0000000000000000e+00);
out[169] = (real_t)(0.0000000000000000e+00);
out[170] = (real_t)(9.3646534939894399e-01);
out[171] = ((real_t)(0.0000000000000000e+00)-(real_t)(4.6804990378125001e-02));
out[172] = (real_t)(0.0000000000000000e+00);
out[173] = (real_t)(0.0000000000000000e+00);
out[174] = (real_t)(0.0000000000000000e+00);
out[175] = (real_t)(5.1259007219671144e-04);
out[176] = (real_t)(0.0000000000000000e+00);
out[177] = (real_t)(0.0000000000000000e+00);
out[178] = (real_t)(0.0000000000000000e+00);
out[179] = (real_t)(0.0000000000000000e+00);
out[180] = (real_t)(0.0000000000000000e+00);
out[181] = (real_t)(0.0000000000000000e+00);
out[182] = (real_t)(0.0000000000000000e+00);
out[183] = (real_t)(0.0000000000000000e+00);
out[184] = (real_t)(0.0000000000000000e+00);
out[185] = (real_t)(0.0000000000000000e+00);
out[186] = (real_t)(2.3691436602558699e-01);
out[187] = (real_t)(8.2032901524311996e-01);
out[188] = (real_t)(0.0000000000000000e+00);
out[189] = (real_t)(0.0000000000000000e+00);
out[190] = (real_t)(0.0000000000000000e+00);
out[191] = ((real_t)(0.0000000000000000e+00)-(real_t)(1.3647220727940000e-02));
}



/* Fixed step size:0.0675676 */
int acado_integrate( real_t* const rk_eta, int resetIntegrator )
{
int error;

int i;
int j;
int k;
int run;
int tmp_index;
acadoWorkspace.rk_xxx[12] = rk_eta[204];
acadoWorkspace.rk_xxx[13] = rk_eta[205];
acadoWorkspace.rk_xxx[14] = rk_eta[206];
acadoWorkspace.rk_xxx[15] = rk_eta[207];


for (run = 0; run < 1; ++run)
{
acadoWorkspace.rk_xxx[0] = rk_eta[0];
acadoWorkspace.rk_xxx[1] = rk_eta[1];
acadoWorkspace.rk_xxx[2] = rk_eta[2];
acadoWorkspace.rk_xxx[3] = rk_eta[3];
acadoWorkspace.rk_xxx[4] = rk_eta[4];
acadoWorkspace.rk_xxx[5] = rk_eta[5];
acadoWorkspace.rk_xxx[6] = rk_eta[6];
acadoWorkspace.rk_xxx[7] = rk_eta[7];
acadoWorkspace.rk_xxx[8] = rk_eta[8];
acadoWorkspace.rk_xxx[9] = rk_eta[9];
acadoWorkspace.rk_xxx[10] = rk_eta[10];
acadoWorkspace.rk_xxx[11] = rk_eta[11];
acado_rhs( acadoWorkspace.rk_xxx, rk_eta );
acado_diffs( acadoWorkspace.rk_xxx, acadoWorkspace.rk_diffsNew2 );
for (i = 0; i < 12; ++i)
{
for (j = 0; j < 12; ++j)
{
tmp_index = (j) + (i * 12);
rk_eta[tmp_index + 12] = acadoWorkspace.rk_diffsNew2[(i * 16) + (j)];
}
for (j = 0; j < 4; ++j)
{
tmp_index = (j) + (i * 4);
rk_eta[tmp_index + 156] = acadoWorkspace.rk_diffsNew2[(i * 16) + (j + 12)];
}
}
}
for (i = 0; i < 12; ++i)
{
}
return error;
}

