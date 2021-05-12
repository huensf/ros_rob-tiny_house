//
//-------------------------------------------------------------
//
//	ROBOTRAN - Version 6.6 (build : february 22, 2008)
//
//	Copyright 
//	Universite catholique de Louvain 
//	Departement de Mecanique 
//	Unite de Production Mecanique et Machines 
//	2, Place du Levant 
//	1348 Louvain-la-Neuve 
//	http://www.robotran.be// 
//
//	==> Generation Date : Sat Jan  9 22:36:43 2021
//
//	==> Project name : Car
//	==> using XML input file 
//
//	==> Number of joints : 53
//
//	==> Function : F18 : Constraints Quadratic Velocity Terms (Jdqd)
//	==> Flops complexity : 955
//
//	==> Generation Time :  0.030 seconds
//	==> Post-Processing :  0.020 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_cons_jdqd(double *Jdqd,
MbsData *s, double tsim)

// double Jdqd[28];
{ 
 
#include "mbs_cons_jdqd_Car.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 

// = = Block_0_0_0_0_0_3 = = 
 
// Trigonometric Variables  

  C8 = cos(q[8]);
  S8 = sin(q[8]);
  C9 = cos(q[9]);
  S9 = sin(q[9]);
  C10 = cos(q[10]);
  S10 = sin(q[10]);
  C11 = cos(q[11]);
  S11 = sin(q[11]);

// = = Block_0_0_0_0_0_4 = = 
 
// Trigonometric Variables  

  C13 = cos(q[13]);
  S13 = sin(q[13]);
  C14 = cos(q[14]);
  S14 = sin(q[14]);
  C15 = cos(q[15]);
  S15 = sin(q[15]);
  C16 = cos(q[16]);
  S16 = sin(q[16]);

// = = Block_0_0_0_0_0_5 = = 
 
// Trigonometric Variables  

  C18 = cos(q[18]);
  S18 = sin(q[18]);

// = = Block_0_0_0_0_0_6 = = 
 
// Trigonometric Variables  

  C19 = cos(q[19]);
  S19 = sin(q[19]);

// = = Block_0_0_0_0_0_7 = = 
 
// Trigonometric Variables  

  C20 = cos(q[20]);
  S20 = sin(q[20]);

// = = Block_0_0_0_0_0_8 = = 
 
// Trigonometric Variables  

  C21 = cos(q[21]);
  S21 = sin(q[21]);

// = = Block_0_0_0_0_0_9 = = 
 
// Trigonometric Variables  

  C22 = cos(q[22]);
  S22 = sin(q[22]);

// = = Block_0_0_0_0_0_10 = = 
 
// Trigonometric Variables  

  C23 = cos(q[23]);
  S23 = sin(q[23]);
  C24 = cos(q[24]);
  S24 = sin(q[24]);

// = = Block_0_0_0_0_0_11 = = 
 
// Trigonometric Variables  

  C25 = cos(q[25]);
  S25 = sin(q[25]);
  C26 = cos(q[26]);
  S26 = sin(q[26]);
  C27 = cos(q[27]);
  S27 = sin(q[27]);

// = = Block_0_0_0_0_0_13 = = 
 
// Trigonometric Variables  

  C29 = cos(q[29]);
  S29 = sin(q[29]);
  C30 = cos(q[30]);
  S30 = sin(q[30]);

// = = Block_0_0_0_0_0_14 = = 
 
// Trigonometric Variables  

  C31 = cos(q[31]);
  S31 = sin(q[31]);
  C32 = cos(q[32]);
  S32 = sin(q[32]);

// = = Block_0_0_0_0_0_15 = = 
 
// Trigonometric Variables  

  C33 = cos(q[33]);
  S33 = sin(q[33]);
  C34 = cos(q[34]);
  S34 = sin(q[34]);

// = = Block_0_0_0_0_0_16 = = 
 
// Trigonometric Variables  

  C36 = cos(q[36]);
  S36 = sin(q[36]);
  C37 = cos(q[37]);
  S37 = sin(q[37]);

// = = Block_0_0_0_0_0_17 = = 
 
// Trigonometric Variables  

  C39 = cos(q[39]);
  S39 = sin(q[39]);

// = = Block_0_0_0_0_0_18 = = 
 
// Trigonometric Variables  

  C40 = cos(q[40]);
  S40 = sin(q[40]);
  C41 = cos(q[41]);
  S41 = sin(q[41]);
  C42 = cos(q[42]);
  S42 = sin(q[42]);

// = = Block_0_0_0_0_0_19 = = 
 
// Trigonometric Variables  

  C43 = cos(q[43]);
  S43 = sin(q[43]);
  C44 = cos(q[44]);
  S44 = sin(q[44]);

// = = Block_0_1_0_0_0_3 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_2_29 = S8*S9;
  RO_2_39 = -C8*S9;
  RO_2_89 = -S8*C9;
  RO_2_99 = C8*C9;
  RO_2_710 = C10*S9;
  RO_2_810 = RO_2_89*C10-S10*C8;
  RO_2_910 = RO_2_99*C10-S10*S8;
  RO_2_111 = S10*S11*S9+C11*C9;
  RO_2_211 = RO_2_29*C11+S11*(RO_2_89*S10+C10*C8);
  RO_2_311 = RO_2_39*C11+S11*(RO_2_99*S10+C10*S8);
  RL_2_156 = RO_2_111*s->dpt[1][21]+RO_2_710*s->dpt[3][21];
  RL_2_256 = RO_2_211*s->dpt[1][21]+RO_2_810*s->dpt[3][21];
  RL_2_356 = RO_2_311*s->dpt[1][21]+RO_2_910*s->dpt[3][21];
//
  RL_16_170 = RO_2_111*s->dpt[1][20]+RO_2_710*s->dpt[3][20];
  RL_16_270 = RO_2_211*s->dpt[1][20]+RO_2_810*s->dpt[3][20];
  RL_16_370 = RO_2_311*s->dpt[1][20]+RO_2_910*s->dpt[3][20];

// = = Block_0_1_0_0_0_4 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_0_214 = S13*S14;
  RO_0_314 = -C13*S14;
  RO_0_814 = -S13*C14;
  RO_0_914 = C13*C14;
  RO_0_715 = S14*C15;
  RO_0_815 = RO_0_814*C15-C13*S15;
  RO_0_915 = RO_0_914*C15-S13*S15;
  RO_0_116 = C14*C16+S14*S15*S16;
  RO_0_216 = RO_0_214*C16+S16*(RO_0_814*S15+C13*C15);
  RO_0_316 = RO_0_314*C16+S16*(RO_0_914*S15+S13*C15);
  RL_0_154 = RO_0_116*s->dpt[1][26]+RO_0_715*s->dpt[3][26];
  RL_0_254 = RO_0_216*s->dpt[1][26]+RO_0_815*s->dpt[3][26];
  RL_0_354 = RO_0_316*s->dpt[1][26]+RO_0_915*s->dpt[3][26];
//
  RL_18_172 = RO_0_116*s->dpt[1][28]+RO_0_715*s->dpt[3][28];
  RL_18_272 = RO_0_216*s->dpt[1][28]+RO_0_815*s->dpt[3][28];
  RL_18_372 = RO_0_316*s->dpt[1][28]+RO_0_915*s->dpt[3][28];

// = = Block_0_1_0_0_0_10 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_8_423 = S22*S23;
  RO_8_623 = C22*S23;
  RL_8_162 = s->dpt[3][40]*(C22*S24+S22*C23*C24);
  RL_8_262 = -s->dpt[3][40]*S23*C24;
  RL_8_362 = s->dpt[3][40]*(C22*C23*C24-S22*S24);

// = = Block_0_1_0_0_0_11 = = 
 
// Trigonometric Variables  

//
  S22p25 = C22*S25+S22*C25;
  C22p25 = C22*C25-S22*S25;
 
// Constraints and Constraints Jacobian 

  RO_10_426 = S22p25*S26;
  RO_10_626 = C22p25*S26;
  RL_10_164 = s->dpt[3][42]*(C22p25*S27+S22p25*C26*C27);
  RL_10_264 = -s->dpt[3][42]*S26*C27;
  RL_10_364 = s->dpt[3][42]*(C22p25*C26*C27-S22p25*S27);

// = = Block_0_1_0_0_0_13 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_17_171 = s->dpt[1][45]*C30-s->dpt[2][45]*S30;
  RL_17_271 = C29*(s->dpt[1][45]*S30+s->dpt[2][45]*C30);
  RL_17_371 = S29*(s->dpt[1][45]*S30+s->dpt[2][45]*C30);

// = = Block_0_1_0_0_0_14 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_19_173 = s->dpt[1][46]*C32-s->dpt[2][46]*S32;
  RL_19_273 = C31*(s->dpt[1][46]*S32+s->dpt[2][46]*C32);
  RL_19_373 = S31*(s->dpt[1][46]*S32+s->dpt[2][46]*C32);

// = = Block_0_1_0_0_0_15 = = 
 
// Trigonometric Variables  

//
  S33p34 = C33*S34+S33*C34;
  C33p34 = C33*C34-S33*S34;

// = = Block_0_1_0_0_0_16 = = 
 
// Trigonometric Variables  

//
  S36p37 = C36*S37+S36*C37;
  C36p37 = C36*C37-S36*S37;

// = = Block_0_1_0_0_0_18 = = 
 
// Trigonometric Variables  

//
  S39p40 = C39*S40+S39*C40;
  C39p40 = C39*C40-S39*S40;
 
// Constraints and Constraints Jacobian 

  RO_14_441 = S39p40*S41;
  RO_14_641 = C39p40*S41;
  RL_14_168 = s->dpt[3][60]*(C39p40*S42+S39p40*C41*C42);
  RL_14_268 = -s->dpt[3][60]*S41*C42;
  RL_14_368 = s->dpt[3][60]*(C39p40*C41*C42-S39p40*S42);

// = = Block_0_1_0_0_0_19 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_13_443 = S39*S43;
  RO_13_643 = C39*S43;
  RL_13_167 = s->dpt[3][61]*(C39*S44+S39*C43*C44);
  RL_13_267 = -s->dpt[3][61]*S43*C44;
  RL_13_367 = s->dpt[3][61]*(C39*C43*C44-S39*S44);

// = = Block_0_2_0_0_0_0 = = 
 
// Constraints Quadratic Terms 

//
  OM_0_214 = qd[14]*C13;
  OM_0_314 = qd[14]*S13;
  Apqp_0_214 = -qd[13]*qd[13]*s->dpt[2][24]*C13;
  Apqp_0_314 = -qd[13]*qd[13]*s->dpt[2][24]*S13;
  OM_0_115 = qd[13]+qd[15]*C14;
  OM_0_215 = OM_0_214+RO_0_214*qd[15];
  OM_0_315 = OM_0_314+RO_0_314*qd[15];
  OM_0_116 = OM_0_115+RO_0_715*qd[16];
  OM_0_216 = OM_0_215+RO_0_815*qd[16];
  OM_0_316 = OM_0_315+RO_0_915*qd[16];
  Ompqp_0_116 = -(qd[14]*qd[15]*S14-qd[16]*(OM_0_215*RO_0_915-OM_0_315*RO_0_815));
  Ompqp_0_216 = -(qd[13]*qd[14]*S13-qd[15]*(OM_0_314*C14-RO_0_314*qd[13])+qd[16]*(OM_0_115*RO_0_915-OM_0_315*RO_0_715));
  Ompqp_0_316 = qd[13]*qd[14]*C13-qd[15]*(OM_0_214*C14-RO_0_214*qd[13])+qd[16]*(OM_0_115*RO_0_815-OM_0_215*RO_0_715);
  OR_0_154 = OM_0_216*RL_0_354-OM_0_316*RL_0_254;
  OR_0_254 = -(OM_0_116*RL_0_354-OM_0_316*RL_0_154);
  OR_0_354 = OM_0_116*RL_0_254-OM_0_216*RL_0_154;
  Apqp_0_154 = OM_0_216*OR_0_354-OM_0_316*OR_0_254+Ompqp_0_216*RL_0_354-Ompqp_0_316*RL_0_254;
//
  OM_2_29 = qd[9]*C8;
  OM_2_39 = qd[9]*S8;
  Apqp_2_29 = -qd[8]*qd[8]*s->dpt[2][18]*C8;
  Apqp_2_39 = -qd[8]*qd[8]*s->dpt[2][18]*S8;
  OM_2_110 = qd[8]+qd[10]*C9;
  OM_2_210 = OM_2_29+RO_2_29*qd[10];
  OM_2_310 = OM_2_39+RO_2_39*qd[10];
  OM_2_111 = OM_2_110+RO_2_710*qd[11];
  OM_2_211 = OM_2_210+RO_2_810*qd[11];
  OM_2_311 = OM_2_310+RO_2_910*qd[11];
  Ompqp_2_111 = -(qd[10]*qd[9]*S9-qd[11]*(OM_2_210*RO_2_910-OM_2_310*RO_2_810));
  Ompqp_2_211 = qd[10]*(OM_2_39*C9-RO_2_39*qd[8])-qd[11]*(OM_2_110*RO_2_910-OM_2_310*RO_2_710)-qd[8]*qd[9]*S8;
  Ompqp_2_311 = qd[8]*qd[9]*C8-qd[10]*(OM_2_29*C9-RO_2_29*qd[8])+qd[11]*(OM_2_110*RO_2_810-OM_2_210*RO_2_710);
  OR_2_156 = OM_2_211*RL_2_356-OM_2_311*RL_2_256;
  OR_2_256 = -(OM_2_111*RL_2_356-OM_2_311*RL_2_156);
  OR_2_356 = OM_2_111*RL_2_256-OM_2_211*RL_2_156;
  Apqp_2_156 = OM_2_211*OR_2_356-OM_2_311*OR_2_256+Ompqp_2_211*RL_2_356-Ompqp_2_311*RL_2_256;
//
  OM_5_137 = qd[36]+qd[37];
//
  OM_7_134 = qd[33]+qd[34];
//
  OM_8_123 = qd[23]*C22;
  OM_8_323 = -qd[23]*S22;
  OM_8_124 = OM_8_123+RO_8_423*qd[24];
  OM_8_224 = qd[22]+qd[24]*C23;
  OM_8_324 = OM_8_323+RO_8_623*qd[24];
  Ompqp_8_124 = -(qd[22]*qd[23]*S22+qd[24]*(OM_8_323*C23-RO_8_623*qd[22]));
  Ompqp_8_224 = -qd[23]*qd[24]*S23;
  Ompqp_8_324 = -(qd[22]*qd[23]*C22-qd[24]*(OM_8_123*C23-RO_8_423*qd[22]));
  OR_8_162 = OM_8_224*RL_8_362-OM_8_324*RL_8_262;
  OR_8_262 = -(OM_8_124*RL_8_362-OM_8_324*RL_8_162);
  OR_8_362 = OM_8_124*RL_8_262-OM_8_224*RL_8_162;
  Apqp_8_162 = OM_8_224*OR_8_362-OM_8_324*OR_8_262+Ompqp_8_224*RL_8_362-Ompqp_8_324*RL_8_262-qd[22]*qd[22]*s->dpt[1][38]
 *C22;
//
  OM_10_225 = qd[22]+qd[25];
  OM_10_126 = qd[26]*C22p25;
  OM_10_326 = -qd[26]*S22p25;
  OM_10_127 = OM_10_126+RO_10_426*qd[27];
  OM_10_227 = OM_10_225+qd[27]*C26;
  OM_10_327 = OM_10_326+RO_10_626*qd[27];
  Ompqp_10_127 = -(OM_10_225*qd[26]*S22p25-qd[27]*(OM_10_225*RO_10_626-OM_10_326*C26));
  Ompqp_10_227 = -qd[26]*qd[27]*S26;
  Ompqp_10_327 = -(OM_10_225*qd[26]*C22p25-qd[27]*(OM_10_126*C26-OM_10_225*RO_10_426));
  OR_10_164 = OM_10_227*RL_10_364-OM_10_327*RL_10_264;
  OR_10_264 = -(OM_10_127*RL_10_364-OM_10_327*RL_10_164);
  OR_10_364 = OM_10_127*RL_10_264-OM_10_227*RL_10_164;
  Apqp_10_164 = -(OM_10_225*OM_10_225*s->dpt[1][41]*C22p25-OM_10_227*OR_10_364+OM_10_327*OR_10_264-Ompqp_10_227*
 RL_10_364+Ompqp_10_327*RL_10_264);
//
  OM_13_143 = qd[43]*C39;
  OM_13_343 = -qd[43]*S39;
  OM_13_144 = OM_13_143+RO_13_443*qd[44];
  OM_13_244 = qd[39]+qd[44]*C43;
  OM_13_344 = OM_13_343+RO_13_643*qd[44];
  Ompqp_13_144 = -(qd[39]*qd[43]*S39+qd[44]*(OM_13_343*C43-RO_13_643*qd[39]));
  Ompqp_13_244 = -qd[43]*qd[44]*S43;
  Ompqp_13_344 = -(qd[39]*qd[43]*C39-qd[44]*(OM_13_143*C43-RO_13_443*qd[39]));
  OR_13_167 = OM_13_244*RL_13_367-OM_13_344*RL_13_267;
  OR_13_267 = -(OM_13_144*RL_13_367-OM_13_344*RL_13_167);
  OR_13_367 = OM_13_144*RL_13_267-OM_13_244*RL_13_167;
  Apqp_13_167 = OM_13_244*OR_13_367-OM_13_344*OR_13_267+Ompqp_13_244*RL_13_367-Ompqp_13_344*RL_13_267-qd[39]*qd[39]*
 s->dpt[1][58]*C39;
//
  OM_14_240 = qd[39]+qd[40];
  OM_14_141 = qd[41]*C39p40;
  OM_14_341 = -qd[41]*S39p40;
  OM_14_142 = OM_14_141+RO_14_441*qd[42];
  OM_14_242 = OM_14_240+qd[42]*C41;
  OM_14_342 = OM_14_341+RO_14_641*qd[42];
  Ompqp_14_142 = -(OM_14_240*qd[41]*S39p40-qd[42]*(OM_14_240*RO_14_641-OM_14_341*C41));
  Ompqp_14_242 = -qd[41]*qd[42]*S41;
  Ompqp_14_342 = -(OM_14_240*qd[41]*C39p40-qd[42]*(OM_14_141*C41-OM_14_240*RO_14_441));
  OR_14_168 = OM_14_242*RL_14_368-OM_14_342*RL_14_268;
  OR_14_268 = -(OM_14_142*RL_14_368-OM_14_342*RL_14_168);
  OR_14_368 = OM_14_142*RL_14_268-OM_14_242*RL_14_168;
  Apqp_14_168 = -(OM_14_240*OM_14_240*s->dpt[1][59]*C39p40-OM_14_242*OR_14_368+OM_14_342*OR_14_268-Ompqp_14_242*
 RL_14_368+Ompqp_14_342*RL_14_268);
//
  OR_16_170 = OM_2_211*RL_16_370-OM_2_311*RL_16_270;
  OR_16_270 = -(OM_2_111*RL_16_370-OM_2_311*RL_16_170);
  OR_16_370 = OM_2_111*RL_16_270-OM_2_211*RL_16_170;
//
  OM_17_230 = -qd[30]*S29;
  OM_17_330 = qd[30]*C29;
  Ompqp_17_230 = -qd[29]*qd[30]*C29;
  Ompqp_17_330 = -qd[29]*qd[30]*S29;
  OR_17_171 = OM_17_230*RL_17_371-OM_17_330*RL_17_271;
  OR_17_271 = OM_17_330*RL_17_171-RL_17_371*qd[29];
  OR_17_371 = -(OM_17_230*RL_17_171-RL_17_271*qd[29]);
//
  OR_18_172 = OM_0_216*RL_18_372-OM_0_316*RL_18_272;
  OR_18_272 = -(OM_0_116*RL_18_372-OM_0_316*RL_18_172);
  OR_18_372 = OM_0_116*RL_18_272-OM_0_216*RL_18_172;
//
  OM_19_232 = -qd[32]*S31;
  OM_19_332 = qd[32]*C31;
  Ompqp_19_232 = -qd[31]*qd[32]*C31;
  Ompqp_19_332 = -qd[31]*qd[32]*S31;
  OR_19_173 = OM_19_232*RL_19_373-OM_19_332*RL_19_273;
  OR_19_273 = OM_19_332*RL_19_173-RL_19_373*qd[31];
  OR_19_373 = -(OM_19_232*RL_19_173-RL_19_273*qd[31]);

// = = Block_0_2_0_0_0_1 = = 
 
// Constraints Quadratic Terms 

//
  jdqd2 = Apqp_0_214-OM_0_116*OR_0_354+OM_0_316*OR_0_154-Ompqp_0_116*RL_0_354+Ompqp_0_316*RL_0_154+qd[18]*qd[18]*
 s->dpt[2][30]*C18;
  jdqd3 = Apqp_0_314+OM_0_116*OR_0_254-OM_0_216*OR_0_154+Ompqp_0_116*RL_0_254-Ompqp_0_216*RL_0_154+qd[18]*qd[18]*
 s->dpt[2][30]*S18;
//
  jdqd5 = Apqp_2_29-OM_2_111*OR_2_356+OM_2_311*OR_2_156-Ompqp_2_111*RL_2_356+Ompqp_2_311*RL_2_156+qd[19]*qd[19]*
 s->dpt[2][32]*C19;
  jdqd6 = Apqp_2_39+OM_2_111*OR_2_256-OM_2_211*OR_2_156+Ompqp_2_111*RL_2_256-Ompqp_2_211*RL_2_156+qd[19]*qd[19]*
 s->dpt[2][32]*S19;
//
  jdqd8 = -(OM_5_137*OM_5_137*s->dpt[3][55]*S36p37+qd[20]*qd[20]*s->dpt[2][34]*C20-qd[36]*qd[36]*s->dpt[2][52]*C36);
  jdqd9 = OM_5_137*OM_5_137*s->dpt[3][55]*C36p37-qd[20]*qd[20]*s->dpt[2][34]*S20+qd[36]*qd[36]*s->dpt[2][52]*S36;
//
  jdqd11 = -(OM_7_134*OM_7_134*s->dpt[3][49]*S33p34+qd[21]*qd[21]*s->dpt[2][36]*C21-qd[33]*qd[33]*s->dpt[2][47]*C33);
  jdqd12 = OM_7_134*OM_7_134*s->dpt[3][49]*C33p34-qd[21]*qd[21]*s->dpt[2][36]*S21+qd[33]*qd[33]*s->dpt[2][47]*S33;
//
  jdqd14 = -(OM_8_124*OR_8_362-OM_8_324*OR_8_162+Ompqp_8_124*RL_8_362-Ompqp_8_324*RL_8_162-qd[13]*qd[13]*s->dpt[2][25]*
 C13);
  jdqd15 = OM_8_124*OR_8_262-OM_8_224*OR_8_162+Ompqp_8_124*RL_8_262-Ompqp_8_224*RL_8_162+qd[13]*qd[13]*s->dpt[2][25]*S13
 +qd[22]*qd[22]*s->dpt[1][38]*S22;
//
  jdqd17 = -(OM_10_127*OR_10_364-OM_10_327*OR_10_164+Ompqp_10_127*RL_10_364-Ompqp_10_327*RL_10_164-qd[8]*qd[8]*
 s->dpt[2][19]*C8);
  jdqd18 = OM_10_127*OR_10_264+OM_10_225*OM_10_225*s->dpt[1][41]*S22p25-OM_10_227*OR_10_164+Ompqp_10_127*RL_10_264-
 Ompqp_10_227*RL_10_164+qd[8]*qd[8]*s->dpt[2][19]*S8;
//
  jdqd20 = OM_13_144*OR_13_367-OM_13_344*OR_13_167+Ompqp_13_144*RL_13_367-Ompqp_13_344*RL_13_167-qd[33]*qd[33]*
 s->dpt[2][48]*C33;
  jdqd21 = -(OM_13_144*OR_13_267-OM_13_244*OR_13_167+Ompqp_13_144*RL_13_267-Ompqp_13_244*RL_13_167+qd[33]*qd[33]*
 s->dpt[2][48]*S33+qd[39]*qd[39]*s->dpt[1][58]*S39);
//
  jdqd23 = -(OM_14_142*OR_14_368-OM_14_342*OR_14_168+Ompqp_14_142*RL_14_368-Ompqp_14_342*RL_14_168-qd[36]*qd[36]*
 s->dpt[2][53]*C36);
  jdqd24 = OM_14_142*OR_14_268+OM_14_240*OM_14_240*s->dpt[1][59]*S39p40-OM_14_242*OR_14_168+Ompqp_14_142*RL_14_268-
 Ompqp_14_242*RL_14_168+qd[36]*qd[36]*s->dpt[2][53]*S36;
//
  jdqd25 = OM_2_211*OR_16_370-OM_2_311*OR_16_270+Ompqp_2_211*RL_16_370-Ompqp_2_311*RL_16_270-(OM_17_230*OR_17_371-
 OM_17_330*OR_17_271+Ompqp_17_230*RL_17_371-Ompqp_17_330*RL_17_271);
  jdqd26 = Apqp_2_29-OM_17_330*OR_17_171-OM_2_111*OR_16_370+OM_2_311*OR_16_170+OR_17_371*qd[29]-Ompqp_17_330*RL_17_171-
 Ompqp_2_111*RL_16_370+Ompqp_2_311*RL_16_170;
  jdqd27 = Apqp_2_39+OM_17_230*OR_17_171+OM_2_111*OR_16_270-OM_2_211*OR_16_170-OR_17_271*qd[29]+Ompqp_17_230*RL_17_171+
 Ompqp_2_111*RL_16_270-Ompqp_2_211*RL_16_170;
//
  jdqd28 = OM_0_216*OR_18_372-OM_0_316*OR_18_272-OM_19_232*OR_19_373+OM_19_332*OR_19_273+Ompqp_0_216*RL_18_372-
 Ompqp_0_316*RL_18_272-Ompqp_19_232*RL_19_373+Ompqp_19_332*RL_19_273;
  jdqd29 = Apqp_0_214-OM_0_116*OR_18_372+OM_0_316*OR_18_172-OM_19_332*OR_19_173+OR_19_373*qd[31]-Ompqp_0_116*RL_18_372+
 Ompqp_0_316*RL_18_172-Ompqp_19_332*RL_19_173;
  jdqd30 = Apqp_0_314+OM_0_116*OR_18_272-OM_0_216*OR_18_172+OM_19_232*OR_19_173-OR_19_273*qd[31]+Ompqp_0_116*RL_18_272-
 Ompqp_0_216*RL_18_172+Ompqp_19_232*RL_19_173;

// = = Block_0_3_0_0_0_0 = = 
 
// Symbolic Outputs  

  Jdqd[1] = Apqp_0_154;
  Jdqd[2] = jdqd2;
  Jdqd[3] = jdqd3;
  Jdqd[4] = Apqp_2_156;
  Jdqd[5] = jdqd5;
  Jdqd[6] = jdqd6;
  Jdqd[7] = jdqd8;
  Jdqd[8] = jdqd9;
  Jdqd[9] = jdqd11;
  Jdqd[10] = jdqd12;
  Jdqd[11] = Apqp_8_162;
  Jdqd[12] = jdqd14;
  Jdqd[13] = jdqd15;
  Jdqd[14] = Apqp_10_164;
  Jdqd[15] = jdqd17;
  Jdqd[16] = jdqd18;
  Jdqd[17] = -Apqp_13_167;
  Jdqd[18] = jdqd20;
  Jdqd[19] = jdqd21;
  Jdqd[20] = Apqp_14_168;
  Jdqd[21] = jdqd23;
  Jdqd[22] = jdqd24;
  Jdqd[23] = jdqd25;
  Jdqd[24] = jdqd26;
  Jdqd[25] = jdqd27;
  Jdqd[26] = jdqd28;
  Jdqd[27] = jdqd29;
  Jdqd[28] = jdqd30;

// ====== END Task 0 ====== 


}
 

