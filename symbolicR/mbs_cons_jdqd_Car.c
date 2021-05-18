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
//	==> Generation Date : Tue May 18 15:39:25 2021
//
//	==> Project name : Car
//	==> using XML input file 
//
//	==> Number of joints : 59
//
//	==> Function : F18 : Constraints Quadratic Velocity Terms (Jdqd)
//	==> Flops complexity : 955
//
//	==> Generation Time :  0.030 seconds
//	==> Post-Processing :  0.010 seconds
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

// = = Block_0_0_0_0_0_16 = = 
 
// Trigonometric Variables  

  C35 = cos(q[35]);
  S35 = sin(q[35]);
  C36 = cos(q[36]);
  S36 = sin(q[36]);

// = = Block_0_0_0_0_0_17 = = 
 
// Trigonometric Variables  

  C38 = cos(q[38]);
  S38 = sin(q[38]);
  C39 = cos(q[39]);
  S39 = sin(q[39]);

// = = Block_0_0_0_0_0_18 = = 
 
// Trigonometric Variables  

  C41 = cos(q[41]);
  S41 = sin(q[41]);

// = = Block_0_0_0_0_0_19 = = 
 
// Trigonometric Variables  

  C42 = cos(q[42]);
  S42 = sin(q[42]);
  C43 = cos(q[43]);
  S43 = sin(q[43]);
  C44 = cos(q[44]);
  S44 = sin(q[44]);

// = = Block_0_0_0_0_0_20 = = 
 
// Trigonometric Variables  

  C45 = cos(q[45]);
  S45 = sin(q[45]);
  C46 = cos(q[46]);
  S46 = sin(q[46]);

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
  RL_2_162 = RO_2_111*s->dpt[1][22]+RO_2_710*s->dpt[3][22];
  RL_2_262 = RO_2_211*s->dpt[1][22]+RO_2_810*s->dpt[3][22];
  RL_2_362 = RO_2_311*s->dpt[1][22]+RO_2_910*s->dpt[3][22];
//
  RL_16_176 = RO_2_111*s->dpt[1][21]+RO_2_710*s->dpt[3][21];
  RL_16_276 = RO_2_211*s->dpt[1][21]+RO_2_810*s->dpt[3][21];
  RL_16_376 = RO_2_311*s->dpt[1][21]+RO_2_910*s->dpt[3][21];

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
  RL_0_160 = RO_0_116*s->dpt[1][28]+RO_0_715*s->dpt[3][28];
  RL_0_260 = RO_0_216*s->dpt[1][28]+RO_0_815*s->dpt[3][28];
  RL_0_360 = RO_0_316*s->dpt[1][28]+RO_0_915*s->dpt[3][28];
//
  RL_18_178 = RO_0_116*s->dpt[1][30]+RO_0_715*s->dpt[3][30];
  RL_18_278 = RO_0_216*s->dpt[1][30]+RO_0_815*s->dpt[3][30];
  RL_18_378 = RO_0_316*s->dpt[1][30]+RO_0_915*s->dpt[3][30];

// = = Block_0_1_0_0_0_10 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_8_423 = S22*S23;
  RO_8_623 = C22*S23;
  RL_8_168 = s->dpt[3][42]*(C22*S24+S22*C23*C24);
  RL_8_268 = -s->dpt[3][42]*S23*C24;
  RL_8_368 = s->dpt[3][42]*(C22*C23*C24-S22*S24);

// = = Block_0_1_0_0_0_11 = = 
 
// Trigonometric Variables  

//
  S22p25 = C22*S25+S22*C25;
  C22p25 = C22*C25-S22*S25;
 
// Constraints and Constraints Jacobian 

  RO_10_426 = S22p25*S26;
  RO_10_626 = C22p25*S26;
  RL_10_170 = s->dpt[3][44]*(C22p25*S27+S22p25*C26*C27);
  RL_10_270 = -s->dpt[3][44]*S26*C27;
  RL_10_370 = s->dpt[3][44]*(C22p25*C26*C27-S22p25*S27);

// = = Block_0_1_0_0_0_13 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_17_177 = s->dpt[1][48]*C30-s->dpt[2][48]*S30;
  RL_17_277 = C29*(s->dpt[1][48]*S30+s->dpt[2][48]*C30);
  RL_17_377 = S29*(s->dpt[1][48]*S30+s->dpt[2][48]*C30);

// = = Block_0_1_0_0_0_14 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_19_179 = s->dpt[1][49]*C32-s->dpt[2][49]*S32;
  RL_19_279 = C31*(s->dpt[1][49]*S32+s->dpt[2][49]*C32);
  RL_19_379 = S31*(s->dpt[1][49]*S32+s->dpt[2][49]*C32);

// = = Block_0_1_0_0_0_16 = = 
 
// Trigonometric Variables  

//
  S35p36 = C35*S36+S35*C36;
  C35p36 = C35*C36-S35*S36;

// = = Block_0_1_0_0_0_17 = = 
 
// Trigonometric Variables  

//
  S38p39 = C38*S39+S38*C39;
  C38p39 = C38*C39-S38*S39;

// = = Block_0_1_0_0_0_19 = = 
 
// Trigonometric Variables  

//
  S41p42 = C41*S42+S41*C42;
  C41p42 = C41*C42-S41*S42;
 
// Constraints and Constraints Jacobian 

  RO_14_443 = S41p42*S43;
  RO_14_643 = C41p42*S43;
  RL_14_174 = s->dpt[3][63]*(C41p42*S44+S41p42*C43*C44);
  RL_14_274 = -s->dpt[3][63]*S43*C44;
  RL_14_374 = s->dpt[3][63]*(C41p42*C43*C44-S41p42*S44);

// = = Block_0_1_0_0_0_20 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_13_445 = S41*S45;
  RO_13_645 = C41*S45;
  RL_13_173 = s->dpt[3][64]*(C41*S46+S41*C45*C46);
  RL_13_273 = -s->dpt[3][64]*S45*C46;
  RL_13_373 = s->dpt[3][64]*(C41*C45*C46-S41*S46);

// = = Block_0_2_0_0_0_0 = = 
 
// Constraints Quadratic Terms 

//
  OM_0_214 = qd[14]*C13;
  OM_0_314 = qd[14]*S13;
  Apqp_0_214 = -qd[13]*qd[13]*s->dpt[2][26]*C13;
  Apqp_0_314 = -qd[13]*qd[13]*s->dpt[2][26]*S13;
  OM_0_115 = qd[13]+qd[15]*C14;
  OM_0_215 = OM_0_214+RO_0_214*qd[15];
  OM_0_315 = OM_0_314+RO_0_314*qd[15];
  OM_0_116 = OM_0_115+RO_0_715*qd[16];
  OM_0_216 = OM_0_215+RO_0_815*qd[16];
  OM_0_316 = OM_0_315+RO_0_915*qd[16];
  Ompqp_0_116 = -(qd[14]*qd[15]*S14-qd[16]*(OM_0_215*RO_0_915-OM_0_315*RO_0_815));
  Ompqp_0_216 = -(qd[13]*qd[14]*S13-qd[15]*(OM_0_314*C14-RO_0_314*qd[13])+qd[16]*(OM_0_115*RO_0_915-OM_0_315*RO_0_715));
  Ompqp_0_316 = qd[13]*qd[14]*C13-qd[15]*(OM_0_214*C14-RO_0_214*qd[13])+qd[16]*(OM_0_115*RO_0_815-OM_0_215*RO_0_715);
  OR_0_160 = OM_0_216*RL_0_360-OM_0_316*RL_0_260;
  OR_0_260 = -(OM_0_116*RL_0_360-OM_0_316*RL_0_160);
  OR_0_360 = OM_0_116*RL_0_260-OM_0_216*RL_0_160;
  Apqp_0_160 = OM_0_216*OR_0_360-OM_0_316*OR_0_260+Ompqp_0_216*RL_0_360-Ompqp_0_316*RL_0_260;
//
  OM_2_29 = qd[9]*C8;
  OM_2_39 = qd[9]*S8;
  Apqp_2_29 = -qd[8]*qd[8]*s->dpt[2][19]*C8;
  Apqp_2_39 = -qd[8]*qd[8]*s->dpt[2][19]*S8;
  OM_2_110 = qd[8]+qd[10]*C9;
  OM_2_210 = OM_2_29+RO_2_29*qd[10];
  OM_2_310 = OM_2_39+RO_2_39*qd[10];
  OM_2_111 = OM_2_110+RO_2_710*qd[11];
  OM_2_211 = OM_2_210+RO_2_810*qd[11];
  OM_2_311 = OM_2_310+RO_2_910*qd[11];
  Ompqp_2_111 = -(qd[10]*qd[9]*S9-qd[11]*(OM_2_210*RO_2_910-OM_2_310*RO_2_810));
  Ompqp_2_211 = qd[10]*(OM_2_39*C9-RO_2_39*qd[8])-qd[11]*(OM_2_110*RO_2_910-OM_2_310*RO_2_710)-qd[8]*qd[9]*S8;
  Ompqp_2_311 = qd[8]*qd[9]*C8-qd[10]*(OM_2_29*C9-RO_2_29*qd[8])+qd[11]*(OM_2_110*RO_2_810-OM_2_210*RO_2_710);
  OR_2_162 = OM_2_211*RL_2_362-OM_2_311*RL_2_262;
  OR_2_262 = -(OM_2_111*RL_2_362-OM_2_311*RL_2_162);
  OR_2_362 = OM_2_111*RL_2_262-OM_2_211*RL_2_162;
  Apqp_2_162 = OM_2_211*OR_2_362-OM_2_311*OR_2_262+Ompqp_2_211*RL_2_362-Ompqp_2_311*RL_2_262;
//
  OM_5_139 = qd[38]+qd[39];
//
  OM_7_136 = qd[35]+qd[36];
//
  OM_8_123 = qd[23]*C22;
  OM_8_323 = -qd[23]*S22;
  OM_8_124 = OM_8_123+RO_8_423*qd[24];
  OM_8_224 = qd[22]+qd[24]*C23;
  OM_8_324 = OM_8_323+RO_8_623*qd[24];
  Ompqp_8_124 = -(qd[22]*qd[23]*S22+qd[24]*(OM_8_323*C23-RO_8_623*qd[22]));
  Ompqp_8_224 = -qd[23]*qd[24]*S23;
  Ompqp_8_324 = -(qd[22]*qd[23]*C22-qd[24]*(OM_8_123*C23-RO_8_423*qd[22]));
  OR_8_168 = OM_8_224*RL_8_368-OM_8_324*RL_8_268;
  OR_8_268 = -(OM_8_124*RL_8_368-OM_8_324*RL_8_168);
  OR_8_368 = OM_8_124*RL_8_268-OM_8_224*RL_8_168;
  Apqp_8_168 = OM_8_224*OR_8_368-OM_8_324*OR_8_268+Ompqp_8_224*RL_8_368-Ompqp_8_324*RL_8_268-qd[22]*qd[22]*s->dpt[1][40]
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
  OR_10_170 = OM_10_227*RL_10_370-OM_10_327*RL_10_270;
  OR_10_270 = -(OM_10_127*RL_10_370-OM_10_327*RL_10_170);
  OR_10_370 = OM_10_127*RL_10_270-OM_10_227*RL_10_170;
  Apqp_10_170 = -(OM_10_225*OM_10_225*s->dpt[1][43]*C22p25-OM_10_227*OR_10_370+OM_10_327*OR_10_270-Ompqp_10_227*
 RL_10_370+Ompqp_10_327*RL_10_270);
//
  OM_13_145 = qd[45]*C41;
  OM_13_345 = -qd[45]*S41;
  OM_13_146 = OM_13_145+RO_13_445*qd[46];
  OM_13_246 = qd[41]+qd[46]*C45;
  OM_13_346 = OM_13_345+RO_13_645*qd[46];
  Ompqp_13_146 = -(qd[41]*qd[45]*S41+qd[46]*(OM_13_345*C45-RO_13_645*qd[41]));
  Ompqp_13_246 = -qd[45]*qd[46]*S45;
  Ompqp_13_346 = -(qd[41]*qd[45]*C41-qd[46]*(OM_13_145*C45-RO_13_445*qd[41]));
  OR_13_173 = OM_13_246*RL_13_373-OM_13_346*RL_13_273;
  OR_13_273 = -(OM_13_146*RL_13_373-OM_13_346*RL_13_173);
  OR_13_373 = OM_13_146*RL_13_273-OM_13_246*RL_13_173;
  Apqp_13_173 = OM_13_246*OR_13_373-OM_13_346*OR_13_273+Ompqp_13_246*RL_13_373-Ompqp_13_346*RL_13_273-qd[41]*qd[41]*
 s->dpt[1][61]*C41;
//
  OM_14_242 = qd[41]+qd[42];
  OM_14_143 = qd[43]*C41p42;
  OM_14_343 = -qd[43]*S41p42;
  OM_14_144 = OM_14_143+RO_14_443*qd[44];
  OM_14_244 = OM_14_242+qd[44]*C43;
  OM_14_344 = OM_14_343+RO_14_643*qd[44];
  Ompqp_14_144 = -(OM_14_242*qd[43]*S41p42-qd[44]*(OM_14_242*RO_14_643-OM_14_343*C43));
  Ompqp_14_244 = -qd[43]*qd[44]*S43;
  Ompqp_14_344 = -(OM_14_242*qd[43]*C41p42-qd[44]*(OM_14_143*C43-OM_14_242*RO_14_443));
  OR_14_174 = OM_14_244*RL_14_374-OM_14_344*RL_14_274;
  OR_14_274 = -(OM_14_144*RL_14_374-OM_14_344*RL_14_174);
  OR_14_374 = OM_14_144*RL_14_274-OM_14_244*RL_14_174;
  Apqp_14_174 = -(OM_14_242*OM_14_242*s->dpt[1][62]*C41p42-OM_14_244*OR_14_374+OM_14_344*OR_14_274-Ompqp_14_244*
 RL_14_374+Ompqp_14_344*RL_14_274);
//
  OR_16_176 = OM_2_211*RL_16_376-OM_2_311*RL_16_276;
  OR_16_276 = -(OM_2_111*RL_16_376-OM_2_311*RL_16_176);
  OR_16_376 = OM_2_111*RL_16_276-OM_2_211*RL_16_176;
//
  OM_17_230 = -qd[30]*S29;
  OM_17_330 = qd[30]*C29;
  Ompqp_17_230 = -qd[29]*qd[30]*C29;
  Ompqp_17_330 = -qd[29]*qd[30]*S29;
  OR_17_177 = OM_17_230*RL_17_377-OM_17_330*RL_17_277;
  OR_17_277 = OM_17_330*RL_17_177-RL_17_377*qd[29];
  OR_17_377 = -(OM_17_230*RL_17_177-RL_17_277*qd[29]);
//
  OR_18_178 = OM_0_216*RL_18_378-OM_0_316*RL_18_278;
  OR_18_278 = -(OM_0_116*RL_18_378-OM_0_316*RL_18_178);
  OR_18_378 = OM_0_116*RL_18_278-OM_0_216*RL_18_178;
//
  OM_19_232 = -qd[32]*S31;
  OM_19_332 = qd[32]*C31;
  Ompqp_19_232 = -qd[31]*qd[32]*C31;
  Ompqp_19_332 = -qd[31]*qd[32]*S31;
  OR_19_179 = OM_19_232*RL_19_379-OM_19_332*RL_19_279;
  OR_19_279 = OM_19_332*RL_19_179-RL_19_379*qd[31];
  OR_19_379 = -(OM_19_232*RL_19_179-RL_19_279*qd[31]);

// = = Block_0_2_0_0_0_1 = = 
 
// Constraints Quadratic Terms 

//
  jdqd2 = Apqp_0_214-OM_0_116*OR_0_360+OM_0_316*OR_0_160-Ompqp_0_116*RL_0_360+Ompqp_0_316*RL_0_160+qd[18]*qd[18]*
 s->dpt[2][32]*C18;
  jdqd3 = Apqp_0_314+OM_0_116*OR_0_260-OM_0_216*OR_0_160+Ompqp_0_116*RL_0_260-Ompqp_0_216*RL_0_160+qd[18]*qd[18]*
 s->dpt[2][32]*S18;
//
  jdqd5 = Apqp_2_29-OM_2_111*OR_2_362+OM_2_311*OR_2_162-Ompqp_2_111*RL_2_362+Ompqp_2_311*RL_2_162+qd[19]*qd[19]*
 s->dpt[2][34]*C19;
  jdqd6 = Apqp_2_39+OM_2_111*OR_2_262-OM_2_211*OR_2_162+Ompqp_2_111*RL_2_262-Ompqp_2_211*RL_2_162+qd[19]*qd[19]*
 s->dpt[2][34]*S19;
//
  jdqd8 = -(OM_5_139*OM_5_139*s->dpt[3][58]*S38p39+qd[20]*qd[20]*s->dpt[2][36]*C20-qd[38]*qd[38]*s->dpt[2][55]*C38);
  jdqd9 = OM_5_139*OM_5_139*s->dpt[3][58]*C38p39-qd[20]*qd[20]*s->dpt[2][36]*S20+qd[38]*qd[38]*s->dpt[2][55]*S38;
//
  jdqd11 = -(OM_7_136*OM_7_136*s->dpt[3][52]*S35p36+qd[21]*qd[21]*s->dpt[2][38]*C21-qd[35]*qd[35]*s->dpt[2][50]*C35);
  jdqd12 = OM_7_136*OM_7_136*s->dpt[3][52]*C35p36-qd[21]*qd[21]*s->dpt[2][38]*S21+qd[35]*qd[35]*s->dpt[2][50]*S35;
//
  jdqd14 = -(OM_8_124*OR_8_368-OM_8_324*OR_8_168+Ompqp_8_124*RL_8_368-Ompqp_8_324*RL_8_168-qd[13]*qd[13]*s->dpt[2][27]*
 C13);
  jdqd15 = OM_8_124*OR_8_268-OM_8_224*OR_8_168+Ompqp_8_124*RL_8_268-Ompqp_8_224*RL_8_168+qd[13]*qd[13]*s->dpt[2][27]*S13
 +qd[22]*qd[22]*s->dpt[1][40]*S22;
//
  jdqd17 = -(OM_10_127*OR_10_370-OM_10_327*OR_10_170+Ompqp_10_127*RL_10_370-Ompqp_10_327*RL_10_170-qd[8]*qd[8]*
 s->dpt[2][20]*C8);
  jdqd18 = OM_10_127*OR_10_270+OM_10_225*OM_10_225*s->dpt[1][43]*S22p25-OM_10_227*OR_10_170+Ompqp_10_127*RL_10_270-
 Ompqp_10_227*RL_10_170+qd[8]*qd[8]*s->dpt[2][20]*S8;
//
  jdqd20 = OM_13_146*OR_13_373-OM_13_346*OR_13_173+Ompqp_13_146*RL_13_373-Ompqp_13_346*RL_13_173-qd[35]*qd[35]*
 s->dpt[2][51]*C35;
  jdqd21 = -(OM_13_146*OR_13_273-OM_13_246*OR_13_173+Ompqp_13_146*RL_13_273-Ompqp_13_246*RL_13_173+qd[35]*qd[35]*
 s->dpt[2][51]*S35+qd[41]*qd[41]*s->dpt[1][61]*S41);
//
  jdqd23 = -(OM_14_144*OR_14_374-OM_14_344*OR_14_174+Ompqp_14_144*RL_14_374-Ompqp_14_344*RL_14_174-qd[38]*qd[38]*
 s->dpt[2][56]*C38);
  jdqd24 = OM_14_144*OR_14_274+OM_14_242*OM_14_242*s->dpt[1][62]*S41p42-OM_14_244*OR_14_174+Ompqp_14_144*RL_14_274-
 Ompqp_14_244*RL_14_174+qd[38]*qd[38]*s->dpt[2][56]*S38;
//
  jdqd25 = OM_2_211*OR_16_376-OM_2_311*OR_16_276+Ompqp_2_211*RL_16_376-Ompqp_2_311*RL_16_276-(OM_17_230*OR_17_377-
 OM_17_330*OR_17_277+Ompqp_17_230*RL_17_377-Ompqp_17_330*RL_17_277);
  jdqd26 = Apqp_2_29-OM_17_330*OR_17_177-OM_2_111*OR_16_376+OM_2_311*OR_16_176+OR_17_377*qd[29]-Ompqp_17_330*RL_17_177-
 Ompqp_2_111*RL_16_376+Ompqp_2_311*RL_16_176;
  jdqd27 = Apqp_2_39+OM_17_230*OR_17_177+OM_2_111*OR_16_276-OM_2_211*OR_16_176-OR_17_277*qd[29]+Ompqp_17_230*RL_17_177+
 Ompqp_2_111*RL_16_276-Ompqp_2_211*RL_16_176;
//
  jdqd28 = OM_0_216*OR_18_378-OM_0_316*OR_18_278-OM_19_232*OR_19_379+OM_19_332*OR_19_279+Ompqp_0_216*RL_18_378-
 Ompqp_0_316*RL_18_278-Ompqp_19_232*RL_19_379+Ompqp_19_332*RL_19_279;
  jdqd29 = Apqp_0_214-OM_0_116*OR_18_378+OM_0_316*OR_18_178-OM_19_332*OR_19_179+OR_19_379*qd[31]-Ompqp_0_116*RL_18_378+
 Ompqp_0_316*RL_18_178-Ompqp_19_332*RL_19_179;
  jdqd30 = Apqp_0_314+OM_0_116*OR_18_278-OM_0_216*OR_18_178+OM_19_232*OR_19_179-OR_19_279*qd[31]+Ompqp_0_116*RL_18_278-
 Ompqp_0_216*RL_18_178+Ompqp_19_232*RL_19_179;

// = = Block_0_3_0_0_0_0 = = 
 
// Symbolic Outputs  

  Jdqd[1] = Apqp_0_160;
  Jdqd[2] = jdqd2;
  Jdqd[3] = jdqd3;
  Jdqd[4] = Apqp_2_162;
  Jdqd[5] = jdqd5;
  Jdqd[6] = jdqd6;
  Jdqd[7] = jdqd8;
  Jdqd[8] = jdqd9;
  Jdqd[9] = jdqd11;
  Jdqd[10] = jdqd12;
  Jdqd[11] = Apqp_8_168;
  Jdqd[12] = jdqd14;
  Jdqd[13] = jdqd15;
  Jdqd[14] = Apqp_10_170;
  Jdqd[15] = jdqd17;
  Jdqd[16] = jdqd18;
  Jdqd[17] = -Apqp_13_173;
  Jdqd[18] = jdqd20;
  Jdqd[19] = jdqd21;
  Jdqd[20] = Apqp_14_174;
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
 

