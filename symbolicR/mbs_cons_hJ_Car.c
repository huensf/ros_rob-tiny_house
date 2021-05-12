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
//	==> Generation Date : Sat Jan  9 22:36:42 2021
//
//	==> Project name : Car
//	==> using XML input file 
//
//	==> Number of joints : 53
//
//	==> Function : F 8 : Constraints Vector (h) and Jacobian Matrix (Jac) 
//	==> Flops complexity : 632
//
//	==> Generation Time :  0.030 seconds
//	==> Post-Processing :  0.010 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_cons_hJ(double *h,double **Jac,
MbsData *s, double tsim)

// double h[28];
// double Jac[28][53];
{ 
 
#include "mbs_cons_hJ_Car.h" 
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
  RL_2_29 = s->dpt[2][18]*C8;
  RL_2_39 = s->dpt[2][18]*S8;
  PO_2_29 = RL_2_29+s->dpt[2][1];
  PO_2_39 = RL_2_39+s->dpt[3][1];
  RL_2_156 = RO_2_111*s->dpt[1][21]+RO_2_710*s->dpt[3][21];
  RL_2_256 = RO_2_211*s->dpt[1][21]+RO_2_810*s->dpt[3][21];
  RL_2_356 = RO_2_311*s->dpt[1][21]+RO_2_910*s->dpt[3][21];
  PO_2_156 = RL_2_156+s->dpt[1][1];
  JT_2_256_8 = -(RL_2_356+RL_2_39);
  JT_2_356_8 = RL_2_256+RL_2_29;
  JT_2_156_9 = -(RL_2_256*S8-RL_2_356*C8);
  JT_2_256_9 = RL_2_156*S8;
  JT_2_356_9 = -RL_2_156*C8;
  JT_2_156_10 = -(RL_2_256*RO_2_39-RL_2_356*RO_2_29);
  JT_2_256_10 = RL_2_156*RO_2_39-RL_2_356*C9;
  JT_2_356_10 = -(RL_2_156*RO_2_29-RL_2_256*C9);
  JT_2_156_11 = -(RL_2_256*RO_2_910-RL_2_356*RO_2_810);
  JT_2_256_11 = RL_2_156*RO_2_910-RL_2_356*RO_2_710;
  JT_2_356_11 = -(RL_2_156*RO_2_810-RL_2_256*RO_2_710);
//
  RL_11_265 = s->dpt[2][19]*C8;
  RL_11_365 = s->dpt[2][19]*S8;
//
  RL_16_170 = RO_2_111*s->dpt[1][20]+RO_2_710*s->dpt[3][20];
  RL_16_270 = RO_2_211*s->dpt[1][20]+RO_2_810*s->dpt[3][20];
  RL_16_370 = RO_2_311*s->dpt[1][20]+RO_2_910*s->dpt[3][20];
  JT_16_270_8 = -(RL_16_370+RL_2_39);
  JT_16_370_8 = RL_16_270+RL_2_29;
  JT_16_170_9 = -(RL_16_270*S8-RL_16_370*C8);
  JT_16_270_9 = RL_16_170*S8;
  JT_16_370_9 = -RL_16_170*C8;
  JT_16_170_10 = -(RL_16_270*RO_2_39-RL_16_370*RO_2_29);
  JT_16_270_10 = RL_16_170*RO_2_39-RL_16_370*C9;
  JT_16_370_10 = -(RL_16_170*RO_2_29-RL_16_270*C9);
  JT_16_170_11 = -(RL_16_270*RO_2_910-RL_16_370*RO_2_810);
  JT_16_270_11 = RL_16_170*RO_2_910-RL_16_370*RO_2_710;
  JT_16_370_11 = -(RL_16_170*RO_2_810-RL_16_270*RO_2_710);

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
  RL_0_214 = s->dpt[2][24]*C13;
  RL_0_314 = s->dpt[2][24]*S13;
  PO_0_214 = RL_0_214+s->dpt[2][2];
  PO_0_314 = RL_0_314+s->dpt[3][2];
  RL_0_154 = RO_0_116*s->dpt[1][26]+RO_0_715*s->dpt[3][26];
  RL_0_254 = RO_0_216*s->dpt[1][26]+RO_0_815*s->dpt[3][26];
  RL_0_354 = RO_0_316*s->dpt[1][26]+RO_0_915*s->dpt[3][26];
  PO_0_154 = RL_0_154+s->dpt[1][2];
  JT_0_254_13 = -(RL_0_314+RL_0_354);
  JT_0_354_13 = RL_0_214+RL_0_254;
  JT_0_154_14 = -(RL_0_254*S13-RL_0_354*C13);
  JT_0_254_14 = RL_0_154*S13;
  JT_0_354_14 = -RL_0_154*C13;
  JT_0_154_15 = -(RL_0_254*RO_0_314-RL_0_354*RO_0_214);
  JT_0_254_15 = RL_0_154*RO_0_314-RL_0_354*C14;
  JT_0_354_15 = -(RL_0_154*RO_0_214-RL_0_254*C14);
  JT_0_154_16 = -(RL_0_254*RO_0_915-RL_0_354*RO_0_815);
  JT_0_254_16 = RL_0_154*RO_0_915-RL_0_354*RO_0_715;
  JT_0_354_16 = -(RL_0_154*RO_0_815-RL_0_254*RO_0_715);
//
  RL_9_263 = s->dpt[2][25]*C13;
  RL_9_363 = s->dpt[2][25]*S13;
//
  RL_18_172 = RO_0_116*s->dpt[1][28]+RO_0_715*s->dpt[3][28];
  RL_18_272 = RO_0_216*s->dpt[1][28]+RO_0_815*s->dpt[3][28];
  RL_18_372 = RO_0_316*s->dpt[1][28]+RO_0_915*s->dpt[3][28];
  JT_18_272_13 = -(RL_0_314+RL_18_372);
  JT_18_372_13 = RL_0_214+RL_18_272;
  JT_18_172_14 = -(RL_18_272*S13-RL_18_372*C13);
  JT_18_272_14 = RL_18_172*S13;
  JT_18_372_14 = -RL_18_172*C13;
  JT_18_172_15 = -(RL_18_272*RO_0_314-RL_18_372*RO_0_214);
  JT_18_272_15 = RL_18_172*RO_0_314-RL_18_372*C14;
  JT_18_372_15 = -(RL_18_172*RO_0_214-RL_18_272*C14);
  JT_18_172_16 = -(RL_18_272*RO_0_915-RL_18_372*RO_0_815);
  JT_18_272_16 = RL_18_172*RO_0_915-RL_18_372*RO_0_715;
  JT_18_372_16 = -(RL_18_172*RO_0_815-RL_18_272*RO_0_715);

// = = Block_0_1_0_0_0_5 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_1_255 = s->dpt[2][30]*C18;
  RL_1_355 = s->dpt[2][30]*S18;

// = = Block_0_1_0_0_0_6 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_3_257 = s->dpt[2][32]*C19;
  RL_3_357 = s->dpt[2][32]*S19;

// = = Block_0_1_0_0_0_7 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_4_258 = s->dpt[2][34]*C20;
  RL_4_358 = s->dpt[2][34]*S20;

// = = Block_0_1_0_0_0_8 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_6_260 = s->dpt[2][36]*C21;
  RL_6_360 = s->dpt[2][36]*S21;

// = = Block_0_1_0_0_0_10 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_8_423 = S22*S23;
  RO_8_623 = C22*S23;
  RL_8_123 = s->dpt[1][38]*C22;
  RL_8_323 = -s->dpt[1][38]*S22;
  RL_8_162 = s->dpt[3][40]*(C22*S24+S22*C23*C24);
  RL_8_262 = -s->dpt[3][40]*S23*C24;
  RL_8_362 = s->dpt[3][40]*(C22*C23*C24-S22*S24);
  JT_8_162_22 = RL_8_323+RL_8_362;
  JT_8_362_22 = -(RL_8_123+RL_8_162);
  JT_8_162_23 = RL_8_262*S22;
  JT_8_262_23 = -(RL_8_162*S22+RL_8_362*C22);
  JT_8_362_23 = RL_8_262*C22;
  JT_8_162_24 = -(RL_8_262*RO_8_623-RL_8_362*C23);
  JT_8_262_24 = RL_8_162*RO_8_623-RL_8_362*RO_8_423;
  JT_8_362_24 = -(RL_8_162*C23-RL_8_262*RO_8_423);

// = = Block_0_1_0_0_0_11 = = 
 
// Trigonometric Variables  

//
  S22p25 = C22*S25+S22*C25;
  C22p25 = C22*C25-S22*S25;
 
// Constraints and Constraints Jacobian 

  RO_10_426 = S22p25*S26;
  RO_10_626 = C22p25*S26;
  RL_10_126 = s->dpt[1][41]*C22p25;
  RL_10_326 = -s->dpt[1][41]*S22p25;
  RL_10_164 = s->dpt[3][42]*(C22p25*S27+S22p25*C26*C27);
  RL_10_264 = -s->dpt[3][42]*S26*C27;
  RL_10_364 = s->dpt[3][42]*(C22p25*C26*C27-S22p25*S27);
  JT_10_164_22 = RL_10_326+RL_10_364;
  JT_10_364_22 = -(RL_10_126+RL_10_164);
  JT_10_164_25 = RL_10_326+RL_10_364;
  JT_10_364_25 = -(RL_10_126+RL_10_164);
  JT_10_164_26 = RL_10_264*S22p25;
  JT_10_264_26 = -(RL_10_164*S22p25+RL_10_364*C22p25);
  JT_10_364_26 = RL_10_264*C22p25;
  JT_10_164_27 = -(RL_10_264*RO_10_626-RL_10_364*C26);
  JT_10_264_27 = RL_10_164*RO_10_626-RL_10_364*RO_10_426;
  JT_10_364_27 = -(RL_10_164*C26-RL_10_264*RO_10_426);

// = = Block_0_1_0_0_0_13 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_17_171 = s->dpt[1][45]*C30-s->dpt[2][45]*S30;
  RL_17_271 = C29*(s->dpt[1][45]*S30+s->dpt[2][45]*C30);
  RL_17_371 = S29*(s->dpt[1][45]*S30+s->dpt[2][45]*C30);
  JT_17_171_30 = -(s->dpt[1][45]*S30+s->dpt[2][45]*C30);
  JT_17_271_30 = RL_17_171*C29;
  JT_17_371_30 = RL_17_171*S29;

// = = Block_0_1_0_0_0_14 = = 
 
// Constraints and Constraints Jacobian 

//
  RL_19_173 = s->dpt[1][46]*C32-s->dpt[2][46]*S32;
  RL_19_273 = C31*(s->dpt[1][46]*S32+s->dpt[2][46]*C32);
  RL_19_373 = S31*(s->dpt[1][46]*S32+s->dpt[2][46]*C32);
  JT_19_173_32 = -(s->dpt[1][46]*S32+s->dpt[2][46]*C32);
  JT_19_273_32 = RL_19_173*C31;
  JT_19_373_32 = RL_19_173*S31;

// = = Block_0_1_0_0_0_15 = = 
 
// Trigonometric Variables  

//
  S33p34 = C33*S34+S33*C34;
  C33p34 = C33*C34-S33*S34;
 
// Constraints and Constraints Jacobian 

  RL_7_234 = s->dpt[2][47]*C33;
  RL_7_334 = s->dpt[2][47]*S33;
  RL_7_261 = -s->dpt[3][49]*S33p34;
  RL_7_361 = s->dpt[3][49]*C33p34;
  JT_7_261_33 = -(RL_7_334+RL_7_361);
  JT_7_361_33 = RL_7_234+RL_7_261;
//
  RL_12_266 = s->dpt[2][48]*C33;
  RL_12_366 = s->dpt[2][48]*S33;

// = = Block_0_1_0_0_0_16 = = 
 
// Trigonometric Variables  

//
  S36p37 = C36*S37+S36*C37;
  C36p37 = C36*C37-S36*S37;
 
// Constraints and Constraints Jacobian 

  RL_5_237 = s->dpt[2][52]*C36;
  RL_5_337 = s->dpt[2][52]*S36;
  RL_5_259 = -s->dpt[3][55]*S36p37;
  RL_5_359 = s->dpt[3][55]*C36p37;
  JT_5_259_36 = -(RL_5_337+RL_5_359);
  JT_5_359_36 = RL_5_237+RL_5_259;
//
  RL_15_269 = s->dpt[2][53]*C36;
  RL_15_369 = s->dpt[2][53]*S36;

// = = Block_0_1_0_0_0_18 = = 
 
// Trigonometric Variables  

//
  S39p40 = C39*S40+S39*C40;
  C39p40 = C39*C40-S39*S40;
 
// Constraints and Constraints Jacobian 

  RO_14_441 = S39p40*S41;
  RO_14_641 = C39p40*S41;
  RL_14_141 = s->dpt[1][59]*C39p40;
  RL_14_341 = -s->dpt[1][59]*S39p40;
  RL_14_168 = s->dpt[3][60]*(C39p40*S42+S39p40*C41*C42);
  RL_14_268 = -s->dpt[3][60]*S41*C42;
  RL_14_368 = s->dpt[3][60]*(C39p40*C41*C42-S39p40*S42);
  JT_14_168_39 = RL_14_341+RL_14_368;
  JT_14_368_39 = -(RL_14_141+RL_14_168);
  JT_14_168_40 = RL_14_341+RL_14_368;
  JT_14_368_40 = -(RL_14_141+RL_14_168);
  JT_14_168_41 = RL_14_268*S39p40;
  JT_14_268_41 = -(RL_14_168*S39p40+RL_14_368*C39p40);
  JT_14_368_41 = RL_14_268*C39p40;
  JT_14_168_42 = -(RL_14_268*RO_14_641-RL_14_368*C41);
  JT_14_268_42 = RL_14_168*RO_14_641-RL_14_368*RO_14_441;
  JT_14_368_42 = -(RL_14_168*C41-RL_14_268*RO_14_441);

// = = Block_0_1_0_0_0_19 = = 
 
// Constraints and Constraints Jacobian 

//
  RO_13_443 = S39*S43;
  RO_13_643 = C39*S43;
  RL_13_143 = s->dpt[1][58]*C39;
  RL_13_343 = -s->dpt[1][58]*S39;
  RL_13_167 = s->dpt[3][61]*(C39*S44+S39*C43*C44);
  RL_13_267 = -s->dpt[3][61]*S43*C44;
  RL_13_367 = s->dpt[3][61]*(C39*C43*C44-S39*S44);
  JT_13_167_39 = RL_13_343+RL_13_367;
  JT_13_367_39 = -(RL_13_143+RL_13_167);
  JT_13_167_43 = RL_13_267*S39;
  JT_13_267_43 = -(RL_13_167*S39+RL_13_367*C39);
  JT_13_367_43 = RL_13_267*C39;
  JT_13_167_44 = -(RL_13_267*RO_13_643-RL_13_367*C43);
  JT_13_267_44 = RL_13_167*RO_13_643-RL_13_367*RO_13_443;
  JT_13_367_44 = -(RL_13_167*C43-RL_13_267*RO_13_443);

// = = Block_0_1_0_0_1_0 = = 
 
// Constraints and Constraints Jacobian 

//
  h_2 = PO_0_214+RL_0_254-RL_1_255-s->dpt[2][3];
  h_3 = PO_0_314+RL_0_354-RL_1_355;
//
  h_5 = PO_2_29+RL_2_256-RL_3_257-s->dpt[2][4];
  h_6 = PO_2_39+RL_2_356-RL_3_357;
//
  h_8 = RL_4_258-RL_5_237-RL_5_259-s->dpt[2][13]+s->dpt[2][5];
  h_9 = RL_4_358-RL_5_337-RL_5_359-s->dpt[3][13];
//
  h_11 = RL_6_260-RL_7_234-RL_7_261-s->dpt[2][12]+s->dpt[2][7];
  h_12 = RL_6_360-RL_7_334-RL_7_361-s->dpt[3][12];
//
  h_13 = RL_8_123+RL_8_162-s->dpt[1][25]-s->dpt[1][2]+s->dpt[1][8];
  h_14 = RL_8_262-RL_9_263-s->dpt[2][2]+s->dpt[2][38];
  h_15 = RL_8_323+RL_8_362-RL_9_363-s->dpt[3][2]+s->dpt[3][8];
//
  h_16 = RL_10_126+RL_10_164-s->dpt[1][19]-s->dpt[1][1]+s->dpt[1][8];
  h_17 = RL_10_264-RL_11_265-s->dpt[2][1]+s->dpt[2][41];
  h_18 = RL_10_326+RL_10_364-RL_11_365-s->dpt[3][1]+s->dpt[3][8];
//
  h_19 = s->dpt[1][12]+s->dpt[1][48]-(RL_13_143+RL_13_167+s->dpt[1][15]);
  h_20 = RL_12_266-RL_13_267+s->dpt[2][12]-s->dpt[2][58];
  h_21 = RL_12_366-RL_13_343-RL_13_367+s->dpt[3][12]-s->dpt[3][15];
//
  h_22 = RL_14_141+RL_14_168-s->dpt[1][13]+s->dpt[1][15]-s->dpt[1][53];
  h_23 = RL_14_268-RL_15_269-s->dpt[2][13]+s->dpt[2][59];
  h_24 = RL_14_341+RL_14_368-RL_15_369-s->dpt[3][13]+s->dpt[3][15];
//
  h_25 = RL_16_170-RL_17_171+s->dpt[1][1]-s->dpt[1][9];
  h_26 = PO_2_29+RL_16_270-RL_17_271-q[28]-s->dpt[2][43];
  h_27 = PO_2_39+RL_16_370-RL_17_371-s->dpt[3][9];
//
  h_28 = RL_18_172-RL_19_173+s->dpt[1][2]-s->dpt[1][9];
  h_29 = PO_0_214+RL_18_272-RL_19_273-q[28]-s->dpt[2][44];
  h_30 = PO_0_314+RL_18_372-RL_19_373-s->dpt[3][9];

// = = Block_0_3_0_0_0_0 = = 
 
// Symbolic Outputs  

  h[1] = PO_0_154;
  h[2] = h_2;
  h[3] = h_3;
  h[4] = PO_2_156;
  h[5] = h_5;
  h[6] = h_6;
  h[7] = h_8;
  h[8] = h_9;
  h[9] = h_11;
  h[10] = h_12;
  h[11] = h_13;
  h[12] = h_14;
  h[13] = h_15;
  h[14] = h_16;
  h[15] = h_17;
  h[16] = h_18;
  h[17] = h_19;
  h[18] = h_20;
  h[19] = h_21;
  h[20] = h_22;
  h[21] = h_23;
  h[22] = h_24;
  h[23] = h_25;
  h[24] = h_26;
  h[25] = h_27;
  h[26] = h_28;
  h[27] = h_29;
  h[28] = h_30;
  Jac[1][14] = JT_0_154_14;
  Jac[1][15] = JT_0_154_15;
  Jac[1][16] = JT_0_154_16;
  Jac[2][13] = JT_0_254_13;
  Jac[2][14] = JT_0_254_14;
  Jac[2][15] = JT_0_254_15;
  Jac[2][16] = JT_0_254_16;
  Jac[2][18] = RL_1_355;
  Jac[3][13] = JT_0_354_13;
  Jac[3][14] = JT_0_354_14;
  Jac[3][15] = JT_0_354_15;
  Jac[3][16] = JT_0_354_16;
  Jac[3][18] = -RL_1_255;
  Jac[4][9] = JT_2_156_9;
  Jac[4][10] = JT_2_156_10;
  Jac[4][11] = JT_2_156_11;
  Jac[5][8] = JT_2_256_8;
  Jac[5][9] = JT_2_256_9;
  Jac[5][10] = JT_2_256_10;
  Jac[5][11] = JT_2_256_11;
  Jac[5][19] = RL_3_357;
  Jac[6][8] = JT_2_356_8;
  Jac[6][9] = JT_2_356_9;
  Jac[6][10] = JT_2_356_10;
  Jac[6][11] = JT_2_356_11;
  Jac[6][19] = -RL_3_257;
  Jac[7][20] = -RL_4_358;
  Jac[7][36] = -JT_5_259_36;
  Jac[7][37] = RL_5_359;
  Jac[8][20] = RL_4_258;
  Jac[8][36] = -JT_5_359_36;
  Jac[8][37] = -RL_5_259;
  Jac[9][21] = -RL_6_360;
  Jac[9][33] = -JT_7_261_33;
  Jac[9][34] = RL_7_361;
  Jac[10][21] = RL_6_260;
  Jac[10][33] = -JT_7_361_33;
  Jac[10][34] = -RL_7_261;
  Jac[11][22] = JT_8_162_22;
  Jac[11][23] = JT_8_162_23;
  Jac[11][24] = JT_8_162_24;
  Jac[12][13] = RL_9_363;
  Jac[12][23] = JT_8_262_23;
  Jac[12][24] = JT_8_262_24;
  Jac[13][13] = -RL_9_263;
  Jac[13][22] = JT_8_362_22;
  Jac[13][23] = JT_8_362_23;
  Jac[13][24] = JT_8_362_24;
  Jac[14][22] = JT_10_164_22;
  Jac[14][25] = JT_10_164_25;
  Jac[14][26] = JT_10_164_26;
  Jac[14][27] = JT_10_164_27;
  Jac[15][8] = RL_11_365;
  Jac[15][26] = JT_10_264_26;
  Jac[15][27] = JT_10_264_27;
  Jac[16][8] = -RL_11_265;
  Jac[16][22] = JT_10_364_22;
  Jac[16][25] = JT_10_364_25;
  Jac[16][26] = JT_10_364_26;
  Jac[16][27] = JT_10_364_27;
  Jac[17][39] = -JT_13_167_39;
  Jac[17][43] = -JT_13_167_43;
  Jac[17][44] = -JT_13_167_44;
  Jac[18][33] = -RL_12_366;
  Jac[18][43] = -JT_13_267_43;
  Jac[18][44] = -JT_13_267_44;
  Jac[19][33] = RL_12_266;
  Jac[19][39] = -JT_13_367_39;
  Jac[19][43] = -JT_13_367_43;
  Jac[19][44] = -JT_13_367_44;
  Jac[20][39] = JT_14_168_39;
  Jac[20][40] = JT_14_168_40;
  Jac[20][41] = JT_14_168_41;
  Jac[20][42] = JT_14_168_42;
  Jac[21][36] = RL_15_369;
  Jac[21][41] = JT_14_268_41;
  Jac[21][42] = JT_14_268_42;
  Jac[22][36] = -RL_15_269;
  Jac[22][39] = JT_14_368_39;
  Jac[22][40] = JT_14_368_40;
  Jac[22][41] = JT_14_368_41;
  Jac[22][42] = JT_14_368_42;
  Jac[23][9] = JT_16_170_9;
  Jac[23][10] = JT_16_170_10;
  Jac[23][11] = JT_16_170_11;
  Jac[23][30] = -JT_17_171_30;
  Jac[24][8] = JT_16_270_8;
  Jac[24][9] = JT_16_270_9;
  Jac[24][10] = JT_16_270_10;
  Jac[24][11] = JT_16_270_11;
  Jac[24][28] = -(1.0);
  Jac[24][29] = RL_17_371;
  Jac[24][30] = -JT_17_271_30;
  Jac[25][8] = JT_16_370_8;
  Jac[25][9] = JT_16_370_9;
  Jac[25][10] = JT_16_370_10;
  Jac[25][11] = JT_16_370_11;
  Jac[25][29] = -RL_17_271;
  Jac[25][30] = -JT_17_371_30;
  Jac[26][14] = JT_18_172_14;
  Jac[26][15] = JT_18_172_15;
  Jac[26][16] = JT_18_172_16;
  Jac[26][32] = -JT_19_173_32;
  Jac[27][13] = JT_18_272_13;
  Jac[27][14] = JT_18_272_14;
  Jac[27][15] = JT_18_272_15;
  Jac[27][16] = JT_18_272_16;
  Jac[27][28] = -(1.0);
  Jac[27][31] = RL_19_373;
  Jac[27][32] = -JT_19_273_32;
  Jac[28][13] = JT_18_372_13;
  Jac[28][14] = JT_18_372_14;
  Jac[28][15] = JT_18_372_15;
  Jac[28][16] = JT_18_372_16;
  Jac[28][31] = -RL_19_273;
  Jac[28][32] = -JT_19_373_32;

// ====== END Task 0 ====== 


}
 

