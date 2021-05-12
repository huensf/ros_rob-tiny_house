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
//	==> Generation Date : Sat Jan  9 22:36:44 2021
//
//	==> Project name : Car
//	==> using XML input file 
//
//	==> Number of joints : 53
//
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 3869
//
//	==> Generation Time :  0.070 seconds
//	==> Post-Processing :  0.070 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
#include "mbs_sensor.h"
 
void  mbs_sensor(MbsSensor *sens, 
              MbsData *s,
              int isens)
{ 
 
#include "mbs_sensor_Car.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 
 
// Sensor Kinematics 



// = = Block_0_0_0_0_0_1 = = 
 
// Trigonometric Variables  

  C4 = cos(q[4]);
  S4 = sin(q[4]);
  C5 = cos(q[5]);
  S5 = sin(q[5]);
  C6 = cos(q[6]);
  S6 = sin(q[6]);

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
  C12 = cos(q[12]);
  S12 = sin(q[12]);

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
  C17 = cos(q[17]);
  S17 = sin(q[17]);

// = = Block_0_0_0_0_0_15 = = 
 
// Trigonometric Variables  

  C33 = cos(q[33]);
  S33 = sin(q[33]);
  C34 = cos(q[34]);
  S34 = sin(q[34]);
  C35 = cos(q[35]);
  S35 = sin(q[35]);

// = = Block_0_0_0_0_0_16 = = 
 
// Trigonometric Variables  

  C36 = cos(q[36]);
  S36 = sin(q[36]);
  C37 = cos(q[37]);
  S37 = sin(q[37]);
  C38 = cos(q[38]);
  S38 = sin(q[38]);

// = = Block_0_0_0_0_0_20 = = 
 
// Trigonometric Variables  

  C45 = cos(q[45]);
  S45 = sin(q[45]);
  C46 = cos(q[46]);
  S46 = sin(q[46]);
  C47 = cos(q[47]);
  S47 = sin(q[47]);

// = = Block_0_0_0_0_0_21 = = 
 
// Trigonometric Variables  

  C49 = cos(q[49]);
  S49 = sin(q[49]);

// = = Block_0_0_0_0_0_22 = = 
 
// Trigonometric Variables  

  C50 = cos(q[50]);
  S50 = sin(q[50]);

// = = Block_0_0_0_0_0_23 = = 
 
// Trigonometric Variables  

  C51 = cos(q[51]);
  S51 = sin(q[51]);

// = = Block_0_0_0_0_0_24 = = 
 
// Trigonometric Variables  

  C52 = cos(q[52]);
  S52 = sin(q[52]);

// ====== END Task 0 ====== 

// ===== BEGIN task 1 ===== 
 
switch(isens)
{
 
// 
break;
case 1:
 


// = = Block_1_0_0_1_0_1 = = 
 
// Sensor Kinematics 


    ROcp0_45 = -S4*C5;
    ROcp0_55 = C4*C5;
    ROcp0_75 = S4*S5;
    ROcp0_85 = -C4*S5;
    ROcp0_16 = -(ROcp0_75*S6-C4*C6);
    ROcp0_26 = -(ROcp0_85*S6-S4*C6);
    ROcp0_36 = -C5*S6;
    ROcp0_76 = ROcp0_75*C6+C4*S6;
    ROcp0_86 = ROcp0_85*C6+S4*S6;
    ROcp0_96 = C5*C6;
    OMcp0_15 = qd[5]*C4;
    OMcp0_25 = qd[5]*S4;
    OMcp0_16 = OMcp0_15+ROcp0_45*qd[6];
    OMcp0_26 = OMcp0_25+ROcp0_55*qd[6];
    OMcp0_36 = qd[4]+qd[6]*S5;
    OPcp0_16 = ROcp0_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp0_25*S5-ROcp0_55*qd[4]);
    OPcp0_26 = ROcp0_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp0_15*S5-ROcp0_45*qd[4]);
    OPcp0_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;
    RLcp0_182 = ROcp0_16*s->dpt[1][16]+ROcp0_76*s->dpt[3][16];
    RLcp0_282 = ROcp0_26*s->dpt[1][16]+ROcp0_86*s->dpt[3][16];
    RLcp0_382 = ROcp0_36*s->dpt[1][16]+ROcp0_96*s->dpt[3][16];
    POcp0_182 = RLcp0_182+q[1];
    POcp0_282 = RLcp0_282+q[2];
    POcp0_382 = RLcp0_382+q[3];
    JTcp0_182_5 = RLcp0_382*S4;
    JTcp0_282_5 = -RLcp0_382*C4;
    JTcp0_382_5 = -(RLcp0_182*S4-RLcp0_282*C4);
    JTcp0_182_6 = -(RLcp0_282*S5-RLcp0_382*ROcp0_55);
    JTcp0_282_6 = RLcp0_182*S5-RLcp0_382*ROcp0_45;
    JTcp0_382_6 = -(RLcp0_182*ROcp0_55-RLcp0_282*ROcp0_45);
    ORcp0_182 = OMcp0_26*RLcp0_382-OMcp0_36*RLcp0_282;
    ORcp0_282 = -(OMcp0_16*RLcp0_382-OMcp0_36*RLcp0_182);
    ORcp0_382 = OMcp0_16*RLcp0_282-OMcp0_26*RLcp0_182;
    VIcp0_182 = ORcp0_182+qd[1];
    VIcp0_282 = ORcp0_282+qd[2];
    VIcp0_382 = ORcp0_382+qd[3];
    ACcp0_182 = qdd[1]+OMcp0_26*ORcp0_382-OMcp0_36*ORcp0_282+OPcp0_26*RLcp0_382-OPcp0_36*RLcp0_282;
    ACcp0_282 = qdd[2]-OMcp0_16*ORcp0_382+OMcp0_36*ORcp0_182-OPcp0_16*RLcp0_382+OPcp0_36*RLcp0_182;
    ACcp0_382 = qdd[3]+OMcp0_16*ORcp0_282-OMcp0_26*ORcp0_182+OPcp0_16*RLcp0_282-OPcp0_26*RLcp0_182;

// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp0_182;
    sens->P[2] = POcp0_282;
    sens->P[3] = POcp0_382;
    sens->R[1][1] = ROcp0_16;
    sens->R[1][2] = ROcp0_26;
    sens->R[1][3] = ROcp0_36;
    sens->R[2][1] = ROcp0_45;
    sens->R[2][2] = ROcp0_55;
    sens->R[2][3] = S5;
    sens->R[3][1] = ROcp0_76;
    sens->R[3][2] = ROcp0_86;
    sens->R[3][3] = ROcp0_96;
    sens->V[1] = VIcp0_182;
    sens->V[2] = VIcp0_282;
    sens->V[3] = VIcp0_382;
    sens->OM[1] = OMcp0_16;
    sens->OM[2] = OMcp0_26;
    sens->OM[3] = OMcp0_36;
    sens->J[1][1] = (1.0);
    sens->J[1][4] = -RLcp0_282;
    sens->J[1][5] = JTcp0_182_5;
    sens->J[1][6] = JTcp0_182_6;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = RLcp0_182;
    sens->J[2][5] = JTcp0_282_5;
    sens->J[2][6] = JTcp0_282_6;
    sens->J[3][3] = (1.0);
    sens->J[3][5] = JTcp0_382_5;
    sens->J[3][6] = JTcp0_382_6;
    sens->J[4][5] = C4;
    sens->J[4][6] = ROcp0_45;
    sens->J[5][5] = S4;
    sens->J[5][6] = ROcp0_55;
    sens->J[6][4] = (1.0);
    sens->J[6][6] = S5;
    sens->A[1] = ACcp0_182;
    sens->A[2] = ACcp0_282;
    sens->A[3] = ACcp0_382;
    sens->OMP[1] = OPcp0_16;
    sens->OMP[2] = OPcp0_26;
    sens->OMP[3] = OPcp0_36;
 
// 
break;
case 2:
 


// = = Block_1_0_0_2_0_1 = = 
 
// Sensor Kinematics 


    ROcp1_45 = -S4*C5;
    ROcp1_55 = C4*C5;
    ROcp1_75 = S4*S5;
    ROcp1_85 = -C4*S5;
    ROcp1_16 = -(ROcp1_75*S6-C4*C6);
    ROcp1_26 = -(ROcp1_85*S6-S4*C6);
    ROcp1_36 = -C5*S6;
    ROcp1_76 = ROcp1_75*C6+C4*S6;
    ROcp1_86 = ROcp1_85*C6+S4*S6;
    ROcp1_96 = C5*C6;
    OMcp1_15 = qd[5]*C4;
    OMcp1_25 = qd[5]*S4;
    OMcp1_16 = OMcp1_15+ROcp1_45*qd[6];
    OMcp1_26 = OMcp1_25+ROcp1_55*qd[6];
    OMcp1_36 = qd[4]+qd[6]*S5;
    OPcp1_16 = ROcp1_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp1_25*S5-ROcp1_55*qd[4]);
    OPcp1_26 = ROcp1_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp1_15*S5-ROcp1_45*qd[4]);
    OPcp1_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_2_0_20 = = 
 
// Sensor Kinematics 


    ROcp1_145 = ROcp1_16*C45+ROcp1_45*S45;
    ROcp1_245 = ROcp1_26*C45+ROcp1_55*S45;
    ROcp1_345 = ROcp1_36*C45+S45*S5;
    ROcp1_445 = -(ROcp1_16*S45-ROcp1_45*C45);
    ROcp1_545 = -(ROcp1_26*S45-ROcp1_55*C45);
    ROcp1_645 = -(ROcp1_36*S45-C45*S5);
    ROcp1_446 = ROcp1_445*C46+ROcp1_76*S46;
    ROcp1_546 = ROcp1_545*C46+ROcp1_86*S46;
    ROcp1_646 = ROcp1_645*C46+ROcp1_96*S46;
    ROcp1_746 = -(ROcp1_445*S46-ROcp1_76*C46);
    ROcp1_846 = -(ROcp1_545*S46-ROcp1_86*C46);
    ROcp1_946 = -(ROcp1_645*S46-ROcp1_96*C46);
    ROcp1_147 = ROcp1_145*C47-ROcp1_746*S47;
    ROcp1_247 = ROcp1_245*C47-ROcp1_846*S47;
    ROcp1_347 = ROcp1_345*C47-ROcp1_946*S47;
    ROcp1_747 = ROcp1_145*S47+ROcp1_746*C47;
    ROcp1_847 = ROcp1_245*S47+ROcp1_846*C47;
    ROcp1_947 = ROcp1_345*S47+ROcp1_946*C47;
    RLcp1_145 = ROcp1_16*s->dpt[1][17]+ROcp1_76*s->dpt[3][17];
    RLcp1_245 = ROcp1_26*s->dpt[1][17]+ROcp1_86*s->dpt[3][17];
    RLcp1_345 = ROcp1_36*s->dpt[1][17]+ROcp1_96*s->dpt[3][17];
    OMcp1_145 = OMcp1_16+ROcp1_76*qd[45];
    OMcp1_245 = OMcp1_26+ROcp1_86*qd[45];
    OMcp1_345 = OMcp1_36+ROcp1_96*qd[45];
    ORcp1_145 = OMcp1_26*RLcp1_345-OMcp1_36*RLcp1_245;
    ORcp1_245 = -(OMcp1_16*RLcp1_345-OMcp1_36*RLcp1_145);
    ORcp1_345 = OMcp1_16*RLcp1_245-OMcp1_26*RLcp1_145;
    OMcp1_146 = OMcp1_145+ROcp1_145*qd[46];
    OMcp1_246 = OMcp1_245+ROcp1_245*qd[46];
    OMcp1_346 = OMcp1_345+ROcp1_345*qd[46];
    OMcp1_147 = OMcp1_146+ROcp1_446*qd[47];
    OMcp1_247 = OMcp1_246+ROcp1_546*qd[47];
    OMcp1_347 = OMcp1_346+ROcp1_646*qd[47];
    OPcp1_147 = OPcp1_16+ROcp1_145*qdd[46]+ROcp1_446*qdd[47]+ROcp1_76*qdd[45]+qd[45]*(OMcp1_26*ROcp1_96-OMcp1_36*ROcp1_86)
 +qd[46]*(OMcp1_245*ROcp1_345-OMcp1_345*ROcp1_245)+qd[47]*(OMcp1_246*ROcp1_646-OMcp1_346*ROcp1_546);
    OPcp1_247 = OPcp1_26+ROcp1_245*qdd[46]+ROcp1_546*qdd[47]+ROcp1_86*qdd[45]-qd[45]*(OMcp1_16*ROcp1_96-OMcp1_36*ROcp1_76)
 -qd[46]*(OMcp1_145*ROcp1_345-OMcp1_345*ROcp1_145)-qd[47]*(OMcp1_146*ROcp1_646-OMcp1_346*ROcp1_446);
    OPcp1_347 = OPcp1_36+ROcp1_345*qdd[46]+ROcp1_646*qdd[47]+ROcp1_96*qdd[45]+qd[45]*(OMcp1_16*ROcp1_86-OMcp1_26*ROcp1_76)
 +qd[46]*(OMcp1_145*ROcp1_245-OMcp1_245*ROcp1_145)+qd[47]*(OMcp1_146*ROcp1_546-OMcp1_246*ROcp1_446);
    RLcp1_148 = ROcp1_147*q[48];
    RLcp1_248 = ROcp1_247*q[48];
    RLcp1_348 = ROcp1_347*q[48];
    ORcp1_148 = OMcp1_247*RLcp1_348-OMcp1_347*RLcp1_248;
    ORcp1_248 = -(OMcp1_147*RLcp1_348-OMcp1_347*RLcp1_148);
    ORcp1_348 = OMcp1_147*RLcp1_248-OMcp1_247*RLcp1_148;
    RLcp1_183 = ROcp1_147*s->dpt[1][67]+ROcp1_747*s->dpt[3][67];
    RLcp1_283 = ROcp1_247*s->dpt[1][67]+ROcp1_847*s->dpt[3][67];
    RLcp1_383 = ROcp1_347*s->dpt[1][67]+ROcp1_947*s->dpt[3][67];
    POcp1_183 = RLcp1_145+RLcp1_148+RLcp1_183+q[1];
    POcp1_283 = RLcp1_245+RLcp1_248+RLcp1_283+q[2];
    POcp1_383 = RLcp1_345+RLcp1_348+RLcp1_383+q[3];
    JTcp1_183_4 = -(RLcp1_245+RLcp1_248+RLcp1_283);
    JTcp1_283_4 = RLcp1_145+RLcp1_148+RLcp1_183;
    JTcp1_183_5 = S4*(RLcp1_345+RLcp1_348+RLcp1_383);
    JTcp1_283_5 = -C4*(RLcp1_345+RLcp1_348+RLcp1_383);
    JTcp1_383_5 = C4*(RLcp1_245+RLcp1_248)-S4*(RLcp1_145+RLcp1_148)-RLcp1_183*S4+RLcp1_283*C4;
    JTcp1_183_6 = ROcp1_55*(RLcp1_345+RLcp1_348)-S5*(RLcp1_245+RLcp1_248)-RLcp1_283*S5+RLcp1_383*ROcp1_55;
    JTcp1_283_6 = RLcp1_183*S5-RLcp1_383*ROcp1_45-ROcp1_45*(RLcp1_345+RLcp1_348)+S5*(RLcp1_145+RLcp1_148);
    JTcp1_383_6 = ROcp1_45*(RLcp1_245+RLcp1_248)-ROcp1_55*(RLcp1_145+RLcp1_148)-RLcp1_183*ROcp1_55+RLcp1_283*ROcp1_45;
    JTcp1_183_7 = ROcp1_86*(RLcp1_348+RLcp1_383)-ROcp1_96*(RLcp1_248+RLcp1_283);
    JTcp1_283_7 = -(ROcp1_76*(RLcp1_348+RLcp1_383)-ROcp1_96*(RLcp1_148+RLcp1_183));
    JTcp1_383_7 = ROcp1_76*(RLcp1_248+RLcp1_283)-ROcp1_86*(RLcp1_148+RLcp1_183);
    JTcp1_183_8 = ROcp1_245*(RLcp1_348+RLcp1_383)-ROcp1_345*(RLcp1_248+RLcp1_283);
    JTcp1_283_8 = -(ROcp1_145*(RLcp1_348+RLcp1_383)-ROcp1_345*(RLcp1_148+RLcp1_183));
    JTcp1_383_8 = ROcp1_145*(RLcp1_248+RLcp1_283)-ROcp1_245*(RLcp1_148+RLcp1_183);
    JTcp1_183_9 = ROcp1_546*(RLcp1_348+RLcp1_383)-ROcp1_646*(RLcp1_248+RLcp1_283);
    JTcp1_283_9 = -(ROcp1_446*(RLcp1_348+RLcp1_383)-ROcp1_646*(RLcp1_148+RLcp1_183));
    JTcp1_383_9 = ROcp1_446*(RLcp1_248+RLcp1_283)-ROcp1_546*(RLcp1_148+RLcp1_183);
    ORcp1_183 = OMcp1_247*RLcp1_383-OMcp1_347*RLcp1_283;
    ORcp1_283 = -(OMcp1_147*RLcp1_383-OMcp1_347*RLcp1_183);
    ORcp1_383 = OMcp1_147*RLcp1_283-OMcp1_247*RLcp1_183;
    VIcp1_183 = ORcp1_145+ORcp1_148+ORcp1_183+qd[1]+ROcp1_147*qd[48];
    VIcp1_283 = ORcp1_245+ORcp1_248+ORcp1_283+qd[2]+ROcp1_247*qd[48];
    VIcp1_383 = ORcp1_345+ORcp1_348+ORcp1_383+qd[3]+ROcp1_347*qd[48];
    ACcp1_183 = qdd[1]+OMcp1_247*ORcp1_348+OMcp1_247*ORcp1_383+OMcp1_26*ORcp1_345-OMcp1_347*ORcp1_248-OMcp1_347*ORcp1_283-
 OMcp1_36*ORcp1_245+OPcp1_247*RLcp1_348+OPcp1_247*RLcp1_383+OPcp1_26*RLcp1_345-OPcp1_347*RLcp1_248-OPcp1_347*RLcp1_283-
 OPcp1_36*RLcp1_245+ROcp1_147*qdd[48]+(2.0)*qd[48]*(OMcp1_247*ROcp1_347-OMcp1_347*ROcp1_247);
    ACcp1_283 = qdd[2]-OMcp1_147*ORcp1_348-OMcp1_147*ORcp1_383-OMcp1_16*ORcp1_345+OMcp1_347*ORcp1_148+OMcp1_347*ORcp1_183+
 OMcp1_36*ORcp1_145-OPcp1_147*RLcp1_348-OPcp1_147*RLcp1_383-OPcp1_16*RLcp1_345+OPcp1_347*RLcp1_148+OPcp1_347*RLcp1_183+
 OPcp1_36*RLcp1_145+ROcp1_247*qdd[48]-(2.0)*qd[48]*(OMcp1_147*ROcp1_347-OMcp1_347*ROcp1_147);
    ACcp1_383 = qdd[3]+OMcp1_147*ORcp1_248+OMcp1_147*ORcp1_283+OMcp1_16*ORcp1_245-OMcp1_247*ORcp1_148-OMcp1_247*ORcp1_183-
 OMcp1_26*ORcp1_145+OPcp1_147*RLcp1_248+OPcp1_147*RLcp1_283+OPcp1_16*RLcp1_245-OPcp1_247*RLcp1_148-OPcp1_247*RLcp1_183-
 OPcp1_26*RLcp1_145+ROcp1_347*qdd[48]+(2.0)*qd[48]*(OMcp1_147*ROcp1_247-OMcp1_247*ROcp1_147);

// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp1_183;
    sens->P[2] = POcp1_283;
    sens->P[3] = POcp1_383;
    sens->R[1][1] = ROcp1_147;
    sens->R[1][2] = ROcp1_247;
    sens->R[1][3] = ROcp1_347;
    sens->R[2][1] = ROcp1_446;
    sens->R[2][2] = ROcp1_546;
    sens->R[2][3] = ROcp1_646;
    sens->R[3][1] = ROcp1_747;
    sens->R[3][2] = ROcp1_847;
    sens->R[3][3] = ROcp1_947;
    sens->V[1] = VIcp1_183;
    sens->V[2] = VIcp1_283;
    sens->V[3] = VIcp1_383;
    sens->OM[1] = OMcp1_147;
    sens->OM[2] = OMcp1_247;
    sens->OM[3] = OMcp1_347;
    sens->J[1][1] = (1.0);
    sens->J[1][4] = JTcp1_183_4;
    sens->J[1][5] = JTcp1_183_5;
    sens->J[1][6] = JTcp1_183_6;
    sens->J[1][45] = JTcp1_183_7;
    sens->J[1][46] = JTcp1_183_8;
    sens->J[1][47] = JTcp1_183_9;
    sens->J[1][48] = ROcp1_147;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp1_283_4;
    sens->J[2][5] = JTcp1_283_5;
    sens->J[2][6] = JTcp1_283_6;
    sens->J[2][45] = JTcp1_283_7;
    sens->J[2][46] = JTcp1_283_8;
    sens->J[2][47] = JTcp1_283_9;
    sens->J[2][48] = ROcp1_247;
    sens->J[3][3] = (1.0);
    sens->J[3][5] = JTcp1_383_5;
    sens->J[3][6] = JTcp1_383_6;
    sens->J[3][45] = JTcp1_383_7;
    sens->J[3][46] = JTcp1_383_8;
    sens->J[3][47] = JTcp1_383_9;
    sens->J[3][48] = ROcp1_347;
    sens->J[4][5] = C4;
    sens->J[4][6] = ROcp1_45;
    sens->J[4][45] = ROcp1_76;
    sens->J[4][46] = ROcp1_145;
    sens->J[4][47] = ROcp1_446;
    sens->J[5][5] = S4;
    sens->J[5][6] = ROcp1_55;
    sens->J[5][45] = ROcp1_86;
    sens->J[5][46] = ROcp1_245;
    sens->J[5][47] = ROcp1_546;
    sens->J[6][4] = (1.0);
    sens->J[6][6] = S5;
    sens->J[6][45] = ROcp1_96;
    sens->J[6][46] = ROcp1_345;
    sens->J[6][47] = ROcp1_646;
    sens->A[1] = ACcp1_183;
    sens->A[2] = ACcp1_283;
    sens->A[3] = ACcp1_383;
    sens->OMP[1] = OPcp1_147;
    sens->OMP[2] = OPcp1_247;
    sens->OMP[3] = OPcp1_347;
 
// 
break;
case 3:
 


// = = Block_1_0_0_3_0_1 = = 
 
// Sensor Kinematics 


    ROcp2_45 = -S4*C5;
    ROcp2_55 = C4*C5;
    ROcp2_75 = S4*S5;
    ROcp2_85 = -C4*S5;
    ROcp2_16 = -(ROcp2_75*S6-C4*C6);
    ROcp2_26 = -(ROcp2_85*S6-S4*C6);
    ROcp2_36 = -C5*S6;
    ROcp2_76 = ROcp2_75*C6+C4*S6;
    ROcp2_86 = ROcp2_85*C6+S4*S6;
    ROcp2_96 = C5*C6;
    OMcp2_15 = qd[5]*C4;
    OMcp2_25 = qd[5]*S4;
    OMcp2_16 = OMcp2_15+ROcp2_45*qd[6];
    OMcp2_26 = OMcp2_25+ROcp2_55*qd[6];
    OMcp2_36 = qd[4]+qd[6]*S5;
    OPcp2_16 = ROcp2_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp2_25*S5-ROcp2_55*qd[4]);
    OPcp2_26 = ROcp2_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp2_15*S5-ROcp2_45*qd[4]);
    OPcp2_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_3_0_3 = = 
 
// Sensor Kinematics 


    ROcp2_48 = ROcp2_45*C8+ROcp2_76*S8;
    ROcp2_58 = ROcp2_55*C8+ROcp2_86*S8;
    ROcp2_68 = ROcp2_96*S8+S5*C8;
    ROcp2_78 = -(ROcp2_45*S8-ROcp2_76*C8);
    ROcp2_88 = -(ROcp2_55*S8-ROcp2_86*C8);
    ROcp2_98 = ROcp2_96*C8-S5*S8;
    ROcp2_19 = ROcp2_16*C9-ROcp2_78*S9;
    ROcp2_29 = ROcp2_26*C9-ROcp2_88*S9;
    ROcp2_39 = ROcp2_36*C9-ROcp2_98*S9;
    ROcp2_79 = ROcp2_16*S9+ROcp2_78*C9;
    ROcp2_89 = ROcp2_26*S9+ROcp2_88*C9;
    ROcp2_99 = ROcp2_36*S9+ROcp2_98*C9;
    ROcp2_410 = ROcp2_48*C10+ROcp2_79*S10;
    ROcp2_510 = ROcp2_58*C10+ROcp2_89*S10;
    ROcp2_610 = ROcp2_68*C10+ROcp2_99*S10;
    ROcp2_710 = -(ROcp2_48*S10-ROcp2_79*C10);
    ROcp2_810 = -(ROcp2_58*S10-ROcp2_89*C10);
    ROcp2_910 = -(ROcp2_68*S10-ROcp2_99*C10);
    ROcp2_111 = ROcp2_19*C11+ROcp2_410*S11;
    ROcp2_211 = ROcp2_29*C11+ROcp2_510*S11;
    ROcp2_311 = ROcp2_39*C11+ROcp2_610*S11;
    ROcp2_411 = -(ROcp2_19*S11-ROcp2_410*C11);
    ROcp2_511 = -(ROcp2_29*S11-ROcp2_510*C11);
    ROcp2_611 = -(ROcp2_39*S11-ROcp2_610*C11);
    ROcp2_112 = ROcp2_111*C12-ROcp2_710*S12;
    ROcp2_212 = ROcp2_211*C12-ROcp2_810*S12;
    ROcp2_312 = ROcp2_311*C12-ROcp2_910*S12;
    ROcp2_712 = ROcp2_111*S12+ROcp2_710*C12;
    ROcp2_812 = ROcp2_211*S12+ROcp2_810*C12;
    ROcp2_912 = ROcp2_311*S12+ROcp2_910*C12;
    RLcp2_18 = ROcp2_16*s->dpt[1][1]+ROcp2_45*s->dpt[2][1]+ROcp2_76*s->dpt[3][1];
    RLcp2_28 = ROcp2_26*s->dpt[1][1]+ROcp2_55*s->dpt[2][1]+ROcp2_86*s->dpt[3][1];
    RLcp2_38 = ROcp2_36*s->dpt[1][1]+ROcp2_96*s->dpt[3][1]+s->dpt[2][1]*S5;
    OMcp2_18 = OMcp2_16+ROcp2_16*qd[8];
    OMcp2_28 = OMcp2_26+ROcp2_26*qd[8];
    OMcp2_38 = OMcp2_36+ROcp2_36*qd[8];
    ORcp2_18 = OMcp2_26*RLcp2_38-OMcp2_36*RLcp2_28;
    ORcp2_28 = -(OMcp2_16*RLcp2_38-OMcp2_36*RLcp2_18);
    ORcp2_38 = OMcp2_16*RLcp2_28-OMcp2_26*RLcp2_18;
    OPcp2_18 = OPcp2_16+ROcp2_16*qdd[8]+qd[8]*(OMcp2_26*ROcp2_36-OMcp2_36*ROcp2_26);
    OPcp2_28 = OPcp2_26+ROcp2_26*qdd[8]-qd[8]*(OMcp2_16*ROcp2_36-OMcp2_36*ROcp2_16);
    OPcp2_38 = OPcp2_36+ROcp2_36*qdd[8]+qd[8]*(OMcp2_16*ROcp2_26-OMcp2_26*ROcp2_16);
    RLcp2_19 = ROcp2_48*s->dpt[2][18];
    RLcp2_29 = ROcp2_58*s->dpt[2][18];
    RLcp2_39 = ROcp2_68*s->dpt[2][18];
    OMcp2_19 = OMcp2_18+ROcp2_48*qd[9];
    OMcp2_29 = OMcp2_28+ROcp2_58*qd[9];
    OMcp2_39 = OMcp2_38+ROcp2_68*qd[9];
    ORcp2_19 = OMcp2_28*RLcp2_39-OMcp2_38*RLcp2_29;
    ORcp2_29 = -(OMcp2_18*RLcp2_39-OMcp2_38*RLcp2_19);
    ORcp2_39 = OMcp2_18*RLcp2_29-OMcp2_28*RLcp2_19;
    OMcp2_110 = OMcp2_19+ROcp2_19*qd[10];
    OMcp2_210 = OMcp2_29+ROcp2_29*qd[10];
    OMcp2_310 = OMcp2_39+ROcp2_39*qd[10];
    OMcp2_111 = OMcp2_110+ROcp2_710*qd[11];
    OMcp2_211 = OMcp2_210+ROcp2_810*qd[11];
    OMcp2_311 = OMcp2_310+ROcp2_910*qd[11];
    OPcp2_111 = OPcp2_18+ROcp2_19*qdd[10]+ROcp2_48*qdd[9]+ROcp2_710*qdd[11]+qd[10]*(OMcp2_29*ROcp2_39-OMcp2_39*ROcp2_29)+
 qd[11]*(OMcp2_210*ROcp2_910-OMcp2_310*ROcp2_810)+qd[9]*(OMcp2_28*ROcp2_68-OMcp2_38*ROcp2_58);
    OPcp2_211 = OPcp2_28+ROcp2_29*qdd[10]+ROcp2_58*qdd[9]+ROcp2_810*qdd[11]-qd[10]*(OMcp2_19*ROcp2_39-OMcp2_39*ROcp2_19)-
 qd[11]*(OMcp2_110*ROcp2_910-OMcp2_310*ROcp2_710)-qd[9]*(OMcp2_18*ROcp2_68-OMcp2_38*ROcp2_48);
    OPcp2_311 = OPcp2_38+ROcp2_39*qdd[10]+ROcp2_68*qdd[9]+ROcp2_910*qdd[11]+qd[10]*(OMcp2_19*ROcp2_29-OMcp2_29*ROcp2_19)+
 qd[11]*(OMcp2_110*ROcp2_810-OMcp2_210*ROcp2_710)+qd[9]*(OMcp2_18*ROcp2_58-OMcp2_28*ROcp2_48);
    RLcp2_112 = ROcp2_710*s->dpt[3][22];
    RLcp2_212 = ROcp2_810*s->dpt[3][22];
    RLcp2_312 = ROcp2_910*s->dpt[3][22];
    POcp2_112 = RLcp2_112+RLcp2_18+RLcp2_19+q[1];
    POcp2_212 = RLcp2_212+RLcp2_28+RLcp2_29+q[2];
    POcp2_312 = RLcp2_312+RLcp2_38+RLcp2_39+q[3];
    OMcp2_112 = OMcp2_111+ROcp2_411*qd[12];
    OMcp2_212 = OMcp2_211+ROcp2_511*qd[12];
    OMcp2_312 = OMcp2_311+ROcp2_611*qd[12];
    ORcp2_112 = OMcp2_211*RLcp2_312-OMcp2_311*RLcp2_212;
    ORcp2_212 = -(OMcp2_111*RLcp2_312-OMcp2_311*RLcp2_112);
    ORcp2_312 = OMcp2_111*RLcp2_212-OMcp2_211*RLcp2_112;
    VIcp2_112 = ORcp2_112+ORcp2_18+ORcp2_19+qd[1];
    VIcp2_212 = ORcp2_212+ORcp2_28+ORcp2_29+qd[2];
    VIcp2_312 = ORcp2_312+ORcp2_38+ORcp2_39+qd[3];
    OPcp2_112 = OPcp2_111+ROcp2_411*qdd[12]+qd[12]*(OMcp2_211*ROcp2_611-OMcp2_311*ROcp2_511);
    OPcp2_212 = OPcp2_211+ROcp2_511*qdd[12]-qd[12]*(OMcp2_111*ROcp2_611-OMcp2_311*ROcp2_411);
    OPcp2_312 = OPcp2_311+ROcp2_611*qdd[12]+qd[12]*(OMcp2_111*ROcp2_511-OMcp2_211*ROcp2_411);
    ACcp2_112 = qdd[1]+OMcp2_211*ORcp2_312+OMcp2_26*ORcp2_38+OMcp2_28*ORcp2_39-OMcp2_311*ORcp2_212-OMcp2_36*ORcp2_28-
 OMcp2_38*ORcp2_29+OPcp2_211*RLcp2_312+OPcp2_26*RLcp2_38+OPcp2_28*RLcp2_39-OPcp2_311*RLcp2_212-OPcp2_36*RLcp2_28-OPcp2_38*
 RLcp2_29;
    ACcp2_212 = qdd[2]-OMcp2_111*ORcp2_312-OMcp2_16*ORcp2_38-OMcp2_18*ORcp2_39+OMcp2_311*ORcp2_112+OMcp2_36*ORcp2_18+
 OMcp2_38*ORcp2_19-OPcp2_111*RLcp2_312-OPcp2_16*RLcp2_38-OPcp2_18*RLcp2_39+OPcp2_311*RLcp2_112+OPcp2_36*RLcp2_18+OPcp2_38*
 RLcp2_19;
    ACcp2_312 = qdd[3]+OMcp2_111*ORcp2_212+OMcp2_16*ORcp2_28+OMcp2_18*ORcp2_29-OMcp2_211*ORcp2_112-OMcp2_26*ORcp2_18-
 OMcp2_28*ORcp2_19+OPcp2_111*RLcp2_212+OPcp2_16*RLcp2_28+OPcp2_18*RLcp2_29-OPcp2_211*RLcp2_112-OPcp2_26*RLcp2_18-OPcp2_28*
 RLcp2_19;

// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp2_112;
    sens->P[2] = POcp2_212;
    sens->P[3] = POcp2_312;
    sens->R[1][1] = ROcp2_112;
    sens->R[1][2] = ROcp2_212;
    sens->R[1][3] = ROcp2_312;
    sens->R[2][1] = ROcp2_411;
    sens->R[2][2] = ROcp2_511;
    sens->R[2][3] = ROcp2_611;
    sens->R[3][1] = ROcp2_712;
    sens->R[3][2] = ROcp2_812;
    sens->R[3][3] = ROcp2_912;
    sens->V[1] = VIcp2_112;
    sens->V[2] = VIcp2_212;
    sens->V[3] = VIcp2_312;
    sens->OM[1] = OMcp2_112;
    sens->OM[2] = OMcp2_212;
    sens->OM[3] = OMcp2_312;
    sens->A[1] = ACcp2_112;
    sens->A[2] = ACcp2_212;
    sens->A[3] = ACcp2_312;
    sens->OMP[1] = OPcp2_112;
    sens->OMP[2] = OPcp2_212;
    sens->OMP[3] = OPcp2_312;
 
// 
break;
case 4:
 


// = = Block_1_0_0_4_0_1 = = 
 
// Sensor Kinematics 


    ROcp3_45 = -S4*C5;
    ROcp3_55 = C4*C5;
    ROcp3_75 = S4*S5;
    ROcp3_85 = -C4*S5;
    ROcp3_16 = -(ROcp3_75*S6-C4*C6);
    ROcp3_26 = -(ROcp3_85*S6-S4*C6);
    ROcp3_36 = -C5*S6;
    ROcp3_76 = ROcp3_75*C6+C4*S6;
    ROcp3_86 = ROcp3_85*C6+S4*S6;
    ROcp3_96 = C5*C6;
    OMcp3_15 = qd[5]*C4;
    OMcp3_25 = qd[5]*S4;
    OMcp3_16 = OMcp3_15+ROcp3_45*qd[6];
    OMcp3_26 = OMcp3_25+ROcp3_55*qd[6];
    OMcp3_36 = qd[4]+qd[6]*S5;
    OPcp3_16 = ROcp3_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp3_25*S5-ROcp3_55*qd[4]);
    OPcp3_26 = ROcp3_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp3_15*S5-ROcp3_45*qd[4]);
    OPcp3_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_4_0_4 = = 
 
// Sensor Kinematics 


    ROcp3_413 = ROcp3_45*C13+ROcp3_76*S13;
    ROcp3_513 = ROcp3_55*C13+ROcp3_86*S13;
    ROcp3_613 = ROcp3_96*S13+C13*S5;
    ROcp3_713 = -(ROcp3_45*S13-ROcp3_76*C13);
    ROcp3_813 = -(ROcp3_55*S13-ROcp3_86*C13);
    ROcp3_913 = ROcp3_96*C13-S13*S5;
    ROcp3_114 = ROcp3_16*C14-ROcp3_713*S14;
    ROcp3_214 = ROcp3_26*C14-ROcp3_813*S14;
    ROcp3_314 = ROcp3_36*C14-ROcp3_913*S14;
    ROcp3_714 = ROcp3_16*S14+ROcp3_713*C14;
    ROcp3_814 = ROcp3_26*S14+ROcp3_813*C14;
    ROcp3_914 = ROcp3_36*S14+ROcp3_913*C14;
    ROcp3_415 = ROcp3_413*C15+ROcp3_714*S15;
    ROcp3_515 = ROcp3_513*C15+ROcp3_814*S15;
    ROcp3_615 = ROcp3_613*C15+ROcp3_914*S15;
    ROcp3_715 = -(ROcp3_413*S15-ROcp3_714*C15);
    ROcp3_815 = -(ROcp3_513*S15-ROcp3_814*C15);
    ROcp3_915 = -(ROcp3_613*S15-ROcp3_914*C15);
    ROcp3_116 = ROcp3_114*C16+ROcp3_415*S16;
    ROcp3_216 = ROcp3_214*C16+ROcp3_515*S16;
    ROcp3_316 = ROcp3_314*C16+ROcp3_615*S16;
    ROcp3_416 = -(ROcp3_114*S16-ROcp3_415*C16);
    ROcp3_516 = -(ROcp3_214*S16-ROcp3_515*C16);
    ROcp3_616 = -(ROcp3_314*S16-ROcp3_615*C16);
    ROcp3_117 = ROcp3_116*C17-ROcp3_715*S17;
    ROcp3_217 = ROcp3_216*C17-ROcp3_815*S17;
    ROcp3_317 = ROcp3_316*C17-ROcp3_915*S17;
    ROcp3_717 = ROcp3_116*S17+ROcp3_715*C17;
    ROcp3_817 = ROcp3_216*S17+ROcp3_815*C17;
    ROcp3_917 = ROcp3_316*S17+ROcp3_915*C17;
    RLcp3_113 = ROcp3_16*s->dpt[1][2]+ROcp3_45*s->dpt[2][2]+ROcp3_76*s->dpt[3][2];
    RLcp3_213 = ROcp3_26*s->dpt[1][2]+ROcp3_55*s->dpt[2][2]+ROcp3_86*s->dpt[3][2];
    RLcp3_313 = ROcp3_36*s->dpt[1][2]+ROcp3_96*s->dpt[3][2]+s->dpt[2][2]*S5;
    OMcp3_113 = OMcp3_16+ROcp3_16*qd[13];
    OMcp3_213 = OMcp3_26+ROcp3_26*qd[13];
    OMcp3_313 = OMcp3_36+ROcp3_36*qd[13];
    ORcp3_113 = OMcp3_26*RLcp3_313-OMcp3_36*RLcp3_213;
    ORcp3_213 = -(OMcp3_16*RLcp3_313-OMcp3_36*RLcp3_113);
    ORcp3_313 = OMcp3_16*RLcp3_213-OMcp3_26*RLcp3_113;
    OPcp3_113 = OPcp3_16+ROcp3_16*qdd[13]+qd[13]*(OMcp3_26*ROcp3_36-OMcp3_36*ROcp3_26);
    OPcp3_213 = OPcp3_26+ROcp3_26*qdd[13]-qd[13]*(OMcp3_16*ROcp3_36-OMcp3_36*ROcp3_16);
    OPcp3_313 = OPcp3_36+ROcp3_36*qdd[13]+qd[13]*(OMcp3_16*ROcp3_26-OMcp3_26*ROcp3_16);
    RLcp3_114 = ROcp3_413*s->dpt[2][24];
    RLcp3_214 = ROcp3_513*s->dpt[2][24];
    RLcp3_314 = ROcp3_613*s->dpt[2][24];
    OMcp3_114 = OMcp3_113+ROcp3_413*qd[14];
    OMcp3_214 = OMcp3_213+ROcp3_513*qd[14];
    OMcp3_314 = OMcp3_313+ROcp3_613*qd[14];
    ORcp3_114 = OMcp3_213*RLcp3_314-OMcp3_313*RLcp3_214;
    ORcp3_214 = -(OMcp3_113*RLcp3_314-OMcp3_313*RLcp3_114);
    ORcp3_314 = OMcp3_113*RLcp3_214-OMcp3_213*RLcp3_114;
    OMcp3_115 = OMcp3_114+ROcp3_114*qd[15];
    OMcp3_215 = OMcp3_214+ROcp3_214*qd[15];
    OMcp3_315 = OMcp3_314+ROcp3_314*qd[15];
    OMcp3_116 = OMcp3_115+ROcp3_715*qd[16];
    OMcp3_216 = OMcp3_215+ROcp3_815*qd[16];
    OMcp3_316 = OMcp3_315+ROcp3_915*qd[16];
    OPcp3_116 = OPcp3_113+ROcp3_114*qdd[15]+ROcp3_413*qdd[14]+ROcp3_715*qdd[16]+qd[14]*(OMcp3_213*ROcp3_613-OMcp3_313*
 ROcp3_513)+qd[15]*(OMcp3_214*ROcp3_314-OMcp3_314*ROcp3_214)+qd[16]*(OMcp3_215*ROcp3_915-OMcp3_315*ROcp3_815);
    OPcp3_216 = OPcp3_213+ROcp3_214*qdd[15]+ROcp3_513*qdd[14]+ROcp3_815*qdd[16]-qd[14]*(OMcp3_113*ROcp3_613-OMcp3_313*
 ROcp3_413)-qd[15]*(OMcp3_114*ROcp3_314-OMcp3_314*ROcp3_114)-qd[16]*(OMcp3_115*ROcp3_915-OMcp3_315*ROcp3_715);
    OPcp3_316 = OPcp3_313+ROcp3_314*qdd[15]+ROcp3_613*qdd[14]+ROcp3_915*qdd[16]+qd[14]*(OMcp3_113*ROcp3_513-OMcp3_213*
 ROcp3_413)+qd[15]*(OMcp3_114*ROcp3_214-OMcp3_214*ROcp3_114)+qd[16]*(OMcp3_115*ROcp3_815-OMcp3_215*ROcp3_715);
    RLcp3_117 = ROcp3_715*s->dpt[3][27];
    RLcp3_217 = ROcp3_815*s->dpt[3][27];
    RLcp3_317 = ROcp3_915*s->dpt[3][27];
    POcp3_117 = RLcp3_113+RLcp3_114+RLcp3_117+q[1];
    POcp3_217 = RLcp3_213+RLcp3_214+RLcp3_217+q[2];
    POcp3_317 = RLcp3_313+RLcp3_314+RLcp3_317+q[3];
    OMcp3_117 = OMcp3_116+ROcp3_416*qd[17];
    OMcp3_217 = OMcp3_216+ROcp3_516*qd[17];
    OMcp3_317 = OMcp3_316+ROcp3_616*qd[17];
    ORcp3_117 = OMcp3_216*RLcp3_317-OMcp3_316*RLcp3_217;
    ORcp3_217 = -(OMcp3_116*RLcp3_317-OMcp3_316*RLcp3_117);
    ORcp3_317 = OMcp3_116*RLcp3_217-OMcp3_216*RLcp3_117;
    VIcp3_117 = ORcp3_113+ORcp3_114+ORcp3_117+qd[1];
    VIcp3_217 = ORcp3_213+ORcp3_214+ORcp3_217+qd[2];
    VIcp3_317 = ORcp3_313+ORcp3_314+ORcp3_317+qd[3];
    OPcp3_117 = OPcp3_116+ROcp3_416*qdd[17]+qd[17]*(OMcp3_216*ROcp3_616-OMcp3_316*ROcp3_516);
    OPcp3_217 = OPcp3_216+ROcp3_516*qdd[17]-qd[17]*(OMcp3_116*ROcp3_616-OMcp3_316*ROcp3_416);
    OPcp3_317 = OPcp3_316+ROcp3_616*qdd[17]+qd[17]*(OMcp3_116*ROcp3_516-OMcp3_216*ROcp3_416);
    ACcp3_117 = qdd[1]+OMcp3_213*ORcp3_314+OMcp3_216*ORcp3_317+OMcp3_26*ORcp3_313-OMcp3_313*ORcp3_214-OMcp3_316*ORcp3_217-
 OMcp3_36*ORcp3_213+OPcp3_213*RLcp3_314+OPcp3_216*RLcp3_317+OPcp3_26*RLcp3_313-OPcp3_313*RLcp3_214-OPcp3_316*RLcp3_217-
 OPcp3_36*RLcp3_213;
    ACcp3_217 = qdd[2]-OMcp3_113*ORcp3_314-OMcp3_116*ORcp3_317-OMcp3_16*ORcp3_313+OMcp3_313*ORcp3_114+OMcp3_316*ORcp3_117+
 OMcp3_36*ORcp3_113-OPcp3_113*RLcp3_314-OPcp3_116*RLcp3_317-OPcp3_16*RLcp3_313+OPcp3_313*RLcp3_114+OPcp3_316*RLcp3_117+
 OPcp3_36*RLcp3_113;
    ACcp3_317 = qdd[3]+OMcp3_113*ORcp3_214+OMcp3_116*ORcp3_217+OMcp3_16*ORcp3_213-OMcp3_213*ORcp3_114-OMcp3_216*ORcp3_117-
 OMcp3_26*ORcp3_113+OPcp3_113*RLcp3_214+OPcp3_116*RLcp3_217+OPcp3_16*RLcp3_213-OPcp3_213*RLcp3_114-OPcp3_216*RLcp3_117-
 OPcp3_26*RLcp3_113;

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_117;
    sens->P[2] = POcp3_217;
    sens->P[3] = POcp3_317;
    sens->R[1][1] = ROcp3_117;
    sens->R[1][2] = ROcp3_217;
    sens->R[1][3] = ROcp3_317;
    sens->R[2][1] = ROcp3_416;
    sens->R[2][2] = ROcp3_516;
    sens->R[2][3] = ROcp3_616;
    sens->R[3][1] = ROcp3_717;
    sens->R[3][2] = ROcp3_817;
    sens->R[3][3] = ROcp3_917;
    sens->V[1] = VIcp3_117;
    sens->V[2] = VIcp3_217;
    sens->V[3] = VIcp3_317;
    sens->OM[1] = OMcp3_117;
    sens->OM[2] = OMcp3_217;
    sens->OM[3] = OMcp3_317;
    sens->A[1] = ACcp3_117;
    sens->A[2] = ACcp3_217;
    sens->A[3] = ACcp3_317;
    sens->OMP[1] = OPcp3_117;
    sens->OMP[2] = OPcp3_217;
    sens->OMP[3] = OPcp3_317;
 
// 
break;
case 5:
 


// = = Block_1_0_0_5_0_1 = = 
 
// Sensor Kinematics 


    ROcp4_45 = -S4*C5;
    ROcp4_55 = C4*C5;
    ROcp4_75 = S4*S5;
    ROcp4_85 = -C4*S5;
    ROcp4_16 = -(ROcp4_75*S6-C4*C6);
    ROcp4_26 = -(ROcp4_85*S6-S4*C6);
    ROcp4_36 = -C5*S6;
    ROcp4_76 = ROcp4_75*C6+C4*S6;
    ROcp4_86 = ROcp4_85*C6+S4*S6;
    ROcp4_96 = C5*C6;
    OMcp4_15 = qd[5]*C4;
    OMcp4_25 = qd[5]*S4;
    OMcp4_16 = OMcp4_15+ROcp4_45*qd[6];
    OMcp4_26 = OMcp4_25+ROcp4_55*qd[6];
    OMcp4_36 = qd[4]+qd[6]*S5;
    OPcp4_16 = ROcp4_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp4_25*S5-ROcp4_55*qd[4]);
    OPcp4_26 = ROcp4_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp4_15*S5-ROcp4_45*qd[4]);
    OPcp4_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_5_0_15 = = 
 
// Sensor Kinematics 


    ROcp4_433 = ROcp4_45*C33+ROcp4_76*S33;
    ROcp4_533 = ROcp4_55*C33+ROcp4_86*S33;
    ROcp4_633 = ROcp4_96*S33+C33*S5;
    ROcp4_733 = -(ROcp4_45*S33-ROcp4_76*C33);
    ROcp4_833 = -(ROcp4_55*S33-ROcp4_86*C33);
    ROcp4_933 = ROcp4_96*C33-S33*S5;
    ROcp4_434 = ROcp4_433*C34+ROcp4_733*S34;
    ROcp4_534 = ROcp4_533*C34+ROcp4_833*S34;
    ROcp4_634 = ROcp4_633*C34+ROcp4_933*S34;
    ROcp4_734 = -(ROcp4_433*S34-ROcp4_733*C34);
    ROcp4_834 = -(ROcp4_533*S34-ROcp4_833*C34);
    ROcp4_934 = -(ROcp4_633*S34-ROcp4_933*C34);
    ROcp4_135 = ROcp4_16*C35-ROcp4_734*S35;
    ROcp4_235 = ROcp4_26*C35-ROcp4_834*S35;
    ROcp4_335 = ROcp4_36*C35-ROcp4_934*S35;
    ROcp4_735 = ROcp4_16*S35+ROcp4_734*C35;
    ROcp4_835 = ROcp4_26*S35+ROcp4_834*C35;
    ROcp4_935 = ROcp4_36*S35+ROcp4_934*C35;
    RLcp4_133 = ROcp4_16*s->dpt[1][12]+ROcp4_45*s->dpt[2][12]+ROcp4_76*s->dpt[3][12];
    RLcp4_233 = ROcp4_26*s->dpt[1][12]+ROcp4_55*s->dpt[2][12]+ROcp4_86*s->dpt[3][12];
    RLcp4_333 = ROcp4_36*s->dpt[1][12]+ROcp4_96*s->dpt[3][12]+s->dpt[2][12]*S5;
    OMcp4_133 = OMcp4_16+ROcp4_16*qd[33];
    OMcp4_233 = OMcp4_26+ROcp4_26*qd[33];
    OMcp4_333 = OMcp4_36+ROcp4_36*qd[33];
    ORcp4_133 = OMcp4_26*RLcp4_333-OMcp4_36*RLcp4_233;
    ORcp4_233 = -(OMcp4_16*RLcp4_333-OMcp4_36*RLcp4_133);
    ORcp4_333 = OMcp4_16*RLcp4_233-OMcp4_26*RLcp4_133;
    OPcp4_133 = OPcp4_16+ROcp4_16*qdd[33]+qd[33]*(OMcp4_26*ROcp4_36-OMcp4_36*ROcp4_26);
    OPcp4_233 = OPcp4_26+ROcp4_26*qdd[33]-qd[33]*(OMcp4_16*ROcp4_36-OMcp4_36*ROcp4_16);
    OPcp4_333 = OPcp4_36+ROcp4_36*qdd[33]+qd[33]*(OMcp4_16*ROcp4_26-OMcp4_26*ROcp4_16);
    RLcp4_134 = ROcp4_433*s->dpt[2][47];
    RLcp4_234 = ROcp4_533*s->dpt[2][47];
    RLcp4_334 = ROcp4_633*s->dpt[2][47];
    OMcp4_134 = OMcp4_133+ROcp4_16*qd[34];
    OMcp4_234 = OMcp4_233+ROcp4_26*qd[34];
    OMcp4_334 = OMcp4_333+ROcp4_36*qd[34];
    ORcp4_134 = OMcp4_233*RLcp4_334-OMcp4_333*RLcp4_234;
    ORcp4_234 = -(OMcp4_133*RLcp4_334-OMcp4_333*RLcp4_134);
    ORcp4_334 = OMcp4_133*RLcp4_234-OMcp4_233*RLcp4_134;
    OPcp4_134 = OPcp4_133+ROcp4_16*qdd[34]+qd[34]*(OMcp4_233*ROcp4_36-OMcp4_333*ROcp4_26);
    OPcp4_234 = OPcp4_233+ROcp4_26*qdd[34]-qd[34]*(OMcp4_133*ROcp4_36-OMcp4_333*ROcp4_16);
    OPcp4_334 = OPcp4_333+ROcp4_36*qdd[34]+qd[34]*(OMcp4_133*ROcp4_26-OMcp4_233*ROcp4_16);
    RLcp4_135 = ROcp4_734*s->dpt[3][50];
    RLcp4_235 = ROcp4_834*s->dpt[3][50];
    RLcp4_335 = ROcp4_934*s->dpt[3][50];
    POcp4_135 = RLcp4_133+RLcp4_134+RLcp4_135+q[1];
    POcp4_235 = RLcp4_233+RLcp4_234+RLcp4_235+q[2];
    POcp4_335 = RLcp4_333+RLcp4_334+RLcp4_335+q[3];
    OMcp4_135 = OMcp4_134+ROcp4_434*qd[35];
    OMcp4_235 = OMcp4_234+ROcp4_534*qd[35];
    OMcp4_335 = OMcp4_334+ROcp4_634*qd[35];
    ORcp4_135 = OMcp4_234*RLcp4_335-OMcp4_334*RLcp4_235;
    ORcp4_235 = -(OMcp4_134*RLcp4_335-OMcp4_334*RLcp4_135);
    ORcp4_335 = OMcp4_134*RLcp4_235-OMcp4_234*RLcp4_135;
    VIcp4_135 = ORcp4_133+ORcp4_134+ORcp4_135+qd[1];
    VIcp4_235 = ORcp4_233+ORcp4_234+ORcp4_235+qd[2];
    VIcp4_335 = ORcp4_333+ORcp4_334+ORcp4_335+qd[3];
    OPcp4_135 = OPcp4_134+ROcp4_434*qdd[35]+qd[35]*(OMcp4_234*ROcp4_634-OMcp4_334*ROcp4_534);
    OPcp4_235 = OPcp4_234+ROcp4_534*qdd[35]-qd[35]*(OMcp4_134*ROcp4_634-OMcp4_334*ROcp4_434);
    OPcp4_335 = OPcp4_334+ROcp4_634*qdd[35]+qd[35]*(OMcp4_134*ROcp4_534-OMcp4_234*ROcp4_434);
    ACcp4_135 = qdd[1]+OMcp4_233*ORcp4_334+OMcp4_234*ORcp4_335+OMcp4_26*ORcp4_333-OMcp4_333*ORcp4_234-OMcp4_334*ORcp4_235-
 OMcp4_36*ORcp4_233+OPcp4_233*RLcp4_334+OPcp4_234*RLcp4_335+OPcp4_26*RLcp4_333-OPcp4_333*RLcp4_234-OPcp4_334*RLcp4_235-
 OPcp4_36*RLcp4_233;
    ACcp4_235 = qdd[2]-OMcp4_133*ORcp4_334-OMcp4_134*ORcp4_335-OMcp4_16*ORcp4_333+OMcp4_333*ORcp4_134+OMcp4_334*ORcp4_135+
 OMcp4_36*ORcp4_133-OPcp4_133*RLcp4_334-OPcp4_134*RLcp4_335-OPcp4_16*RLcp4_333+OPcp4_333*RLcp4_134+OPcp4_334*RLcp4_135+
 OPcp4_36*RLcp4_133;
    ACcp4_335 = qdd[3]+OMcp4_133*ORcp4_234+OMcp4_134*ORcp4_235+OMcp4_16*ORcp4_233-OMcp4_233*ORcp4_134-OMcp4_234*ORcp4_135-
 OMcp4_26*ORcp4_133+OPcp4_133*RLcp4_234+OPcp4_134*RLcp4_235+OPcp4_16*RLcp4_233-OPcp4_233*RLcp4_134-OPcp4_234*RLcp4_135-
 OPcp4_26*RLcp4_133;

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp4_135;
    sens->P[2] = POcp4_235;
    sens->P[3] = POcp4_335;
    sens->R[1][1] = ROcp4_135;
    sens->R[1][2] = ROcp4_235;
    sens->R[1][3] = ROcp4_335;
    sens->R[2][1] = ROcp4_434;
    sens->R[2][2] = ROcp4_534;
    sens->R[2][3] = ROcp4_634;
    sens->R[3][1] = ROcp4_735;
    sens->R[3][2] = ROcp4_835;
    sens->R[3][3] = ROcp4_935;
    sens->V[1] = VIcp4_135;
    sens->V[2] = VIcp4_235;
    sens->V[3] = VIcp4_335;
    sens->OM[1] = OMcp4_135;
    sens->OM[2] = OMcp4_235;
    sens->OM[3] = OMcp4_335;
    sens->A[1] = ACcp4_135;
    sens->A[2] = ACcp4_235;
    sens->A[3] = ACcp4_335;
    sens->OMP[1] = OPcp4_135;
    sens->OMP[2] = OPcp4_235;
    sens->OMP[3] = OPcp4_335;
 
// 
break;
case 6:
 


// = = Block_1_0_0_6_0_1 = = 
 
// Sensor Kinematics 


    ROcp5_45 = -S4*C5;
    ROcp5_55 = C4*C5;
    ROcp5_75 = S4*S5;
    ROcp5_85 = -C4*S5;
    ROcp5_16 = -(ROcp5_75*S6-C4*C6);
    ROcp5_26 = -(ROcp5_85*S6-S4*C6);
    ROcp5_36 = -C5*S6;
    ROcp5_76 = ROcp5_75*C6+C4*S6;
    ROcp5_86 = ROcp5_85*C6+S4*S6;
    ROcp5_96 = C5*C6;
    OMcp5_15 = qd[5]*C4;
    OMcp5_25 = qd[5]*S4;
    OMcp5_16 = OMcp5_15+ROcp5_45*qd[6];
    OMcp5_26 = OMcp5_25+ROcp5_55*qd[6];
    OMcp5_36 = qd[4]+qd[6]*S5;
    OPcp5_16 = ROcp5_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp5_25*S5-ROcp5_55*qd[4]);
    OPcp5_26 = ROcp5_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp5_15*S5-ROcp5_45*qd[4]);
    OPcp5_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_6_0_16 = = 
 
// Sensor Kinematics 


    ROcp5_436 = ROcp5_45*C36+ROcp5_76*S36;
    ROcp5_536 = ROcp5_55*C36+ROcp5_86*S36;
    ROcp5_636 = ROcp5_96*S36+C36*S5;
    ROcp5_736 = -(ROcp5_45*S36-ROcp5_76*C36);
    ROcp5_836 = -(ROcp5_55*S36-ROcp5_86*C36);
    ROcp5_936 = ROcp5_96*C36-S36*S5;
    ROcp5_437 = ROcp5_436*C37+ROcp5_736*S37;
    ROcp5_537 = ROcp5_536*C37+ROcp5_836*S37;
    ROcp5_637 = ROcp5_636*C37+ROcp5_936*S37;
    ROcp5_737 = -(ROcp5_436*S37-ROcp5_736*C37);
    ROcp5_837 = -(ROcp5_536*S37-ROcp5_836*C37);
    ROcp5_937 = -(ROcp5_636*S37-ROcp5_936*C37);
    ROcp5_138 = ROcp5_16*C38-ROcp5_737*S38;
    ROcp5_238 = ROcp5_26*C38-ROcp5_837*S38;
    ROcp5_338 = ROcp5_36*C38-ROcp5_937*S38;
    ROcp5_738 = ROcp5_16*S38+ROcp5_737*C38;
    ROcp5_838 = ROcp5_26*S38+ROcp5_837*C38;
    ROcp5_938 = ROcp5_36*S38+ROcp5_937*C38;
    RLcp5_136 = ROcp5_16*s->dpt[1][13]+ROcp5_45*s->dpt[2][13]+ROcp5_76*s->dpt[3][13];
    RLcp5_236 = ROcp5_26*s->dpt[1][13]+ROcp5_55*s->dpt[2][13]+ROcp5_86*s->dpt[3][13];
    RLcp5_336 = ROcp5_36*s->dpt[1][13]+ROcp5_96*s->dpt[3][13]+s->dpt[2][13]*S5;
    OMcp5_136 = OMcp5_16+ROcp5_16*qd[36];
    OMcp5_236 = OMcp5_26+ROcp5_26*qd[36];
    OMcp5_336 = OMcp5_36+ROcp5_36*qd[36];
    ORcp5_136 = OMcp5_26*RLcp5_336-OMcp5_36*RLcp5_236;
    ORcp5_236 = -(OMcp5_16*RLcp5_336-OMcp5_36*RLcp5_136);
    ORcp5_336 = OMcp5_16*RLcp5_236-OMcp5_26*RLcp5_136;
    OPcp5_136 = OPcp5_16+ROcp5_16*qdd[36]+qd[36]*(OMcp5_26*ROcp5_36-OMcp5_36*ROcp5_26);
    OPcp5_236 = OPcp5_26+ROcp5_26*qdd[36]-qd[36]*(OMcp5_16*ROcp5_36-OMcp5_36*ROcp5_16);
    OPcp5_336 = OPcp5_36+ROcp5_36*qdd[36]+qd[36]*(OMcp5_16*ROcp5_26-OMcp5_26*ROcp5_16);
    RLcp5_137 = ROcp5_436*s->dpt[2][52];
    RLcp5_237 = ROcp5_536*s->dpt[2][52];
    RLcp5_337 = ROcp5_636*s->dpt[2][52];
    OMcp5_137 = OMcp5_136+ROcp5_16*qd[37];
    OMcp5_237 = OMcp5_236+ROcp5_26*qd[37];
    OMcp5_337 = OMcp5_336+ROcp5_36*qd[37];
    ORcp5_137 = OMcp5_236*RLcp5_337-OMcp5_336*RLcp5_237;
    ORcp5_237 = -(OMcp5_136*RLcp5_337-OMcp5_336*RLcp5_137);
    ORcp5_337 = OMcp5_136*RLcp5_237-OMcp5_236*RLcp5_137;
    OPcp5_137 = OPcp5_136+ROcp5_16*qdd[37]+qd[37]*(OMcp5_236*ROcp5_36-OMcp5_336*ROcp5_26);
    OPcp5_237 = OPcp5_236+ROcp5_26*qdd[37]-qd[37]*(OMcp5_136*ROcp5_36-OMcp5_336*ROcp5_16);
    OPcp5_337 = OPcp5_336+ROcp5_36*qdd[37]+qd[37]*(OMcp5_136*ROcp5_26-OMcp5_236*ROcp5_16);
    RLcp5_138 = ROcp5_737*s->dpt[3][54];
    RLcp5_238 = ROcp5_837*s->dpt[3][54];
    RLcp5_338 = ROcp5_937*s->dpt[3][54];
    POcp5_138 = RLcp5_136+RLcp5_137+RLcp5_138+q[1];
    POcp5_238 = RLcp5_236+RLcp5_237+RLcp5_238+q[2];
    POcp5_338 = RLcp5_336+RLcp5_337+RLcp5_338+q[3];
    OMcp5_138 = OMcp5_137+ROcp5_437*qd[38];
    OMcp5_238 = OMcp5_237+ROcp5_537*qd[38];
    OMcp5_338 = OMcp5_337+ROcp5_637*qd[38];
    ORcp5_138 = OMcp5_237*RLcp5_338-OMcp5_337*RLcp5_238;
    ORcp5_238 = -(OMcp5_137*RLcp5_338-OMcp5_337*RLcp5_138);
    ORcp5_338 = OMcp5_137*RLcp5_238-OMcp5_237*RLcp5_138;
    VIcp5_138 = ORcp5_136+ORcp5_137+ORcp5_138+qd[1];
    VIcp5_238 = ORcp5_236+ORcp5_237+ORcp5_238+qd[2];
    VIcp5_338 = ORcp5_336+ORcp5_337+ORcp5_338+qd[3];
    OPcp5_138 = OPcp5_137+ROcp5_437*qdd[38]+qd[38]*(OMcp5_237*ROcp5_637-OMcp5_337*ROcp5_537);
    OPcp5_238 = OPcp5_237+ROcp5_537*qdd[38]-qd[38]*(OMcp5_137*ROcp5_637-OMcp5_337*ROcp5_437);
    OPcp5_338 = OPcp5_337+ROcp5_637*qdd[38]+qd[38]*(OMcp5_137*ROcp5_537-OMcp5_237*ROcp5_437);
    ACcp5_138 = qdd[1]+OMcp5_236*ORcp5_337+OMcp5_237*ORcp5_338+OMcp5_26*ORcp5_336-OMcp5_336*ORcp5_237-OMcp5_337*ORcp5_238-
 OMcp5_36*ORcp5_236+OPcp5_236*RLcp5_337+OPcp5_237*RLcp5_338+OPcp5_26*RLcp5_336-OPcp5_336*RLcp5_237-OPcp5_337*RLcp5_238-
 OPcp5_36*RLcp5_236;
    ACcp5_238 = qdd[2]-OMcp5_136*ORcp5_337-OMcp5_137*ORcp5_338-OMcp5_16*ORcp5_336+OMcp5_336*ORcp5_137+OMcp5_337*ORcp5_138+
 OMcp5_36*ORcp5_136-OPcp5_136*RLcp5_337-OPcp5_137*RLcp5_338-OPcp5_16*RLcp5_336+OPcp5_336*RLcp5_137+OPcp5_337*RLcp5_138+
 OPcp5_36*RLcp5_136;
    ACcp5_338 = qdd[3]+OMcp5_136*ORcp5_237+OMcp5_137*ORcp5_238+OMcp5_16*ORcp5_236-OMcp5_236*ORcp5_137-OMcp5_237*ORcp5_138-
 OMcp5_26*ORcp5_136+OPcp5_136*RLcp5_237+OPcp5_137*RLcp5_238+OPcp5_16*RLcp5_236-OPcp5_236*RLcp5_137-OPcp5_237*RLcp5_138-
 OPcp5_26*RLcp5_136;

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp5_138;
    sens->P[2] = POcp5_238;
    sens->P[3] = POcp5_338;
    sens->R[1][1] = ROcp5_138;
    sens->R[1][2] = ROcp5_238;
    sens->R[1][3] = ROcp5_338;
    sens->R[2][1] = ROcp5_437;
    sens->R[2][2] = ROcp5_537;
    sens->R[2][3] = ROcp5_637;
    sens->R[3][1] = ROcp5_738;
    sens->R[3][2] = ROcp5_838;
    sens->R[3][3] = ROcp5_938;
    sens->V[1] = VIcp5_138;
    sens->V[2] = VIcp5_238;
    sens->V[3] = VIcp5_338;
    sens->OM[1] = OMcp5_138;
    sens->OM[2] = OMcp5_238;
    sens->OM[3] = OMcp5_338;
    sens->A[1] = ACcp5_138;
    sens->A[2] = ACcp5_238;
    sens->A[3] = ACcp5_338;
    sens->OMP[1] = OPcp5_138;
    sens->OMP[2] = OPcp5_238;
    sens->OMP[3] = OPcp5_338;
 
// 
break;
case 7:
 


// = = Block_1_0_0_7_0_1 = = 
 
// Sensor Kinematics 


    ROcp6_45 = -S4*C5;
    ROcp6_55 = C4*C5;
    ROcp6_75 = S4*S5;
    ROcp6_85 = -C4*S5;
    ROcp6_16 = -(ROcp6_75*S6-C4*C6);
    ROcp6_26 = -(ROcp6_85*S6-S4*C6);
    ROcp6_36 = -C5*S6;
    ROcp6_76 = ROcp6_75*C6+C4*S6;
    ROcp6_86 = ROcp6_85*C6+S4*S6;
    ROcp6_96 = C5*C6;
    OMcp6_15 = qd[5]*C4;
    OMcp6_25 = qd[5]*S4;
    OMcp6_16 = OMcp6_15+ROcp6_45*qd[6];
    OMcp6_26 = OMcp6_25+ROcp6_55*qd[6];
    OMcp6_36 = qd[4]+qd[6]*S5;
    OPcp6_16 = ROcp6_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp6_25*S5-ROcp6_55*qd[4]);
    OPcp6_26 = ROcp6_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp6_15*S5-ROcp6_45*qd[4]);
    OPcp6_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_7_0_20 = = 
 
// Sensor Kinematics 


    ROcp6_145 = ROcp6_16*C45+ROcp6_45*S45;
    ROcp6_245 = ROcp6_26*C45+ROcp6_55*S45;
    ROcp6_345 = ROcp6_36*C45+S45*S5;
    ROcp6_445 = -(ROcp6_16*S45-ROcp6_45*C45);
    ROcp6_545 = -(ROcp6_26*S45-ROcp6_55*C45);
    ROcp6_645 = -(ROcp6_36*S45-C45*S5);
    ROcp6_446 = ROcp6_445*C46+ROcp6_76*S46;
    ROcp6_546 = ROcp6_545*C46+ROcp6_86*S46;
    ROcp6_646 = ROcp6_645*C46+ROcp6_96*S46;
    ROcp6_746 = -(ROcp6_445*S46-ROcp6_76*C46);
    ROcp6_846 = -(ROcp6_545*S46-ROcp6_86*C46);
    ROcp6_946 = -(ROcp6_645*S46-ROcp6_96*C46);
    ROcp6_147 = ROcp6_145*C47-ROcp6_746*S47;
    ROcp6_247 = ROcp6_245*C47-ROcp6_846*S47;
    ROcp6_347 = ROcp6_345*C47-ROcp6_946*S47;
    ROcp6_747 = ROcp6_145*S47+ROcp6_746*C47;
    ROcp6_847 = ROcp6_245*S47+ROcp6_846*C47;
    ROcp6_947 = ROcp6_345*S47+ROcp6_946*C47;
    RLcp6_145 = ROcp6_16*s->dpt[1][17]+ROcp6_76*s->dpt[3][17];
    RLcp6_245 = ROcp6_26*s->dpt[1][17]+ROcp6_86*s->dpt[3][17];
    RLcp6_345 = ROcp6_36*s->dpt[1][17]+ROcp6_96*s->dpt[3][17];
    OMcp6_145 = OMcp6_16+ROcp6_76*qd[45];
    OMcp6_245 = OMcp6_26+ROcp6_86*qd[45];
    OMcp6_345 = OMcp6_36+ROcp6_96*qd[45];
    ORcp6_145 = OMcp6_26*RLcp6_345-OMcp6_36*RLcp6_245;
    ORcp6_245 = -(OMcp6_16*RLcp6_345-OMcp6_36*RLcp6_145);
    ORcp6_345 = OMcp6_16*RLcp6_245-OMcp6_26*RLcp6_145;
    OMcp6_146 = OMcp6_145+ROcp6_145*qd[46];
    OMcp6_246 = OMcp6_245+ROcp6_245*qd[46];
    OMcp6_346 = OMcp6_345+ROcp6_345*qd[46];
    OMcp6_147 = OMcp6_146+ROcp6_446*qd[47];
    OMcp6_247 = OMcp6_246+ROcp6_546*qd[47];
    OMcp6_347 = OMcp6_346+ROcp6_646*qd[47];
    OPcp6_147 = OPcp6_16+ROcp6_145*qdd[46]+ROcp6_446*qdd[47]+ROcp6_76*qdd[45]+qd[45]*(OMcp6_26*ROcp6_96-OMcp6_36*ROcp6_86)
 +qd[46]*(OMcp6_245*ROcp6_345-OMcp6_345*ROcp6_245)+qd[47]*(OMcp6_246*ROcp6_646-OMcp6_346*ROcp6_546);
    OPcp6_247 = OPcp6_26+ROcp6_245*qdd[46]+ROcp6_546*qdd[47]+ROcp6_86*qdd[45]-qd[45]*(OMcp6_16*ROcp6_96-OMcp6_36*ROcp6_76)
 -qd[46]*(OMcp6_145*ROcp6_345-OMcp6_345*ROcp6_145)-qd[47]*(OMcp6_146*ROcp6_646-OMcp6_346*ROcp6_446);
    OPcp6_347 = OPcp6_36+ROcp6_345*qdd[46]+ROcp6_646*qdd[47]+ROcp6_96*qdd[45]+qd[45]*(OMcp6_16*ROcp6_86-OMcp6_26*ROcp6_76)
 +qd[46]*(OMcp6_145*ROcp6_245-OMcp6_245*ROcp6_145)+qd[47]*(OMcp6_146*ROcp6_546-OMcp6_246*ROcp6_446);
    RLcp6_148 = ROcp6_147*q[48];
    RLcp6_248 = ROcp6_247*q[48];
    RLcp6_348 = ROcp6_347*q[48];
    ORcp6_148 = OMcp6_247*RLcp6_348-OMcp6_347*RLcp6_248;
    ORcp6_248 = -(OMcp6_147*RLcp6_348-OMcp6_347*RLcp6_148);
    ORcp6_348 = OMcp6_147*RLcp6_248-OMcp6_247*RLcp6_148;

// = = Block_1_0_0_7_0_21 = = 
 
// Sensor Kinematics 


    ROcp6_149 = ROcp6_147*C49-ROcp6_747*S49;
    ROcp6_249 = ROcp6_247*C49-ROcp6_847*S49;
    ROcp6_349 = ROcp6_347*C49-ROcp6_947*S49;
    ROcp6_749 = ROcp6_147*S49+ROcp6_747*C49;
    ROcp6_849 = ROcp6_247*S49+ROcp6_847*C49;
    ROcp6_949 = ROcp6_347*S49+ROcp6_947*C49;
    RLcp6_149 = ROcp6_147*s->dpt[1][62]+ROcp6_446*s->dpt[2][62]+ROcp6_747*s->dpt[3][62];
    RLcp6_249 = ROcp6_247*s->dpt[1][62]+ROcp6_546*s->dpt[2][62]+ROcp6_847*s->dpt[3][62];
    RLcp6_349 = ROcp6_347*s->dpt[1][62]+ROcp6_646*s->dpt[2][62]+ROcp6_947*s->dpt[3][62];
    POcp6_149 = RLcp6_145+RLcp6_148+RLcp6_149+q[1];
    POcp6_249 = RLcp6_245+RLcp6_248+RLcp6_249+q[2];
    POcp6_349 = RLcp6_345+RLcp6_348+RLcp6_349+q[3];
    OMcp6_149 = OMcp6_147+ROcp6_446*qd[49];
    OMcp6_249 = OMcp6_247+ROcp6_546*qd[49];
    OMcp6_349 = OMcp6_347+ROcp6_646*qd[49];
    ORcp6_149 = OMcp6_247*RLcp6_349-OMcp6_347*RLcp6_249;
    ORcp6_249 = -(OMcp6_147*RLcp6_349-OMcp6_347*RLcp6_149);
    ORcp6_349 = OMcp6_147*RLcp6_249-OMcp6_247*RLcp6_149;
    VIcp6_149 = ORcp6_145+ORcp6_148+ORcp6_149+qd[1]+ROcp6_147*qd[48];
    VIcp6_249 = ORcp6_245+ORcp6_248+ORcp6_249+qd[2]+ROcp6_247*qd[48];
    VIcp6_349 = ORcp6_345+ORcp6_348+ORcp6_349+qd[3]+ROcp6_347*qd[48];
    OPcp6_149 = OPcp6_147+ROcp6_446*qdd[49]+qd[49]*(OMcp6_247*ROcp6_646-OMcp6_347*ROcp6_546);
    OPcp6_249 = OPcp6_247+ROcp6_546*qdd[49]-qd[49]*(OMcp6_147*ROcp6_646-OMcp6_347*ROcp6_446);
    OPcp6_349 = OPcp6_347+ROcp6_646*qdd[49]+qd[49]*(OMcp6_147*ROcp6_546-OMcp6_247*ROcp6_446);
    ACcp6_149 = qdd[1]+OMcp6_247*ORcp6_348+OMcp6_247*ORcp6_349+OMcp6_26*ORcp6_345-OMcp6_347*ORcp6_248-OMcp6_347*ORcp6_249-
 OMcp6_36*ORcp6_245+OPcp6_247*RLcp6_348+OPcp6_247*RLcp6_349+OPcp6_26*RLcp6_345-OPcp6_347*RLcp6_248-OPcp6_347*RLcp6_249-
 OPcp6_36*RLcp6_245+ROcp6_147*qdd[48]+(2.0)*qd[48]*(OMcp6_247*ROcp6_347-OMcp6_347*ROcp6_247);
    ACcp6_249 = qdd[2]-OMcp6_147*ORcp6_348-OMcp6_147*ORcp6_349-OMcp6_16*ORcp6_345+OMcp6_347*ORcp6_148+OMcp6_347*ORcp6_149+
 OMcp6_36*ORcp6_145-OPcp6_147*RLcp6_348-OPcp6_147*RLcp6_349-OPcp6_16*RLcp6_345+OPcp6_347*RLcp6_148+OPcp6_347*RLcp6_149+
 OPcp6_36*RLcp6_145+ROcp6_247*qdd[48]-(2.0)*qd[48]*(OMcp6_147*ROcp6_347-OMcp6_347*ROcp6_147);
    ACcp6_349 = qdd[3]+OMcp6_147*ORcp6_248+OMcp6_147*ORcp6_249+OMcp6_16*ORcp6_245-OMcp6_247*ORcp6_148-OMcp6_247*ORcp6_149-
 OMcp6_26*ORcp6_145+OPcp6_147*RLcp6_248+OPcp6_147*RLcp6_249+OPcp6_16*RLcp6_245-OPcp6_247*RLcp6_148-OPcp6_247*RLcp6_149-
 OPcp6_26*RLcp6_145+ROcp6_347*qdd[48]+(2.0)*qd[48]*(OMcp6_147*ROcp6_247-OMcp6_247*ROcp6_147);

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp6_149;
    sens->P[2] = POcp6_249;
    sens->P[3] = POcp6_349;
    sens->R[1][1] = ROcp6_149;
    sens->R[1][2] = ROcp6_249;
    sens->R[1][3] = ROcp6_349;
    sens->R[2][1] = ROcp6_446;
    sens->R[2][2] = ROcp6_546;
    sens->R[2][3] = ROcp6_646;
    sens->R[3][1] = ROcp6_749;
    sens->R[3][2] = ROcp6_849;
    sens->R[3][3] = ROcp6_949;
    sens->V[1] = VIcp6_149;
    sens->V[2] = VIcp6_249;
    sens->V[3] = VIcp6_349;
    sens->OM[1] = OMcp6_149;
    sens->OM[2] = OMcp6_249;
    sens->OM[3] = OMcp6_349;
    sens->A[1] = ACcp6_149;
    sens->A[2] = ACcp6_249;
    sens->A[3] = ACcp6_349;
    sens->OMP[1] = OPcp6_149;
    sens->OMP[2] = OPcp6_249;
    sens->OMP[3] = OPcp6_349;
 
// 
break;
case 8:
 


// = = Block_1_0_0_8_0_1 = = 
 
// Sensor Kinematics 


    ROcp7_45 = -S4*C5;
    ROcp7_55 = C4*C5;
    ROcp7_75 = S4*S5;
    ROcp7_85 = -C4*S5;
    ROcp7_16 = -(ROcp7_75*S6-C4*C6);
    ROcp7_26 = -(ROcp7_85*S6-S4*C6);
    ROcp7_36 = -C5*S6;
    ROcp7_76 = ROcp7_75*C6+C4*S6;
    ROcp7_86 = ROcp7_85*C6+S4*S6;
    ROcp7_96 = C5*C6;
    OMcp7_15 = qd[5]*C4;
    OMcp7_25 = qd[5]*S4;
    OMcp7_16 = OMcp7_15+ROcp7_45*qd[6];
    OMcp7_26 = OMcp7_25+ROcp7_55*qd[6];
    OMcp7_36 = qd[4]+qd[6]*S5;
    OPcp7_16 = ROcp7_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp7_25*S5-ROcp7_55*qd[4]);
    OPcp7_26 = ROcp7_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp7_15*S5-ROcp7_45*qd[4]);
    OPcp7_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_8_0_20 = = 
 
// Sensor Kinematics 


    ROcp7_145 = ROcp7_16*C45+ROcp7_45*S45;
    ROcp7_245 = ROcp7_26*C45+ROcp7_55*S45;
    ROcp7_345 = ROcp7_36*C45+S45*S5;
    ROcp7_445 = -(ROcp7_16*S45-ROcp7_45*C45);
    ROcp7_545 = -(ROcp7_26*S45-ROcp7_55*C45);
    ROcp7_645 = -(ROcp7_36*S45-C45*S5);
    ROcp7_446 = ROcp7_445*C46+ROcp7_76*S46;
    ROcp7_546 = ROcp7_545*C46+ROcp7_86*S46;
    ROcp7_646 = ROcp7_645*C46+ROcp7_96*S46;
    ROcp7_746 = -(ROcp7_445*S46-ROcp7_76*C46);
    ROcp7_846 = -(ROcp7_545*S46-ROcp7_86*C46);
    ROcp7_946 = -(ROcp7_645*S46-ROcp7_96*C46);
    ROcp7_147 = ROcp7_145*C47-ROcp7_746*S47;
    ROcp7_247 = ROcp7_245*C47-ROcp7_846*S47;
    ROcp7_347 = ROcp7_345*C47-ROcp7_946*S47;
    ROcp7_747 = ROcp7_145*S47+ROcp7_746*C47;
    ROcp7_847 = ROcp7_245*S47+ROcp7_846*C47;
    ROcp7_947 = ROcp7_345*S47+ROcp7_946*C47;
    RLcp7_145 = ROcp7_16*s->dpt[1][17]+ROcp7_76*s->dpt[3][17];
    RLcp7_245 = ROcp7_26*s->dpt[1][17]+ROcp7_86*s->dpt[3][17];
    RLcp7_345 = ROcp7_36*s->dpt[1][17]+ROcp7_96*s->dpt[3][17];
    OMcp7_145 = OMcp7_16+ROcp7_76*qd[45];
    OMcp7_245 = OMcp7_26+ROcp7_86*qd[45];
    OMcp7_345 = OMcp7_36+ROcp7_96*qd[45];
    ORcp7_145 = OMcp7_26*RLcp7_345-OMcp7_36*RLcp7_245;
    ORcp7_245 = -(OMcp7_16*RLcp7_345-OMcp7_36*RLcp7_145);
    ORcp7_345 = OMcp7_16*RLcp7_245-OMcp7_26*RLcp7_145;
    OMcp7_146 = OMcp7_145+ROcp7_145*qd[46];
    OMcp7_246 = OMcp7_245+ROcp7_245*qd[46];
    OMcp7_346 = OMcp7_345+ROcp7_345*qd[46];
    OMcp7_147 = OMcp7_146+ROcp7_446*qd[47];
    OMcp7_247 = OMcp7_246+ROcp7_546*qd[47];
    OMcp7_347 = OMcp7_346+ROcp7_646*qd[47];
    OPcp7_147 = OPcp7_16+ROcp7_145*qdd[46]+ROcp7_446*qdd[47]+ROcp7_76*qdd[45]+qd[45]*(OMcp7_26*ROcp7_96-OMcp7_36*ROcp7_86)
 +qd[46]*(OMcp7_245*ROcp7_345-OMcp7_345*ROcp7_245)+qd[47]*(OMcp7_246*ROcp7_646-OMcp7_346*ROcp7_546);
    OPcp7_247 = OPcp7_26+ROcp7_245*qdd[46]+ROcp7_546*qdd[47]+ROcp7_86*qdd[45]-qd[45]*(OMcp7_16*ROcp7_96-OMcp7_36*ROcp7_76)
 -qd[46]*(OMcp7_145*ROcp7_345-OMcp7_345*ROcp7_145)-qd[47]*(OMcp7_146*ROcp7_646-OMcp7_346*ROcp7_446);
    OPcp7_347 = OPcp7_36+ROcp7_345*qdd[46]+ROcp7_646*qdd[47]+ROcp7_96*qdd[45]+qd[45]*(OMcp7_16*ROcp7_86-OMcp7_26*ROcp7_76)
 +qd[46]*(OMcp7_145*ROcp7_245-OMcp7_245*ROcp7_145)+qd[47]*(OMcp7_146*ROcp7_546-OMcp7_246*ROcp7_446);
    RLcp7_148 = ROcp7_147*q[48];
    RLcp7_248 = ROcp7_247*q[48];
    RLcp7_348 = ROcp7_347*q[48];
    ORcp7_148 = OMcp7_247*RLcp7_348-OMcp7_347*RLcp7_248;
    ORcp7_248 = -(OMcp7_147*RLcp7_348-OMcp7_347*RLcp7_148);
    ORcp7_348 = OMcp7_147*RLcp7_248-OMcp7_247*RLcp7_148;

// = = Block_1_0_0_8_0_22 = = 
 
// Sensor Kinematics 


    ROcp7_150 = ROcp7_147*C50-ROcp7_747*S50;
    ROcp7_250 = ROcp7_247*C50-ROcp7_847*S50;
    ROcp7_350 = ROcp7_347*C50-ROcp7_947*S50;
    ROcp7_750 = ROcp7_147*S50+ROcp7_747*C50;
    ROcp7_850 = ROcp7_247*S50+ROcp7_847*C50;
    ROcp7_950 = ROcp7_347*S50+ROcp7_947*C50;
    RLcp7_150 = ROcp7_147*s->dpt[1][63]+ROcp7_446*s->dpt[2][63]+ROcp7_747*s->dpt[3][63];
    RLcp7_250 = ROcp7_247*s->dpt[1][63]+ROcp7_546*s->dpt[2][63]+ROcp7_847*s->dpt[3][63];
    RLcp7_350 = ROcp7_347*s->dpt[1][63]+ROcp7_646*s->dpt[2][63]+ROcp7_947*s->dpt[3][63];
    POcp7_150 = RLcp7_145+RLcp7_148+RLcp7_150+q[1];
    POcp7_250 = RLcp7_245+RLcp7_248+RLcp7_250+q[2];
    POcp7_350 = RLcp7_345+RLcp7_348+RLcp7_350+q[3];
    OMcp7_150 = OMcp7_147+ROcp7_446*qd[50];
    OMcp7_250 = OMcp7_247+ROcp7_546*qd[50];
    OMcp7_350 = OMcp7_347+ROcp7_646*qd[50];
    ORcp7_150 = OMcp7_247*RLcp7_350-OMcp7_347*RLcp7_250;
    ORcp7_250 = -(OMcp7_147*RLcp7_350-OMcp7_347*RLcp7_150);
    ORcp7_350 = OMcp7_147*RLcp7_250-OMcp7_247*RLcp7_150;
    VIcp7_150 = ORcp7_145+ORcp7_148+ORcp7_150+qd[1]+ROcp7_147*qd[48];
    VIcp7_250 = ORcp7_245+ORcp7_248+ORcp7_250+qd[2]+ROcp7_247*qd[48];
    VIcp7_350 = ORcp7_345+ORcp7_348+ORcp7_350+qd[3]+ROcp7_347*qd[48];
    OPcp7_150 = OPcp7_147+ROcp7_446*qdd[50]+qd[50]*(OMcp7_247*ROcp7_646-OMcp7_347*ROcp7_546);
    OPcp7_250 = OPcp7_247+ROcp7_546*qdd[50]-qd[50]*(OMcp7_147*ROcp7_646-OMcp7_347*ROcp7_446);
    OPcp7_350 = OPcp7_347+ROcp7_646*qdd[50]+qd[50]*(OMcp7_147*ROcp7_546-OMcp7_247*ROcp7_446);
    ACcp7_150 = qdd[1]+OMcp7_247*ORcp7_348+OMcp7_247*ORcp7_350+OMcp7_26*ORcp7_345-OMcp7_347*ORcp7_248-OMcp7_347*ORcp7_250-
 OMcp7_36*ORcp7_245+OPcp7_247*RLcp7_348+OPcp7_247*RLcp7_350+OPcp7_26*RLcp7_345-OPcp7_347*RLcp7_248-OPcp7_347*RLcp7_250-
 OPcp7_36*RLcp7_245+ROcp7_147*qdd[48]+(2.0)*qd[48]*(OMcp7_247*ROcp7_347-OMcp7_347*ROcp7_247);
    ACcp7_250 = qdd[2]-OMcp7_147*ORcp7_348-OMcp7_147*ORcp7_350-OMcp7_16*ORcp7_345+OMcp7_347*ORcp7_148+OMcp7_347*ORcp7_150+
 OMcp7_36*ORcp7_145-OPcp7_147*RLcp7_348-OPcp7_147*RLcp7_350-OPcp7_16*RLcp7_345+OPcp7_347*RLcp7_148+OPcp7_347*RLcp7_150+
 OPcp7_36*RLcp7_145+ROcp7_247*qdd[48]-(2.0)*qd[48]*(OMcp7_147*ROcp7_347-OMcp7_347*ROcp7_147);
    ACcp7_350 = qdd[3]+OMcp7_147*ORcp7_248+OMcp7_147*ORcp7_250+OMcp7_16*ORcp7_245-OMcp7_247*ORcp7_148-OMcp7_247*ORcp7_150-
 OMcp7_26*ORcp7_145+OPcp7_147*RLcp7_248+OPcp7_147*RLcp7_250+OPcp7_16*RLcp7_245-OPcp7_247*RLcp7_148-OPcp7_247*RLcp7_150-
 OPcp7_26*RLcp7_145+ROcp7_347*qdd[48]+(2.0)*qd[48]*(OMcp7_147*ROcp7_247-OMcp7_247*ROcp7_147);

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_150;
    sens->P[2] = POcp7_250;
    sens->P[3] = POcp7_350;
    sens->R[1][1] = ROcp7_150;
    sens->R[1][2] = ROcp7_250;
    sens->R[1][3] = ROcp7_350;
    sens->R[2][1] = ROcp7_446;
    sens->R[2][2] = ROcp7_546;
    sens->R[2][3] = ROcp7_646;
    sens->R[3][1] = ROcp7_750;
    sens->R[3][2] = ROcp7_850;
    sens->R[3][3] = ROcp7_950;
    sens->V[1] = VIcp7_150;
    sens->V[2] = VIcp7_250;
    sens->V[3] = VIcp7_350;
    sens->OM[1] = OMcp7_150;
    sens->OM[2] = OMcp7_250;
    sens->OM[3] = OMcp7_350;
    sens->A[1] = ACcp7_150;
    sens->A[2] = ACcp7_250;
    sens->A[3] = ACcp7_350;
    sens->OMP[1] = OPcp7_150;
    sens->OMP[2] = OPcp7_250;
    sens->OMP[3] = OPcp7_350;
 
// 
break;
case 9:
 


// = = Block_1_0_0_9_0_1 = = 
 
// Sensor Kinematics 


    ROcp8_45 = -S4*C5;
    ROcp8_55 = C4*C5;
    ROcp8_75 = S4*S5;
    ROcp8_85 = -C4*S5;
    ROcp8_16 = -(ROcp8_75*S6-C4*C6);
    ROcp8_26 = -(ROcp8_85*S6-S4*C6);
    ROcp8_36 = -C5*S6;
    ROcp8_76 = ROcp8_75*C6+C4*S6;
    ROcp8_86 = ROcp8_85*C6+S4*S6;
    ROcp8_96 = C5*C6;
    OMcp8_15 = qd[5]*C4;
    OMcp8_25 = qd[5]*S4;
    OMcp8_16 = OMcp8_15+ROcp8_45*qd[6];
    OMcp8_26 = OMcp8_25+ROcp8_55*qd[6];
    OMcp8_36 = qd[4]+qd[6]*S5;
    OPcp8_16 = ROcp8_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp8_25*S5-ROcp8_55*qd[4]);
    OPcp8_26 = ROcp8_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp8_15*S5-ROcp8_45*qd[4]);
    OPcp8_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_9_0_20 = = 
 
// Sensor Kinematics 


    ROcp8_145 = ROcp8_16*C45+ROcp8_45*S45;
    ROcp8_245 = ROcp8_26*C45+ROcp8_55*S45;
    ROcp8_345 = ROcp8_36*C45+S45*S5;
    ROcp8_445 = -(ROcp8_16*S45-ROcp8_45*C45);
    ROcp8_545 = -(ROcp8_26*S45-ROcp8_55*C45);
    ROcp8_645 = -(ROcp8_36*S45-C45*S5);
    ROcp8_446 = ROcp8_445*C46+ROcp8_76*S46;
    ROcp8_546 = ROcp8_545*C46+ROcp8_86*S46;
    ROcp8_646 = ROcp8_645*C46+ROcp8_96*S46;
    ROcp8_746 = -(ROcp8_445*S46-ROcp8_76*C46);
    ROcp8_846 = -(ROcp8_545*S46-ROcp8_86*C46);
    ROcp8_946 = -(ROcp8_645*S46-ROcp8_96*C46);
    ROcp8_147 = ROcp8_145*C47-ROcp8_746*S47;
    ROcp8_247 = ROcp8_245*C47-ROcp8_846*S47;
    ROcp8_347 = ROcp8_345*C47-ROcp8_946*S47;
    ROcp8_747 = ROcp8_145*S47+ROcp8_746*C47;
    ROcp8_847 = ROcp8_245*S47+ROcp8_846*C47;
    ROcp8_947 = ROcp8_345*S47+ROcp8_946*C47;
    RLcp8_145 = ROcp8_16*s->dpt[1][17]+ROcp8_76*s->dpt[3][17];
    RLcp8_245 = ROcp8_26*s->dpt[1][17]+ROcp8_86*s->dpt[3][17];
    RLcp8_345 = ROcp8_36*s->dpt[1][17]+ROcp8_96*s->dpt[3][17];
    OMcp8_145 = OMcp8_16+ROcp8_76*qd[45];
    OMcp8_245 = OMcp8_26+ROcp8_86*qd[45];
    OMcp8_345 = OMcp8_36+ROcp8_96*qd[45];
    ORcp8_145 = OMcp8_26*RLcp8_345-OMcp8_36*RLcp8_245;
    ORcp8_245 = -(OMcp8_16*RLcp8_345-OMcp8_36*RLcp8_145);
    ORcp8_345 = OMcp8_16*RLcp8_245-OMcp8_26*RLcp8_145;
    OMcp8_146 = OMcp8_145+ROcp8_145*qd[46];
    OMcp8_246 = OMcp8_245+ROcp8_245*qd[46];
    OMcp8_346 = OMcp8_345+ROcp8_345*qd[46];
    OMcp8_147 = OMcp8_146+ROcp8_446*qd[47];
    OMcp8_247 = OMcp8_246+ROcp8_546*qd[47];
    OMcp8_347 = OMcp8_346+ROcp8_646*qd[47];
    OPcp8_147 = OPcp8_16+ROcp8_145*qdd[46]+ROcp8_446*qdd[47]+ROcp8_76*qdd[45]+qd[45]*(OMcp8_26*ROcp8_96-OMcp8_36*ROcp8_86)
 +qd[46]*(OMcp8_245*ROcp8_345-OMcp8_345*ROcp8_245)+qd[47]*(OMcp8_246*ROcp8_646-OMcp8_346*ROcp8_546);
    OPcp8_247 = OPcp8_26+ROcp8_245*qdd[46]+ROcp8_546*qdd[47]+ROcp8_86*qdd[45]-qd[45]*(OMcp8_16*ROcp8_96-OMcp8_36*ROcp8_76)
 -qd[46]*(OMcp8_145*ROcp8_345-OMcp8_345*ROcp8_145)-qd[47]*(OMcp8_146*ROcp8_646-OMcp8_346*ROcp8_446);
    OPcp8_347 = OPcp8_36+ROcp8_345*qdd[46]+ROcp8_646*qdd[47]+ROcp8_96*qdd[45]+qd[45]*(OMcp8_16*ROcp8_86-OMcp8_26*ROcp8_76)
 +qd[46]*(OMcp8_145*ROcp8_245-OMcp8_245*ROcp8_145)+qd[47]*(OMcp8_146*ROcp8_546-OMcp8_246*ROcp8_446);
    RLcp8_148 = ROcp8_147*q[48];
    RLcp8_248 = ROcp8_247*q[48];
    RLcp8_348 = ROcp8_347*q[48];
    ORcp8_148 = OMcp8_247*RLcp8_348-OMcp8_347*RLcp8_248;
    ORcp8_248 = -(OMcp8_147*RLcp8_348-OMcp8_347*RLcp8_148);
    ORcp8_348 = OMcp8_147*RLcp8_248-OMcp8_247*RLcp8_148;

// = = Block_1_0_0_9_0_23 = = 
 
// Sensor Kinematics 


    ROcp8_151 = ROcp8_147*C51-ROcp8_747*S51;
    ROcp8_251 = ROcp8_247*C51-ROcp8_847*S51;
    ROcp8_351 = ROcp8_347*C51-ROcp8_947*S51;
    ROcp8_751 = ROcp8_147*S51+ROcp8_747*C51;
    ROcp8_851 = ROcp8_247*S51+ROcp8_847*C51;
    ROcp8_951 = ROcp8_347*S51+ROcp8_947*C51;
    RLcp8_151 = ROcp8_147*s->dpt[1][64]+ROcp8_446*s->dpt[2][64]+ROcp8_747*s->dpt[3][64];
    RLcp8_251 = ROcp8_247*s->dpt[1][64]+ROcp8_546*s->dpt[2][64]+ROcp8_847*s->dpt[3][64];
    RLcp8_351 = ROcp8_347*s->dpt[1][64]+ROcp8_646*s->dpt[2][64]+ROcp8_947*s->dpt[3][64];
    POcp8_151 = RLcp8_145+RLcp8_148+RLcp8_151+q[1];
    POcp8_251 = RLcp8_245+RLcp8_248+RLcp8_251+q[2];
    POcp8_351 = RLcp8_345+RLcp8_348+RLcp8_351+q[3];
    OMcp8_151 = OMcp8_147+ROcp8_446*qd[51];
    OMcp8_251 = OMcp8_247+ROcp8_546*qd[51];
    OMcp8_351 = OMcp8_347+ROcp8_646*qd[51];
    ORcp8_151 = OMcp8_247*RLcp8_351-OMcp8_347*RLcp8_251;
    ORcp8_251 = -(OMcp8_147*RLcp8_351-OMcp8_347*RLcp8_151);
    ORcp8_351 = OMcp8_147*RLcp8_251-OMcp8_247*RLcp8_151;
    VIcp8_151 = ORcp8_145+ORcp8_148+ORcp8_151+qd[1]+ROcp8_147*qd[48];
    VIcp8_251 = ORcp8_245+ORcp8_248+ORcp8_251+qd[2]+ROcp8_247*qd[48];
    VIcp8_351 = ORcp8_345+ORcp8_348+ORcp8_351+qd[3]+ROcp8_347*qd[48];
    OPcp8_151 = OPcp8_147+ROcp8_446*qdd[51]+qd[51]*(OMcp8_247*ROcp8_646-OMcp8_347*ROcp8_546);
    OPcp8_251 = OPcp8_247+ROcp8_546*qdd[51]-qd[51]*(OMcp8_147*ROcp8_646-OMcp8_347*ROcp8_446);
    OPcp8_351 = OPcp8_347+ROcp8_646*qdd[51]+qd[51]*(OMcp8_147*ROcp8_546-OMcp8_247*ROcp8_446);
    ACcp8_151 = qdd[1]+OMcp8_247*ORcp8_348+OMcp8_247*ORcp8_351+OMcp8_26*ORcp8_345-OMcp8_347*ORcp8_248-OMcp8_347*ORcp8_251-
 OMcp8_36*ORcp8_245+OPcp8_247*RLcp8_348+OPcp8_247*RLcp8_351+OPcp8_26*RLcp8_345-OPcp8_347*RLcp8_248-OPcp8_347*RLcp8_251-
 OPcp8_36*RLcp8_245+ROcp8_147*qdd[48]+(2.0)*qd[48]*(OMcp8_247*ROcp8_347-OMcp8_347*ROcp8_247);
    ACcp8_251 = qdd[2]-OMcp8_147*ORcp8_348-OMcp8_147*ORcp8_351-OMcp8_16*ORcp8_345+OMcp8_347*ORcp8_148+OMcp8_347*ORcp8_151+
 OMcp8_36*ORcp8_145-OPcp8_147*RLcp8_348-OPcp8_147*RLcp8_351-OPcp8_16*RLcp8_345+OPcp8_347*RLcp8_148+OPcp8_347*RLcp8_151+
 OPcp8_36*RLcp8_145+ROcp8_247*qdd[48]-(2.0)*qd[48]*(OMcp8_147*ROcp8_347-OMcp8_347*ROcp8_147);
    ACcp8_351 = qdd[3]+OMcp8_147*ORcp8_248+OMcp8_147*ORcp8_251+OMcp8_16*ORcp8_245-OMcp8_247*ORcp8_148-OMcp8_247*ORcp8_151-
 OMcp8_26*ORcp8_145+OPcp8_147*RLcp8_248+OPcp8_147*RLcp8_251+OPcp8_16*RLcp8_245-OPcp8_247*RLcp8_148-OPcp8_247*RLcp8_151-
 OPcp8_26*RLcp8_145+ROcp8_347*qdd[48]+(2.0)*qd[48]*(OMcp8_147*ROcp8_247-OMcp8_247*ROcp8_147);

// = = Block_1_0_0_9_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp8_151;
    sens->P[2] = POcp8_251;
    sens->P[3] = POcp8_351;
    sens->R[1][1] = ROcp8_151;
    sens->R[1][2] = ROcp8_251;
    sens->R[1][3] = ROcp8_351;
    sens->R[2][1] = ROcp8_446;
    sens->R[2][2] = ROcp8_546;
    sens->R[2][3] = ROcp8_646;
    sens->R[3][1] = ROcp8_751;
    sens->R[3][2] = ROcp8_851;
    sens->R[3][3] = ROcp8_951;
    sens->V[1] = VIcp8_151;
    sens->V[2] = VIcp8_251;
    sens->V[3] = VIcp8_351;
    sens->OM[1] = OMcp8_151;
    sens->OM[2] = OMcp8_251;
    sens->OM[3] = OMcp8_351;
    sens->A[1] = ACcp8_151;
    sens->A[2] = ACcp8_251;
    sens->A[3] = ACcp8_351;
    sens->OMP[1] = OPcp8_151;
    sens->OMP[2] = OPcp8_251;
    sens->OMP[3] = OPcp8_351;
 
// 
break;
case 10:
 


// = = Block_1_0_0_10_0_1 = = 
 
// Sensor Kinematics 


    ROcp9_45 = -S4*C5;
    ROcp9_55 = C4*C5;
    ROcp9_75 = S4*S5;
    ROcp9_85 = -C4*S5;
    ROcp9_16 = -(ROcp9_75*S6-C4*C6);
    ROcp9_26 = -(ROcp9_85*S6-S4*C6);
    ROcp9_36 = -C5*S6;
    ROcp9_76 = ROcp9_75*C6+C4*S6;
    ROcp9_86 = ROcp9_85*C6+S4*S6;
    ROcp9_96 = C5*C6;
    OMcp9_15 = qd[5]*C4;
    OMcp9_25 = qd[5]*S4;
    OMcp9_16 = OMcp9_15+ROcp9_45*qd[6];
    OMcp9_26 = OMcp9_25+ROcp9_55*qd[6];
    OMcp9_36 = qd[4]+qd[6]*S5;
    OPcp9_16 = ROcp9_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp9_25*S5-ROcp9_55*qd[4]);
    OPcp9_26 = ROcp9_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp9_15*S5-ROcp9_45*qd[4]);
    OPcp9_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_10_0_20 = = 
 
// Sensor Kinematics 


    ROcp9_145 = ROcp9_16*C45+ROcp9_45*S45;
    ROcp9_245 = ROcp9_26*C45+ROcp9_55*S45;
    ROcp9_345 = ROcp9_36*C45+S45*S5;
    ROcp9_445 = -(ROcp9_16*S45-ROcp9_45*C45);
    ROcp9_545 = -(ROcp9_26*S45-ROcp9_55*C45);
    ROcp9_645 = -(ROcp9_36*S45-C45*S5);
    ROcp9_446 = ROcp9_445*C46+ROcp9_76*S46;
    ROcp9_546 = ROcp9_545*C46+ROcp9_86*S46;
    ROcp9_646 = ROcp9_645*C46+ROcp9_96*S46;
    ROcp9_746 = -(ROcp9_445*S46-ROcp9_76*C46);
    ROcp9_846 = -(ROcp9_545*S46-ROcp9_86*C46);
    ROcp9_946 = -(ROcp9_645*S46-ROcp9_96*C46);
    ROcp9_147 = ROcp9_145*C47-ROcp9_746*S47;
    ROcp9_247 = ROcp9_245*C47-ROcp9_846*S47;
    ROcp9_347 = ROcp9_345*C47-ROcp9_946*S47;
    ROcp9_747 = ROcp9_145*S47+ROcp9_746*C47;
    ROcp9_847 = ROcp9_245*S47+ROcp9_846*C47;
    ROcp9_947 = ROcp9_345*S47+ROcp9_946*C47;
    RLcp9_145 = ROcp9_16*s->dpt[1][17]+ROcp9_76*s->dpt[3][17];
    RLcp9_245 = ROcp9_26*s->dpt[1][17]+ROcp9_86*s->dpt[3][17];
    RLcp9_345 = ROcp9_36*s->dpt[1][17]+ROcp9_96*s->dpt[3][17];
    OMcp9_145 = OMcp9_16+ROcp9_76*qd[45];
    OMcp9_245 = OMcp9_26+ROcp9_86*qd[45];
    OMcp9_345 = OMcp9_36+ROcp9_96*qd[45];
    ORcp9_145 = OMcp9_26*RLcp9_345-OMcp9_36*RLcp9_245;
    ORcp9_245 = -(OMcp9_16*RLcp9_345-OMcp9_36*RLcp9_145);
    ORcp9_345 = OMcp9_16*RLcp9_245-OMcp9_26*RLcp9_145;
    OMcp9_146 = OMcp9_145+ROcp9_145*qd[46];
    OMcp9_246 = OMcp9_245+ROcp9_245*qd[46];
    OMcp9_346 = OMcp9_345+ROcp9_345*qd[46];
    OMcp9_147 = OMcp9_146+ROcp9_446*qd[47];
    OMcp9_247 = OMcp9_246+ROcp9_546*qd[47];
    OMcp9_347 = OMcp9_346+ROcp9_646*qd[47];
    OPcp9_147 = OPcp9_16+ROcp9_145*qdd[46]+ROcp9_446*qdd[47]+ROcp9_76*qdd[45]+qd[45]*(OMcp9_26*ROcp9_96-OMcp9_36*ROcp9_86)
 +qd[46]*(OMcp9_245*ROcp9_345-OMcp9_345*ROcp9_245)+qd[47]*(OMcp9_246*ROcp9_646-OMcp9_346*ROcp9_546);
    OPcp9_247 = OPcp9_26+ROcp9_245*qdd[46]+ROcp9_546*qdd[47]+ROcp9_86*qdd[45]-qd[45]*(OMcp9_16*ROcp9_96-OMcp9_36*ROcp9_76)
 -qd[46]*(OMcp9_145*ROcp9_345-OMcp9_345*ROcp9_145)-qd[47]*(OMcp9_146*ROcp9_646-OMcp9_346*ROcp9_446);
    OPcp9_347 = OPcp9_36+ROcp9_345*qdd[46]+ROcp9_646*qdd[47]+ROcp9_96*qdd[45]+qd[45]*(OMcp9_16*ROcp9_86-OMcp9_26*ROcp9_76)
 +qd[46]*(OMcp9_145*ROcp9_245-OMcp9_245*ROcp9_145)+qd[47]*(OMcp9_146*ROcp9_546-OMcp9_246*ROcp9_446);
    RLcp9_148 = ROcp9_147*q[48];
    RLcp9_248 = ROcp9_247*q[48];
    RLcp9_348 = ROcp9_347*q[48];
    ORcp9_148 = OMcp9_247*RLcp9_348-OMcp9_347*RLcp9_248;
    ORcp9_248 = -(OMcp9_147*RLcp9_348-OMcp9_347*RLcp9_148);
    ORcp9_348 = OMcp9_147*RLcp9_248-OMcp9_247*RLcp9_148;

// = = Block_1_0_0_10_0_24 = = 
 
// Sensor Kinematics 


    ROcp9_152 = ROcp9_147*C52-ROcp9_747*S52;
    ROcp9_252 = ROcp9_247*C52-ROcp9_847*S52;
    ROcp9_352 = ROcp9_347*C52-ROcp9_947*S52;
    ROcp9_752 = ROcp9_147*S52+ROcp9_747*C52;
    ROcp9_852 = ROcp9_247*S52+ROcp9_847*C52;
    ROcp9_952 = ROcp9_347*S52+ROcp9_947*C52;
    RLcp9_152 = ROcp9_147*s->dpt[1][65]+ROcp9_446*s->dpt[2][65]+ROcp9_747*s->dpt[3][65];
    RLcp9_252 = ROcp9_247*s->dpt[1][65]+ROcp9_546*s->dpt[2][65]+ROcp9_847*s->dpt[3][65];
    RLcp9_352 = ROcp9_347*s->dpt[1][65]+ROcp9_646*s->dpt[2][65]+ROcp9_947*s->dpt[3][65];
    POcp9_152 = RLcp9_145+RLcp9_148+RLcp9_152+q[1];
    POcp9_252 = RLcp9_245+RLcp9_248+RLcp9_252+q[2];
    POcp9_352 = RLcp9_345+RLcp9_348+RLcp9_352+q[3];
    OMcp9_152 = OMcp9_147+ROcp9_446*qd[52];
    OMcp9_252 = OMcp9_247+ROcp9_546*qd[52];
    OMcp9_352 = OMcp9_347+ROcp9_646*qd[52];
    ORcp9_152 = OMcp9_247*RLcp9_352-OMcp9_347*RLcp9_252;
    ORcp9_252 = -(OMcp9_147*RLcp9_352-OMcp9_347*RLcp9_152);
    ORcp9_352 = OMcp9_147*RLcp9_252-OMcp9_247*RLcp9_152;
    VIcp9_152 = ORcp9_145+ORcp9_148+ORcp9_152+qd[1]+ROcp9_147*qd[48];
    VIcp9_252 = ORcp9_245+ORcp9_248+ORcp9_252+qd[2]+ROcp9_247*qd[48];
    VIcp9_352 = ORcp9_345+ORcp9_348+ORcp9_352+qd[3]+ROcp9_347*qd[48];
    OPcp9_152 = OPcp9_147+ROcp9_446*qdd[52]+qd[52]*(OMcp9_247*ROcp9_646-OMcp9_347*ROcp9_546);
    OPcp9_252 = OPcp9_247+ROcp9_546*qdd[52]-qd[52]*(OMcp9_147*ROcp9_646-OMcp9_347*ROcp9_446);
    OPcp9_352 = OPcp9_347+ROcp9_646*qdd[52]+qd[52]*(OMcp9_147*ROcp9_546-OMcp9_247*ROcp9_446);
    ACcp9_152 = qdd[1]+OMcp9_247*ORcp9_348+OMcp9_247*ORcp9_352+OMcp9_26*ORcp9_345-OMcp9_347*ORcp9_248-OMcp9_347*ORcp9_252-
 OMcp9_36*ORcp9_245+OPcp9_247*RLcp9_348+OPcp9_247*RLcp9_352+OPcp9_26*RLcp9_345-OPcp9_347*RLcp9_248-OPcp9_347*RLcp9_252-
 OPcp9_36*RLcp9_245+ROcp9_147*qdd[48]+(2.0)*qd[48]*(OMcp9_247*ROcp9_347-OMcp9_347*ROcp9_247);
    ACcp9_252 = qdd[2]-OMcp9_147*ORcp9_348-OMcp9_147*ORcp9_352-OMcp9_16*ORcp9_345+OMcp9_347*ORcp9_148+OMcp9_347*ORcp9_152+
 OMcp9_36*ORcp9_145-OPcp9_147*RLcp9_348-OPcp9_147*RLcp9_352-OPcp9_16*RLcp9_345+OPcp9_347*RLcp9_148+OPcp9_347*RLcp9_152+
 OPcp9_36*RLcp9_145+ROcp9_247*qdd[48]-(2.0)*qd[48]*(OMcp9_147*ROcp9_347-OMcp9_347*ROcp9_147);
    ACcp9_352 = qdd[3]+OMcp9_147*ORcp9_248+OMcp9_147*ORcp9_252+OMcp9_16*ORcp9_245-OMcp9_247*ORcp9_148-OMcp9_247*ORcp9_152-
 OMcp9_26*ORcp9_145+OPcp9_147*RLcp9_248+OPcp9_147*RLcp9_252+OPcp9_16*RLcp9_245-OPcp9_247*RLcp9_148-OPcp9_247*RLcp9_152-
 OPcp9_26*RLcp9_145+ROcp9_347*qdd[48]+(2.0)*qd[48]*(OMcp9_147*ROcp9_247-OMcp9_247*ROcp9_147);

// = = Block_1_0_0_10_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp9_152;
    sens->P[2] = POcp9_252;
    sens->P[3] = POcp9_352;
    sens->R[1][1] = ROcp9_152;
    sens->R[1][2] = ROcp9_252;
    sens->R[1][3] = ROcp9_352;
    sens->R[2][1] = ROcp9_446;
    sens->R[2][2] = ROcp9_546;
    sens->R[2][3] = ROcp9_646;
    sens->R[3][1] = ROcp9_752;
    sens->R[3][2] = ROcp9_852;
    sens->R[3][3] = ROcp9_952;
    sens->V[1] = VIcp9_152;
    sens->V[2] = VIcp9_252;
    sens->V[3] = VIcp9_352;
    sens->OM[1] = OMcp9_152;
    sens->OM[2] = OMcp9_252;
    sens->OM[3] = OMcp9_352;
    sens->A[1] = ACcp9_152;
    sens->A[2] = ACcp9_252;
    sens->A[3] = ACcp9_352;
    sens->OMP[1] = OPcp9_152;
    sens->OMP[2] = OPcp9_252;
    sens->OMP[3] = OPcp9_352;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 

