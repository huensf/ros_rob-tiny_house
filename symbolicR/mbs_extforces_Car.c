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
//	==> Function : F19 : External Forces
//	==> Flops complexity : 3534
//
//	==> Generation Time :  0.060 seconds
//	==> Post-Processing :  0.070 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_extforces(double **frc,double **trq,
MbsData *s, double tsim)

// double frc[3][53];
// double trq[3][53];
{ 
double PxF1[4]; 
double RxF1[4][4]; 
double VxF1[4]; 
double OMxF1[4]; 
double AxF1[4]; 
double OMPxF1[4]; 
double *SWr1; 
double PxF2[4]; 
double RxF2[4][4]; 
double VxF2[4]; 
double OMxF2[4]; 
double AxF2[4]; 
double OMPxF2[4]; 
double *SWr2; 
double PxF3[4]; 
double RxF3[4][4]; 
double VxF3[4]; 
double OMxF3[4]; 
double AxF3[4]; 
double OMPxF3[4]; 
double *SWr3; 
double PxF4[4]; 
double RxF4[4][4]; 
double VxF4[4]; 
double OMxF4[4]; 
double AxF4[4]; 
double OMPxF4[4]; 
double *SWr4; 
double PxF5[4]; 
double RxF5[4][4]; 
double VxF5[4]; 
double OMxF5[4]; 
double AxF5[4]; 
double OMPxF5[4]; 
double *SWr5; 
double PxF6[4]; 
double RxF6[4][4]; 
double VxF6[4]; 
double OMxF6[4]; 
double AxF6[4]; 
double OMPxF6[4]; 
double *SWr6; 
double PxF7[4]; 
double RxF7[4][4]; 
double VxF7[4]; 
double OMxF7[4]; 
double AxF7[4]; 
double OMPxF7[4]; 
double *SWr7; 
double PxF8[4]; 
double RxF8[4][4]; 
double VxF8[4]; 
double OMxF8[4]; 
double AxF8[4]; 
double OMPxF8[4]; 
double *SWr8; 
 
#include "mbs_extforces_Car.h" 
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

// = = Block_0_0_1_1_0_1 = = 
 
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
  OMcp2_16 = OMcp2_15+qd[6]*ROcp2_45;
  OMcp2_26 = OMcp2_25+qd[6]*ROcp2_55;
  OMcp2_36 = qd[4]+qd[6]*S5;
  OPcp2_16 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp2_55-OMcp2_25*S5)-qdd[5]*C4-qdd[6]*ROcp2_45);
  OPcp2_26 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp2_45-OMcp2_15*S5)+qdd[5]*S4+qdd[6]*ROcp2_55;
  OPcp2_36 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;

// = = Block_0_0_1_1_0_3 = = 
 
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
  OMcp2_18 = OMcp2_16+qd[8]*ROcp2_16;
  OMcp2_28 = OMcp2_26+qd[8]*ROcp2_26;
  OMcp2_38 = OMcp2_36+qd[8]*ROcp2_36;
  ORcp2_18 = OMcp2_26*RLcp2_38-OMcp2_36*RLcp2_28;
  ORcp2_28 = -(OMcp2_16*RLcp2_38-OMcp2_36*RLcp2_18);
  ORcp2_38 = OMcp2_16*RLcp2_28-OMcp2_26*RLcp2_18;
  OPcp2_18 = OPcp2_16+qd[8]*(OMcp2_26*ROcp2_36-OMcp2_36*ROcp2_26)+qdd[8]*ROcp2_16;
  OPcp2_28 = OPcp2_26-qd[8]*(OMcp2_16*ROcp2_36-OMcp2_36*ROcp2_16)+qdd[8]*ROcp2_26;
  OPcp2_38 = OPcp2_36+qd[8]*(OMcp2_16*ROcp2_26-OMcp2_26*ROcp2_16)+qdd[8]*ROcp2_36;
  RLcp2_19 = ROcp2_48*s->dpt[2][18];
  RLcp2_29 = ROcp2_58*s->dpt[2][18];
  RLcp2_39 = ROcp2_68*s->dpt[2][18];
  OMcp2_19 = OMcp2_18+qd[9]*ROcp2_48;
  OMcp2_29 = OMcp2_28+qd[9]*ROcp2_58;
  OMcp2_39 = OMcp2_38+qd[9]*ROcp2_68;
  ORcp2_19 = OMcp2_28*RLcp2_39-OMcp2_38*RLcp2_29;
  ORcp2_29 = -(OMcp2_18*RLcp2_39-OMcp2_38*RLcp2_19);
  ORcp2_39 = OMcp2_18*RLcp2_29-OMcp2_28*RLcp2_19;
  OMcp2_110 = OMcp2_19+qd[10]*ROcp2_19;
  OMcp2_210 = OMcp2_29+qd[10]*ROcp2_29;
  OMcp2_310 = OMcp2_39+qd[10]*ROcp2_39;
  OMcp2_111 = OMcp2_110+qd[11]*ROcp2_710;
  OMcp2_211 = OMcp2_210+qd[11]*ROcp2_810;
  OMcp2_311 = OMcp2_310+qd[11]*ROcp2_910;
  OPcp2_111 = OPcp2_18+qd[10]*(OMcp2_29*ROcp2_39-OMcp2_39*ROcp2_29)+qd[11]*(OMcp2_210*ROcp2_910-OMcp2_310*ROcp2_810)+
 qd[9]*(OMcp2_28*ROcp2_68-OMcp2_38*ROcp2_58)+qdd[10]*ROcp2_19+qdd[11]*ROcp2_710+qdd[9]*ROcp2_48;
  OPcp2_211 = OPcp2_28-qd[10]*(OMcp2_19*ROcp2_39-OMcp2_39*ROcp2_19)-qd[11]*(OMcp2_110*ROcp2_910-OMcp2_310*ROcp2_710)-
 qd[9]*(OMcp2_18*ROcp2_68-OMcp2_38*ROcp2_48)+qdd[10]*ROcp2_29+qdd[11]*ROcp2_810+qdd[9]*ROcp2_58;
  OPcp2_311 = OPcp2_38+qd[10]*(OMcp2_19*ROcp2_29-OMcp2_29*ROcp2_19)+qd[11]*(OMcp2_110*ROcp2_810-OMcp2_210*ROcp2_710)+
 qd[9]*(OMcp2_18*ROcp2_58-OMcp2_28*ROcp2_48)+qdd[10]*ROcp2_39+qdd[11]*ROcp2_910+qdd[9]*ROcp2_68;
  RLcp2_112 = ROcp2_710*s->dpt[3][22];
  RLcp2_212 = ROcp2_810*s->dpt[3][22];
  RLcp2_312 = ROcp2_910*s->dpt[3][22];
  ORcp2_112 = OMcp2_211*RLcp2_312-OMcp2_311*RLcp2_212;
  ORcp2_212 = -(OMcp2_111*RLcp2_312-OMcp2_311*RLcp2_112);
  ORcp2_312 = OMcp2_111*RLcp2_212-OMcp2_211*RLcp2_112;
  PxF1[1] = q[1]+RLcp2_112+RLcp2_18+RLcp2_19;
  PxF1[2] = q[2]+RLcp2_212+RLcp2_28+RLcp2_29;
  PxF1[3] = q[3]+RLcp2_312+RLcp2_38+RLcp2_39;
  RxF1[1][1] = ROcp2_112;
  RxF1[1][2] = ROcp2_212;
  RxF1[1][3] = ROcp2_312;
  RxF1[2][1] = ROcp2_411;
  RxF1[2][2] = ROcp2_511;
  RxF1[2][3] = ROcp2_611;
  RxF1[3][1] = ROcp2_712;
  RxF1[3][2] = ROcp2_812;
  RxF1[3][3] = ROcp2_912;
  VxF1[1] = qd[1]+ORcp2_112+ORcp2_18+ORcp2_19;
  VxF1[2] = qd[2]+ORcp2_212+ORcp2_28+ORcp2_29;
  VxF1[3] = qd[3]+ORcp2_312+ORcp2_38+ORcp2_39;
  OMxF1[1] = OMcp2_111+qd[12]*ROcp2_411;
  OMxF1[2] = OMcp2_211+qd[12]*ROcp2_511;
  OMxF1[3] = OMcp2_311+qd[12]*ROcp2_611;
  AxF1[1] = qdd[1]+OMcp2_211*ORcp2_312+OMcp2_26*ORcp2_38+OMcp2_28*ORcp2_39-OMcp2_311*ORcp2_212-OMcp2_36*ORcp2_28-
 OMcp2_38*ORcp2_29+OPcp2_211*RLcp2_312+OPcp2_26*RLcp2_38+OPcp2_28*RLcp2_39-OPcp2_311*RLcp2_212-OPcp2_36*RLcp2_28-OPcp2_38*
 RLcp2_29;
  AxF1[2] = qdd[2]-OMcp2_111*ORcp2_312-OMcp2_16*ORcp2_38-OMcp2_18*ORcp2_39+OMcp2_311*ORcp2_112+OMcp2_36*ORcp2_18+
 OMcp2_38*ORcp2_19-OPcp2_111*RLcp2_312-OPcp2_16*RLcp2_38-OPcp2_18*RLcp2_39+OPcp2_311*RLcp2_112+OPcp2_36*RLcp2_18+OPcp2_38*
 RLcp2_19;
  AxF1[3] = qdd[3]+OMcp2_111*ORcp2_212+OMcp2_16*ORcp2_28+OMcp2_18*ORcp2_29-OMcp2_211*ORcp2_112-OMcp2_26*ORcp2_18-
 OMcp2_28*ORcp2_19+OPcp2_111*RLcp2_212+OPcp2_16*RLcp2_28+OPcp2_18*RLcp2_29-OPcp2_211*RLcp2_112-OPcp2_26*RLcp2_18-OPcp2_28*
 RLcp2_19;
  OMPxF1[1] = OPcp2_111+qd[12]*(OMcp2_211*ROcp2_611-OMcp2_311*ROcp2_511)+qdd[12]*ROcp2_411;
  OMPxF1[2] = OPcp2_211-qd[12]*(OMcp2_111*ROcp2_611-OMcp2_311*ROcp2_411)+qdd[12]*ROcp2_511;
  OMPxF1[3] = OPcp2_311+qd[12]*(OMcp2_111*ROcp2_511-OMcp2_211*ROcp2_411)+qdd[12]*ROcp2_611;
 
// Sensor Forces Computation 

  SWr1 = user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc13 = ROcp2_112*SWr1[1]+ROcp2_212*SWr1[2]+ROcp2_312*SWr1[3];
  xfrc23 = ROcp2_411*SWr1[1]+ROcp2_511*SWr1[2]+ROcp2_611*SWr1[3];
  xfrc33 = ROcp2_712*SWr1[1]+ROcp2_812*SWr1[2]+ROcp2_912*SWr1[3];
  frc[1][12] = s->frc[1][12]+xfrc13;
  frc[2][12] = s->frc[2][12]+xfrc23;
  frc[3][12] = s->frc[3][12]+xfrc33;
  xtrq13 = ROcp2_112*SWr1[4]+ROcp2_212*SWr1[5]+ROcp2_312*SWr1[6];
  xtrq23 = ROcp2_411*SWr1[4]+ROcp2_511*SWr1[5]+ROcp2_611*SWr1[6];
  xtrq33 = ROcp2_712*SWr1[4]+ROcp2_812*SWr1[5]+ROcp2_912*SWr1[6];
  trq[1][12] = s->trq[1][12]+xtrq13-xfrc23*SWr1[9]+xfrc33*SWr1[8];
  trq[2][12] = s->trq[2][12]+xtrq23+xfrc13*SWr1[9]-xfrc33*SWr1[7];
  trq[3][12] = s->trq[3][12]+xtrq33-xfrc13*SWr1[8]+xfrc23*SWr1[7];

// = = Block_0_0_1_2_0_1 = = 
 
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
  OMcp3_16 = OMcp3_15+qd[6]*ROcp3_45;
  OMcp3_26 = OMcp3_25+qd[6]*ROcp3_55;
  OMcp3_36 = qd[4]+qd[6]*S5;
  OPcp3_16 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp3_55-OMcp3_25*S5)-qdd[5]*C4-qdd[6]*ROcp3_45);
  OPcp3_26 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp3_45-OMcp3_15*S5)+qdd[5]*S4+qdd[6]*ROcp3_55;
  OPcp3_36 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;

// = = Block_0_0_1_2_0_4 = = 
 
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
  OMcp3_113 = OMcp3_16+qd[13]*ROcp3_16;
  OMcp3_213 = OMcp3_26+qd[13]*ROcp3_26;
  OMcp3_313 = OMcp3_36+qd[13]*ROcp3_36;
  ORcp3_113 = OMcp3_26*RLcp3_313-OMcp3_36*RLcp3_213;
  ORcp3_213 = -(OMcp3_16*RLcp3_313-OMcp3_36*RLcp3_113);
  ORcp3_313 = OMcp3_16*RLcp3_213-OMcp3_26*RLcp3_113;
  OPcp3_113 = OPcp3_16+qd[13]*(OMcp3_26*ROcp3_36-OMcp3_36*ROcp3_26)+qdd[13]*ROcp3_16;
  OPcp3_213 = OPcp3_26-qd[13]*(OMcp3_16*ROcp3_36-OMcp3_36*ROcp3_16)+qdd[13]*ROcp3_26;
  OPcp3_313 = OPcp3_36+qd[13]*(OMcp3_16*ROcp3_26-OMcp3_26*ROcp3_16)+qdd[13]*ROcp3_36;
  RLcp3_114 = ROcp3_413*s->dpt[2][24];
  RLcp3_214 = ROcp3_513*s->dpt[2][24];
  RLcp3_314 = ROcp3_613*s->dpt[2][24];
  OMcp3_114 = OMcp3_113+qd[14]*ROcp3_413;
  OMcp3_214 = OMcp3_213+qd[14]*ROcp3_513;
  OMcp3_314 = OMcp3_313+qd[14]*ROcp3_613;
  ORcp3_114 = OMcp3_213*RLcp3_314-OMcp3_313*RLcp3_214;
  ORcp3_214 = -(OMcp3_113*RLcp3_314-OMcp3_313*RLcp3_114);
  ORcp3_314 = OMcp3_113*RLcp3_214-OMcp3_213*RLcp3_114;
  OMcp3_115 = OMcp3_114+qd[15]*ROcp3_114;
  OMcp3_215 = OMcp3_214+qd[15]*ROcp3_214;
  OMcp3_315 = OMcp3_314+qd[15]*ROcp3_314;
  OMcp3_116 = OMcp3_115+qd[16]*ROcp3_715;
  OMcp3_216 = OMcp3_215+qd[16]*ROcp3_815;
  OMcp3_316 = OMcp3_315+qd[16]*ROcp3_915;
  OPcp3_116 = OPcp3_113+qd[14]*(OMcp3_213*ROcp3_613-OMcp3_313*ROcp3_513)+qd[15]*(OMcp3_214*ROcp3_314-OMcp3_314*ROcp3_214
 )+qd[16]*(OMcp3_215*ROcp3_915-OMcp3_315*ROcp3_815)+qdd[14]*ROcp3_413+qdd[15]*ROcp3_114+qdd[16]*ROcp3_715;
  OPcp3_216 = OPcp3_213-qd[14]*(OMcp3_113*ROcp3_613-OMcp3_313*ROcp3_413)-qd[15]*(OMcp3_114*ROcp3_314-OMcp3_314*ROcp3_114
 )-qd[16]*(OMcp3_115*ROcp3_915-OMcp3_315*ROcp3_715)+qdd[14]*ROcp3_513+qdd[15]*ROcp3_214+qdd[16]*ROcp3_815;
  OPcp3_316 = OPcp3_313+qd[14]*(OMcp3_113*ROcp3_513-OMcp3_213*ROcp3_413)+qd[15]*(OMcp3_114*ROcp3_214-OMcp3_214*ROcp3_114
 )+qd[16]*(OMcp3_115*ROcp3_815-OMcp3_215*ROcp3_715)+qdd[14]*ROcp3_613+qdd[15]*ROcp3_314+qdd[16]*ROcp3_915;
  RLcp3_117 = ROcp3_715*s->dpt[3][27];
  RLcp3_217 = ROcp3_815*s->dpt[3][27];
  RLcp3_317 = ROcp3_915*s->dpt[3][27];
  ORcp3_117 = OMcp3_216*RLcp3_317-OMcp3_316*RLcp3_217;
  ORcp3_217 = -(OMcp3_116*RLcp3_317-OMcp3_316*RLcp3_117);
  ORcp3_317 = OMcp3_116*RLcp3_217-OMcp3_216*RLcp3_117;
  PxF2[1] = q[1]+RLcp3_113+RLcp3_114+RLcp3_117;
  PxF2[2] = q[2]+RLcp3_213+RLcp3_214+RLcp3_217;
  PxF2[3] = q[3]+RLcp3_313+RLcp3_314+RLcp3_317;
  RxF2[1][1] = ROcp3_117;
  RxF2[1][2] = ROcp3_217;
  RxF2[1][3] = ROcp3_317;
  RxF2[2][1] = ROcp3_416;
  RxF2[2][2] = ROcp3_516;
  RxF2[2][3] = ROcp3_616;
  RxF2[3][1] = ROcp3_717;
  RxF2[3][2] = ROcp3_817;
  RxF2[3][3] = ROcp3_917;
  VxF2[1] = qd[1]+ORcp3_113+ORcp3_114+ORcp3_117;
  VxF2[2] = qd[2]+ORcp3_213+ORcp3_214+ORcp3_217;
  VxF2[3] = qd[3]+ORcp3_313+ORcp3_314+ORcp3_317;
  OMxF2[1] = OMcp3_116+qd[17]*ROcp3_416;
  OMxF2[2] = OMcp3_216+qd[17]*ROcp3_516;
  OMxF2[3] = OMcp3_316+qd[17]*ROcp3_616;
  AxF2[1] = qdd[1]+OMcp3_213*ORcp3_314+OMcp3_216*ORcp3_317+OMcp3_26*ORcp3_313-OMcp3_313*ORcp3_214-OMcp3_316*ORcp3_217-
 OMcp3_36*ORcp3_213+OPcp3_213*RLcp3_314+OPcp3_216*RLcp3_317+OPcp3_26*RLcp3_313-OPcp3_313*RLcp3_214-OPcp3_316*RLcp3_217-
 OPcp3_36*RLcp3_213;
  AxF2[2] = qdd[2]-OMcp3_113*ORcp3_314-OMcp3_116*ORcp3_317-OMcp3_16*ORcp3_313+OMcp3_313*ORcp3_114+OMcp3_316*ORcp3_117+
 OMcp3_36*ORcp3_113-OPcp3_113*RLcp3_314-OPcp3_116*RLcp3_317-OPcp3_16*RLcp3_313+OPcp3_313*RLcp3_114+OPcp3_316*RLcp3_117+
 OPcp3_36*RLcp3_113;
  AxF2[3] = qdd[3]+OMcp3_113*ORcp3_214+OMcp3_116*ORcp3_217+OMcp3_16*ORcp3_213-OMcp3_213*ORcp3_114-OMcp3_216*ORcp3_117-
 OMcp3_26*ORcp3_113+OPcp3_113*RLcp3_214+OPcp3_116*RLcp3_217+OPcp3_16*RLcp3_213-OPcp3_213*RLcp3_114-OPcp3_216*RLcp3_117-
 OPcp3_26*RLcp3_113;
  OMPxF2[1] = OPcp3_116+qd[17]*(OMcp3_216*ROcp3_616-OMcp3_316*ROcp3_516)+qdd[17]*ROcp3_416;
  OMPxF2[2] = OPcp3_216-qd[17]*(OMcp3_116*ROcp3_616-OMcp3_316*ROcp3_416)+qdd[17]*ROcp3_516;
  OMPxF2[3] = OPcp3_316+qd[17]*(OMcp3_116*ROcp3_516-OMcp3_216*ROcp3_416)+qdd[17]*ROcp3_616;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc14 = ROcp3_117*SWr2[1]+ROcp3_217*SWr2[2]+ROcp3_317*SWr2[3];
  xfrc24 = ROcp3_416*SWr2[1]+ROcp3_516*SWr2[2]+ROcp3_616*SWr2[3];
  xfrc34 = ROcp3_717*SWr2[1]+ROcp3_817*SWr2[2]+ROcp3_917*SWr2[3];
  frc[1][17] = s->frc[1][17]+xfrc14;
  frc[2][17] = s->frc[2][17]+xfrc24;
  frc[3][17] = s->frc[3][17]+xfrc34;
  xtrq14 = ROcp3_117*SWr2[4]+ROcp3_217*SWr2[5]+ROcp3_317*SWr2[6];
  xtrq24 = ROcp3_416*SWr2[4]+ROcp3_516*SWr2[5]+ROcp3_616*SWr2[6];
  xtrq34 = ROcp3_717*SWr2[4]+ROcp3_817*SWr2[5]+ROcp3_917*SWr2[6];
  trq[1][17] = s->trq[1][17]+xtrq14-xfrc24*SWr2[9]+xfrc34*SWr2[8];
  trq[2][17] = s->trq[2][17]+xtrq24+xfrc14*SWr2[9]-xfrc34*SWr2[7];
  trq[3][17] = s->trq[3][17]+xtrq34-xfrc14*SWr2[8]+xfrc24*SWr2[7];

// = = Block_0_0_1_3_0_1 = = 
 
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
  OMcp4_16 = OMcp4_15+qd[6]*ROcp4_45;
  OMcp4_26 = OMcp4_25+qd[6]*ROcp4_55;
  OMcp4_36 = qd[4]+qd[6]*S5;
  OPcp4_16 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp4_55-OMcp4_25*S5)-qdd[5]*C4-qdd[6]*ROcp4_45);
  OPcp4_26 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp4_45-OMcp4_15*S5)+qdd[5]*S4+qdd[6]*ROcp4_55;
  OPcp4_36 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;

// = = Block_0_0_1_3_0_15 = = 
 
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
  OMcp4_133 = OMcp4_16+qd[33]*ROcp4_16;
  OMcp4_233 = OMcp4_26+qd[33]*ROcp4_26;
  OMcp4_333 = OMcp4_36+qd[33]*ROcp4_36;
  ORcp4_133 = OMcp4_26*RLcp4_333-OMcp4_36*RLcp4_233;
  ORcp4_233 = -(OMcp4_16*RLcp4_333-OMcp4_36*RLcp4_133);
  ORcp4_333 = OMcp4_16*RLcp4_233-OMcp4_26*RLcp4_133;
  OPcp4_133 = OPcp4_16+qd[33]*(OMcp4_26*ROcp4_36-OMcp4_36*ROcp4_26)+qdd[33]*ROcp4_16;
  OPcp4_233 = OPcp4_26-qd[33]*(OMcp4_16*ROcp4_36-OMcp4_36*ROcp4_16)+qdd[33]*ROcp4_26;
  OPcp4_333 = OPcp4_36+qd[33]*(OMcp4_16*ROcp4_26-OMcp4_26*ROcp4_16)+qdd[33]*ROcp4_36;
  RLcp4_134 = ROcp4_433*s->dpt[2][47];
  RLcp4_234 = ROcp4_533*s->dpt[2][47];
  RLcp4_334 = ROcp4_633*s->dpt[2][47];
  OMcp4_134 = OMcp4_133+qd[34]*ROcp4_16;
  OMcp4_234 = OMcp4_233+qd[34]*ROcp4_26;
  OMcp4_334 = OMcp4_333+qd[34]*ROcp4_36;
  ORcp4_134 = OMcp4_233*RLcp4_334-OMcp4_333*RLcp4_234;
  ORcp4_234 = -(OMcp4_133*RLcp4_334-OMcp4_333*RLcp4_134);
  ORcp4_334 = OMcp4_133*RLcp4_234-OMcp4_233*RLcp4_134;
  OPcp4_134 = OPcp4_133+qd[34]*(OMcp4_233*ROcp4_36-OMcp4_333*ROcp4_26)+qdd[34]*ROcp4_16;
  OPcp4_234 = OPcp4_233-qd[34]*(OMcp4_133*ROcp4_36-OMcp4_333*ROcp4_16)+qdd[34]*ROcp4_26;
  OPcp4_334 = OPcp4_333+qd[34]*(OMcp4_133*ROcp4_26-OMcp4_233*ROcp4_16)+qdd[34]*ROcp4_36;
  RLcp4_135 = ROcp4_734*s->dpt[3][50];
  RLcp4_235 = ROcp4_834*s->dpt[3][50];
  RLcp4_335 = ROcp4_934*s->dpt[3][50];
  ORcp4_135 = OMcp4_234*RLcp4_335-OMcp4_334*RLcp4_235;
  ORcp4_235 = -(OMcp4_134*RLcp4_335-OMcp4_334*RLcp4_135);
  ORcp4_335 = OMcp4_134*RLcp4_235-OMcp4_234*RLcp4_135;
  PxF3[1] = q[1]+RLcp4_133+RLcp4_134+RLcp4_135;
  PxF3[2] = q[2]+RLcp4_233+RLcp4_234+RLcp4_235;
  PxF3[3] = q[3]+RLcp4_333+RLcp4_334+RLcp4_335;
  RxF3[1][1] = ROcp4_135;
  RxF3[1][2] = ROcp4_235;
  RxF3[1][3] = ROcp4_335;
  RxF3[2][1] = ROcp4_434;
  RxF3[2][2] = ROcp4_534;
  RxF3[2][3] = ROcp4_634;
  RxF3[3][1] = ROcp4_735;
  RxF3[3][2] = ROcp4_835;
  RxF3[3][3] = ROcp4_935;
  VxF3[1] = qd[1]+ORcp4_133+ORcp4_134+ORcp4_135;
  VxF3[2] = qd[2]+ORcp4_233+ORcp4_234+ORcp4_235;
  VxF3[3] = qd[3]+ORcp4_333+ORcp4_334+ORcp4_335;
  OMxF3[1] = OMcp4_134+qd[35]*ROcp4_434;
  OMxF3[2] = OMcp4_234+qd[35]*ROcp4_534;
  OMxF3[3] = OMcp4_334+qd[35]*ROcp4_634;
  AxF3[1] = qdd[1]+OMcp4_233*ORcp4_334+OMcp4_234*ORcp4_335+OMcp4_26*ORcp4_333-OMcp4_333*ORcp4_234-OMcp4_334*ORcp4_235-
 OMcp4_36*ORcp4_233+OPcp4_233*RLcp4_334+OPcp4_234*RLcp4_335+OPcp4_26*RLcp4_333-OPcp4_333*RLcp4_234-OPcp4_334*RLcp4_235-
 OPcp4_36*RLcp4_233;
  AxF3[2] = qdd[2]-OMcp4_133*ORcp4_334-OMcp4_134*ORcp4_335-OMcp4_16*ORcp4_333+OMcp4_333*ORcp4_134+OMcp4_334*ORcp4_135+
 OMcp4_36*ORcp4_133-OPcp4_133*RLcp4_334-OPcp4_134*RLcp4_335-OPcp4_16*RLcp4_333+OPcp4_333*RLcp4_134+OPcp4_334*RLcp4_135+
 OPcp4_36*RLcp4_133;
  AxF3[3] = qdd[3]+OMcp4_133*ORcp4_234+OMcp4_134*ORcp4_235+OMcp4_16*ORcp4_233-OMcp4_233*ORcp4_134-OMcp4_234*ORcp4_135-
 OMcp4_26*ORcp4_133+OPcp4_133*RLcp4_234+OPcp4_134*RLcp4_235+OPcp4_16*RLcp4_233-OPcp4_233*RLcp4_134-OPcp4_234*RLcp4_135-
 OPcp4_26*RLcp4_133;
  OMPxF3[1] = OPcp4_134+qd[35]*(OMcp4_234*ROcp4_634-OMcp4_334*ROcp4_534)+qdd[35]*ROcp4_434;
  OMPxF3[2] = OPcp4_234-qd[35]*(OMcp4_134*ROcp4_634-OMcp4_334*ROcp4_434)+qdd[35]*ROcp4_534;
  OMPxF3[3] = OPcp4_334+qd[35]*(OMcp4_134*ROcp4_534-OMcp4_234*ROcp4_434)+qdd[35]*ROcp4_634;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc15 = ROcp4_135*SWr3[1]+ROcp4_235*SWr3[2]+ROcp4_335*SWr3[3];
  xfrc25 = ROcp4_434*SWr3[1]+ROcp4_534*SWr3[2]+ROcp4_634*SWr3[3];
  xfrc35 = ROcp4_735*SWr3[1]+ROcp4_835*SWr3[2]+ROcp4_935*SWr3[3];
  frc[1][35] = s->frc[1][35]+xfrc15;
  frc[2][35] = s->frc[2][35]+xfrc25;
  frc[3][35] = s->frc[3][35]+xfrc35;
  xtrq15 = ROcp4_135*SWr3[4]+ROcp4_235*SWr3[5]+ROcp4_335*SWr3[6];
  xtrq25 = ROcp4_434*SWr3[4]+ROcp4_534*SWr3[5]+ROcp4_634*SWr3[6];
  xtrq35 = ROcp4_735*SWr3[4]+ROcp4_835*SWr3[5]+ROcp4_935*SWr3[6];
  trq[1][35] = s->trq[1][35]+xtrq15-xfrc25*SWr3[9]+xfrc35*SWr3[8];
  trq[2][35] = s->trq[2][35]+xtrq25+xfrc15*SWr3[9]-xfrc35*SWr3[7];
  trq[3][35] = s->trq[3][35]+xtrq35-xfrc15*SWr3[8]+xfrc25*SWr3[7];

// = = Block_0_0_1_4_0_1 = = 
 
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
  OMcp5_16 = OMcp5_15+qd[6]*ROcp5_45;
  OMcp5_26 = OMcp5_25+qd[6]*ROcp5_55;
  OMcp5_36 = qd[4]+qd[6]*S5;
  OPcp5_16 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp5_55-OMcp5_25*S5)-qdd[5]*C4-qdd[6]*ROcp5_45);
  OPcp5_26 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp5_45-OMcp5_15*S5)+qdd[5]*S4+qdd[6]*ROcp5_55;
  OPcp5_36 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;

// = = Block_0_0_1_4_0_16 = = 
 
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
  OMcp5_136 = OMcp5_16+qd[36]*ROcp5_16;
  OMcp5_236 = OMcp5_26+qd[36]*ROcp5_26;
  OMcp5_336 = OMcp5_36+qd[36]*ROcp5_36;
  ORcp5_136 = OMcp5_26*RLcp5_336-OMcp5_36*RLcp5_236;
  ORcp5_236 = -(OMcp5_16*RLcp5_336-OMcp5_36*RLcp5_136);
  ORcp5_336 = OMcp5_16*RLcp5_236-OMcp5_26*RLcp5_136;
  OPcp5_136 = OPcp5_16+qd[36]*(OMcp5_26*ROcp5_36-OMcp5_36*ROcp5_26)+qdd[36]*ROcp5_16;
  OPcp5_236 = OPcp5_26-qd[36]*(OMcp5_16*ROcp5_36-OMcp5_36*ROcp5_16)+qdd[36]*ROcp5_26;
  OPcp5_336 = OPcp5_36+qd[36]*(OMcp5_16*ROcp5_26-OMcp5_26*ROcp5_16)+qdd[36]*ROcp5_36;
  RLcp5_137 = ROcp5_436*s->dpt[2][52];
  RLcp5_237 = ROcp5_536*s->dpt[2][52];
  RLcp5_337 = ROcp5_636*s->dpt[2][52];
  OMcp5_137 = OMcp5_136+qd[37]*ROcp5_16;
  OMcp5_237 = OMcp5_236+qd[37]*ROcp5_26;
  OMcp5_337 = OMcp5_336+qd[37]*ROcp5_36;
  ORcp5_137 = OMcp5_236*RLcp5_337-OMcp5_336*RLcp5_237;
  ORcp5_237 = -(OMcp5_136*RLcp5_337-OMcp5_336*RLcp5_137);
  ORcp5_337 = OMcp5_136*RLcp5_237-OMcp5_236*RLcp5_137;
  OPcp5_137 = OPcp5_136+qd[37]*(OMcp5_236*ROcp5_36-OMcp5_336*ROcp5_26)+qdd[37]*ROcp5_16;
  OPcp5_237 = OPcp5_236-qd[37]*(OMcp5_136*ROcp5_36-OMcp5_336*ROcp5_16)+qdd[37]*ROcp5_26;
  OPcp5_337 = OPcp5_336+qd[37]*(OMcp5_136*ROcp5_26-OMcp5_236*ROcp5_16)+qdd[37]*ROcp5_36;
  RLcp5_138 = ROcp5_737*s->dpt[3][54];
  RLcp5_238 = ROcp5_837*s->dpt[3][54];
  RLcp5_338 = ROcp5_937*s->dpt[3][54];
  ORcp5_138 = OMcp5_237*RLcp5_338-OMcp5_337*RLcp5_238;
  ORcp5_238 = -(OMcp5_137*RLcp5_338-OMcp5_337*RLcp5_138);
  ORcp5_338 = OMcp5_137*RLcp5_238-OMcp5_237*RLcp5_138;
  PxF4[1] = q[1]+RLcp5_136+RLcp5_137+RLcp5_138;
  PxF4[2] = q[2]+RLcp5_236+RLcp5_237+RLcp5_238;
  PxF4[3] = q[3]+RLcp5_336+RLcp5_337+RLcp5_338;
  RxF4[1][1] = ROcp5_138;
  RxF4[1][2] = ROcp5_238;
  RxF4[1][3] = ROcp5_338;
  RxF4[2][1] = ROcp5_437;
  RxF4[2][2] = ROcp5_537;
  RxF4[2][3] = ROcp5_637;
  RxF4[3][1] = ROcp5_738;
  RxF4[3][2] = ROcp5_838;
  RxF4[3][3] = ROcp5_938;
  VxF4[1] = qd[1]+ORcp5_136+ORcp5_137+ORcp5_138;
  VxF4[2] = qd[2]+ORcp5_236+ORcp5_237+ORcp5_238;
  VxF4[3] = qd[3]+ORcp5_336+ORcp5_337+ORcp5_338;
  OMxF4[1] = OMcp5_137+qd[38]*ROcp5_437;
  OMxF4[2] = OMcp5_237+qd[38]*ROcp5_537;
  OMxF4[3] = OMcp5_337+qd[38]*ROcp5_637;
  AxF4[1] = qdd[1]+OMcp5_236*ORcp5_337+OMcp5_237*ORcp5_338+OMcp5_26*ORcp5_336-OMcp5_336*ORcp5_237-OMcp5_337*ORcp5_238-
 OMcp5_36*ORcp5_236+OPcp5_236*RLcp5_337+OPcp5_237*RLcp5_338+OPcp5_26*RLcp5_336-OPcp5_336*RLcp5_237-OPcp5_337*RLcp5_238-
 OPcp5_36*RLcp5_236;
  AxF4[2] = qdd[2]-OMcp5_136*ORcp5_337-OMcp5_137*ORcp5_338-OMcp5_16*ORcp5_336+OMcp5_336*ORcp5_137+OMcp5_337*ORcp5_138+
 OMcp5_36*ORcp5_136-OPcp5_136*RLcp5_337-OPcp5_137*RLcp5_338-OPcp5_16*RLcp5_336+OPcp5_336*RLcp5_137+OPcp5_337*RLcp5_138+
 OPcp5_36*RLcp5_136;
  AxF4[3] = qdd[3]+OMcp5_136*ORcp5_237+OMcp5_137*ORcp5_238+OMcp5_16*ORcp5_236-OMcp5_236*ORcp5_137-OMcp5_237*ORcp5_138-
 OMcp5_26*ORcp5_136+OPcp5_136*RLcp5_237+OPcp5_137*RLcp5_238+OPcp5_16*RLcp5_236-OPcp5_236*RLcp5_137-OPcp5_237*RLcp5_138-
 OPcp5_26*RLcp5_136;
  OMPxF4[1] = OPcp5_137+qd[38]*(OMcp5_237*ROcp5_637-OMcp5_337*ROcp5_537)+qdd[38]*ROcp5_437;
  OMPxF4[2] = OPcp5_237-qd[38]*(OMcp5_137*ROcp5_637-OMcp5_337*ROcp5_437)+qdd[38]*ROcp5_537;
  OMPxF4[3] = OPcp5_337+qd[38]*(OMcp5_137*ROcp5_537-OMcp5_237*ROcp5_437)+qdd[38]*ROcp5_637;
 
// Sensor Forces Computation 

  SWr4 = user_ExtForces(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc16 = ROcp5_138*SWr4[1]+ROcp5_238*SWr4[2]+ROcp5_338*SWr4[3];
  xfrc26 = ROcp5_437*SWr4[1]+ROcp5_537*SWr4[2]+ROcp5_637*SWr4[3];
  xfrc36 = ROcp5_738*SWr4[1]+ROcp5_838*SWr4[2]+ROcp5_938*SWr4[3];
  frc[1][38] = s->frc[1][38]+xfrc16;
  frc[2][38] = s->frc[2][38]+xfrc26;
  frc[3][38] = s->frc[3][38]+xfrc36;
  xtrq16 = ROcp5_138*SWr4[4]+ROcp5_238*SWr4[5]+ROcp5_338*SWr4[6];
  xtrq26 = ROcp5_437*SWr4[4]+ROcp5_537*SWr4[5]+ROcp5_637*SWr4[6];
  xtrq36 = ROcp5_738*SWr4[4]+ROcp5_838*SWr4[5]+ROcp5_938*SWr4[6];
  trq[1][38] = s->trq[1][38]+xtrq16-xfrc26*SWr4[9]+xfrc36*SWr4[8];
  trq[2][38] = s->trq[2][38]+xtrq26+xfrc16*SWr4[9]-xfrc36*SWr4[7];
  trq[3][38] = s->trq[3][38]+xtrq36-xfrc16*SWr4[8]+xfrc26*SWr4[7];

// = = Block_0_0_1_5_0_1 = = 
 
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
  OMcp6_16 = OMcp6_15+qd[6]*ROcp6_45;
  OMcp6_26 = OMcp6_25+qd[6]*ROcp6_55;
  OMcp6_36 = qd[4]+qd[6]*S5;
  OPcp6_16 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp6_55-OMcp6_25*S5)-qdd[5]*C4-qdd[6]*ROcp6_45);
  OPcp6_26 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp6_45-OMcp6_15*S5)+qdd[5]*S4+qdd[6]*ROcp6_55;
  OPcp6_36 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;

// = = Block_0_0_1_5_0_20 = = 
 
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
  OMcp6_145 = OMcp6_16+qd[45]*ROcp6_76;
  OMcp6_245 = OMcp6_26+qd[45]*ROcp6_86;
  OMcp6_345 = OMcp6_36+qd[45]*ROcp6_96;
  ORcp6_145 = OMcp6_26*RLcp6_345-OMcp6_36*RLcp6_245;
  ORcp6_245 = -(OMcp6_16*RLcp6_345-OMcp6_36*RLcp6_145);
  ORcp6_345 = OMcp6_16*RLcp6_245-OMcp6_26*RLcp6_145;
  OMcp6_146 = OMcp6_145+qd[46]*ROcp6_145;
  OMcp6_246 = OMcp6_245+qd[46]*ROcp6_245;
  OMcp6_346 = OMcp6_345+qd[46]*ROcp6_345;
  OMcp6_147 = OMcp6_146+qd[47]*ROcp6_446;
  OMcp6_247 = OMcp6_246+qd[47]*ROcp6_546;
  OMcp6_347 = OMcp6_346+qd[47]*ROcp6_646;
  OPcp6_147 = OPcp6_16+qd[45]*(OMcp6_26*ROcp6_96-OMcp6_36*ROcp6_86)+qd[46]*(OMcp6_245*ROcp6_345-OMcp6_345*ROcp6_245)+
 qd[47]*(OMcp6_246*ROcp6_646-OMcp6_346*ROcp6_546)+qdd[45]*ROcp6_76+qdd[46]*ROcp6_145+qdd[47]*ROcp6_446;
  OPcp6_247 = OPcp6_26-qd[45]*(OMcp6_16*ROcp6_96-OMcp6_36*ROcp6_76)-qd[46]*(OMcp6_145*ROcp6_345-OMcp6_345*ROcp6_145)-
 qd[47]*(OMcp6_146*ROcp6_646-OMcp6_346*ROcp6_446)+qdd[45]*ROcp6_86+qdd[46]*ROcp6_245+qdd[47]*ROcp6_546;
  OPcp6_347 = OPcp6_36+qd[45]*(OMcp6_16*ROcp6_86-OMcp6_26*ROcp6_76)+qd[46]*(OMcp6_145*ROcp6_245-OMcp6_245*ROcp6_145)+
 qd[47]*(OMcp6_146*ROcp6_546-OMcp6_246*ROcp6_446)+qdd[45]*ROcp6_96+qdd[46]*ROcp6_345+qdd[47]*ROcp6_646;
  RLcp6_148 = q[48]*ROcp6_147;
  RLcp6_248 = q[48]*ROcp6_247;
  RLcp6_348 = q[48]*ROcp6_347;
  ORcp6_148 = OMcp6_247*RLcp6_348-OMcp6_347*RLcp6_248;
  ORcp6_248 = -(OMcp6_147*RLcp6_348-OMcp6_347*RLcp6_148);
  ORcp6_348 = OMcp6_147*RLcp6_248-OMcp6_247*RLcp6_148;

// = = Block_0_0_1_5_0_21 = = 
 
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
  ORcp6_149 = OMcp6_247*RLcp6_349-OMcp6_347*RLcp6_249;
  ORcp6_249 = -(OMcp6_147*RLcp6_349-OMcp6_347*RLcp6_149);
  ORcp6_349 = OMcp6_147*RLcp6_249-OMcp6_247*RLcp6_149;
  PxF5[1] = q[1]+RLcp6_145+RLcp6_148+RLcp6_149;
  PxF5[2] = q[2]+RLcp6_245+RLcp6_248+RLcp6_249;
  PxF5[3] = q[3]+RLcp6_345+RLcp6_348+RLcp6_349;
  RxF5[1][1] = ROcp6_149;
  RxF5[1][2] = ROcp6_249;
  RxF5[1][3] = ROcp6_349;
  RxF5[2][1] = ROcp6_446;
  RxF5[2][2] = ROcp6_546;
  RxF5[2][3] = ROcp6_646;
  RxF5[3][1] = ROcp6_749;
  RxF5[3][2] = ROcp6_849;
  RxF5[3][3] = ROcp6_949;
  VxF5[1] = qd[1]+ORcp6_145+ORcp6_148+ORcp6_149+qd[48]*ROcp6_147;
  VxF5[2] = qd[2]+ORcp6_245+ORcp6_248+ORcp6_249+qd[48]*ROcp6_247;
  VxF5[3] = qd[3]+ORcp6_345+ORcp6_348+ORcp6_349+qd[48]*ROcp6_347;
  OMxF5[1] = OMcp6_147+qd[49]*ROcp6_446;
  OMxF5[2] = OMcp6_247+qd[49]*ROcp6_546;
  OMxF5[3] = OMcp6_347+qd[49]*ROcp6_646;
  AxF5[1] = qdd[1]+(2.0)*qd[48]*(OMcp6_247*ROcp6_347-OMcp6_347*ROcp6_247)+qdd[48]*ROcp6_147+OMcp6_247*ORcp6_348+OMcp6_247*
 ORcp6_349+OMcp6_26*ORcp6_345-OMcp6_347*ORcp6_248-OMcp6_347*ORcp6_249-OMcp6_36*ORcp6_245+OPcp6_247*RLcp6_348+OPcp6_247*
 RLcp6_349+OPcp6_26*RLcp6_345-OPcp6_347*RLcp6_248-OPcp6_347*RLcp6_249-OPcp6_36*RLcp6_245;
  AxF5[2] = qdd[2]-(2.0)*qd[48]*(OMcp6_147*ROcp6_347-OMcp6_347*ROcp6_147)+qdd[48]*ROcp6_247-OMcp6_147*ORcp6_348-OMcp6_147*
 ORcp6_349-OMcp6_16*ORcp6_345+OMcp6_347*ORcp6_148+OMcp6_347*ORcp6_149+OMcp6_36*ORcp6_145-OPcp6_147*RLcp6_348-OPcp6_147*
 RLcp6_349-OPcp6_16*RLcp6_345+OPcp6_347*RLcp6_148+OPcp6_347*RLcp6_149+OPcp6_36*RLcp6_145;
  AxF5[3] = qdd[3]+(2.0)*qd[48]*(OMcp6_147*ROcp6_247-OMcp6_247*ROcp6_147)+qdd[48]*ROcp6_347+OMcp6_147*ORcp6_248+OMcp6_147*
 ORcp6_249+OMcp6_16*ORcp6_245-OMcp6_247*ORcp6_148-OMcp6_247*ORcp6_149-OMcp6_26*ORcp6_145+OPcp6_147*RLcp6_248+OPcp6_147*
 RLcp6_249+OPcp6_16*RLcp6_245-OPcp6_247*RLcp6_148-OPcp6_247*RLcp6_149-OPcp6_26*RLcp6_145;
  OMPxF5[1] = OPcp6_147+qd[49]*(OMcp6_247*ROcp6_646-OMcp6_347*ROcp6_546)+qdd[49]*ROcp6_446;
  OMPxF5[2] = OPcp6_247-qd[49]*(OMcp6_147*ROcp6_646-OMcp6_347*ROcp6_446)+qdd[49]*ROcp6_546;
  OMPxF5[3] = OPcp6_347+qd[49]*(OMcp6_147*ROcp6_546-OMcp6_247*ROcp6_446)+qdd[49]*ROcp6_646;
 
// Sensor Forces Computation 

  SWr5 = user_ExtForces(PxF5,RxF5,VxF5,OMxF5,AxF5,OMPxF5,s,tsim,5);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc17 = ROcp6_149*SWr5[1]+ROcp6_249*SWr5[2]+ROcp6_349*SWr5[3];
  xfrc27 = ROcp6_446*SWr5[1]+ROcp6_546*SWr5[2]+ROcp6_646*SWr5[3];
  xfrc37 = ROcp6_749*SWr5[1]+ROcp6_849*SWr5[2]+ROcp6_949*SWr5[3];
  frc[1][49] = s->frc[1][49]+xfrc17;
  frc[2][49] = s->frc[2][49]+xfrc27;
  frc[3][49] = s->frc[3][49]+xfrc37;
  xtrq17 = ROcp6_149*SWr5[4]+ROcp6_249*SWr5[5]+ROcp6_349*SWr5[6];
  xtrq27 = ROcp6_446*SWr5[4]+ROcp6_546*SWr5[5]+ROcp6_646*SWr5[6];
  xtrq37 = ROcp6_749*SWr5[4]+ROcp6_849*SWr5[5]+ROcp6_949*SWr5[6];
  trq[1][49] = s->trq[1][49]+xtrq17-xfrc27*SWr5[9]+xfrc37*SWr5[8];
  trq[2][49] = s->trq[2][49]+xtrq27+xfrc17*SWr5[9]-xfrc37*SWr5[7];
  trq[3][49] = s->trq[3][49]+xtrq37-xfrc17*SWr5[8]+xfrc27*SWr5[7];

// = = Block_0_0_1_6_0_1 = = 
 
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
  OMcp7_16 = OMcp7_15+qd[6]*ROcp7_45;
  OMcp7_26 = OMcp7_25+qd[6]*ROcp7_55;
  OMcp7_36 = qd[4]+qd[6]*S5;
  OPcp7_16 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp7_55-OMcp7_25*S5)-qdd[5]*C4-qdd[6]*ROcp7_45);
  OPcp7_26 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp7_45-OMcp7_15*S5)+qdd[5]*S4+qdd[6]*ROcp7_55;
  OPcp7_36 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;

// = = Block_0_0_1_6_0_20 = = 
 
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
  OMcp7_145 = OMcp7_16+qd[45]*ROcp7_76;
  OMcp7_245 = OMcp7_26+qd[45]*ROcp7_86;
  OMcp7_345 = OMcp7_36+qd[45]*ROcp7_96;
  ORcp7_145 = OMcp7_26*RLcp7_345-OMcp7_36*RLcp7_245;
  ORcp7_245 = -(OMcp7_16*RLcp7_345-OMcp7_36*RLcp7_145);
  ORcp7_345 = OMcp7_16*RLcp7_245-OMcp7_26*RLcp7_145;
  OMcp7_146 = OMcp7_145+qd[46]*ROcp7_145;
  OMcp7_246 = OMcp7_245+qd[46]*ROcp7_245;
  OMcp7_346 = OMcp7_345+qd[46]*ROcp7_345;
  OMcp7_147 = OMcp7_146+qd[47]*ROcp7_446;
  OMcp7_247 = OMcp7_246+qd[47]*ROcp7_546;
  OMcp7_347 = OMcp7_346+qd[47]*ROcp7_646;
  OPcp7_147 = OPcp7_16+qd[45]*(OMcp7_26*ROcp7_96-OMcp7_36*ROcp7_86)+qd[46]*(OMcp7_245*ROcp7_345-OMcp7_345*ROcp7_245)+
 qd[47]*(OMcp7_246*ROcp7_646-OMcp7_346*ROcp7_546)+qdd[45]*ROcp7_76+qdd[46]*ROcp7_145+qdd[47]*ROcp7_446;
  OPcp7_247 = OPcp7_26-qd[45]*(OMcp7_16*ROcp7_96-OMcp7_36*ROcp7_76)-qd[46]*(OMcp7_145*ROcp7_345-OMcp7_345*ROcp7_145)-
 qd[47]*(OMcp7_146*ROcp7_646-OMcp7_346*ROcp7_446)+qdd[45]*ROcp7_86+qdd[46]*ROcp7_245+qdd[47]*ROcp7_546;
  OPcp7_347 = OPcp7_36+qd[45]*(OMcp7_16*ROcp7_86-OMcp7_26*ROcp7_76)+qd[46]*(OMcp7_145*ROcp7_245-OMcp7_245*ROcp7_145)+
 qd[47]*(OMcp7_146*ROcp7_546-OMcp7_246*ROcp7_446)+qdd[45]*ROcp7_96+qdd[46]*ROcp7_345+qdd[47]*ROcp7_646;
  RLcp7_148 = q[48]*ROcp7_147;
  RLcp7_248 = q[48]*ROcp7_247;
  RLcp7_348 = q[48]*ROcp7_347;
  ORcp7_148 = OMcp7_247*RLcp7_348-OMcp7_347*RLcp7_248;
  ORcp7_248 = -(OMcp7_147*RLcp7_348-OMcp7_347*RLcp7_148);
  ORcp7_348 = OMcp7_147*RLcp7_248-OMcp7_247*RLcp7_148;

// = = Block_0_0_1_6_0_22 = = 
 
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
  ORcp7_150 = OMcp7_247*RLcp7_350-OMcp7_347*RLcp7_250;
  ORcp7_250 = -(OMcp7_147*RLcp7_350-OMcp7_347*RLcp7_150);
  ORcp7_350 = OMcp7_147*RLcp7_250-OMcp7_247*RLcp7_150;
  PxF6[1] = q[1]+RLcp7_145+RLcp7_148+RLcp7_150;
  PxF6[2] = q[2]+RLcp7_245+RLcp7_248+RLcp7_250;
  PxF6[3] = q[3]+RLcp7_345+RLcp7_348+RLcp7_350;
  RxF6[1][1] = ROcp7_150;
  RxF6[1][2] = ROcp7_250;
  RxF6[1][3] = ROcp7_350;
  RxF6[2][1] = ROcp7_446;
  RxF6[2][2] = ROcp7_546;
  RxF6[2][3] = ROcp7_646;
  RxF6[3][1] = ROcp7_750;
  RxF6[3][2] = ROcp7_850;
  RxF6[3][3] = ROcp7_950;
  VxF6[1] = qd[1]+ORcp7_145+ORcp7_148+ORcp7_150+qd[48]*ROcp7_147;
  VxF6[2] = qd[2]+ORcp7_245+ORcp7_248+ORcp7_250+qd[48]*ROcp7_247;
  VxF6[3] = qd[3]+ORcp7_345+ORcp7_348+ORcp7_350+qd[48]*ROcp7_347;
  OMxF6[1] = OMcp7_147+qd[50]*ROcp7_446;
  OMxF6[2] = OMcp7_247+qd[50]*ROcp7_546;
  OMxF6[3] = OMcp7_347+qd[50]*ROcp7_646;
  AxF6[1] = qdd[1]+(2.0)*qd[48]*(OMcp7_247*ROcp7_347-OMcp7_347*ROcp7_247)+qdd[48]*ROcp7_147+OMcp7_247*ORcp7_348+OMcp7_247*
 ORcp7_350+OMcp7_26*ORcp7_345-OMcp7_347*ORcp7_248-OMcp7_347*ORcp7_250-OMcp7_36*ORcp7_245+OPcp7_247*RLcp7_348+OPcp7_247*
 RLcp7_350+OPcp7_26*RLcp7_345-OPcp7_347*RLcp7_248-OPcp7_347*RLcp7_250-OPcp7_36*RLcp7_245;
  AxF6[2] = qdd[2]-(2.0)*qd[48]*(OMcp7_147*ROcp7_347-OMcp7_347*ROcp7_147)+qdd[48]*ROcp7_247-OMcp7_147*ORcp7_348-OMcp7_147*
 ORcp7_350-OMcp7_16*ORcp7_345+OMcp7_347*ORcp7_148+OMcp7_347*ORcp7_150+OMcp7_36*ORcp7_145-OPcp7_147*RLcp7_348-OPcp7_147*
 RLcp7_350-OPcp7_16*RLcp7_345+OPcp7_347*RLcp7_148+OPcp7_347*RLcp7_150+OPcp7_36*RLcp7_145;
  AxF6[3] = qdd[3]+(2.0)*qd[48]*(OMcp7_147*ROcp7_247-OMcp7_247*ROcp7_147)+qdd[48]*ROcp7_347+OMcp7_147*ORcp7_248+OMcp7_147*
 ORcp7_250+OMcp7_16*ORcp7_245-OMcp7_247*ORcp7_148-OMcp7_247*ORcp7_150-OMcp7_26*ORcp7_145+OPcp7_147*RLcp7_248+OPcp7_147*
 RLcp7_250+OPcp7_16*RLcp7_245-OPcp7_247*RLcp7_148-OPcp7_247*RLcp7_150-OPcp7_26*RLcp7_145;
  OMPxF6[1] = OPcp7_147+qd[50]*(OMcp7_247*ROcp7_646-OMcp7_347*ROcp7_546)+qdd[50]*ROcp7_446;
  OMPxF6[2] = OPcp7_247-qd[50]*(OMcp7_147*ROcp7_646-OMcp7_347*ROcp7_446)+qdd[50]*ROcp7_546;
  OMPxF6[3] = OPcp7_347+qd[50]*(OMcp7_147*ROcp7_546-OMcp7_247*ROcp7_446)+qdd[50]*ROcp7_646;
 
// Sensor Forces Computation 

  SWr6 = user_ExtForces(PxF6,RxF6,VxF6,OMxF6,AxF6,OMPxF6,s,tsim,6);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc18 = ROcp7_150*SWr6[1]+ROcp7_250*SWr6[2]+ROcp7_350*SWr6[3];
  xfrc28 = ROcp7_446*SWr6[1]+ROcp7_546*SWr6[2]+ROcp7_646*SWr6[3];
  xfrc38 = ROcp7_750*SWr6[1]+ROcp7_850*SWr6[2]+ROcp7_950*SWr6[3];
  frc[1][50] = s->frc[1][50]+xfrc18;
  frc[2][50] = s->frc[2][50]+xfrc28;
  frc[3][50] = s->frc[3][50]+xfrc38;
  xtrq18 = ROcp7_150*SWr6[4]+ROcp7_250*SWr6[5]+ROcp7_350*SWr6[6];
  xtrq28 = ROcp7_446*SWr6[4]+ROcp7_546*SWr6[5]+ROcp7_646*SWr6[6];
  xtrq38 = ROcp7_750*SWr6[4]+ROcp7_850*SWr6[5]+ROcp7_950*SWr6[6];
  trq[1][50] = s->trq[1][50]+xtrq18-xfrc28*SWr6[9]+xfrc38*SWr6[8];
  trq[2][50] = s->trq[2][50]+xtrq28+xfrc18*SWr6[9]-xfrc38*SWr6[7];
  trq[3][50] = s->trq[3][50]+xtrq38-xfrc18*SWr6[8]+xfrc28*SWr6[7];

// = = Block_0_0_1_7_0_1 = = 
 
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
  OMcp8_16 = OMcp8_15+qd[6]*ROcp8_45;
  OMcp8_26 = OMcp8_25+qd[6]*ROcp8_55;
  OMcp8_36 = qd[4]+qd[6]*S5;
  OPcp8_16 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp8_55-OMcp8_25*S5)-qdd[5]*C4-qdd[6]*ROcp8_45);
  OPcp8_26 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp8_45-OMcp8_15*S5)+qdd[5]*S4+qdd[6]*ROcp8_55;
  OPcp8_36 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;

// = = Block_0_0_1_7_0_20 = = 
 
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
  OMcp8_145 = OMcp8_16+qd[45]*ROcp8_76;
  OMcp8_245 = OMcp8_26+qd[45]*ROcp8_86;
  OMcp8_345 = OMcp8_36+qd[45]*ROcp8_96;
  ORcp8_145 = OMcp8_26*RLcp8_345-OMcp8_36*RLcp8_245;
  ORcp8_245 = -(OMcp8_16*RLcp8_345-OMcp8_36*RLcp8_145);
  ORcp8_345 = OMcp8_16*RLcp8_245-OMcp8_26*RLcp8_145;
  OMcp8_146 = OMcp8_145+qd[46]*ROcp8_145;
  OMcp8_246 = OMcp8_245+qd[46]*ROcp8_245;
  OMcp8_346 = OMcp8_345+qd[46]*ROcp8_345;
  OMcp8_147 = OMcp8_146+qd[47]*ROcp8_446;
  OMcp8_247 = OMcp8_246+qd[47]*ROcp8_546;
  OMcp8_347 = OMcp8_346+qd[47]*ROcp8_646;
  OPcp8_147 = OPcp8_16+qd[45]*(OMcp8_26*ROcp8_96-OMcp8_36*ROcp8_86)+qd[46]*(OMcp8_245*ROcp8_345-OMcp8_345*ROcp8_245)+
 qd[47]*(OMcp8_246*ROcp8_646-OMcp8_346*ROcp8_546)+qdd[45]*ROcp8_76+qdd[46]*ROcp8_145+qdd[47]*ROcp8_446;
  OPcp8_247 = OPcp8_26-qd[45]*(OMcp8_16*ROcp8_96-OMcp8_36*ROcp8_76)-qd[46]*(OMcp8_145*ROcp8_345-OMcp8_345*ROcp8_145)-
 qd[47]*(OMcp8_146*ROcp8_646-OMcp8_346*ROcp8_446)+qdd[45]*ROcp8_86+qdd[46]*ROcp8_245+qdd[47]*ROcp8_546;
  OPcp8_347 = OPcp8_36+qd[45]*(OMcp8_16*ROcp8_86-OMcp8_26*ROcp8_76)+qd[46]*(OMcp8_145*ROcp8_245-OMcp8_245*ROcp8_145)+
 qd[47]*(OMcp8_146*ROcp8_546-OMcp8_246*ROcp8_446)+qdd[45]*ROcp8_96+qdd[46]*ROcp8_345+qdd[47]*ROcp8_646;
  RLcp8_148 = q[48]*ROcp8_147;
  RLcp8_248 = q[48]*ROcp8_247;
  RLcp8_348 = q[48]*ROcp8_347;
  ORcp8_148 = OMcp8_247*RLcp8_348-OMcp8_347*RLcp8_248;
  ORcp8_248 = -(OMcp8_147*RLcp8_348-OMcp8_347*RLcp8_148);
  ORcp8_348 = OMcp8_147*RLcp8_248-OMcp8_247*RLcp8_148;

// = = Block_0_0_1_7_0_23 = = 
 
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
  ORcp8_151 = OMcp8_247*RLcp8_351-OMcp8_347*RLcp8_251;
  ORcp8_251 = -(OMcp8_147*RLcp8_351-OMcp8_347*RLcp8_151);
  ORcp8_351 = OMcp8_147*RLcp8_251-OMcp8_247*RLcp8_151;
  PxF7[1] = q[1]+RLcp8_145+RLcp8_148+RLcp8_151;
  PxF7[2] = q[2]+RLcp8_245+RLcp8_248+RLcp8_251;
  PxF7[3] = q[3]+RLcp8_345+RLcp8_348+RLcp8_351;
  RxF7[1][1] = ROcp8_151;
  RxF7[1][2] = ROcp8_251;
  RxF7[1][3] = ROcp8_351;
  RxF7[2][1] = ROcp8_446;
  RxF7[2][2] = ROcp8_546;
  RxF7[2][3] = ROcp8_646;
  RxF7[3][1] = ROcp8_751;
  RxF7[3][2] = ROcp8_851;
  RxF7[3][3] = ROcp8_951;
  VxF7[1] = qd[1]+ORcp8_145+ORcp8_148+ORcp8_151+qd[48]*ROcp8_147;
  VxF7[2] = qd[2]+ORcp8_245+ORcp8_248+ORcp8_251+qd[48]*ROcp8_247;
  VxF7[3] = qd[3]+ORcp8_345+ORcp8_348+ORcp8_351+qd[48]*ROcp8_347;
  OMxF7[1] = OMcp8_147+qd[51]*ROcp8_446;
  OMxF7[2] = OMcp8_247+qd[51]*ROcp8_546;
  OMxF7[3] = OMcp8_347+qd[51]*ROcp8_646;
  AxF7[1] = qdd[1]+(2.0)*qd[48]*(OMcp8_247*ROcp8_347-OMcp8_347*ROcp8_247)+qdd[48]*ROcp8_147+OMcp8_247*ORcp8_348+OMcp8_247*
 ORcp8_351+OMcp8_26*ORcp8_345-OMcp8_347*ORcp8_248-OMcp8_347*ORcp8_251-OMcp8_36*ORcp8_245+OPcp8_247*RLcp8_348+OPcp8_247*
 RLcp8_351+OPcp8_26*RLcp8_345-OPcp8_347*RLcp8_248-OPcp8_347*RLcp8_251-OPcp8_36*RLcp8_245;
  AxF7[2] = qdd[2]-(2.0)*qd[48]*(OMcp8_147*ROcp8_347-OMcp8_347*ROcp8_147)+qdd[48]*ROcp8_247-OMcp8_147*ORcp8_348-OMcp8_147*
 ORcp8_351-OMcp8_16*ORcp8_345+OMcp8_347*ORcp8_148+OMcp8_347*ORcp8_151+OMcp8_36*ORcp8_145-OPcp8_147*RLcp8_348-OPcp8_147*
 RLcp8_351-OPcp8_16*RLcp8_345+OPcp8_347*RLcp8_148+OPcp8_347*RLcp8_151+OPcp8_36*RLcp8_145;
  AxF7[3] = qdd[3]+(2.0)*qd[48]*(OMcp8_147*ROcp8_247-OMcp8_247*ROcp8_147)+qdd[48]*ROcp8_347+OMcp8_147*ORcp8_248+OMcp8_147*
 ORcp8_251+OMcp8_16*ORcp8_245-OMcp8_247*ORcp8_148-OMcp8_247*ORcp8_151-OMcp8_26*ORcp8_145+OPcp8_147*RLcp8_248+OPcp8_147*
 RLcp8_251+OPcp8_16*RLcp8_245-OPcp8_247*RLcp8_148-OPcp8_247*RLcp8_151-OPcp8_26*RLcp8_145;
  OMPxF7[1] = OPcp8_147+qd[51]*(OMcp8_247*ROcp8_646-OMcp8_347*ROcp8_546)+qdd[51]*ROcp8_446;
  OMPxF7[2] = OPcp8_247-qd[51]*(OMcp8_147*ROcp8_646-OMcp8_347*ROcp8_446)+qdd[51]*ROcp8_546;
  OMPxF7[3] = OPcp8_347+qd[51]*(OMcp8_147*ROcp8_546-OMcp8_247*ROcp8_446)+qdd[51]*ROcp8_646;
 
// Sensor Forces Computation 

  SWr7 = user_ExtForces(PxF7,RxF7,VxF7,OMxF7,AxF7,OMPxF7,s,tsim,7);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc19 = ROcp8_151*SWr7[1]+ROcp8_251*SWr7[2]+ROcp8_351*SWr7[3];
  xfrc29 = ROcp8_446*SWr7[1]+ROcp8_546*SWr7[2]+ROcp8_646*SWr7[3];
  xfrc39 = ROcp8_751*SWr7[1]+ROcp8_851*SWr7[2]+ROcp8_951*SWr7[3];
  frc[1][51] = s->frc[1][51]+xfrc19;
  frc[2][51] = s->frc[2][51]+xfrc29;
  frc[3][51] = s->frc[3][51]+xfrc39;
  xtrq19 = ROcp8_151*SWr7[4]+ROcp8_251*SWr7[5]+ROcp8_351*SWr7[6];
  xtrq29 = ROcp8_446*SWr7[4]+ROcp8_546*SWr7[5]+ROcp8_646*SWr7[6];
  xtrq39 = ROcp8_751*SWr7[4]+ROcp8_851*SWr7[5]+ROcp8_951*SWr7[6];
  trq[1][51] = s->trq[1][51]+xtrq19-xfrc29*SWr7[9]+xfrc39*SWr7[8];
  trq[2][51] = s->trq[2][51]+xtrq29+xfrc19*SWr7[9]-xfrc39*SWr7[7];
  trq[3][51] = s->trq[3][51]+xtrq39-xfrc19*SWr7[8]+xfrc29*SWr7[7];

// = = Block_0_0_1_8_0_1 = = 
 
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
  OMcp9_16 = OMcp9_15+qd[6]*ROcp9_45;
  OMcp9_26 = OMcp9_25+qd[6]*ROcp9_55;
  OMcp9_36 = qd[4]+qd[6]*S5;
  OPcp9_16 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp9_55-OMcp9_25*S5)-qdd[5]*C4-qdd[6]*ROcp9_45);
  OPcp9_26 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp9_45-OMcp9_15*S5)+qdd[5]*S4+qdd[6]*ROcp9_55;
  OPcp9_36 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;

// = = Block_0_0_1_8_0_20 = = 
 
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
  OMcp9_145 = OMcp9_16+qd[45]*ROcp9_76;
  OMcp9_245 = OMcp9_26+qd[45]*ROcp9_86;
  OMcp9_345 = OMcp9_36+qd[45]*ROcp9_96;
  ORcp9_145 = OMcp9_26*RLcp9_345-OMcp9_36*RLcp9_245;
  ORcp9_245 = -(OMcp9_16*RLcp9_345-OMcp9_36*RLcp9_145);
  ORcp9_345 = OMcp9_16*RLcp9_245-OMcp9_26*RLcp9_145;
  OMcp9_146 = OMcp9_145+qd[46]*ROcp9_145;
  OMcp9_246 = OMcp9_245+qd[46]*ROcp9_245;
  OMcp9_346 = OMcp9_345+qd[46]*ROcp9_345;
  OMcp9_147 = OMcp9_146+qd[47]*ROcp9_446;
  OMcp9_247 = OMcp9_246+qd[47]*ROcp9_546;
  OMcp9_347 = OMcp9_346+qd[47]*ROcp9_646;
  OPcp9_147 = OPcp9_16+qd[45]*(OMcp9_26*ROcp9_96-OMcp9_36*ROcp9_86)+qd[46]*(OMcp9_245*ROcp9_345-OMcp9_345*ROcp9_245)+
 qd[47]*(OMcp9_246*ROcp9_646-OMcp9_346*ROcp9_546)+qdd[45]*ROcp9_76+qdd[46]*ROcp9_145+qdd[47]*ROcp9_446;
  OPcp9_247 = OPcp9_26-qd[45]*(OMcp9_16*ROcp9_96-OMcp9_36*ROcp9_76)-qd[46]*(OMcp9_145*ROcp9_345-OMcp9_345*ROcp9_145)-
 qd[47]*(OMcp9_146*ROcp9_646-OMcp9_346*ROcp9_446)+qdd[45]*ROcp9_86+qdd[46]*ROcp9_245+qdd[47]*ROcp9_546;
  OPcp9_347 = OPcp9_36+qd[45]*(OMcp9_16*ROcp9_86-OMcp9_26*ROcp9_76)+qd[46]*(OMcp9_145*ROcp9_245-OMcp9_245*ROcp9_145)+
 qd[47]*(OMcp9_146*ROcp9_546-OMcp9_246*ROcp9_446)+qdd[45]*ROcp9_96+qdd[46]*ROcp9_345+qdd[47]*ROcp9_646;
  RLcp9_148 = q[48]*ROcp9_147;
  RLcp9_248 = q[48]*ROcp9_247;
  RLcp9_348 = q[48]*ROcp9_347;
  ORcp9_148 = OMcp9_247*RLcp9_348-OMcp9_347*RLcp9_248;
  ORcp9_248 = -(OMcp9_147*RLcp9_348-OMcp9_347*RLcp9_148);
  ORcp9_348 = OMcp9_147*RLcp9_248-OMcp9_247*RLcp9_148;

// = = Block_0_0_1_8_0_24 = = 
 
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
  ORcp9_152 = OMcp9_247*RLcp9_352-OMcp9_347*RLcp9_252;
  ORcp9_252 = -(OMcp9_147*RLcp9_352-OMcp9_347*RLcp9_152);
  ORcp9_352 = OMcp9_147*RLcp9_252-OMcp9_247*RLcp9_152;
  PxF8[1] = q[1]+RLcp9_145+RLcp9_148+RLcp9_152;
  PxF8[2] = q[2]+RLcp9_245+RLcp9_248+RLcp9_252;
  PxF8[3] = q[3]+RLcp9_345+RLcp9_348+RLcp9_352;
  RxF8[1][1] = ROcp9_152;
  RxF8[1][2] = ROcp9_252;
  RxF8[1][3] = ROcp9_352;
  RxF8[2][1] = ROcp9_446;
  RxF8[2][2] = ROcp9_546;
  RxF8[2][3] = ROcp9_646;
  RxF8[3][1] = ROcp9_752;
  RxF8[3][2] = ROcp9_852;
  RxF8[3][3] = ROcp9_952;
  VxF8[1] = qd[1]+ORcp9_145+ORcp9_148+ORcp9_152+qd[48]*ROcp9_147;
  VxF8[2] = qd[2]+ORcp9_245+ORcp9_248+ORcp9_252+qd[48]*ROcp9_247;
  VxF8[3] = qd[3]+ORcp9_345+ORcp9_348+ORcp9_352+qd[48]*ROcp9_347;
  OMxF8[1] = OMcp9_147+qd[52]*ROcp9_446;
  OMxF8[2] = OMcp9_247+qd[52]*ROcp9_546;
  OMxF8[3] = OMcp9_347+qd[52]*ROcp9_646;
  AxF8[1] = qdd[1]+(2.0)*qd[48]*(OMcp9_247*ROcp9_347-OMcp9_347*ROcp9_247)+qdd[48]*ROcp9_147+OMcp9_247*ORcp9_348+OMcp9_247*
 ORcp9_352+OMcp9_26*ORcp9_345-OMcp9_347*ORcp9_248-OMcp9_347*ORcp9_252-OMcp9_36*ORcp9_245+OPcp9_247*RLcp9_348+OPcp9_247*
 RLcp9_352+OPcp9_26*RLcp9_345-OPcp9_347*RLcp9_248-OPcp9_347*RLcp9_252-OPcp9_36*RLcp9_245;
  AxF8[2] = qdd[2]-(2.0)*qd[48]*(OMcp9_147*ROcp9_347-OMcp9_347*ROcp9_147)+qdd[48]*ROcp9_247-OMcp9_147*ORcp9_348-OMcp9_147*
 ORcp9_352-OMcp9_16*ORcp9_345+OMcp9_347*ORcp9_148+OMcp9_347*ORcp9_152+OMcp9_36*ORcp9_145-OPcp9_147*RLcp9_348-OPcp9_147*
 RLcp9_352-OPcp9_16*RLcp9_345+OPcp9_347*RLcp9_148+OPcp9_347*RLcp9_152+OPcp9_36*RLcp9_145;
  AxF8[3] = qdd[3]+(2.0)*qd[48]*(OMcp9_147*ROcp9_247-OMcp9_247*ROcp9_147)+qdd[48]*ROcp9_347+OMcp9_147*ORcp9_248+OMcp9_147*
 ORcp9_252+OMcp9_16*ORcp9_245-OMcp9_247*ORcp9_148-OMcp9_247*ORcp9_152-OMcp9_26*ORcp9_145+OPcp9_147*RLcp9_248+OPcp9_147*
 RLcp9_252+OPcp9_16*RLcp9_245-OPcp9_247*RLcp9_148-OPcp9_247*RLcp9_152-OPcp9_26*RLcp9_145;
  OMPxF8[1] = OPcp9_147+qd[52]*(OMcp9_247*ROcp9_646-OMcp9_347*ROcp9_546)+qdd[52]*ROcp9_446;
  OMPxF8[2] = OPcp9_247-qd[52]*(OMcp9_147*ROcp9_646-OMcp9_347*ROcp9_446)+qdd[52]*ROcp9_546;
  OMPxF8[3] = OPcp9_347+qd[52]*(OMcp9_147*ROcp9_546-OMcp9_247*ROcp9_446)+qdd[52]*ROcp9_646;
 
// Sensor Forces Computation 

  SWr8 = user_ExtForces(PxF8,RxF8,VxF8,OMxF8,AxF8,OMPxF8,s,tsim,8);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc110 = ROcp9_152*SWr8[1]+ROcp9_252*SWr8[2]+ROcp9_352*SWr8[3];
  xfrc210 = ROcp9_446*SWr8[1]+ROcp9_546*SWr8[2]+ROcp9_646*SWr8[3];
  xfrc310 = ROcp9_752*SWr8[1]+ROcp9_852*SWr8[2]+ROcp9_952*SWr8[3];
  frc[1][52] = s->frc[1][52]+xfrc110;
  frc[2][52] = s->frc[2][52]+xfrc210;
  frc[3][52] = s->frc[3][52]+xfrc310;
  xtrq110 = ROcp9_152*SWr8[4]+ROcp9_252*SWr8[5]+ROcp9_352*SWr8[6];
  xtrq210 = ROcp9_446*SWr8[4]+ROcp9_546*SWr8[5]+ROcp9_646*SWr8[6];
  xtrq310 = ROcp9_752*SWr8[4]+ROcp9_852*SWr8[5]+ROcp9_952*SWr8[6];
  trq[1][52] = s->trq[1][52]+xtrq110-xfrc210*SWr8[9]+xfrc310*SWr8[8];
  trq[2][52] = s->trq[2][52]+xtrq210+xfrc110*SWr8[9]-xfrc310*SWr8[7];
  trq[3][52] = s->trq[3][52]+xtrq310-xfrc110*SWr8[8]+xfrc210*SWr8[7];

// = = Block_0_0_1_8_1_0 = = 
 
// Symbolic Outputs  

  frc[1][6] = s->frc[1][6];
  frc[2][6] = s->frc[2][6];
  frc[3][6] = s->frc[3][6];
  frc[1][7] = s->frc[1][7];
  frc[2][7] = s->frc[2][7];
  frc[3][7] = s->frc[3][7];
  frc[1][8] = s->frc[1][8];
  frc[2][8] = s->frc[2][8];
  frc[3][8] = s->frc[3][8];
  frc[1][11] = s->frc[1][11];
  frc[2][11] = s->frc[2][11];
  frc[3][11] = s->frc[3][11];
  frc[1][13] = s->frc[1][13];
  frc[2][13] = s->frc[2][13];
  frc[3][13] = s->frc[3][13];
  frc[1][16] = s->frc[1][16];
  frc[2][16] = s->frc[2][16];
  frc[3][16] = s->frc[3][16];
  frc[1][18] = s->frc[1][18];
  frc[2][18] = s->frc[2][18];
  frc[3][18] = s->frc[3][18];
  frc[1][19] = s->frc[1][19];
  frc[2][19] = s->frc[2][19];
  frc[3][19] = s->frc[3][19];
  frc[1][20] = s->frc[1][20];
  frc[2][20] = s->frc[2][20];
  frc[3][20] = s->frc[3][20];
  frc[1][21] = s->frc[1][21];
  frc[2][21] = s->frc[2][21];
  frc[3][21] = s->frc[3][21];
  frc[1][22] = s->frc[1][22];
  frc[2][22] = s->frc[2][22];
  frc[3][22] = s->frc[3][22];
  frc[1][24] = s->frc[1][24];
  frc[2][24] = s->frc[2][24];
  frc[3][24] = s->frc[3][24];
  frc[1][25] = s->frc[1][25];
  frc[2][25] = s->frc[2][25];
  frc[3][25] = s->frc[3][25];
  frc[1][27] = s->frc[1][27];
  frc[2][27] = s->frc[2][27];
  frc[3][27] = s->frc[3][27];
  frc[1][28] = s->frc[1][28];
  frc[2][28] = s->frc[2][28];
  frc[3][28] = s->frc[3][28];
  frc[1][30] = s->frc[1][30];
  frc[2][30] = s->frc[2][30];
  frc[3][30] = s->frc[3][30];
  frc[1][32] = s->frc[1][32];
  frc[2][32] = s->frc[2][32];
  frc[3][32] = s->frc[3][32];
  frc[1][33] = s->frc[1][33];
  frc[2][33] = s->frc[2][33];
  frc[3][33] = s->frc[3][33];
  frc[1][34] = s->frc[1][34];
  frc[2][34] = s->frc[2][34];
  frc[3][34] = s->frc[3][34];
  frc[1][36] = s->frc[1][36];
  frc[2][36] = s->frc[2][36];
  frc[3][36] = s->frc[3][36];
  frc[1][37] = s->frc[1][37];
  frc[2][37] = s->frc[2][37];
  frc[3][37] = s->frc[3][37];
  frc[1][39] = s->frc[1][39];
  frc[2][39] = s->frc[2][39];
  frc[3][39] = s->frc[3][39];
  frc[1][40] = s->frc[1][40];
  frc[2][40] = s->frc[2][40];
  frc[3][40] = s->frc[3][40];
  frc[1][42] = s->frc[1][42];
  frc[2][42] = s->frc[2][42];
  frc[3][42] = s->frc[3][42];
  frc[1][44] = s->frc[1][44];
  frc[2][44] = s->frc[2][44];
  frc[3][44] = s->frc[3][44];
  frc[1][48] = s->frc[1][48];
  frc[2][48] = s->frc[2][48];
  frc[3][48] = s->frc[3][48];
  frc[1][53] = s->frc[1][53];
  frc[2][53] = s->frc[2][53];
  frc[3][53] = s->frc[3][53];
  trq[1][6] = s->trq[1][6];
  trq[2][6] = s->trq[2][6];
  trq[3][6] = s->trq[3][6];
  trq[1][7] = s->trq[1][7];
  trq[2][7] = s->trq[2][7];
  trq[3][7] = s->trq[3][7];
  trq[1][8] = s->trq[1][8];
  trq[2][8] = s->trq[2][8];
  trq[3][8] = s->trq[3][8];
  trq[1][11] = s->trq[1][11];
  trq[2][11] = s->trq[2][11];
  trq[3][11] = s->trq[3][11];
  trq[1][13] = s->trq[1][13];
  trq[2][13] = s->trq[2][13];
  trq[3][13] = s->trq[3][13];
  trq[1][16] = s->trq[1][16];
  trq[2][16] = s->trq[2][16];
  trq[3][16] = s->trq[3][16];
  trq[1][18] = s->trq[1][18];
  trq[2][18] = s->trq[2][18];
  trq[3][18] = s->trq[3][18];
  trq[1][19] = s->trq[1][19];
  trq[2][19] = s->trq[2][19];
  trq[3][19] = s->trq[3][19];
  trq[1][20] = s->trq[1][20];
  trq[2][20] = s->trq[2][20];
  trq[3][20] = s->trq[3][20];
  trq[1][21] = s->trq[1][21];
  trq[2][21] = s->trq[2][21];
  trq[3][21] = s->trq[3][21];
  trq[1][22] = s->trq[1][22];
  trq[2][22] = s->trq[2][22];
  trq[3][22] = s->trq[3][22];
  trq[1][24] = s->trq[1][24];
  trq[2][24] = s->trq[2][24];
  trq[3][24] = s->trq[3][24];
  trq[1][25] = s->trq[1][25];
  trq[2][25] = s->trq[2][25];
  trq[3][25] = s->trq[3][25];
  trq[1][27] = s->trq[1][27];
  trq[2][27] = s->trq[2][27];
  trq[3][27] = s->trq[3][27];
  trq[1][28] = s->trq[1][28];
  trq[2][28] = s->trq[2][28];
  trq[3][28] = s->trq[3][28];
  trq[1][30] = s->trq[1][30];
  trq[2][30] = s->trq[2][30];
  trq[3][30] = s->trq[3][30];
  trq[1][32] = s->trq[1][32];
  trq[2][32] = s->trq[2][32];
  trq[3][32] = s->trq[3][32];
  trq[1][33] = s->trq[1][33];
  trq[2][33] = s->trq[2][33];
  trq[3][33] = s->trq[3][33];
  trq[1][34] = s->trq[1][34];
  trq[2][34] = s->trq[2][34];
  trq[3][34] = s->trq[3][34];
  trq[1][36] = s->trq[1][36];
  trq[2][36] = s->trq[2][36];
  trq[3][36] = s->trq[3][36];
  trq[1][37] = s->trq[1][37];
  trq[2][37] = s->trq[2][37];
  trq[3][37] = s->trq[3][37];
  trq[1][39] = s->trq[1][39];
  trq[2][39] = s->trq[2][39];
  trq[3][39] = s->trq[3][39];
  trq[1][40] = s->trq[1][40];
  trq[2][40] = s->trq[2][40];
  trq[3][40] = s->trq[3][40];
  trq[1][42] = s->trq[1][42];
  trq[2][42] = s->trq[2][42];
  trq[3][42] = s->trq[3][42];
  trq[1][44] = s->trq[1][44];
  trq[2][44] = s->trq[2][44];
  trq[3][44] = s->trq[3][44];
  trq[1][48] = s->trq[1][48];
  trq[2][48] = s->trq[2][48];
  trq[3][48] = s->trq[3][48];
  trq[1][53] = s->trq[1][53];
  trq[2][53] = s->trq[2][53];
  trq[3][53] = s->trq[3][53];

// ====== END Task 0 ====== 


}
 

