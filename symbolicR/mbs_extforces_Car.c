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
//	==> Generation Date : Tue May 18 15:39:26 2021
//
//	==> Project name : Car
//	==> using XML input file 
//
//	==> Number of joints : 59
//
//	==> Function : F19 : External Forces
//	==> Flops complexity : 3534
//
//	==> Generation Time :  0.070 seconds
//	==> Post-Processing :  0.060 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_extforces(double **frc,double **trq,
MbsData *s, double tsim)

// double frc[3][59];
// double trq[3][59];
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

// = = Block_0_0_0_0_0_16 = = 
 
// Trigonometric Variables  

  C35 = cos(q[35]);
  S35 = sin(q[35]);
  C36 = cos(q[36]);
  S36 = sin(q[36]);
  C37 = cos(q[37]);
  S37 = sin(q[37]);

// = = Block_0_0_0_0_0_17 = = 
 
// Trigonometric Variables  

  C38 = cos(q[38]);
  S38 = sin(q[38]);
  C39 = cos(q[39]);
  S39 = sin(q[39]);
  C40 = cos(q[40]);
  S40 = sin(q[40]);

// = = Block_0_0_0_0_0_22 = = 
 
// Trigonometric Variables  

  C51 = cos(q[51]);
  S51 = sin(q[51]);
  C52 = cos(q[52]);
  S52 = sin(q[52]);
  C53 = cos(q[53]);
  S53 = sin(q[53]);

// = = Block_0_0_0_0_0_23 = = 
 
// Trigonometric Variables  

  C55 = cos(q[55]);
  S55 = sin(q[55]);

// = = Block_0_0_0_0_0_24 = = 
 
// Trigonometric Variables  

  C56 = cos(q[56]);
  S56 = sin(q[56]);

// = = Block_0_0_0_0_0_25 = = 
 
// Trigonometric Variables  

  C57 = cos(q[57]);
  S57 = sin(q[57]);

// = = Block_0_0_0_0_0_26 = = 
 
// Trigonometric Variables  

  C58 = cos(q[58]);
  S58 = sin(q[58]);

// = = Block_0_0_1_1_0_1 = = 
 
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

// = = Block_0_0_1_1_0_3 = = 
 
// Sensor Kinematics 


  ROcp4_48 = ROcp4_45*C8+ROcp4_76*S8;
  ROcp4_58 = ROcp4_55*C8+ROcp4_86*S8;
  ROcp4_68 = ROcp4_96*S8+S5*C8;
  ROcp4_78 = -(ROcp4_45*S8-ROcp4_76*C8);
  ROcp4_88 = -(ROcp4_55*S8-ROcp4_86*C8);
  ROcp4_98 = ROcp4_96*C8-S5*S8;
  ROcp4_19 = ROcp4_16*C9-ROcp4_78*S9;
  ROcp4_29 = ROcp4_26*C9-ROcp4_88*S9;
  ROcp4_39 = ROcp4_36*C9-ROcp4_98*S9;
  ROcp4_79 = ROcp4_16*S9+ROcp4_78*C9;
  ROcp4_89 = ROcp4_26*S9+ROcp4_88*C9;
  ROcp4_99 = ROcp4_36*S9+ROcp4_98*C9;
  ROcp4_410 = ROcp4_48*C10+ROcp4_79*S10;
  ROcp4_510 = ROcp4_58*C10+ROcp4_89*S10;
  ROcp4_610 = ROcp4_68*C10+ROcp4_99*S10;
  ROcp4_710 = -(ROcp4_48*S10-ROcp4_79*C10);
  ROcp4_810 = -(ROcp4_58*S10-ROcp4_89*C10);
  ROcp4_910 = -(ROcp4_68*S10-ROcp4_99*C10);
  ROcp4_111 = ROcp4_19*C11+ROcp4_410*S11;
  ROcp4_211 = ROcp4_29*C11+ROcp4_510*S11;
  ROcp4_311 = ROcp4_39*C11+ROcp4_610*S11;
  ROcp4_411 = -(ROcp4_19*S11-ROcp4_410*C11);
  ROcp4_511 = -(ROcp4_29*S11-ROcp4_510*C11);
  ROcp4_611 = -(ROcp4_39*S11-ROcp4_610*C11);
  ROcp4_112 = ROcp4_111*C12-ROcp4_710*S12;
  ROcp4_212 = ROcp4_211*C12-ROcp4_810*S12;
  ROcp4_312 = ROcp4_311*C12-ROcp4_910*S12;
  ROcp4_712 = ROcp4_111*S12+ROcp4_710*C12;
  ROcp4_812 = ROcp4_211*S12+ROcp4_810*C12;
  ROcp4_912 = ROcp4_311*S12+ROcp4_910*C12;
  RLcp4_18 = ROcp4_16*s->dpt[1][1]+ROcp4_45*s->dpt[2][1]+ROcp4_76*s->dpt[3][1];
  RLcp4_28 = ROcp4_26*s->dpt[1][1]+ROcp4_55*s->dpt[2][1]+ROcp4_86*s->dpt[3][1];
  RLcp4_38 = ROcp4_36*s->dpt[1][1]+ROcp4_96*s->dpt[3][1]+s->dpt[2][1]*S5;
  OMcp4_18 = OMcp4_16+qd[8]*ROcp4_16;
  OMcp4_28 = OMcp4_26+qd[8]*ROcp4_26;
  OMcp4_38 = OMcp4_36+qd[8]*ROcp4_36;
  ORcp4_18 = OMcp4_26*RLcp4_38-OMcp4_36*RLcp4_28;
  ORcp4_28 = -(OMcp4_16*RLcp4_38-OMcp4_36*RLcp4_18);
  ORcp4_38 = OMcp4_16*RLcp4_28-OMcp4_26*RLcp4_18;
  OPcp4_18 = OPcp4_16+qd[8]*(OMcp4_26*ROcp4_36-OMcp4_36*ROcp4_26)+qdd[8]*ROcp4_16;
  OPcp4_28 = OPcp4_26-qd[8]*(OMcp4_16*ROcp4_36-OMcp4_36*ROcp4_16)+qdd[8]*ROcp4_26;
  OPcp4_38 = OPcp4_36+qd[8]*(OMcp4_16*ROcp4_26-OMcp4_26*ROcp4_16)+qdd[8]*ROcp4_36;
  RLcp4_19 = ROcp4_48*s->dpt[2][19];
  RLcp4_29 = ROcp4_58*s->dpt[2][19];
  RLcp4_39 = ROcp4_68*s->dpt[2][19];
  OMcp4_19 = OMcp4_18+qd[9]*ROcp4_48;
  OMcp4_29 = OMcp4_28+qd[9]*ROcp4_58;
  OMcp4_39 = OMcp4_38+qd[9]*ROcp4_68;
  ORcp4_19 = OMcp4_28*RLcp4_39-OMcp4_38*RLcp4_29;
  ORcp4_29 = -(OMcp4_18*RLcp4_39-OMcp4_38*RLcp4_19);
  ORcp4_39 = OMcp4_18*RLcp4_29-OMcp4_28*RLcp4_19;
  OMcp4_110 = OMcp4_19+qd[10]*ROcp4_19;
  OMcp4_210 = OMcp4_29+qd[10]*ROcp4_29;
  OMcp4_310 = OMcp4_39+qd[10]*ROcp4_39;
  OMcp4_111 = OMcp4_110+qd[11]*ROcp4_710;
  OMcp4_211 = OMcp4_210+qd[11]*ROcp4_810;
  OMcp4_311 = OMcp4_310+qd[11]*ROcp4_910;
  OPcp4_111 = OPcp4_18+qd[10]*(OMcp4_29*ROcp4_39-OMcp4_39*ROcp4_29)+qd[11]*(OMcp4_210*ROcp4_910-OMcp4_310*ROcp4_810)+
 qd[9]*(OMcp4_28*ROcp4_68-OMcp4_38*ROcp4_58)+qdd[10]*ROcp4_19+qdd[11]*ROcp4_710+qdd[9]*ROcp4_48;
  OPcp4_211 = OPcp4_28-qd[10]*(OMcp4_19*ROcp4_39-OMcp4_39*ROcp4_19)-qd[11]*(OMcp4_110*ROcp4_910-OMcp4_310*ROcp4_710)-
 qd[9]*(OMcp4_18*ROcp4_68-OMcp4_38*ROcp4_48)+qdd[10]*ROcp4_29+qdd[11]*ROcp4_810+qdd[9]*ROcp4_58;
  OPcp4_311 = OPcp4_38+qd[10]*(OMcp4_19*ROcp4_29-OMcp4_29*ROcp4_19)+qd[11]*(OMcp4_110*ROcp4_810-OMcp4_210*ROcp4_710)+
 qd[9]*(OMcp4_18*ROcp4_58-OMcp4_28*ROcp4_48)+qdd[10]*ROcp4_39+qdd[11]*ROcp4_910+qdd[9]*ROcp4_68;
  RLcp4_112 = ROcp4_710*s->dpt[3][23];
  RLcp4_212 = ROcp4_810*s->dpt[3][23];
  RLcp4_312 = ROcp4_910*s->dpt[3][23];
  ORcp4_112 = OMcp4_211*RLcp4_312-OMcp4_311*RLcp4_212;
  ORcp4_212 = -(OMcp4_111*RLcp4_312-OMcp4_311*RLcp4_112);
  ORcp4_312 = OMcp4_111*RLcp4_212-OMcp4_211*RLcp4_112;
  PxF1[1] = q[1]+RLcp4_112+RLcp4_18+RLcp4_19;
  PxF1[2] = q[2]+RLcp4_212+RLcp4_28+RLcp4_29;
  PxF1[3] = q[3]+RLcp4_312+RLcp4_38+RLcp4_39;
  RxF1[1][1] = ROcp4_112;
  RxF1[1][2] = ROcp4_212;
  RxF1[1][3] = ROcp4_312;
  RxF1[2][1] = ROcp4_411;
  RxF1[2][2] = ROcp4_511;
  RxF1[2][3] = ROcp4_611;
  RxF1[3][1] = ROcp4_712;
  RxF1[3][2] = ROcp4_812;
  RxF1[3][3] = ROcp4_912;
  VxF1[1] = qd[1]+ORcp4_112+ORcp4_18+ORcp4_19;
  VxF1[2] = qd[2]+ORcp4_212+ORcp4_28+ORcp4_29;
  VxF1[3] = qd[3]+ORcp4_312+ORcp4_38+ORcp4_39;
  OMxF1[1] = OMcp4_111+qd[12]*ROcp4_411;
  OMxF1[2] = OMcp4_211+qd[12]*ROcp4_511;
  OMxF1[3] = OMcp4_311+qd[12]*ROcp4_611;
  AxF1[1] = qdd[1]+OMcp4_211*ORcp4_312+OMcp4_26*ORcp4_38+OMcp4_28*ORcp4_39-OMcp4_311*ORcp4_212-OMcp4_36*ORcp4_28-
 OMcp4_38*ORcp4_29+OPcp4_211*RLcp4_312+OPcp4_26*RLcp4_38+OPcp4_28*RLcp4_39-OPcp4_311*RLcp4_212-OPcp4_36*RLcp4_28-OPcp4_38*
 RLcp4_29;
  AxF1[2] = qdd[2]-OMcp4_111*ORcp4_312-OMcp4_16*ORcp4_38-OMcp4_18*ORcp4_39+OMcp4_311*ORcp4_112+OMcp4_36*ORcp4_18+
 OMcp4_38*ORcp4_19-OPcp4_111*RLcp4_312-OPcp4_16*RLcp4_38-OPcp4_18*RLcp4_39+OPcp4_311*RLcp4_112+OPcp4_36*RLcp4_18+OPcp4_38*
 RLcp4_19;
  AxF1[3] = qdd[3]+OMcp4_111*ORcp4_212+OMcp4_16*ORcp4_28+OMcp4_18*ORcp4_29-OMcp4_211*ORcp4_112-OMcp4_26*ORcp4_18-
 OMcp4_28*ORcp4_19+OPcp4_111*RLcp4_212+OPcp4_16*RLcp4_28+OPcp4_18*RLcp4_29-OPcp4_211*RLcp4_112-OPcp4_26*RLcp4_18-OPcp4_28*
 RLcp4_19;
  OMPxF1[1] = OPcp4_111+qd[12]*(OMcp4_211*ROcp4_611-OMcp4_311*ROcp4_511)+qdd[12]*ROcp4_411;
  OMPxF1[2] = OPcp4_211-qd[12]*(OMcp4_111*ROcp4_611-OMcp4_311*ROcp4_411)+qdd[12]*ROcp4_511;
  OMPxF1[3] = OPcp4_311+qd[12]*(OMcp4_111*ROcp4_511-OMcp4_211*ROcp4_411)+qdd[12]*ROcp4_611;
 
// Sensor Forces Computation 

  SWr1 = user_ExtForces(PxF1,RxF1,VxF1,OMxF1,AxF1,OMPxF1,s,tsim,1);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc15 = ROcp4_112*SWr1[1]+ROcp4_212*SWr1[2]+ROcp4_312*SWr1[3];
  xfrc25 = ROcp4_411*SWr1[1]+ROcp4_511*SWr1[2]+ROcp4_611*SWr1[3];
  xfrc35 = ROcp4_712*SWr1[1]+ROcp4_812*SWr1[2]+ROcp4_912*SWr1[3];
  frc[1][12] = s->frc[1][12]+xfrc15;
  frc[2][12] = s->frc[2][12]+xfrc25;
  frc[3][12] = s->frc[3][12]+xfrc35;
  xtrq15 = ROcp4_112*SWr1[4]+ROcp4_212*SWr1[5]+ROcp4_312*SWr1[6];
  xtrq25 = ROcp4_411*SWr1[4]+ROcp4_511*SWr1[5]+ROcp4_611*SWr1[6];
  xtrq35 = ROcp4_712*SWr1[4]+ROcp4_812*SWr1[5]+ROcp4_912*SWr1[6];
  trq[1][12] = s->trq[1][12]+xtrq15-xfrc25*SWr1[9]+xfrc35*SWr1[8];
  trq[2][12] = s->trq[2][12]+xtrq25+xfrc15*SWr1[9]-xfrc35*SWr1[7];
  trq[3][12] = s->trq[3][12]+xtrq35-xfrc15*SWr1[8]+xfrc25*SWr1[7];

// = = Block_0_0_1_2_0_1 = = 
 
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

// = = Block_0_0_1_2_0_4 = = 
 
// Sensor Kinematics 


  ROcp5_413 = ROcp5_45*C13+ROcp5_76*S13;
  ROcp5_513 = ROcp5_55*C13+ROcp5_86*S13;
  ROcp5_613 = ROcp5_96*S13+C13*S5;
  ROcp5_713 = -(ROcp5_45*S13-ROcp5_76*C13);
  ROcp5_813 = -(ROcp5_55*S13-ROcp5_86*C13);
  ROcp5_913 = ROcp5_96*C13-S13*S5;
  ROcp5_114 = ROcp5_16*C14-ROcp5_713*S14;
  ROcp5_214 = ROcp5_26*C14-ROcp5_813*S14;
  ROcp5_314 = ROcp5_36*C14-ROcp5_913*S14;
  ROcp5_714 = ROcp5_16*S14+ROcp5_713*C14;
  ROcp5_814 = ROcp5_26*S14+ROcp5_813*C14;
  ROcp5_914 = ROcp5_36*S14+ROcp5_913*C14;
  ROcp5_415 = ROcp5_413*C15+ROcp5_714*S15;
  ROcp5_515 = ROcp5_513*C15+ROcp5_814*S15;
  ROcp5_615 = ROcp5_613*C15+ROcp5_914*S15;
  ROcp5_715 = -(ROcp5_413*S15-ROcp5_714*C15);
  ROcp5_815 = -(ROcp5_513*S15-ROcp5_814*C15);
  ROcp5_915 = -(ROcp5_613*S15-ROcp5_914*C15);
  ROcp5_116 = ROcp5_114*C16+ROcp5_415*S16;
  ROcp5_216 = ROcp5_214*C16+ROcp5_515*S16;
  ROcp5_316 = ROcp5_314*C16+ROcp5_615*S16;
  ROcp5_416 = -(ROcp5_114*S16-ROcp5_415*C16);
  ROcp5_516 = -(ROcp5_214*S16-ROcp5_515*C16);
  ROcp5_616 = -(ROcp5_314*S16-ROcp5_615*C16);
  ROcp5_117 = ROcp5_116*C17-ROcp5_715*S17;
  ROcp5_217 = ROcp5_216*C17-ROcp5_815*S17;
  ROcp5_317 = ROcp5_316*C17-ROcp5_915*S17;
  ROcp5_717 = ROcp5_116*S17+ROcp5_715*C17;
  ROcp5_817 = ROcp5_216*S17+ROcp5_815*C17;
  ROcp5_917 = ROcp5_316*S17+ROcp5_915*C17;
  RLcp5_113 = ROcp5_16*s->dpt[1][2]+ROcp5_45*s->dpt[2][2]+ROcp5_76*s->dpt[3][2];
  RLcp5_213 = ROcp5_26*s->dpt[1][2]+ROcp5_55*s->dpt[2][2]+ROcp5_86*s->dpt[3][2];
  RLcp5_313 = ROcp5_36*s->dpt[1][2]+ROcp5_96*s->dpt[3][2]+s->dpt[2][2]*S5;
  OMcp5_113 = OMcp5_16+qd[13]*ROcp5_16;
  OMcp5_213 = OMcp5_26+qd[13]*ROcp5_26;
  OMcp5_313 = OMcp5_36+qd[13]*ROcp5_36;
  ORcp5_113 = OMcp5_26*RLcp5_313-OMcp5_36*RLcp5_213;
  ORcp5_213 = -(OMcp5_16*RLcp5_313-OMcp5_36*RLcp5_113);
  ORcp5_313 = OMcp5_16*RLcp5_213-OMcp5_26*RLcp5_113;
  OPcp5_113 = OPcp5_16+qd[13]*(OMcp5_26*ROcp5_36-OMcp5_36*ROcp5_26)+qdd[13]*ROcp5_16;
  OPcp5_213 = OPcp5_26-qd[13]*(OMcp5_16*ROcp5_36-OMcp5_36*ROcp5_16)+qdd[13]*ROcp5_26;
  OPcp5_313 = OPcp5_36+qd[13]*(OMcp5_16*ROcp5_26-OMcp5_26*ROcp5_16)+qdd[13]*ROcp5_36;
  RLcp5_114 = ROcp5_413*s->dpt[2][26];
  RLcp5_214 = ROcp5_513*s->dpt[2][26];
  RLcp5_314 = ROcp5_613*s->dpt[2][26];
  OMcp5_114 = OMcp5_113+qd[14]*ROcp5_413;
  OMcp5_214 = OMcp5_213+qd[14]*ROcp5_513;
  OMcp5_314 = OMcp5_313+qd[14]*ROcp5_613;
  ORcp5_114 = OMcp5_213*RLcp5_314-OMcp5_313*RLcp5_214;
  ORcp5_214 = -(OMcp5_113*RLcp5_314-OMcp5_313*RLcp5_114);
  ORcp5_314 = OMcp5_113*RLcp5_214-OMcp5_213*RLcp5_114;
  OMcp5_115 = OMcp5_114+qd[15]*ROcp5_114;
  OMcp5_215 = OMcp5_214+qd[15]*ROcp5_214;
  OMcp5_315 = OMcp5_314+qd[15]*ROcp5_314;
  OMcp5_116 = OMcp5_115+qd[16]*ROcp5_715;
  OMcp5_216 = OMcp5_215+qd[16]*ROcp5_815;
  OMcp5_316 = OMcp5_315+qd[16]*ROcp5_915;
  OPcp5_116 = OPcp5_113+qd[14]*(OMcp5_213*ROcp5_613-OMcp5_313*ROcp5_513)+qd[15]*(OMcp5_214*ROcp5_314-OMcp5_314*ROcp5_214
 )+qd[16]*(OMcp5_215*ROcp5_915-OMcp5_315*ROcp5_815)+qdd[14]*ROcp5_413+qdd[15]*ROcp5_114+qdd[16]*ROcp5_715;
  OPcp5_216 = OPcp5_213-qd[14]*(OMcp5_113*ROcp5_613-OMcp5_313*ROcp5_413)-qd[15]*(OMcp5_114*ROcp5_314-OMcp5_314*ROcp5_114
 )-qd[16]*(OMcp5_115*ROcp5_915-OMcp5_315*ROcp5_715)+qdd[14]*ROcp5_513+qdd[15]*ROcp5_214+qdd[16]*ROcp5_815;
  OPcp5_316 = OPcp5_313+qd[14]*(OMcp5_113*ROcp5_513-OMcp5_213*ROcp5_413)+qd[15]*(OMcp5_114*ROcp5_214-OMcp5_214*ROcp5_114
 )+qd[16]*(OMcp5_115*ROcp5_815-OMcp5_215*ROcp5_715)+qdd[14]*ROcp5_613+qdd[15]*ROcp5_314+qdd[16]*ROcp5_915;
  RLcp5_117 = ROcp5_715*s->dpt[3][29];
  RLcp5_217 = ROcp5_815*s->dpt[3][29];
  RLcp5_317 = ROcp5_915*s->dpt[3][29];
  ORcp5_117 = OMcp5_216*RLcp5_317-OMcp5_316*RLcp5_217;
  ORcp5_217 = -(OMcp5_116*RLcp5_317-OMcp5_316*RLcp5_117);
  ORcp5_317 = OMcp5_116*RLcp5_217-OMcp5_216*RLcp5_117;
  PxF2[1] = q[1]+RLcp5_113+RLcp5_114+RLcp5_117;
  PxF2[2] = q[2]+RLcp5_213+RLcp5_214+RLcp5_217;
  PxF2[3] = q[3]+RLcp5_313+RLcp5_314+RLcp5_317;
  RxF2[1][1] = ROcp5_117;
  RxF2[1][2] = ROcp5_217;
  RxF2[1][3] = ROcp5_317;
  RxF2[2][1] = ROcp5_416;
  RxF2[2][2] = ROcp5_516;
  RxF2[2][3] = ROcp5_616;
  RxF2[3][1] = ROcp5_717;
  RxF2[3][2] = ROcp5_817;
  RxF2[3][3] = ROcp5_917;
  VxF2[1] = qd[1]+ORcp5_113+ORcp5_114+ORcp5_117;
  VxF2[2] = qd[2]+ORcp5_213+ORcp5_214+ORcp5_217;
  VxF2[3] = qd[3]+ORcp5_313+ORcp5_314+ORcp5_317;
  OMxF2[1] = OMcp5_116+qd[17]*ROcp5_416;
  OMxF2[2] = OMcp5_216+qd[17]*ROcp5_516;
  OMxF2[3] = OMcp5_316+qd[17]*ROcp5_616;
  AxF2[1] = qdd[1]+OMcp5_213*ORcp5_314+OMcp5_216*ORcp5_317+OMcp5_26*ORcp5_313-OMcp5_313*ORcp5_214-OMcp5_316*ORcp5_217-
 OMcp5_36*ORcp5_213+OPcp5_213*RLcp5_314+OPcp5_216*RLcp5_317+OPcp5_26*RLcp5_313-OPcp5_313*RLcp5_214-OPcp5_316*RLcp5_217-
 OPcp5_36*RLcp5_213;
  AxF2[2] = qdd[2]-OMcp5_113*ORcp5_314-OMcp5_116*ORcp5_317-OMcp5_16*ORcp5_313+OMcp5_313*ORcp5_114+OMcp5_316*ORcp5_117+
 OMcp5_36*ORcp5_113-OPcp5_113*RLcp5_314-OPcp5_116*RLcp5_317-OPcp5_16*RLcp5_313+OPcp5_313*RLcp5_114+OPcp5_316*RLcp5_117+
 OPcp5_36*RLcp5_113;
  AxF2[3] = qdd[3]+OMcp5_113*ORcp5_214+OMcp5_116*ORcp5_217+OMcp5_16*ORcp5_213-OMcp5_213*ORcp5_114-OMcp5_216*ORcp5_117-
 OMcp5_26*ORcp5_113+OPcp5_113*RLcp5_214+OPcp5_116*RLcp5_217+OPcp5_16*RLcp5_213-OPcp5_213*RLcp5_114-OPcp5_216*RLcp5_117-
 OPcp5_26*RLcp5_113;
  OMPxF2[1] = OPcp5_116+qd[17]*(OMcp5_216*ROcp5_616-OMcp5_316*ROcp5_516)+qdd[17]*ROcp5_416;
  OMPxF2[2] = OPcp5_216-qd[17]*(OMcp5_116*ROcp5_616-OMcp5_316*ROcp5_416)+qdd[17]*ROcp5_516;
  OMPxF2[3] = OPcp5_316+qd[17]*(OMcp5_116*ROcp5_516-OMcp5_216*ROcp5_416)+qdd[17]*ROcp5_616;
 
// Sensor Forces Computation 

  SWr2 = user_ExtForces(PxF2,RxF2,VxF2,OMxF2,AxF2,OMPxF2,s,tsim,2);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc16 = ROcp5_117*SWr2[1]+ROcp5_217*SWr2[2]+ROcp5_317*SWr2[3];
  xfrc26 = ROcp5_416*SWr2[1]+ROcp5_516*SWr2[2]+ROcp5_616*SWr2[3];
  xfrc36 = ROcp5_717*SWr2[1]+ROcp5_817*SWr2[2]+ROcp5_917*SWr2[3];
  frc[1][17] = s->frc[1][17]+xfrc16;
  frc[2][17] = s->frc[2][17]+xfrc26;
  frc[3][17] = s->frc[3][17]+xfrc36;
  xtrq16 = ROcp5_117*SWr2[4]+ROcp5_217*SWr2[5]+ROcp5_317*SWr2[6];
  xtrq26 = ROcp5_416*SWr2[4]+ROcp5_516*SWr2[5]+ROcp5_616*SWr2[6];
  xtrq36 = ROcp5_717*SWr2[4]+ROcp5_817*SWr2[5]+ROcp5_917*SWr2[6];
  trq[1][17] = s->trq[1][17]+xtrq16-xfrc26*SWr2[9]+xfrc36*SWr2[8];
  trq[2][17] = s->trq[2][17]+xtrq26+xfrc16*SWr2[9]-xfrc36*SWr2[7];
  trq[3][17] = s->trq[3][17]+xtrq36-xfrc16*SWr2[8]+xfrc26*SWr2[7];

// = = Block_0_0_1_3_0_1 = = 
 
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

// = = Block_0_0_1_3_0_16 = = 
 
// Sensor Kinematics 


  ROcp6_435 = ROcp6_45*C35+ROcp6_76*S35;
  ROcp6_535 = ROcp6_55*C35+ROcp6_86*S35;
  ROcp6_635 = ROcp6_96*S35+C35*S5;
  ROcp6_735 = -(ROcp6_45*S35-ROcp6_76*C35);
  ROcp6_835 = -(ROcp6_55*S35-ROcp6_86*C35);
  ROcp6_935 = ROcp6_96*C35-S35*S5;
  ROcp6_436 = ROcp6_435*C36+ROcp6_735*S36;
  ROcp6_536 = ROcp6_535*C36+ROcp6_835*S36;
  ROcp6_636 = ROcp6_635*C36+ROcp6_935*S36;
  ROcp6_736 = -(ROcp6_435*S36-ROcp6_735*C36);
  ROcp6_836 = -(ROcp6_535*S36-ROcp6_835*C36);
  ROcp6_936 = -(ROcp6_635*S36-ROcp6_935*C36);
  ROcp6_137 = ROcp6_16*C37-ROcp6_736*S37;
  ROcp6_237 = ROcp6_26*C37-ROcp6_836*S37;
  ROcp6_337 = ROcp6_36*C37-ROcp6_936*S37;
  ROcp6_737 = ROcp6_16*S37+ROcp6_736*C37;
  ROcp6_837 = ROcp6_26*S37+ROcp6_836*C37;
  ROcp6_937 = ROcp6_36*S37+ROcp6_936*C37;
  RLcp6_135 = ROcp6_16*s->dpt[1][12]+ROcp6_45*s->dpt[2][12]+ROcp6_76*s->dpt[3][12];
  RLcp6_235 = ROcp6_26*s->dpt[1][12]+ROcp6_55*s->dpt[2][12]+ROcp6_86*s->dpt[3][12];
  RLcp6_335 = ROcp6_36*s->dpt[1][12]+ROcp6_96*s->dpt[3][12]+s->dpt[2][12]*S5;
  OMcp6_135 = OMcp6_16+qd[35]*ROcp6_16;
  OMcp6_235 = OMcp6_26+qd[35]*ROcp6_26;
  OMcp6_335 = OMcp6_36+qd[35]*ROcp6_36;
  ORcp6_135 = OMcp6_26*RLcp6_335-OMcp6_36*RLcp6_235;
  ORcp6_235 = -(OMcp6_16*RLcp6_335-OMcp6_36*RLcp6_135);
  ORcp6_335 = OMcp6_16*RLcp6_235-OMcp6_26*RLcp6_135;
  OPcp6_135 = OPcp6_16+qd[35]*(OMcp6_26*ROcp6_36-OMcp6_36*ROcp6_26)+qdd[35]*ROcp6_16;
  OPcp6_235 = OPcp6_26-qd[35]*(OMcp6_16*ROcp6_36-OMcp6_36*ROcp6_16)+qdd[35]*ROcp6_26;
  OPcp6_335 = OPcp6_36+qd[35]*(OMcp6_16*ROcp6_26-OMcp6_26*ROcp6_16)+qdd[35]*ROcp6_36;
  RLcp6_136 = ROcp6_435*s->dpt[2][50];
  RLcp6_236 = ROcp6_535*s->dpt[2][50];
  RLcp6_336 = ROcp6_635*s->dpt[2][50];
  OMcp6_136 = OMcp6_135+qd[36]*ROcp6_16;
  OMcp6_236 = OMcp6_235+qd[36]*ROcp6_26;
  OMcp6_336 = OMcp6_335+qd[36]*ROcp6_36;
  ORcp6_136 = OMcp6_235*RLcp6_336-OMcp6_335*RLcp6_236;
  ORcp6_236 = -(OMcp6_135*RLcp6_336-OMcp6_335*RLcp6_136);
  ORcp6_336 = OMcp6_135*RLcp6_236-OMcp6_235*RLcp6_136;
  OPcp6_136 = OPcp6_135+qd[36]*(OMcp6_235*ROcp6_36-OMcp6_335*ROcp6_26)+qdd[36]*ROcp6_16;
  OPcp6_236 = OPcp6_235-qd[36]*(OMcp6_135*ROcp6_36-OMcp6_335*ROcp6_16)+qdd[36]*ROcp6_26;
  OPcp6_336 = OPcp6_335+qd[36]*(OMcp6_135*ROcp6_26-OMcp6_235*ROcp6_16)+qdd[36]*ROcp6_36;
  RLcp6_137 = ROcp6_736*s->dpt[3][53];
  RLcp6_237 = ROcp6_836*s->dpt[3][53];
  RLcp6_337 = ROcp6_936*s->dpt[3][53];
  ORcp6_137 = OMcp6_236*RLcp6_337-OMcp6_336*RLcp6_237;
  ORcp6_237 = -(OMcp6_136*RLcp6_337-OMcp6_336*RLcp6_137);
  ORcp6_337 = OMcp6_136*RLcp6_237-OMcp6_236*RLcp6_137;
  PxF3[1] = q[1]+RLcp6_135+RLcp6_136+RLcp6_137;
  PxF3[2] = q[2]+RLcp6_235+RLcp6_236+RLcp6_237;
  PxF3[3] = q[3]+RLcp6_335+RLcp6_336+RLcp6_337;
  RxF3[1][1] = ROcp6_137;
  RxF3[1][2] = ROcp6_237;
  RxF3[1][3] = ROcp6_337;
  RxF3[2][1] = ROcp6_436;
  RxF3[2][2] = ROcp6_536;
  RxF3[2][3] = ROcp6_636;
  RxF3[3][1] = ROcp6_737;
  RxF3[3][2] = ROcp6_837;
  RxF3[3][3] = ROcp6_937;
  VxF3[1] = qd[1]+ORcp6_135+ORcp6_136+ORcp6_137;
  VxF3[2] = qd[2]+ORcp6_235+ORcp6_236+ORcp6_237;
  VxF3[3] = qd[3]+ORcp6_335+ORcp6_336+ORcp6_337;
  OMxF3[1] = OMcp6_136+qd[37]*ROcp6_436;
  OMxF3[2] = OMcp6_236+qd[37]*ROcp6_536;
  OMxF3[3] = OMcp6_336+qd[37]*ROcp6_636;
  AxF3[1] = qdd[1]+OMcp6_235*ORcp6_336+OMcp6_236*ORcp6_337+OMcp6_26*ORcp6_335-OMcp6_335*ORcp6_236-OMcp6_336*ORcp6_237-
 OMcp6_36*ORcp6_235+OPcp6_235*RLcp6_336+OPcp6_236*RLcp6_337+OPcp6_26*RLcp6_335-OPcp6_335*RLcp6_236-OPcp6_336*RLcp6_237-
 OPcp6_36*RLcp6_235;
  AxF3[2] = qdd[2]-OMcp6_135*ORcp6_336-OMcp6_136*ORcp6_337-OMcp6_16*ORcp6_335+OMcp6_335*ORcp6_136+OMcp6_336*ORcp6_137+
 OMcp6_36*ORcp6_135-OPcp6_135*RLcp6_336-OPcp6_136*RLcp6_337-OPcp6_16*RLcp6_335+OPcp6_335*RLcp6_136+OPcp6_336*RLcp6_137+
 OPcp6_36*RLcp6_135;
  AxF3[3] = qdd[3]+OMcp6_135*ORcp6_236+OMcp6_136*ORcp6_237+OMcp6_16*ORcp6_235-OMcp6_235*ORcp6_136-OMcp6_236*ORcp6_137-
 OMcp6_26*ORcp6_135+OPcp6_135*RLcp6_236+OPcp6_136*RLcp6_237+OPcp6_16*RLcp6_235-OPcp6_235*RLcp6_136-OPcp6_236*RLcp6_137-
 OPcp6_26*RLcp6_135;
  OMPxF3[1] = OPcp6_136+qd[37]*(OMcp6_236*ROcp6_636-OMcp6_336*ROcp6_536)+qdd[37]*ROcp6_436;
  OMPxF3[2] = OPcp6_236-qd[37]*(OMcp6_136*ROcp6_636-OMcp6_336*ROcp6_436)+qdd[37]*ROcp6_536;
  OMPxF3[3] = OPcp6_336+qd[37]*(OMcp6_136*ROcp6_536-OMcp6_236*ROcp6_436)+qdd[37]*ROcp6_636;
 
// Sensor Forces Computation 

  SWr3 = user_ExtForces(PxF3,RxF3,VxF3,OMxF3,AxF3,OMPxF3,s,tsim,3);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc17 = ROcp6_137*SWr3[1]+ROcp6_237*SWr3[2]+ROcp6_337*SWr3[3];
  xfrc27 = ROcp6_436*SWr3[1]+ROcp6_536*SWr3[2]+ROcp6_636*SWr3[3];
  xfrc37 = ROcp6_737*SWr3[1]+ROcp6_837*SWr3[2]+ROcp6_937*SWr3[3];
  frc[1][37] = s->frc[1][37]+xfrc17;
  frc[2][37] = s->frc[2][37]+xfrc27;
  frc[3][37] = s->frc[3][37]+xfrc37;
  xtrq17 = ROcp6_137*SWr3[4]+ROcp6_237*SWr3[5]+ROcp6_337*SWr3[6];
  xtrq27 = ROcp6_436*SWr3[4]+ROcp6_536*SWr3[5]+ROcp6_636*SWr3[6];
  xtrq37 = ROcp6_737*SWr3[4]+ROcp6_837*SWr3[5]+ROcp6_937*SWr3[6];
  trq[1][37] = s->trq[1][37]+xtrq17-xfrc27*SWr3[9]+xfrc37*SWr3[8];
  trq[2][37] = s->trq[2][37]+xtrq27+xfrc17*SWr3[9]-xfrc37*SWr3[7];
  trq[3][37] = s->trq[3][37]+xtrq37-xfrc17*SWr3[8]+xfrc27*SWr3[7];

// = = Block_0_0_1_4_0_1 = = 
 
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

// = = Block_0_0_1_4_0_17 = = 
 
// Sensor Kinematics 


  ROcp7_438 = ROcp7_45*C38+ROcp7_76*S38;
  ROcp7_538 = ROcp7_55*C38+ROcp7_86*S38;
  ROcp7_638 = ROcp7_96*S38+C38*S5;
  ROcp7_738 = -(ROcp7_45*S38-ROcp7_76*C38);
  ROcp7_838 = -(ROcp7_55*S38-ROcp7_86*C38);
  ROcp7_938 = ROcp7_96*C38-S38*S5;
  ROcp7_439 = ROcp7_438*C39+ROcp7_738*S39;
  ROcp7_539 = ROcp7_538*C39+ROcp7_838*S39;
  ROcp7_639 = ROcp7_638*C39+ROcp7_938*S39;
  ROcp7_739 = -(ROcp7_438*S39-ROcp7_738*C39);
  ROcp7_839 = -(ROcp7_538*S39-ROcp7_838*C39);
  ROcp7_939 = -(ROcp7_638*S39-ROcp7_938*C39);
  ROcp7_140 = ROcp7_16*C40-ROcp7_739*S40;
  ROcp7_240 = ROcp7_26*C40-ROcp7_839*S40;
  ROcp7_340 = ROcp7_36*C40-ROcp7_939*S40;
  ROcp7_740 = ROcp7_16*S40+ROcp7_739*C40;
  ROcp7_840 = ROcp7_26*S40+ROcp7_839*C40;
  ROcp7_940 = ROcp7_36*S40+ROcp7_939*C40;
  RLcp7_138 = ROcp7_16*s->dpt[1][13]+ROcp7_45*s->dpt[2][13]+ROcp7_76*s->dpt[3][13];
  RLcp7_238 = ROcp7_26*s->dpt[1][13]+ROcp7_55*s->dpt[2][13]+ROcp7_86*s->dpt[3][13];
  RLcp7_338 = ROcp7_36*s->dpt[1][13]+ROcp7_96*s->dpt[3][13]+s->dpt[2][13]*S5;
  OMcp7_138 = OMcp7_16+qd[38]*ROcp7_16;
  OMcp7_238 = OMcp7_26+qd[38]*ROcp7_26;
  OMcp7_338 = OMcp7_36+qd[38]*ROcp7_36;
  ORcp7_138 = OMcp7_26*RLcp7_338-OMcp7_36*RLcp7_238;
  ORcp7_238 = -(OMcp7_16*RLcp7_338-OMcp7_36*RLcp7_138);
  ORcp7_338 = OMcp7_16*RLcp7_238-OMcp7_26*RLcp7_138;
  OPcp7_138 = OPcp7_16+qd[38]*(OMcp7_26*ROcp7_36-OMcp7_36*ROcp7_26)+qdd[38]*ROcp7_16;
  OPcp7_238 = OPcp7_26-qd[38]*(OMcp7_16*ROcp7_36-OMcp7_36*ROcp7_16)+qdd[38]*ROcp7_26;
  OPcp7_338 = OPcp7_36+qd[38]*(OMcp7_16*ROcp7_26-OMcp7_26*ROcp7_16)+qdd[38]*ROcp7_36;
  RLcp7_139 = ROcp7_438*s->dpt[2][55];
  RLcp7_239 = ROcp7_538*s->dpt[2][55];
  RLcp7_339 = ROcp7_638*s->dpt[2][55];
  OMcp7_139 = OMcp7_138+qd[39]*ROcp7_16;
  OMcp7_239 = OMcp7_238+qd[39]*ROcp7_26;
  OMcp7_339 = OMcp7_338+qd[39]*ROcp7_36;
  ORcp7_139 = OMcp7_238*RLcp7_339-OMcp7_338*RLcp7_239;
  ORcp7_239 = -(OMcp7_138*RLcp7_339-OMcp7_338*RLcp7_139);
  ORcp7_339 = OMcp7_138*RLcp7_239-OMcp7_238*RLcp7_139;
  OPcp7_139 = OPcp7_138+qd[39]*(OMcp7_238*ROcp7_36-OMcp7_338*ROcp7_26)+qdd[39]*ROcp7_16;
  OPcp7_239 = OPcp7_238-qd[39]*(OMcp7_138*ROcp7_36-OMcp7_338*ROcp7_16)+qdd[39]*ROcp7_26;
  OPcp7_339 = OPcp7_338+qd[39]*(OMcp7_138*ROcp7_26-OMcp7_238*ROcp7_16)+qdd[39]*ROcp7_36;
  RLcp7_140 = ROcp7_739*s->dpt[3][57];
  RLcp7_240 = ROcp7_839*s->dpt[3][57];
  RLcp7_340 = ROcp7_939*s->dpt[3][57];
  ORcp7_140 = OMcp7_239*RLcp7_340-OMcp7_339*RLcp7_240;
  ORcp7_240 = -(OMcp7_139*RLcp7_340-OMcp7_339*RLcp7_140);
  ORcp7_340 = OMcp7_139*RLcp7_240-OMcp7_239*RLcp7_140;
  PxF4[1] = q[1]+RLcp7_138+RLcp7_139+RLcp7_140;
  PxF4[2] = q[2]+RLcp7_238+RLcp7_239+RLcp7_240;
  PxF4[3] = q[3]+RLcp7_338+RLcp7_339+RLcp7_340;
  RxF4[1][1] = ROcp7_140;
  RxF4[1][2] = ROcp7_240;
  RxF4[1][3] = ROcp7_340;
  RxF4[2][1] = ROcp7_439;
  RxF4[2][2] = ROcp7_539;
  RxF4[2][3] = ROcp7_639;
  RxF4[3][1] = ROcp7_740;
  RxF4[3][2] = ROcp7_840;
  RxF4[3][3] = ROcp7_940;
  VxF4[1] = qd[1]+ORcp7_138+ORcp7_139+ORcp7_140;
  VxF4[2] = qd[2]+ORcp7_238+ORcp7_239+ORcp7_240;
  VxF4[3] = qd[3]+ORcp7_338+ORcp7_339+ORcp7_340;
  OMxF4[1] = OMcp7_139+qd[40]*ROcp7_439;
  OMxF4[2] = OMcp7_239+qd[40]*ROcp7_539;
  OMxF4[3] = OMcp7_339+qd[40]*ROcp7_639;
  AxF4[1] = qdd[1]+OMcp7_238*ORcp7_339+OMcp7_239*ORcp7_340+OMcp7_26*ORcp7_338-OMcp7_338*ORcp7_239-OMcp7_339*ORcp7_240-
 OMcp7_36*ORcp7_238+OPcp7_238*RLcp7_339+OPcp7_239*RLcp7_340+OPcp7_26*RLcp7_338-OPcp7_338*RLcp7_239-OPcp7_339*RLcp7_240-
 OPcp7_36*RLcp7_238;
  AxF4[2] = qdd[2]-OMcp7_138*ORcp7_339-OMcp7_139*ORcp7_340-OMcp7_16*ORcp7_338+OMcp7_338*ORcp7_139+OMcp7_339*ORcp7_140+
 OMcp7_36*ORcp7_138-OPcp7_138*RLcp7_339-OPcp7_139*RLcp7_340-OPcp7_16*RLcp7_338+OPcp7_338*RLcp7_139+OPcp7_339*RLcp7_140+
 OPcp7_36*RLcp7_138;
  AxF4[3] = qdd[3]+OMcp7_138*ORcp7_239+OMcp7_139*ORcp7_240+OMcp7_16*ORcp7_238-OMcp7_238*ORcp7_139-OMcp7_239*ORcp7_140-
 OMcp7_26*ORcp7_138+OPcp7_138*RLcp7_239+OPcp7_139*RLcp7_240+OPcp7_16*RLcp7_238-OPcp7_238*RLcp7_139-OPcp7_239*RLcp7_140-
 OPcp7_26*RLcp7_138;
  OMPxF4[1] = OPcp7_139+qd[40]*(OMcp7_239*ROcp7_639-OMcp7_339*ROcp7_539)+qdd[40]*ROcp7_439;
  OMPxF4[2] = OPcp7_239-qd[40]*(OMcp7_139*ROcp7_639-OMcp7_339*ROcp7_439)+qdd[40]*ROcp7_539;
  OMPxF4[3] = OPcp7_339+qd[40]*(OMcp7_139*ROcp7_539-OMcp7_239*ROcp7_439)+qdd[40]*ROcp7_639;
 
// Sensor Forces Computation 

  SWr4 = user_ExtForces(PxF4,RxF4,VxF4,OMxF4,AxF4,OMPxF4,s,tsim,4);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc18 = ROcp7_140*SWr4[1]+ROcp7_240*SWr4[2]+ROcp7_340*SWr4[3];
  xfrc28 = ROcp7_439*SWr4[1]+ROcp7_539*SWr4[2]+ROcp7_639*SWr4[3];
  xfrc38 = ROcp7_740*SWr4[1]+ROcp7_840*SWr4[2]+ROcp7_940*SWr4[3];
  frc[1][40] = s->frc[1][40]+xfrc18;
  frc[2][40] = s->frc[2][40]+xfrc28;
  frc[3][40] = s->frc[3][40]+xfrc38;
  xtrq18 = ROcp7_140*SWr4[4]+ROcp7_240*SWr4[5]+ROcp7_340*SWr4[6];
  xtrq28 = ROcp7_439*SWr4[4]+ROcp7_539*SWr4[5]+ROcp7_639*SWr4[6];
  xtrq38 = ROcp7_740*SWr4[4]+ROcp7_840*SWr4[5]+ROcp7_940*SWr4[6];
  trq[1][40] = s->trq[1][40]+xtrq18-xfrc28*SWr4[9]+xfrc38*SWr4[8];
  trq[2][40] = s->trq[2][40]+xtrq28+xfrc18*SWr4[9]-xfrc38*SWr4[7];
  trq[3][40] = s->trq[3][40]+xtrq38-xfrc18*SWr4[8]+xfrc28*SWr4[7];

// = = Block_0_0_1_5_0_1 = = 
 
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

// = = Block_0_0_1_5_0_22 = = 
 
// Sensor Kinematics 


  ROcp8_151 = ROcp8_16*C51+ROcp8_45*S51;
  ROcp8_251 = ROcp8_26*C51+ROcp8_55*S51;
  ROcp8_351 = ROcp8_36*C51+S51*S5;
  ROcp8_451 = -(ROcp8_16*S51-ROcp8_45*C51);
  ROcp8_551 = -(ROcp8_26*S51-ROcp8_55*C51);
  ROcp8_651 = -(ROcp8_36*S51-C51*S5);
  ROcp8_452 = ROcp8_451*C52+ROcp8_76*S52;
  ROcp8_552 = ROcp8_551*C52+ROcp8_86*S52;
  ROcp8_652 = ROcp8_651*C52+ROcp8_96*S52;
  ROcp8_752 = -(ROcp8_451*S52-ROcp8_76*C52);
  ROcp8_852 = -(ROcp8_551*S52-ROcp8_86*C52);
  ROcp8_952 = -(ROcp8_651*S52-ROcp8_96*C52);
  ROcp8_153 = ROcp8_151*C53-ROcp8_752*S53;
  ROcp8_253 = ROcp8_251*C53-ROcp8_852*S53;
  ROcp8_353 = ROcp8_351*C53-ROcp8_952*S53;
  ROcp8_753 = ROcp8_151*S53+ROcp8_752*C53;
  ROcp8_853 = ROcp8_251*S53+ROcp8_852*C53;
  ROcp8_953 = ROcp8_351*S53+ROcp8_952*C53;
  RLcp8_151 = ROcp8_16*s->dpt[1][18]+ROcp8_76*s->dpt[3][18];
  RLcp8_251 = ROcp8_26*s->dpt[1][18]+ROcp8_86*s->dpt[3][18];
  RLcp8_351 = ROcp8_36*s->dpt[1][18]+ROcp8_96*s->dpt[3][18];
  OMcp8_151 = OMcp8_16+qd[51]*ROcp8_76;
  OMcp8_251 = OMcp8_26+qd[51]*ROcp8_86;
  OMcp8_351 = OMcp8_36+qd[51]*ROcp8_96;
  ORcp8_151 = OMcp8_26*RLcp8_351-OMcp8_36*RLcp8_251;
  ORcp8_251 = -(OMcp8_16*RLcp8_351-OMcp8_36*RLcp8_151);
  ORcp8_351 = OMcp8_16*RLcp8_251-OMcp8_26*RLcp8_151;
  OMcp8_152 = OMcp8_151+qd[52]*ROcp8_151;
  OMcp8_252 = OMcp8_251+qd[52]*ROcp8_251;
  OMcp8_352 = OMcp8_351+qd[52]*ROcp8_351;
  OMcp8_153 = OMcp8_152+qd[53]*ROcp8_452;
  OMcp8_253 = OMcp8_252+qd[53]*ROcp8_552;
  OMcp8_353 = OMcp8_352+qd[53]*ROcp8_652;
  OPcp8_153 = OPcp8_16+qd[51]*(OMcp8_26*ROcp8_96-OMcp8_36*ROcp8_86)+qd[52]*(OMcp8_251*ROcp8_351-OMcp8_351*ROcp8_251)+
 qd[53]*(OMcp8_252*ROcp8_652-OMcp8_352*ROcp8_552)+qdd[51]*ROcp8_76+qdd[52]*ROcp8_151+qdd[53]*ROcp8_452;
  OPcp8_253 = OPcp8_26-qd[51]*(OMcp8_16*ROcp8_96-OMcp8_36*ROcp8_76)-qd[52]*(OMcp8_151*ROcp8_351-OMcp8_351*ROcp8_151)-
 qd[53]*(OMcp8_152*ROcp8_652-OMcp8_352*ROcp8_452)+qdd[51]*ROcp8_86+qdd[52]*ROcp8_251+qdd[53]*ROcp8_552;
  OPcp8_353 = OPcp8_36+qd[51]*(OMcp8_16*ROcp8_86-OMcp8_26*ROcp8_76)+qd[52]*(OMcp8_151*ROcp8_251-OMcp8_251*ROcp8_151)+
 qd[53]*(OMcp8_152*ROcp8_552-OMcp8_252*ROcp8_452)+qdd[51]*ROcp8_96+qdd[52]*ROcp8_351+qdd[53]*ROcp8_652;
  RLcp8_154 = q[54]*ROcp8_153;
  RLcp8_254 = q[54]*ROcp8_253;
  RLcp8_354 = q[54]*ROcp8_353;
  ORcp8_154 = OMcp8_253*RLcp8_354-OMcp8_353*RLcp8_254;
  ORcp8_254 = -(OMcp8_153*RLcp8_354-OMcp8_353*RLcp8_154);
  ORcp8_354 = OMcp8_153*RLcp8_254-OMcp8_253*RLcp8_154;

// = = Block_0_0_1_5_0_23 = = 
 
// Sensor Kinematics 


  ROcp8_155 = ROcp8_153*C55-ROcp8_753*S55;
  ROcp8_255 = ROcp8_253*C55-ROcp8_853*S55;
  ROcp8_355 = ROcp8_353*C55-ROcp8_953*S55;
  ROcp8_755 = ROcp8_153*S55+ROcp8_753*C55;
  ROcp8_855 = ROcp8_253*S55+ROcp8_853*C55;
  ROcp8_955 = ROcp8_353*S55+ROcp8_953*C55;
  RLcp8_155 = ROcp8_153*s->dpt[1][65]+ROcp8_452*s->dpt[2][65]+ROcp8_753*s->dpt[3][65];
  RLcp8_255 = ROcp8_253*s->dpt[1][65]+ROcp8_552*s->dpt[2][65]+ROcp8_853*s->dpt[3][65];
  RLcp8_355 = ROcp8_353*s->dpt[1][65]+ROcp8_652*s->dpt[2][65]+ROcp8_953*s->dpt[3][65];
  ORcp8_155 = OMcp8_253*RLcp8_355-OMcp8_353*RLcp8_255;
  ORcp8_255 = -(OMcp8_153*RLcp8_355-OMcp8_353*RLcp8_155);
  ORcp8_355 = OMcp8_153*RLcp8_255-OMcp8_253*RLcp8_155;
  PxF5[1] = q[1]+RLcp8_151+RLcp8_154+RLcp8_155;
  PxF5[2] = q[2]+RLcp8_251+RLcp8_254+RLcp8_255;
  PxF5[3] = q[3]+RLcp8_351+RLcp8_354+RLcp8_355;
  RxF5[1][1] = ROcp8_155;
  RxF5[1][2] = ROcp8_255;
  RxF5[1][3] = ROcp8_355;
  RxF5[2][1] = ROcp8_452;
  RxF5[2][2] = ROcp8_552;
  RxF5[2][3] = ROcp8_652;
  RxF5[3][1] = ROcp8_755;
  RxF5[3][2] = ROcp8_855;
  RxF5[3][3] = ROcp8_955;
  VxF5[1] = qd[1]+ORcp8_151+ORcp8_154+ORcp8_155+qd[54]*ROcp8_153;
  VxF5[2] = qd[2]+ORcp8_251+ORcp8_254+ORcp8_255+qd[54]*ROcp8_253;
  VxF5[3] = qd[3]+ORcp8_351+ORcp8_354+ORcp8_355+qd[54]*ROcp8_353;
  OMxF5[1] = OMcp8_153+qd[55]*ROcp8_452;
  OMxF5[2] = OMcp8_253+qd[55]*ROcp8_552;
  OMxF5[3] = OMcp8_353+qd[55]*ROcp8_652;
  AxF5[1] = qdd[1]+(2.0)*qd[54]*(OMcp8_253*ROcp8_353-OMcp8_353*ROcp8_253)+qdd[54]*ROcp8_153+OMcp8_253*ORcp8_354+OMcp8_253*
 ORcp8_355+OMcp8_26*ORcp8_351-OMcp8_353*ORcp8_254-OMcp8_353*ORcp8_255-OMcp8_36*ORcp8_251+OPcp8_253*RLcp8_354+OPcp8_253*
 RLcp8_355+OPcp8_26*RLcp8_351-OPcp8_353*RLcp8_254-OPcp8_353*RLcp8_255-OPcp8_36*RLcp8_251;
  AxF5[2] = qdd[2]-(2.0)*qd[54]*(OMcp8_153*ROcp8_353-OMcp8_353*ROcp8_153)+qdd[54]*ROcp8_253-OMcp8_153*ORcp8_354-OMcp8_153*
 ORcp8_355-OMcp8_16*ORcp8_351+OMcp8_353*ORcp8_154+OMcp8_353*ORcp8_155+OMcp8_36*ORcp8_151-OPcp8_153*RLcp8_354-OPcp8_153*
 RLcp8_355-OPcp8_16*RLcp8_351+OPcp8_353*RLcp8_154+OPcp8_353*RLcp8_155+OPcp8_36*RLcp8_151;
  AxF5[3] = qdd[3]+(2.0)*qd[54]*(OMcp8_153*ROcp8_253-OMcp8_253*ROcp8_153)+qdd[54]*ROcp8_353+OMcp8_153*ORcp8_254+OMcp8_153*
 ORcp8_255+OMcp8_16*ORcp8_251-OMcp8_253*ORcp8_154-OMcp8_253*ORcp8_155-OMcp8_26*ORcp8_151+OPcp8_153*RLcp8_254+OPcp8_153*
 RLcp8_255+OPcp8_16*RLcp8_251-OPcp8_253*RLcp8_154-OPcp8_253*RLcp8_155-OPcp8_26*RLcp8_151;
  OMPxF5[1] = OPcp8_153+qd[55]*(OMcp8_253*ROcp8_652-OMcp8_353*ROcp8_552)+qdd[55]*ROcp8_452;
  OMPxF5[2] = OPcp8_253-qd[55]*(OMcp8_153*ROcp8_652-OMcp8_353*ROcp8_452)+qdd[55]*ROcp8_552;
  OMPxF5[3] = OPcp8_353+qd[55]*(OMcp8_153*ROcp8_552-OMcp8_253*ROcp8_452)+qdd[55]*ROcp8_652;
 
// Sensor Forces Computation 

  SWr5 = user_ExtForces(PxF5,RxF5,VxF5,OMxF5,AxF5,OMPxF5,s,tsim,5);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc19 = ROcp8_155*SWr5[1]+ROcp8_255*SWr5[2]+ROcp8_355*SWr5[3];
  xfrc29 = ROcp8_452*SWr5[1]+ROcp8_552*SWr5[2]+ROcp8_652*SWr5[3];
  xfrc39 = ROcp8_755*SWr5[1]+ROcp8_855*SWr5[2]+ROcp8_955*SWr5[3];
  frc[1][55] = s->frc[1][55]+xfrc19;
  frc[2][55] = s->frc[2][55]+xfrc29;
  frc[3][55] = s->frc[3][55]+xfrc39;
  xtrq19 = ROcp8_155*SWr5[4]+ROcp8_255*SWr5[5]+ROcp8_355*SWr5[6];
  xtrq29 = ROcp8_452*SWr5[4]+ROcp8_552*SWr5[5]+ROcp8_652*SWr5[6];
  xtrq39 = ROcp8_755*SWr5[4]+ROcp8_855*SWr5[5]+ROcp8_955*SWr5[6];
  trq[1][55] = s->trq[1][55]+xtrq19-xfrc29*SWr5[9]+xfrc39*SWr5[8];
  trq[2][55] = s->trq[2][55]+xtrq29+xfrc19*SWr5[9]-xfrc39*SWr5[7];
  trq[3][55] = s->trq[3][55]+xtrq39-xfrc19*SWr5[8]+xfrc29*SWr5[7];

// = = Block_0_0_1_6_0_1 = = 
 
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

// = = Block_0_0_1_6_0_22 = = 
 
// Sensor Kinematics 


  ROcp9_151 = ROcp9_16*C51+ROcp9_45*S51;
  ROcp9_251 = ROcp9_26*C51+ROcp9_55*S51;
  ROcp9_351 = ROcp9_36*C51+S51*S5;
  ROcp9_451 = -(ROcp9_16*S51-ROcp9_45*C51);
  ROcp9_551 = -(ROcp9_26*S51-ROcp9_55*C51);
  ROcp9_651 = -(ROcp9_36*S51-C51*S5);
  ROcp9_452 = ROcp9_451*C52+ROcp9_76*S52;
  ROcp9_552 = ROcp9_551*C52+ROcp9_86*S52;
  ROcp9_652 = ROcp9_651*C52+ROcp9_96*S52;
  ROcp9_752 = -(ROcp9_451*S52-ROcp9_76*C52);
  ROcp9_852 = -(ROcp9_551*S52-ROcp9_86*C52);
  ROcp9_952 = -(ROcp9_651*S52-ROcp9_96*C52);
  ROcp9_153 = ROcp9_151*C53-ROcp9_752*S53;
  ROcp9_253 = ROcp9_251*C53-ROcp9_852*S53;
  ROcp9_353 = ROcp9_351*C53-ROcp9_952*S53;
  ROcp9_753 = ROcp9_151*S53+ROcp9_752*C53;
  ROcp9_853 = ROcp9_251*S53+ROcp9_852*C53;
  ROcp9_953 = ROcp9_351*S53+ROcp9_952*C53;
  RLcp9_151 = ROcp9_16*s->dpt[1][18]+ROcp9_76*s->dpt[3][18];
  RLcp9_251 = ROcp9_26*s->dpt[1][18]+ROcp9_86*s->dpt[3][18];
  RLcp9_351 = ROcp9_36*s->dpt[1][18]+ROcp9_96*s->dpt[3][18];
  OMcp9_151 = OMcp9_16+qd[51]*ROcp9_76;
  OMcp9_251 = OMcp9_26+qd[51]*ROcp9_86;
  OMcp9_351 = OMcp9_36+qd[51]*ROcp9_96;
  ORcp9_151 = OMcp9_26*RLcp9_351-OMcp9_36*RLcp9_251;
  ORcp9_251 = -(OMcp9_16*RLcp9_351-OMcp9_36*RLcp9_151);
  ORcp9_351 = OMcp9_16*RLcp9_251-OMcp9_26*RLcp9_151;
  OMcp9_152 = OMcp9_151+qd[52]*ROcp9_151;
  OMcp9_252 = OMcp9_251+qd[52]*ROcp9_251;
  OMcp9_352 = OMcp9_351+qd[52]*ROcp9_351;
  OMcp9_153 = OMcp9_152+qd[53]*ROcp9_452;
  OMcp9_253 = OMcp9_252+qd[53]*ROcp9_552;
  OMcp9_353 = OMcp9_352+qd[53]*ROcp9_652;
  OPcp9_153 = OPcp9_16+qd[51]*(OMcp9_26*ROcp9_96-OMcp9_36*ROcp9_86)+qd[52]*(OMcp9_251*ROcp9_351-OMcp9_351*ROcp9_251)+
 qd[53]*(OMcp9_252*ROcp9_652-OMcp9_352*ROcp9_552)+qdd[51]*ROcp9_76+qdd[52]*ROcp9_151+qdd[53]*ROcp9_452;
  OPcp9_253 = OPcp9_26-qd[51]*(OMcp9_16*ROcp9_96-OMcp9_36*ROcp9_76)-qd[52]*(OMcp9_151*ROcp9_351-OMcp9_351*ROcp9_151)-
 qd[53]*(OMcp9_152*ROcp9_652-OMcp9_352*ROcp9_452)+qdd[51]*ROcp9_86+qdd[52]*ROcp9_251+qdd[53]*ROcp9_552;
  OPcp9_353 = OPcp9_36+qd[51]*(OMcp9_16*ROcp9_86-OMcp9_26*ROcp9_76)+qd[52]*(OMcp9_151*ROcp9_251-OMcp9_251*ROcp9_151)+
 qd[53]*(OMcp9_152*ROcp9_552-OMcp9_252*ROcp9_452)+qdd[51]*ROcp9_96+qdd[52]*ROcp9_351+qdd[53]*ROcp9_652;
  RLcp9_154 = q[54]*ROcp9_153;
  RLcp9_254 = q[54]*ROcp9_253;
  RLcp9_354 = q[54]*ROcp9_353;
  ORcp9_154 = OMcp9_253*RLcp9_354-OMcp9_353*RLcp9_254;
  ORcp9_254 = -(OMcp9_153*RLcp9_354-OMcp9_353*RLcp9_154);
  ORcp9_354 = OMcp9_153*RLcp9_254-OMcp9_253*RLcp9_154;

// = = Block_0_0_1_6_0_24 = = 
 
// Sensor Kinematics 


  ROcp9_156 = ROcp9_153*C56-ROcp9_753*S56;
  ROcp9_256 = ROcp9_253*C56-ROcp9_853*S56;
  ROcp9_356 = ROcp9_353*C56-ROcp9_953*S56;
  ROcp9_756 = ROcp9_153*S56+ROcp9_753*C56;
  ROcp9_856 = ROcp9_253*S56+ROcp9_853*C56;
  ROcp9_956 = ROcp9_353*S56+ROcp9_953*C56;
  RLcp9_156 = ROcp9_153*s->dpt[1][66]+ROcp9_452*s->dpt[2][66]+ROcp9_753*s->dpt[3][66];
  RLcp9_256 = ROcp9_253*s->dpt[1][66]+ROcp9_552*s->dpt[2][66]+ROcp9_853*s->dpt[3][66];
  RLcp9_356 = ROcp9_353*s->dpt[1][66]+ROcp9_652*s->dpt[2][66]+ROcp9_953*s->dpt[3][66];
  ORcp9_156 = OMcp9_253*RLcp9_356-OMcp9_353*RLcp9_256;
  ORcp9_256 = -(OMcp9_153*RLcp9_356-OMcp9_353*RLcp9_156);
  ORcp9_356 = OMcp9_153*RLcp9_256-OMcp9_253*RLcp9_156;
  PxF6[1] = q[1]+RLcp9_151+RLcp9_154+RLcp9_156;
  PxF6[2] = q[2]+RLcp9_251+RLcp9_254+RLcp9_256;
  PxF6[3] = q[3]+RLcp9_351+RLcp9_354+RLcp9_356;
  RxF6[1][1] = ROcp9_156;
  RxF6[1][2] = ROcp9_256;
  RxF6[1][3] = ROcp9_356;
  RxF6[2][1] = ROcp9_452;
  RxF6[2][2] = ROcp9_552;
  RxF6[2][3] = ROcp9_652;
  RxF6[3][1] = ROcp9_756;
  RxF6[3][2] = ROcp9_856;
  RxF6[3][3] = ROcp9_956;
  VxF6[1] = qd[1]+ORcp9_151+ORcp9_154+ORcp9_156+qd[54]*ROcp9_153;
  VxF6[2] = qd[2]+ORcp9_251+ORcp9_254+ORcp9_256+qd[54]*ROcp9_253;
  VxF6[3] = qd[3]+ORcp9_351+ORcp9_354+ORcp9_356+qd[54]*ROcp9_353;
  OMxF6[1] = OMcp9_153+qd[56]*ROcp9_452;
  OMxF6[2] = OMcp9_253+qd[56]*ROcp9_552;
  OMxF6[3] = OMcp9_353+qd[56]*ROcp9_652;
  AxF6[1] = qdd[1]+(2.0)*qd[54]*(OMcp9_253*ROcp9_353-OMcp9_353*ROcp9_253)+qdd[54]*ROcp9_153+OMcp9_253*ORcp9_354+OMcp9_253*
 ORcp9_356+OMcp9_26*ORcp9_351-OMcp9_353*ORcp9_254-OMcp9_353*ORcp9_256-OMcp9_36*ORcp9_251+OPcp9_253*RLcp9_354+OPcp9_253*
 RLcp9_356+OPcp9_26*RLcp9_351-OPcp9_353*RLcp9_254-OPcp9_353*RLcp9_256-OPcp9_36*RLcp9_251;
  AxF6[2] = qdd[2]-(2.0)*qd[54]*(OMcp9_153*ROcp9_353-OMcp9_353*ROcp9_153)+qdd[54]*ROcp9_253-OMcp9_153*ORcp9_354-OMcp9_153*
 ORcp9_356-OMcp9_16*ORcp9_351+OMcp9_353*ORcp9_154+OMcp9_353*ORcp9_156+OMcp9_36*ORcp9_151-OPcp9_153*RLcp9_354-OPcp9_153*
 RLcp9_356-OPcp9_16*RLcp9_351+OPcp9_353*RLcp9_154+OPcp9_353*RLcp9_156+OPcp9_36*RLcp9_151;
  AxF6[3] = qdd[3]+(2.0)*qd[54]*(OMcp9_153*ROcp9_253-OMcp9_253*ROcp9_153)+qdd[54]*ROcp9_353+OMcp9_153*ORcp9_254+OMcp9_153*
 ORcp9_256+OMcp9_16*ORcp9_251-OMcp9_253*ORcp9_154-OMcp9_253*ORcp9_156-OMcp9_26*ORcp9_151+OPcp9_153*RLcp9_254+OPcp9_153*
 RLcp9_256+OPcp9_16*RLcp9_251-OPcp9_253*RLcp9_154-OPcp9_253*RLcp9_156-OPcp9_26*RLcp9_151;
  OMPxF6[1] = OPcp9_153+qd[56]*(OMcp9_253*ROcp9_652-OMcp9_353*ROcp9_552)+qdd[56]*ROcp9_452;
  OMPxF6[2] = OPcp9_253-qd[56]*(OMcp9_153*ROcp9_652-OMcp9_353*ROcp9_452)+qdd[56]*ROcp9_552;
  OMPxF6[3] = OPcp9_353+qd[56]*(OMcp9_153*ROcp9_552-OMcp9_253*ROcp9_452)+qdd[56]*ROcp9_652;
 
// Sensor Forces Computation 

  SWr6 = user_ExtForces(PxF6,RxF6,VxF6,OMxF6,AxF6,OMPxF6,s,tsim,6);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc110 = ROcp9_156*SWr6[1]+ROcp9_256*SWr6[2]+ROcp9_356*SWr6[3];
  xfrc210 = ROcp9_452*SWr6[1]+ROcp9_552*SWr6[2]+ROcp9_652*SWr6[3];
  xfrc310 = ROcp9_756*SWr6[1]+ROcp9_856*SWr6[2]+ROcp9_956*SWr6[3];
  frc[1][56] = s->frc[1][56]+xfrc110;
  frc[2][56] = s->frc[2][56]+xfrc210;
  frc[3][56] = s->frc[3][56]+xfrc310;
  xtrq110 = ROcp9_156*SWr6[4]+ROcp9_256*SWr6[5]+ROcp9_356*SWr6[6];
  xtrq210 = ROcp9_452*SWr6[4]+ROcp9_552*SWr6[5]+ROcp9_652*SWr6[6];
  xtrq310 = ROcp9_756*SWr6[4]+ROcp9_856*SWr6[5]+ROcp9_956*SWr6[6];
  trq[1][56] = s->trq[1][56]+xtrq110-xfrc210*SWr6[9]+xfrc310*SWr6[8];
  trq[2][56] = s->trq[2][56]+xtrq210+xfrc110*SWr6[9]-xfrc310*SWr6[7];
  trq[3][56] = s->trq[3][56]+xtrq310-xfrc110*SWr6[8]+xfrc210*SWr6[7];

// = = Block_0_0_1_7_0_1 = = 
 
// Sensor Kinematics 


  ROcp10_45 = -S4*C5;
  ROcp10_55 = C4*C5;
  ROcp10_75 = S4*S5;
  ROcp10_85 = -C4*S5;
  ROcp10_16 = -(ROcp10_75*S6-C4*C6);
  ROcp10_26 = -(ROcp10_85*S6-S4*C6);
  ROcp10_36 = -C5*S6;
  ROcp10_76 = ROcp10_75*C6+C4*S6;
  ROcp10_86 = ROcp10_85*C6+S4*S6;
  ROcp10_96 = C5*C6;
  OMcp10_15 = qd[5]*C4;
  OMcp10_25 = qd[5]*S4;
  OMcp10_16 = OMcp10_15+qd[6]*ROcp10_45;
  OMcp10_26 = OMcp10_25+qd[6]*ROcp10_55;
  OMcp10_36 = qd[4]+qd[6]*S5;
  OPcp10_16 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp10_55-OMcp10_25*S5)-qdd[5]*C4-qdd[6]*ROcp10_45);
  OPcp10_26 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp10_45-OMcp10_15*S5)+qdd[5]*S4+qdd[6]*ROcp10_55;
  OPcp10_36 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;

// = = Block_0_0_1_7_0_22 = = 
 
// Sensor Kinematics 


  ROcp10_151 = ROcp10_16*C51+ROcp10_45*S51;
  ROcp10_251 = ROcp10_26*C51+ROcp10_55*S51;
  ROcp10_351 = ROcp10_36*C51+S51*S5;
  ROcp10_451 = -(ROcp10_16*S51-ROcp10_45*C51);
  ROcp10_551 = -(ROcp10_26*S51-ROcp10_55*C51);
  ROcp10_651 = -(ROcp10_36*S51-C51*S5);
  ROcp10_452 = ROcp10_451*C52+ROcp10_76*S52;
  ROcp10_552 = ROcp10_551*C52+ROcp10_86*S52;
  ROcp10_652 = ROcp10_651*C52+ROcp10_96*S52;
  ROcp10_752 = -(ROcp10_451*S52-ROcp10_76*C52);
  ROcp10_852 = -(ROcp10_551*S52-ROcp10_86*C52);
  ROcp10_952 = -(ROcp10_651*S52-ROcp10_96*C52);
  ROcp10_153 = ROcp10_151*C53-ROcp10_752*S53;
  ROcp10_253 = ROcp10_251*C53-ROcp10_852*S53;
  ROcp10_353 = ROcp10_351*C53-ROcp10_952*S53;
  ROcp10_753 = ROcp10_151*S53+ROcp10_752*C53;
  ROcp10_853 = ROcp10_251*S53+ROcp10_852*C53;
  ROcp10_953 = ROcp10_351*S53+ROcp10_952*C53;
  RLcp10_151 = ROcp10_16*s->dpt[1][18]+ROcp10_76*s->dpt[3][18];
  RLcp10_251 = ROcp10_26*s->dpt[1][18]+ROcp10_86*s->dpt[3][18];
  RLcp10_351 = ROcp10_36*s->dpt[1][18]+ROcp10_96*s->dpt[3][18];
  OMcp10_151 = OMcp10_16+qd[51]*ROcp10_76;
  OMcp10_251 = OMcp10_26+qd[51]*ROcp10_86;
  OMcp10_351 = OMcp10_36+qd[51]*ROcp10_96;
  ORcp10_151 = OMcp10_26*RLcp10_351-OMcp10_36*RLcp10_251;
  ORcp10_251 = -(OMcp10_16*RLcp10_351-OMcp10_36*RLcp10_151);
  ORcp10_351 = OMcp10_16*RLcp10_251-OMcp10_26*RLcp10_151;
  OMcp10_152 = OMcp10_151+qd[52]*ROcp10_151;
  OMcp10_252 = OMcp10_251+qd[52]*ROcp10_251;
  OMcp10_352 = OMcp10_351+qd[52]*ROcp10_351;
  OMcp10_153 = OMcp10_152+qd[53]*ROcp10_452;
  OMcp10_253 = OMcp10_252+qd[53]*ROcp10_552;
  OMcp10_353 = OMcp10_352+qd[53]*ROcp10_652;
  OPcp10_153 = OPcp10_16+qd[51]*(OMcp10_26*ROcp10_96-OMcp10_36*ROcp10_86)+qd[52]*(OMcp10_251*ROcp10_351-OMcp10_351*
 ROcp10_251)+qd[53]*(OMcp10_252*ROcp10_652-OMcp10_352*ROcp10_552)+qdd[51]*ROcp10_76+qdd[52]*ROcp10_151+qdd[53]*ROcp10_452;
  OPcp10_253 = OPcp10_26-qd[51]*(OMcp10_16*ROcp10_96-OMcp10_36*ROcp10_76)-qd[52]*(OMcp10_151*ROcp10_351-OMcp10_351*
 ROcp10_151)-qd[53]*(OMcp10_152*ROcp10_652-OMcp10_352*ROcp10_452)+qdd[51]*ROcp10_86+qdd[52]*ROcp10_251+qdd[53]*ROcp10_552;
  OPcp10_353 = OPcp10_36+qd[51]*(OMcp10_16*ROcp10_86-OMcp10_26*ROcp10_76)+qd[52]*(OMcp10_151*ROcp10_251-OMcp10_251*
 ROcp10_151)+qd[53]*(OMcp10_152*ROcp10_552-OMcp10_252*ROcp10_452)+qdd[51]*ROcp10_96+qdd[52]*ROcp10_351+qdd[53]*ROcp10_652;
  RLcp10_154 = q[54]*ROcp10_153;
  RLcp10_254 = q[54]*ROcp10_253;
  RLcp10_354 = q[54]*ROcp10_353;
  ORcp10_154 = OMcp10_253*RLcp10_354-OMcp10_353*RLcp10_254;
  ORcp10_254 = -(OMcp10_153*RLcp10_354-OMcp10_353*RLcp10_154);
  ORcp10_354 = OMcp10_153*RLcp10_254-OMcp10_253*RLcp10_154;

// = = Block_0_0_1_7_0_25 = = 
 
// Sensor Kinematics 


  ROcp10_157 = ROcp10_153*C57-ROcp10_753*S57;
  ROcp10_257 = ROcp10_253*C57-ROcp10_853*S57;
  ROcp10_357 = ROcp10_353*C57-ROcp10_953*S57;
  ROcp10_757 = ROcp10_153*S57+ROcp10_753*C57;
  ROcp10_857 = ROcp10_253*S57+ROcp10_853*C57;
  ROcp10_957 = ROcp10_353*S57+ROcp10_953*C57;
  RLcp10_157 = ROcp10_153*s->dpt[1][67]+ROcp10_452*s->dpt[2][67]+ROcp10_753*s->dpt[3][67];
  RLcp10_257 = ROcp10_253*s->dpt[1][67]+ROcp10_552*s->dpt[2][67]+ROcp10_853*s->dpt[3][67];
  RLcp10_357 = ROcp10_353*s->dpt[1][67]+ROcp10_652*s->dpt[2][67]+ROcp10_953*s->dpt[3][67];
  ORcp10_157 = OMcp10_253*RLcp10_357-OMcp10_353*RLcp10_257;
  ORcp10_257 = -(OMcp10_153*RLcp10_357-OMcp10_353*RLcp10_157);
  ORcp10_357 = OMcp10_153*RLcp10_257-OMcp10_253*RLcp10_157;
  PxF7[1] = q[1]+RLcp10_151+RLcp10_154+RLcp10_157;
  PxF7[2] = q[2]+RLcp10_251+RLcp10_254+RLcp10_257;
  PxF7[3] = q[3]+RLcp10_351+RLcp10_354+RLcp10_357;
  RxF7[1][1] = ROcp10_157;
  RxF7[1][2] = ROcp10_257;
  RxF7[1][3] = ROcp10_357;
  RxF7[2][1] = ROcp10_452;
  RxF7[2][2] = ROcp10_552;
  RxF7[2][3] = ROcp10_652;
  RxF7[3][1] = ROcp10_757;
  RxF7[3][2] = ROcp10_857;
  RxF7[3][3] = ROcp10_957;
  VxF7[1] = qd[1]+ORcp10_151+ORcp10_154+ORcp10_157+qd[54]*ROcp10_153;
  VxF7[2] = qd[2]+ORcp10_251+ORcp10_254+ORcp10_257+qd[54]*ROcp10_253;
  VxF7[3] = qd[3]+ORcp10_351+ORcp10_354+ORcp10_357+qd[54]*ROcp10_353;
  OMxF7[1] = OMcp10_153+qd[57]*ROcp10_452;
  OMxF7[2] = OMcp10_253+qd[57]*ROcp10_552;
  OMxF7[3] = OMcp10_353+qd[57]*ROcp10_652;
  AxF7[1] = qdd[1]+(2.0)*qd[54]*(OMcp10_253*ROcp10_353-OMcp10_353*ROcp10_253)+qdd[54]*ROcp10_153+OMcp10_253*ORcp10_354+
 OMcp10_253*ORcp10_357+OMcp10_26*ORcp10_351-OMcp10_353*ORcp10_254-OMcp10_353*ORcp10_257-OMcp10_36*ORcp10_251+OPcp10_253*
 RLcp10_354+OPcp10_253*RLcp10_357+OPcp10_26*RLcp10_351-OPcp10_353*RLcp10_254-OPcp10_353*RLcp10_257-OPcp10_36*RLcp10_251;
  AxF7[2] = qdd[2]-(2.0)*qd[54]*(OMcp10_153*ROcp10_353-OMcp10_353*ROcp10_153)+qdd[54]*ROcp10_253-OMcp10_153*ORcp10_354-
 OMcp10_153*ORcp10_357-OMcp10_16*ORcp10_351+OMcp10_353*ORcp10_154+OMcp10_353*ORcp10_157+OMcp10_36*ORcp10_151-OPcp10_153*
 RLcp10_354-OPcp10_153*RLcp10_357-OPcp10_16*RLcp10_351+OPcp10_353*RLcp10_154+OPcp10_353*RLcp10_157+OPcp10_36*RLcp10_151;
  AxF7[3] = qdd[3]+(2.0)*qd[54]*(OMcp10_153*ROcp10_253-OMcp10_253*ROcp10_153)+qdd[54]*ROcp10_353+OMcp10_153*ORcp10_254+
 OMcp10_153*ORcp10_257+OMcp10_16*ORcp10_251-OMcp10_253*ORcp10_154-OMcp10_253*ORcp10_157-OMcp10_26*ORcp10_151+OPcp10_153*
 RLcp10_254+OPcp10_153*RLcp10_257+OPcp10_16*RLcp10_251-OPcp10_253*RLcp10_154-OPcp10_253*RLcp10_157-OPcp10_26*RLcp10_151;
  OMPxF7[1] = OPcp10_153+qd[57]*(OMcp10_253*ROcp10_652-OMcp10_353*ROcp10_552)+qdd[57]*ROcp10_452;
  OMPxF7[2] = OPcp10_253-qd[57]*(OMcp10_153*ROcp10_652-OMcp10_353*ROcp10_452)+qdd[57]*ROcp10_552;
  OMPxF7[3] = OPcp10_353+qd[57]*(OMcp10_153*ROcp10_552-OMcp10_253*ROcp10_452)+qdd[57]*ROcp10_652;
 
// Sensor Forces Computation 

  SWr7 = user_ExtForces(PxF7,RxF7,VxF7,OMxF7,AxF7,OMPxF7,s,tsim,7);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc111 = ROcp10_157*SWr7[1]+ROcp10_257*SWr7[2]+ROcp10_357*SWr7[3];
  xfrc211 = ROcp10_452*SWr7[1]+ROcp10_552*SWr7[2]+ROcp10_652*SWr7[3];
  xfrc311 = ROcp10_757*SWr7[1]+ROcp10_857*SWr7[2]+ROcp10_957*SWr7[3];
  frc[1][57] = s->frc[1][57]+xfrc111;
  frc[2][57] = s->frc[2][57]+xfrc211;
  frc[3][57] = s->frc[3][57]+xfrc311;
  xtrq111 = ROcp10_157*SWr7[4]+ROcp10_257*SWr7[5]+ROcp10_357*SWr7[6];
  xtrq211 = ROcp10_452*SWr7[4]+ROcp10_552*SWr7[5]+ROcp10_652*SWr7[6];
  xtrq311 = ROcp10_757*SWr7[4]+ROcp10_857*SWr7[5]+ROcp10_957*SWr7[6];
  trq[1][57] = s->trq[1][57]+xtrq111-xfrc211*SWr7[9]+xfrc311*SWr7[8];
  trq[2][57] = s->trq[2][57]+xtrq211+xfrc111*SWr7[9]-xfrc311*SWr7[7];
  trq[3][57] = s->trq[3][57]+xtrq311-xfrc111*SWr7[8]+xfrc211*SWr7[7];

// = = Block_0_0_1_8_0_1 = = 
 
// Sensor Kinematics 


  ROcp11_45 = -S4*C5;
  ROcp11_55 = C4*C5;
  ROcp11_75 = S4*S5;
  ROcp11_85 = -C4*S5;
  ROcp11_16 = -(ROcp11_75*S6-C4*C6);
  ROcp11_26 = -(ROcp11_85*S6-S4*C6);
  ROcp11_36 = -C5*S6;
  ROcp11_76 = ROcp11_75*C6+C4*S6;
  ROcp11_86 = ROcp11_85*C6+S4*S6;
  ROcp11_96 = C5*C6;
  OMcp11_15 = qd[5]*C4;
  OMcp11_25 = qd[5]*S4;
  OMcp11_16 = OMcp11_15+qd[6]*ROcp11_45;
  OMcp11_26 = OMcp11_25+qd[6]*ROcp11_55;
  OMcp11_36 = qd[4]+qd[6]*S5;
  OPcp11_16 = -(qd[4]*qd[5]*S4+qd[6]*(qd[4]*ROcp11_55-OMcp11_25*S5)-qdd[5]*C4-qdd[6]*ROcp11_45);
  OPcp11_26 = qd[4]*qd[5]*C4+qd[6]*(qd[4]*ROcp11_45-OMcp11_15*S5)+qdd[5]*S4+qdd[6]*ROcp11_55;
  OPcp11_36 = qdd[4]+qd[5]*qd[6]*C5+qdd[6]*S5;

// = = Block_0_0_1_8_0_22 = = 
 
// Sensor Kinematics 


  ROcp11_151 = ROcp11_16*C51+ROcp11_45*S51;
  ROcp11_251 = ROcp11_26*C51+ROcp11_55*S51;
  ROcp11_351 = ROcp11_36*C51+S51*S5;
  ROcp11_451 = -(ROcp11_16*S51-ROcp11_45*C51);
  ROcp11_551 = -(ROcp11_26*S51-ROcp11_55*C51);
  ROcp11_651 = -(ROcp11_36*S51-C51*S5);
  ROcp11_452 = ROcp11_451*C52+ROcp11_76*S52;
  ROcp11_552 = ROcp11_551*C52+ROcp11_86*S52;
  ROcp11_652 = ROcp11_651*C52+ROcp11_96*S52;
  ROcp11_752 = -(ROcp11_451*S52-ROcp11_76*C52);
  ROcp11_852 = -(ROcp11_551*S52-ROcp11_86*C52);
  ROcp11_952 = -(ROcp11_651*S52-ROcp11_96*C52);
  ROcp11_153 = ROcp11_151*C53-ROcp11_752*S53;
  ROcp11_253 = ROcp11_251*C53-ROcp11_852*S53;
  ROcp11_353 = ROcp11_351*C53-ROcp11_952*S53;
  ROcp11_753 = ROcp11_151*S53+ROcp11_752*C53;
  ROcp11_853 = ROcp11_251*S53+ROcp11_852*C53;
  ROcp11_953 = ROcp11_351*S53+ROcp11_952*C53;
  RLcp11_151 = ROcp11_16*s->dpt[1][18]+ROcp11_76*s->dpt[3][18];
  RLcp11_251 = ROcp11_26*s->dpt[1][18]+ROcp11_86*s->dpt[3][18];
  RLcp11_351 = ROcp11_36*s->dpt[1][18]+ROcp11_96*s->dpt[3][18];
  OMcp11_151 = OMcp11_16+qd[51]*ROcp11_76;
  OMcp11_251 = OMcp11_26+qd[51]*ROcp11_86;
  OMcp11_351 = OMcp11_36+qd[51]*ROcp11_96;
  ORcp11_151 = OMcp11_26*RLcp11_351-OMcp11_36*RLcp11_251;
  ORcp11_251 = -(OMcp11_16*RLcp11_351-OMcp11_36*RLcp11_151);
  ORcp11_351 = OMcp11_16*RLcp11_251-OMcp11_26*RLcp11_151;
  OMcp11_152 = OMcp11_151+qd[52]*ROcp11_151;
  OMcp11_252 = OMcp11_251+qd[52]*ROcp11_251;
  OMcp11_352 = OMcp11_351+qd[52]*ROcp11_351;
  OMcp11_153 = OMcp11_152+qd[53]*ROcp11_452;
  OMcp11_253 = OMcp11_252+qd[53]*ROcp11_552;
  OMcp11_353 = OMcp11_352+qd[53]*ROcp11_652;
  OPcp11_153 = OPcp11_16+qd[51]*(OMcp11_26*ROcp11_96-OMcp11_36*ROcp11_86)+qd[52]*(OMcp11_251*ROcp11_351-OMcp11_351*
 ROcp11_251)+qd[53]*(OMcp11_252*ROcp11_652-OMcp11_352*ROcp11_552)+qdd[51]*ROcp11_76+qdd[52]*ROcp11_151+qdd[53]*ROcp11_452;
  OPcp11_253 = OPcp11_26-qd[51]*(OMcp11_16*ROcp11_96-OMcp11_36*ROcp11_76)-qd[52]*(OMcp11_151*ROcp11_351-OMcp11_351*
 ROcp11_151)-qd[53]*(OMcp11_152*ROcp11_652-OMcp11_352*ROcp11_452)+qdd[51]*ROcp11_86+qdd[52]*ROcp11_251+qdd[53]*ROcp11_552;
  OPcp11_353 = OPcp11_36+qd[51]*(OMcp11_16*ROcp11_86-OMcp11_26*ROcp11_76)+qd[52]*(OMcp11_151*ROcp11_251-OMcp11_251*
 ROcp11_151)+qd[53]*(OMcp11_152*ROcp11_552-OMcp11_252*ROcp11_452)+qdd[51]*ROcp11_96+qdd[52]*ROcp11_351+qdd[53]*ROcp11_652;
  RLcp11_154 = q[54]*ROcp11_153;
  RLcp11_254 = q[54]*ROcp11_253;
  RLcp11_354 = q[54]*ROcp11_353;
  ORcp11_154 = OMcp11_253*RLcp11_354-OMcp11_353*RLcp11_254;
  ORcp11_254 = -(OMcp11_153*RLcp11_354-OMcp11_353*RLcp11_154);
  ORcp11_354 = OMcp11_153*RLcp11_254-OMcp11_253*RLcp11_154;

// = = Block_0_0_1_8_0_26 = = 
 
// Sensor Kinematics 


  ROcp11_158 = ROcp11_153*C58-ROcp11_753*S58;
  ROcp11_258 = ROcp11_253*C58-ROcp11_853*S58;
  ROcp11_358 = ROcp11_353*C58-ROcp11_953*S58;
  ROcp11_758 = ROcp11_153*S58+ROcp11_753*C58;
  ROcp11_858 = ROcp11_253*S58+ROcp11_853*C58;
  ROcp11_958 = ROcp11_353*S58+ROcp11_953*C58;
  RLcp11_158 = ROcp11_153*s->dpt[1][68]+ROcp11_452*s->dpt[2][68]+ROcp11_753*s->dpt[3][68];
  RLcp11_258 = ROcp11_253*s->dpt[1][68]+ROcp11_552*s->dpt[2][68]+ROcp11_853*s->dpt[3][68];
  RLcp11_358 = ROcp11_353*s->dpt[1][68]+ROcp11_652*s->dpt[2][68]+ROcp11_953*s->dpt[3][68];
  ORcp11_158 = OMcp11_253*RLcp11_358-OMcp11_353*RLcp11_258;
  ORcp11_258 = -(OMcp11_153*RLcp11_358-OMcp11_353*RLcp11_158);
  ORcp11_358 = OMcp11_153*RLcp11_258-OMcp11_253*RLcp11_158;
  PxF8[1] = q[1]+RLcp11_151+RLcp11_154+RLcp11_158;
  PxF8[2] = q[2]+RLcp11_251+RLcp11_254+RLcp11_258;
  PxF8[3] = q[3]+RLcp11_351+RLcp11_354+RLcp11_358;
  RxF8[1][1] = ROcp11_158;
  RxF8[1][2] = ROcp11_258;
  RxF8[1][3] = ROcp11_358;
  RxF8[2][1] = ROcp11_452;
  RxF8[2][2] = ROcp11_552;
  RxF8[2][3] = ROcp11_652;
  RxF8[3][1] = ROcp11_758;
  RxF8[3][2] = ROcp11_858;
  RxF8[3][3] = ROcp11_958;
  VxF8[1] = qd[1]+ORcp11_151+ORcp11_154+ORcp11_158+qd[54]*ROcp11_153;
  VxF8[2] = qd[2]+ORcp11_251+ORcp11_254+ORcp11_258+qd[54]*ROcp11_253;
  VxF8[3] = qd[3]+ORcp11_351+ORcp11_354+ORcp11_358+qd[54]*ROcp11_353;
  OMxF8[1] = OMcp11_153+qd[58]*ROcp11_452;
  OMxF8[2] = OMcp11_253+qd[58]*ROcp11_552;
  OMxF8[3] = OMcp11_353+qd[58]*ROcp11_652;
  AxF8[1] = qdd[1]+(2.0)*qd[54]*(OMcp11_253*ROcp11_353-OMcp11_353*ROcp11_253)+qdd[54]*ROcp11_153+OMcp11_253*ORcp11_354+
 OMcp11_253*ORcp11_358+OMcp11_26*ORcp11_351-OMcp11_353*ORcp11_254-OMcp11_353*ORcp11_258-OMcp11_36*ORcp11_251+OPcp11_253*
 RLcp11_354+OPcp11_253*RLcp11_358+OPcp11_26*RLcp11_351-OPcp11_353*RLcp11_254-OPcp11_353*RLcp11_258-OPcp11_36*RLcp11_251;
  AxF8[2] = qdd[2]-(2.0)*qd[54]*(OMcp11_153*ROcp11_353-OMcp11_353*ROcp11_153)+qdd[54]*ROcp11_253-OMcp11_153*ORcp11_354-
 OMcp11_153*ORcp11_358-OMcp11_16*ORcp11_351+OMcp11_353*ORcp11_154+OMcp11_353*ORcp11_158+OMcp11_36*ORcp11_151-OPcp11_153*
 RLcp11_354-OPcp11_153*RLcp11_358-OPcp11_16*RLcp11_351+OPcp11_353*RLcp11_154+OPcp11_353*RLcp11_158+OPcp11_36*RLcp11_151;
  AxF8[3] = qdd[3]+(2.0)*qd[54]*(OMcp11_153*ROcp11_253-OMcp11_253*ROcp11_153)+qdd[54]*ROcp11_353+OMcp11_153*ORcp11_254+
 OMcp11_153*ORcp11_258+OMcp11_16*ORcp11_251-OMcp11_253*ORcp11_154-OMcp11_253*ORcp11_158-OMcp11_26*ORcp11_151+OPcp11_153*
 RLcp11_254+OPcp11_153*RLcp11_258+OPcp11_16*RLcp11_251-OPcp11_253*RLcp11_154-OPcp11_253*RLcp11_158-OPcp11_26*RLcp11_151;
  OMPxF8[1] = OPcp11_153+qd[58]*(OMcp11_253*ROcp11_652-OMcp11_353*ROcp11_552)+qdd[58]*ROcp11_452;
  OMPxF8[2] = OPcp11_253-qd[58]*(OMcp11_153*ROcp11_652-OMcp11_353*ROcp11_452)+qdd[58]*ROcp11_552;
  OMPxF8[3] = OPcp11_353+qd[58]*(OMcp11_153*ROcp11_552-OMcp11_253*ROcp11_452)+qdd[58]*ROcp11_652;
 
// Sensor Forces Computation 

  SWr8 = user_ExtForces(PxF8,RxF8,VxF8,OMxF8,AxF8,OMPxF8,s,tsim,8);
 
// Sensor Dynamics : Forces projection on body-fixed frames 

  xfrc112 = ROcp11_158*SWr8[1]+ROcp11_258*SWr8[2]+ROcp11_358*SWr8[3];
  xfrc212 = ROcp11_452*SWr8[1]+ROcp11_552*SWr8[2]+ROcp11_652*SWr8[3];
  xfrc312 = ROcp11_758*SWr8[1]+ROcp11_858*SWr8[2]+ROcp11_958*SWr8[3];
  frc[1][58] = s->frc[1][58]+xfrc112;
  frc[2][58] = s->frc[2][58]+xfrc212;
  frc[3][58] = s->frc[3][58]+xfrc312;
  xtrq112 = ROcp11_158*SWr8[4]+ROcp11_258*SWr8[5]+ROcp11_358*SWr8[6];
  xtrq212 = ROcp11_452*SWr8[4]+ROcp11_552*SWr8[5]+ROcp11_652*SWr8[6];
  xtrq312 = ROcp11_758*SWr8[4]+ROcp11_858*SWr8[5]+ROcp11_958*SWr8[6];
  trq[1][58] = s->trq[1][58]+xtrq112-xfrc212*SWr8[9]+xfrc312*SWr8[8];
  trq[2][58] = s->trq[2][58]+xtrq212+xfrc112*SWr8[9]-xfrc312*SWr8[7];
  trq[3][58] = s->trq[3][58]+xtrq312-xfrc112*SWr8[8]+xfrc212*SWr8[7];

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
  frc[1][34] = s->frc[1][34];
  frc[2][34] = s->frc[2][34];
  frc[3][34] = s->frc[3][34];
  frc[1][35] = s->frc[1][35];
  frc[2][35] = s->frc[2][35];
  frc[3][35] = s->frc[3][35];
  frc[1][36] = s->frc[1][36];
  frc[2][36] = s->frc[2][36];
  frc[3][36] = s->frc[3][36];
  frc[1][38] = s->frc[1][38];
  frc[2][38] = s->frc[2][38];
  frc[3][38] = s->frc[3][38];
  frc[1][39] = s->frc[1][39];
  frc[2][39] = s->frc[2][39];
  frc[3][39] = s->frc[3][39];
  frc[1][41] = s->frc[1][41];
  frc[2][41] = s->frc[2][41];
  frc[3][41] = s->frc[3][41];
  frc[1][42] = s->frc[1][42];
  frc[2][42] = s->frc[2][42];
  frc[3][42] = s->frc[3][42];
  frc[1][44] = s->frc[1][44];
  frc[2][44] = s->frc[2][44];
  frc[3][44] = s->frc[3][44];
  frc[1][46] = s->frc[1][46];
  frc[2][46] = s->frc[2][46];
  frc[3][46] = s->frc[3][46];
  frc[1][50] = s->frc[1][50];
  frc[2][50] = s->frc[2][50];
  frc[3][50] = s->frc[3][50];
  frc[1][54] = s->frc[1][54];
  frc[2][54] = s->frc[2][54];
  frc[3][54] = s->frc[3][54];
  frc[1][59] = s->frc[1][59];
  frc[2][59] = s->frc[2][59];
  frc[3][59] = s->frc[3][59];
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
  trq[1][34] = s->trq[1][34];
  trq[2][34] = s->trq[2][34];
  trq[3][34] = s->trq[3][34];
  trq[1][35] = s->trq[1][35];
  trq[2][35] = s->trq[2][35];
  trq[3][35] = s->trq[3][35];
  trq[1][36] = s->trq[1][36];
  trq[2][36] = s->trq[2][36];
  trq[3][36] = s->trq[3][36];
  trq[1][38] = s->trq[1][38];
  trq[2][38] = s->trq[2][38];
  trq[3][38] = s->trq[3][38];
  trq[1][39] = s->trq[1][39];
  trq[2][39] = s->trq[2][39];
  trq[3][39] = s->trq[3][39];
  trq[1][41] = s->trq[1][41];
  trq[2][41] = s->trq[2][41];
  trq[3][41] = s->trq[3][41];
  trq[1][42] = s->trq[1][42];
  trq[2][42] = s->trq[2][42];
  trq[3][42] = s->trq[3][42];
  trq[1][44] = s->trq[1][44];
  trq[2][44] = s->trq[2][44];
  trq[3][44] = s->trq[3][44];
  trq[1][46] = s->trq[1][46];
  trq[2][46] = s->trq[2][46];
  trq[3][46] = s->trq[3][46];
  trq[1][50] = s->trq[1][50];
  trq[2][50] = s->trq[2][50];
  trq[3][50] = s->trq[3][50];
  trq[1][54] = s->trq[1][54];
  trq[2][54] = s->trq[2][54];
  trq[3][54] = s->trq[3][54];
  trq[1][59] = s->trq[1][59];
  trq[2][59] = s->trq[2][59];
  trq[3][59] = s->trq[3][59];

// ====== END Task 0 ====== 


}
 

