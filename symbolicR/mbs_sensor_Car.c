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
//	==> Function : F 6 : Sensors Kinematical Informations (sens) 
//	==> Flops complexity : 4761
//
//	==> Generation Time :  0.090 seconds
//	==> Post-Processing :  0.080 seconds
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
    RLcp0_188 = ROcp0_16*s->dpt[1][16]+ROcp0_76*s->dpt[3][16];
    RLcp0_288 = ROcp0_26*s->dpt[1][16]+ROcp0_86*s->dpt[3][16];
    RLcp0_388 = ROcp0_36*s->dpt[1][16]+ROcp0_96*s->dpt[3][16];
    POcp0_188 = RLcp0_188+q[1];
    POcp0_288 = RLcp0_288+q[2];
    POcp0_388 = RLcp0_388+q[3];
    JTcp0_188_5 = RLcp0_388*S4;
    JTcp0_288_5 = -RLcp0_388*C4;
    JTcp0_388_5 = -(RLcp0_188*S4-RLcp0_288*C4);
    JTcp0_188_6 = -(RLcp0_288*S5-RLcp0_388*ROcp0_55);
    JTcp0_288_6 = RLcp0_188*S5-RLcp0_388*ROcp0_45;
    JTcp0_388_6 = -(RLcp0_188*ROcp0_55-RLcp0_288*ROcp0_45);
    ORcp0_188 = OMcp0_26*RLcp0_388-OMcp0_36*RLcp0_288;
    ORcp0_288 = -(OMcp0_16*RLcp0_388-OMcp0_36*RLcp0_188);
    ORcp0_388 = OMcp0_16*RLcp0_288-OMcp0_26*RLcp0_188;
    VIcp0_188 = ORcp0_188+qd[1];
    VIcp0_288 = ORcp0_288+qd[2];
    VIcp0_388 = ORcp0_388+qd[3];
    ACcp0_188 = qdd[1]+OMcp0_26*ORcp0_388-OMcp0_36*ORcp0_288+OPcp0_26*RLcp0_388-OPcp0_36*RLcp0_288;
    ACcp0_288 = qdd[2]-OMcp0_16*ORcp0_388+OMcp0_36*ORcp0_188-OPcp0_16*RLcp0_388+OPcp0_36*RLcp0_188;
    ACcp0_388 = qdd[3]+OMcp0_16*ORcp0_288-OMcp0_26*ORcp0_188+OPcp0_16*RLcp0_288-OPcp0_26*RLcp0_188;

// = = Block_1_0_0_1_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp0_188;
    sens->P[2] = POcp0_288;
    sens->P[3] = POcp0_388;
    sens->R[1][1] = ROcp0_16;
    sens->R[1][2] = ROcp0_26;
    sens->R[1][3] = ROcp0_36;
    sens->R[2][1] = ROcp0_45;
    sens->R[2][2] = ROcp0_55;
    sens->R[2][3] = S5;
    sens->R[3][1] = ROcp0_76;
    sens->R[3][2] = ROcp0_86;
    sens->R[3][3] = ROcp0_96;
    sens->V[1] = VIcp0_188;
    sens->V[2] = VIcp0_288;
    sens->V[3] = VIcp0_388;
    sens->OM[1] = OMcp0_16;
    sens->OM[2] = OMcp0_26;
    sens->OM[3] = OMcp0_36;
    sens->J[1][1] = (1.0);
    sens->J[1][4] = -RLcp0_288;
    sens->J[1][5] = JTcp0_188_5;
    sens->J[1][6] = JTcp0_188_6;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = RLcp0_188;
    sens->J[2][5] = JTcp0_288_5;
    sens->J[2][6] = JTcp0_288_6;
    sens->J[3][3] = (1.0);
    sens->J[3][5] = JTcp0_388_5;
    sens->J[3][6] = JTcp0_388_6;
    sens->J[4][5] = C4;
    sens->J[4][6] = ROcp0_45;
    sens->J[5][5] = S4;
    sens->J[5][6] = ROcp0_55;
    sens->J[6][4] = (1.0);
    sens->J[6][6] = S5;
    sens->A[1] = ACcp0_188;
    sens->A[2] = ACcp0_288;
    sens->A[3] = ACcp0_388;
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

// = = Block_1_0_0_2_0_3 = = 
 
// Sensor Kinematics 


    ROcp1_48 = ROcp1_45*C8+ROcp1_76*S8;
    ROcp1_58 = ROcp1_55*C8+ROcp1_86*S8;
    ROcp1_68 = ROcp1_96*S8+S5*C8;
    ROcp1_78 = -(ROcp1_45*S8-ROcp1_76*C8);
    ROcp1_88 = -(ROcp1_55*S8-ROcp1_86*C8);
    ROcp1_98 = ROcp1_96*C8-S5*S8;
    ROcp1_19 = ROcp1_16*C9-ROcp1_78*S9;
    ROcp1_29 = ROcp1_26*C9-ROcp1_88*S9;
    ROcp1_39 = ROcp1_36*C9-ROcp1_98*S9;
    ROcp1_79 = ROcp1_16*S9+ROcp1_78*C9;
    ROcp1_89 = ROcp1_26*S9+ROcp1_88*C9;
    ROcp1_99 = ROcp1_36*S9+ROcp1_98*C9;
    ROcp1_410 = ROcp1_48*C10+ROcp1_79*S10;
    ROcp1_510 = ROcp1_58*C10+ROcp1_89*S10;
    ROcp1_610 = ROcp1_68*C10+ROcp1_99*S10;
    ROcp1_710 = -(ROcp1_48*S10-ROcp1_79*C10);
    ROcp1_810 = -(ROcp1_58*S10-ROcp1_89*C10);
    ROcp1_910 = -(ROcp1_68*S10-ROcp1_99*C10);
    ROcp1_111 = ROcp1_19*C11+ROcp1_410*S11;
    ROcp1_211 = ROcp1_29*C11+ROcp1_510*S11;
    ROcp1_311 = ROcp1_39*C11+ROcp1_610*S11;
    ROcp1_411 = -(ROcp1_19*S11-ROcp1_410*C11);
    ROcp1_511 = -(ROcp1_29*S11-ROcp1_510*C11);
    ROcp1_611 = -(ROcp1_39*S11-ROcp1_610*C11);
    RLcp1_18 = ROcp1_16*s->dpt[1][1]+ROcp1_45*s->dpt[2][1]+ROcp1_76*s->dpt[3][1];
    RLcp1_28 = ROcp1_26*s->dpt[1][1]+ROcp1_55*s->dpt[2][1]+ROcp1_86*s->dpt[3][1];
    RLcp1_38 = ROcp1_36*s->dpt[1][1]+ROcp1_96*s->dpt[3][1]+s->dpt[2][1]*S5;
    OMcp1_18 = OMcp1_16+ROcp1_16*qd[8];
    OMcp1_28 = OMcp1_26+ROcp1_26*qd[8];
    OMcp1_38 = OMcp1_36+ROcp1_36*qd[8];
    ORcp1_18 = OMcp1_26*RLcp1_38-OMcp1_36*RLcp1_28;
    ORcp1_28 = -(OMcp1_16*RLcp1_38-OMcp1_36*RLcp1_18);
    ORcp1_38 = OMcp1_16*RLcp1_28-OMcp1_26*RLcp1_18;
    OPcp1_18 = OPcp1_16+ROcp1_16*qdd[8]+qd[8]*(OMcp1_26*ROcp1_36-OMcp1_36*ROcp1_26);
    OPcp1_28 = OPcp1_26+ROcp1_26*qdd[8]-qd[8]*(OMcp1_16*ROcp1_36-OMcp1_36*ROcp1_16);
    OPcp1_38 = OPcp1_36+ROcp1_36*qdd[8]+qd[8]*(OMcp1_16*ROcp1_26-OMcp1_26*ROcp1_16);
    RLcp1_19 = ROcp1_48*s->dpt[2][19];
    RLcp1_29 = ROcp1_58*s->dpt[2][19];
    RLcp1_39 = ROcp1_68*s->dpt[2][19];
    OMcp1_19 = OMcp1_18+ROcp1_48*qd[9];
    OMcp1_29 = OMcp1_28+ROcp1_58*qd[9];
    OMcp1_39 = OMcp1_38+ROcp1_68*qd[9];
    ORcp1_19 = OMcp1_28*RLcp1_39-OMcp1_38*RLcp1_29;
    ORcp1_29 = -(OMcp1_18*RLcp1_39-OMcp1_38*RLcp1_19);
    ORcp1_39 = OMcp1_18*RLcp1_29-OMcp1_28*RLcp1_19;
    OMcp1_110 = OMcp1_19+ROcp1_19*qd[10];
    OMcp1_210 = OMcp1_29+ROcp1_29*qd[10];
    OMcp1_310 = OMcp1_39+ROcp1_39*qd[10];
    OMcp1_111 = OMcp1_110+ROcp1_710*qd[11];
    OMcp1_211 = OMcp1_210+ROcp1_810*qd[11];
    OMcp1_311 = OMcp1_310+ROcp1_910*qd[11];
    OPcp1_111 = OPcp1_18+ROcp1_19*qdd[10]+ROcp1_48*qdd[9]+ROcp1_710*qdd[11]+qd[10]*(OMcp1_29*ROcp1_39-OMcp1_39*ROcp1_29)+
 qd[11]*(OMcp1_210*ROcp1_910-OMcp1_310*ROcp1_810)+qd[9]*(OMcp1_28*ROcp1_68-OMcp1_38*ROcp1_58);
    OPcp1_211 = OPcp1_28+ROcp1_29*qdd[10]+ROcp1_58*qdd[9]+ROcp1_810*qdd[11]-qd[10]*(OMcp1_19*ROcp1_39-OMcp1_39*ROcp1_19)-
 qd[11]*(OMcp1_110*ROcp1_910-OMcp1_310*ROcp1_710)-qd[9]*(OMcp1_18*ROcp1_68-OMcp1_38*ROcp1_48);
    OPcp1_311 = OPcp1_38+ROcp1_39*qdd[10]+ROcp1_68*qdd[9]+ROcp1_910*qdd[11]+qd[10]*(OMcp1_19*ROcp1_29-OMcp1_29*ROcp1_19)+
 qd[11]*(OMcp1_110*ROcp1_810-OMcp1_210*ROcp1_710)+qd[9]*(OMcp1_18*ROcp1_58-OMcp1_28*ROcp1_48);
    RLcp1_189 = ROcp1_111*s->dpt[1][22]+ROcp1_710*s->dpt[3][22];
    RLcp1_289 = ROcp1_211*s->dpt[1][22]+ROcp1_810*s->dpt[3][22];
    RLcp1_389 = ROcp1_311*s->dpt[1][22]+ROcp1_910*s->dpt[3][22];
    POcp1_189 = RLcp1_18+RLcp1_189+RLcp1_19+q[1];
    POcp1_289 = RLcp1_28+RLcp1_289+RLcp1_29+q[2];
    POcp1_389 = RLcp1_38+RLcp1_389+RLcp1_39+q[3];
    JTcp1_189_4 = -(RLcp1_28+RLcp1_289+RLcp1_29);
    JTcp1_289_4 = RLcp1_18+RLcp1_189+RLcp1_19;
    JTcp1_189_5 = S4*(RLcp1_38+RLcp1_389+RLcp1_39);
    JTcp1_289_5 = -C4*(RLcp1_38+RLcp1_389+RLcp1_39);
    JTcp1_389_5 = C4*(RLcp1_28+RLcp1_29)-S4*(RLcp1_18+RLcp1_19)-RLcp1_189*S4+RLcp1_289*C4;
    JTcp1_189_6 = ROcp1_55*(RLcp1_38+RLcp1_39)-S5*(RLcp1_28+RLcp1_29)-RLcp1_289*S5+RLcp1_389*ROcp1_55;
    JTcp1_289_6 = RLcp1_189*S5-RLcp1_389*ROcp1_45-ROcp1_45*(RLcp1_38+RLcp1_39)+S5*(RLcp1_18+RLcp1_19);
    JTcp1_389_6 = ROcp1_45*(RLcp1_28+RLcp1_29)-ROcp1_55*(RLcp1_18+RLcp1_19)-RLcp1_189*ROcp1_55+RLcp1_289*ROcp1_45;
    JTcp1_189_7 = ROcp1_26*(RLcp1_389+RLcp1_39)-ROcp1_36*(RLcp1_289+RLcp1_29);
    JTcp1_289_7 = -(ROcp1_16*(RLcp1_389+RLcp1_39)-ROcp1_36*(RLcp1_189+RLcp1_19));
    JTcp1_389_7 = ROcp1_16*(RLcp1_289+RLcp1_29)-ROcp1_26*(RLcp1_189+RLcp1_19);
    JTcp1_189_8 = -(RLcp1_289*ROcp1_68-RLcp1_389*ROcp1_58);
    JTcp1_289_8 = RLcp1_189*ROcp1_68-RLcp1_389*ROcp1_48;
    JTcp1_389_8 = -(RLcp1_189*ROcp1_58-RLcp1_289*ROcp1_48);
    JTcp1_189_9 = -(RLcp1_289*ROcp1_39-RLcp1_389*ROcp1_29);
    JTcp1_289_9 = RLcp1_189*ROcp1_39-RLcp1_389*ROcp1_19;
    JTcp1_389_9 = -(RLcp1_189*ROcp1_29-RLcp1_289*ROcp1_19);
    JTcp1_189_10 = -(RLcp1_289*ROcp1_910-RLcp1_389*ROcp1_810);
    JTcp1_289_10 = RLcp1_189*ROcp1_910-RLcp1_389*ROcp1_710;
    JTcp1_389_10 = -(RLcp1_189*ROcp1_810-RLcp1_289*ROcp1_710);
    ORcp1_189 = OMcp1_211*RLcp1_389-OMcp1_311*RLcp1_289;
    ORcp1_289 = -(OMcp1_111*RLcp1_389-OMcp1_311*RLcp1_189);
    ORcp1_389 = OMcp1_111*RLcp1_289-OMcp1_211*RLcp1_189;
    VIcp1_189 = ORcp1_18+ORcp1_189+ORcp1_19+qd[1];
    VIcp1_289 = ORcp1_28+ORcp1_289+ORcp1_29+qd[2];
    VIcp1_389 = ORcp1_38+ORcp1_389+ORcp1_39+qd[3];
    ACcp1_189 = qdd[1]+OMcp1_211*ORcp1_389+OMcp1_26*ORcp1_38+OMcp1_28*ORcp1_39-OMcp1_311*ORcp1_289-OMcp1_36*ORcp1_28-
 OMcp1_38*ORcp1_29+OPcp1_211*RLcp1_389+OPcp1_26*RLcp1_38+OPcp1_28*RLcp1_39-OPcp1_311*RLcp1_289-OPcp1_36*RLcp1_28-OPcp1_38*
 RLcp1_29;
    ACcp1_289 = qdd[2]-OMcp1_111*ORcp1_389-OMcp1_16*ORcp1_38-OMcp1_18*ORcp1_39+OMcp1_311*ORcp1_189+OMcp1_36*ORcp1_18+
 OMcp1_38*ORcp1_19-OPcp1_111*RLcp1_389-OPcp1_16*RLcp1_38-OPcp1_18*RLcp1_39+OPcp1_311*RLcp1_189+OPcp1_36*RLcp1_18+OPcp1_38*
 RLcp1_19;
    ACcp1_389 = qdd[3]+OMcp1_111*ORcp1_289+OMcp1_16*ORcp1_28+OMcp1_18*ORcp1_29-OMcp1_211*ORcp1_189-OMcp1_26*ORcp1_18-
 OMcp1_28*ORcp1_19+OPcp1_111*RLcp1_289+OPcp1_16*RLcp1_28+OPcp1_18*RLcp1_29-OPcp1_211*RLcp1_189-OPcp1_26*RLcp1_18-OPcp1_28*
 RLcp1_19;

// = = Block_1_0_0_2_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp1_189;
    sens->P[2] = POcp1_289;
    sens->P[3] = POcp1_389;
    sens->R[1][1] = ROcp1_111;
    sens->R[1][2] = ROcp1_211;
    sens->R[1][3] = ROcp1_311;
    sens->R[2][1] = ROcp1_411;
    sens->R[2][2] = ROcp1_511;
    sens->R[2][3] = ROcp1_611;
    sens->R[3][1] = ROcp1_710;
    sens->R[3][2] = ROcp1_810;
    sens->R[3][3] = ROcp1_910;
    sens->V[1] = VIcp1_189;
    sens->V[2] = VIcp1_289;
    sens->V[3] = VIcp1_389;
    sens->OM[1] = OMcp1_111;
    sens->OM[2] = OMcp1_211;
    sens->OM[3] = OMcp1_311;
    sens->J[1][1] = (1.0);
    sens->J[1][4] = JTcp1_189_4;
    sens->J[1][5] = JTcp1_189_5;
    sens->J[1][6] = JTcp1_189_6;
    sens->J[1][8] = JTcp1_189_7;
    sens->J[1][9] = JTcp1_189_8;
    sens->J[1][10] = JTcp1_189_9;
    sens->J[1][11] = JTcp1_189_10;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp1_289_4;
    sens->J[2][5] = JTcp1_289_5;
    sens->J[2][6] = JTcp1_289_6;
    sens->J[2][8] = JTcp1_289_7;
    sens->J[2][9] = JTcp1_289_8;
    sens->J[2][10] = JTcp1_289_9;
    sens->J[2][11] = JTcp1_289_10;
    sens->J[3][3] = (1.0);
    sens->J[3][5] = JTcp1_389_5;
    sens->J[3][6] = JTcp1_389_6;
    sens->J[3][8] = JTcp1_389_7;
    sens->J[3][9] = JTcp1_389_8;
    sens->J[3][10] = JTcp1_389_9;
    sens->J[3][11] = JTcp1_389_10;
    sens->J[4][5] = C4;
    sens->J[4][6] = ROcp1_45;
    sens->J[4][8] = ROcp1_16;
    sens->J[4][9] = ROcp1_48;
    sens->J[4][10] = ROcp1_19;
    sens->J[4][11] = ROcp1_710;
    sens->J[5][5] = S4;
    sens->J[5][6] = ROcp1_55;
    sens->J[5][8] = ROcp1_26;
    sens->J[5][9] = ROcp1_58;
    sens->J[5][10] = ROcp1_29;
    sens->J[5][11] = ROcp1_810;
    sens->J[6][4] = (1.0);
    sens->J[6][6] = S5;
    sens->J[6][8] = ROcp1_36;
    sens->J[6][9] = ROcp1_68;
    sens->J[6][10] = ROcp1_39;
    sens->J[6][11] = ROcp1_910;
    sens->A[1] = ACcp1_189;
    sens->A[2] = ACcp1_289;
    sens->A[3] = ACcp1_389;
    sens->OMP[1] = OPcp1_111;
    sens->OMP[2] = OPcp1_211;
    sens->OMP[3] = OPcp1_311;
 
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
    RLcp2_19 = ROcp2_48*s->dpt[2][19];
    RLcp2_29 = ROcp2_58*s->dpt[2][19];
    RLcp2_39 = ROcp2_68*s->dpt[2][19];
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
    RLcp2_190 = ROcp2_710*s->dpt[3][23];
    RLcp2_290 = ROcp2_810*s->dpt[3][23];
    RLcp2_390 = ROcp2_910*s->dpt[3][23];
    POcp2_190 = RLcp2_18+RLcp2_19+RLcp2_190+q[1];
    POcp2_290 = RLcp2_28+RLcp2_29+RLcp2_290+q[2];
    POcp2_390 = RLcp2_38+RLcp2_39+RLcp2_390+q[3];
    JTcp2_190_4 = -(RLcp2_28+RLcp2_29+RLcp2_290);
    JTcp2_290_4 = RLcp2_18+RLcp2_19+RLcp2_190;
    JTcp2_190_5 = S4*(RLcp2_38+RLcp2_39+RLcp2_390);
    JTcp2_290_5 = -C4*(RLcp2_38+RLcp2_39+RLcp2_390);
    JTcp2_390_5 = C4*(RLcp2_28+RLcp2_29)-S4*(RLcp2_18+RLcp2_19)-RLcp2_190*S4+RLcp2_290*C4;
    JTcp2_190_6 = ROcp2_55*(RLcp2_38+RLcp2_39)-S5*(RLcp2_28+RLcp2_29)-RLcp2_290*S5+RLcp2_390*ROcp2_55;
    JTcp2_290_6 = RLcp2_190*S5-RLcp2_390*ROcp2_45-ROcp2_45*(RLcp2_38+RLcp2_39)+S5*(RLcp2_18+RLcp2_19);
    JTcp2_390_6 = ROcp2_45*(RLcp2_28+RLcp2_29)-ROcp2_55*(RLcp2_18+RLcp2_19)-RLcp2_190*ROcp2_55+RLcp2_290*ROcp2_45;
    JTcp2_190_7 = ROcp2_26*(RLcp2_39+RLcp2_390)-ROcp2_36*(RLcp2_29+RLcp2_290);
    JTcp2_290_7 = -(ROcp2_16*(RLcp2_39+RLcp2_390)-ROcp2_36*(RLcp2_19+RLcp2_190));
    JTcp2_390_7 = ROcp2_16*(RLcp2_29+RLcp2_290)-ROcp2_26*(RLcp2_19+RLcp2_190);
    JTcp2_190_8 = -(RLcp2_290*ROcp2_68-RLcp2_390*ROcp2_58);
    JTcp2_290_8 = RLcp2_190*ROcp2_68-RLcp2_390*ROcp2_48;
    JTcp2_390_8 = -(RLcp2_190*ROcp2_58-RLcp2_290*ROcp2_48);
    JTcp2_190_9 = -(RLcp2_290*ROcp2_39-RLcp2_390*ROcp2_29);
    JTcp2_290_9 = RLcp2_190*ROcp2_39-RLcp2_390*ROcp2_19;
    JTcp2_390_9 = -(RLcp2_190*ROcp2_29-RLcp2_290*ROcp2_19);
    JTcp2_190_10 = -(RLcp2_290*ROcp2_910-RLcp2_390*ROcp2_810);
    JTcp2_290_10 = RLcp2_190*ROcp2_910-RLcp2_390*ROcp2_710;
    JTcp2_390_10 = -(RLcp2_190*ROcp2_810-RLcp2_290*ROcp2_710);
    ORcp2_190 = OMcp2_211*RLcp2_390-OMcp2_311*RLcp2_290;
    ORcp2_290 = -(OMcp2_111*RLcp2_390-OMcp2_311*RLcp2_190);
    ORcp2_390 = OMcp2_111*RLcp2_290-OMcp2_211*RLcp2_190;
    VIcp2_190 = ORcp2_18+ORcp2_19+ORcp2_190+qd[1];
    VIcp2_290 = ORcp2_28+ORcp2_29+ORcp2_290+qd[2];
    VIcp2_390 = ORcp2_38+ORcp2_39+ORcp2_390+qd[3];
    ACcp2_190 = qdd[1]+OMcp2_211*ORcp2_390+OMcp2_26*ORcp2_38+OMcp2_28*ORcp2_39-OMcp2_311*ORcp2_290-OMcp2_36*ORcp2_28-
 OMcp2_38*ORcp2_29+OPcp2_211*RLcp2_390+OPcp2_26*RLcp2_38+OPcp2_28*RLcp2_39-OPcp2_311*RLcp2_290-OPcp2_36*RLcp2_28-OPcp2_38*
 RLcp2_29;
    ACcp2_290 = qdd[2]-OMcp2_111*ORcp2_390-OMcp2_16*ORcp2_38-OMcp2_18*ORcp2_39+OMcp2_311*ORcp2_190+OMcp2_36*ORcp2_18+
 OMcp2_38*ORcp2_19-OPcp2_111*RLcp2_390-OPcp2_16*RLcp2_38-OPcp2_18*RLcp2_39+OPcp2_311*RLcp2_190+OPcp2_36*RLcp2_18+OPcp2_38*
 RLcp2_19;
    ACcp2_390 = qdd[3]+OMcp2_111*ORcp2_290+OMcp2_16*ORcp2_28+OMcp2_18*ORcp2_29-OMcp2_211*ORcp2_190-OMcp2_26*ORcp2_18-
 OMcp2_28*ORcp2_19+OPcp2_111*RLcp2_290+OPcp2_16*RLcp2_28+OPcp2_18*RLcp2_29-OPcp2_211*RLcp2_190-OPcp2_26*RLcp2_18-OPcp2_28*
 RLcp2_19;

// = = Block_1_0_0_3_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp2_190;
    sens->P[2] = POcp2_290;
    sens->P[3] = POcp2_390;
    sens->R[1][1] = ROcp2_111;
    sens->R[1][2] = ROcp2_211;
    sens->R[1][3] = ROcp2_311;
    sens->R[2][1] = ROcp2_411;
    sens->R[2][2] = ROcp2_511;
    sens->R[2][3] = ROcp2_611;
    sens->R[3][1] = ROcp2_710;
    sens->R[3][2] = ROcp2_810;
    sens->R[3][3] = ROcp2_910;
    sens->V[1] = VIcp2_190;
    sens->V[2] = VIcp2_290;
    sens->V[3] = VIcp2_390;
    sens->OM[1] = OMcp2_111;
    sens->OM[2] = OMcp2_211;
    sens->OM[3] = OMcp2_311;
    sens->J[1][1] = (1.0);
    sens->J[1][4] = JTcp2_190_4;
    sens->J[1][5] = JTcp2_190_5;
    sens->J[1][6] = JTcp2_190_6;
    sens->J[1][8] = JTcp2_190_7;
    sens->J[1][9] = JTcp2_190_8;
    sens->J[1][10] = JTcp2_190_9;
    sens->J[1][11] = JTcp2_190_10;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp2_290_4;
    sens->J[2][5] = JTcp2_290_5;
    sens->J[2][6] = JTcp2_290_6;
    sens->J[2][8] = JTcp2_290_7;
    sens->J[2][9] = JTcp2_290_8;
    sens->J[2][10] = JTcp2_290_9;
    sens->J[2][11] = JTcp2_290_10;
    sens->J[3][3] = (1.0);
    sens->J[3][5] = JTcp2_390_5;
    sens->J[3][6] = JTcp2_390_6;
    sens->J[3][8] = JTcp2_390_7;
    sens->J[3][9] = JTcp2_390_8;
    sens->J[3][10] = JTcp2_390_9;
    sens->J[3][11] = JTcp2_390_10;
    sens->J[4][5] = C4;
    sens->J[4][6] = ROcp2_45;
    sens->J[4][8] = ROcp2_16;
    sens->J[4][9] = ROcp2_48;
    sens->J[4][10] = ROcp2_19;
    sens->J[4][11] = ROcp2_710;
    sens->J[5][5] = S4;
    sens->J[5][6] = ROcp2_55;
    sens->J[5][8] = ROcp2_26;
    sens->J[5][9] = ROcp2_58;
    sens->J[5][10] = ROcp2_29;
    sens->J[5][11] = ROcp2_810;
    sens->J[6][4] = (1.0);
    sens->J[6][6] = S5;
    sens->J[6][8] = ROcp2_36;
    sens->J[6][9] = ROcp2_68;
    sens->J[6][10] = ROcp2_39;
    sens->J[6][11] = ROcp2_910;
    sens->A[1] = ACcp2_190;
    sens->A[2] = ACcp2_290;
    sens->A[3] = ACcp2_390;
    sens->OMP[1] = OPcp2_111;
    sens->OMP[2] = OPcp2_211;
    sens->OMP[3] = OPcp2_311;
 
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

// = = Block_1_0_0_4_0_3 = = 
 
// Sensor Kinematics 


    ROcp3_48 = ROcp3_45*C8+ROcp3_76*S8;
    ROcp3_58 = ROcp3_55*C8+ROcp3_86*S8;
    ROcp3_68 = ROcp3_96*S8+S5*C8;
    ROcp3_78 = -(ROcp3_45*S8-ROcp3_76*C8);
    ROcp3_88 = -(ROcp3_55*S8-ROcp3_86*C8);
    ROcp3_98 = ROcp3_96*C8-S5*S8;
    ROcp3_19 = ROcp3_16*C9-ROcp3_78*S9;
    ROcp3_29 = ROcp3_26*C9-ROcp3_88*S9;
    ROcp3_39 = ROcp3_36*C9-ROcp3_98*S9;
    ROcp3_79 = ROcp3_16*S9+ROcp3_78*C9;
    ROcp3_89 = ROcp3_26*S9+ROcp3_88*C9;
    ROcp3_99 = ROcp3_36*S9+ROcp3_98*C9;
    ROcp3_410 = ROcp3_48*C10+ROcp3_79*S10;
    ROcp3_510 = ROcp3_58*C10+ROcp3_89*S10;
    ROcp3_610 = ROcp3_68*C10+ROcp3_99*S10;
    ROcp3_710 = -(ROcp3_48*S10-ROcp3_79*C10);
    ROcp3_810 = -(ROcp3_58*S10-ROcp3_89*C10);
    ROcp3_910 = -(ROcp3_68*S10-ROcp3_99*C10);
    ROcp3_111 = ROcp3_19*C11+ROcp3_410*S11;
    ROcp3_211 = ROcp3_29*C11+ROcp3_510*S11;
    ROcp3_311 = ROcp3_39*C11+ROcp3_610*S11;
    ROcp3_411 = -(ROcp3_19*S11-ROcp3_410*C11);
    ROcp3_511 = -(ROcp3_29*S11-ROcp3_510*C11);
    ROcp3_611 = -(ROcp3_39*S11-ROcp3_610*C11);
    RLcp3_18 = ROcp3_16*s->dpt[1][1]+ROcp3_45*s->dpt[2][1]+ROcp3_76*s->dpt[3][1];
    RLcp3_28 = ROcp3_26*s->dpt[1][1]+ROcp3_55*s->dpt[2][1]+ROcp3_86*s->dpt[3][1];
    RLcp3_38 = ROcp3_36*s->dpt[1][1]+ROcp3_96*s->dpt[3][1]+s->dpt[2][1]*S5;
    OMcp3_18 = OMcp3_16+ROcp3_16*qd[8];
    OMcp3_28 = OMcp3_26+ROcp3_26*qd[8];
    OMcp3_38 = OMcp3_36+ROcp3_36*qd[8];
    ORcp3_18 = OMcp3_26*RLcp3_38-OMcp3_36*RLcp3_28;
    ORcp3_28 = -(OMcp3_16*RLcp3_38-OMcp3_36*RLcp3_18);
    ORcp3_38 = OMcp3_16*RLcp3_28-OMcp3_26*RLcp3_18;
    OPcp3_18 = OPcp3_16+ROcp3_16*qdd[8]+qd[8]*(OMcp3_26*ROcp3_36-OMcp3_36*ROcp3_26);
    OPcp3_28 = OPcp3_26+ROcp3_26*qdd[8]-qd[8]*(OMcp3_16*ROcp3_36-OMcp3_36*ROcp3_16);
    OPcp3_38 = OPcp3_36+ROcp3_36*qdd[8]+qd[8]*(OMcp3_16*ROcp3_26-OMcp3_26*ROcp3_16);
    RLcp3_19 = ROcp3_48*s->dpt[2][19];
    RLcp3_29 = ROcp3_58*s->dpt[2][19];
    RLcp3_39 = ROcp3_68*s->dpt[2][19];
    POcp3_19 = RLcp3_18+RLcp3_19+q[1];
    POcp3_29 = RLcp3_28+RLcp3_29+q[2];
    POcp3_39 = RLcp3_38+RLcp3_39+q[3];
    JTcp3_19_4 = -(RLcp3_28+RLcp3_29);
    JTcp3_29_4 = RLcp3_18+RLcp3_19;
    JTcp3_19_5 = S4*(RLcp3_38+RLcp3_39);
    JTcp3_29_5 = -C4*(RLcp3_38+RLcp3_39);
    JTcp3_39_5 = C4*(RLcp3_28+RLcp3_29)-S4*(RLcp3_18+RLcp3_19);
    JTcp3_19_6 = ROcp3_55*(RLcp3_38+RLcp3_39)-S5*(RLcp3_28+RLcp3_29);
    JTcp3_29_6 = -(ROcp3_45*(RLcp3_38+RLcp3_39)-S5*(RLcp3_18+RLcp3_19));
    JTcp3_39_6 = ROcp3_45*(RLcp3_28+RLcp3_29)-ROcp3_55*(RLcp3_18+RLcp3_19);
    JTcp3_19_7 = -(RLcp3_29*ROcp3_36-RLcp3_39*ROcp3_26);
    JTcp3_29_7 = RLcp3_19*ROcp3_36-RLcp3_39*ROcp3_16;
    JTcp3_39_7 = -(RLcp3_19*ROcp3_26-RLcp3_29*ROcp3_16);
    OMcp3_19 = OMcp3_18+ROcp3_48*qd[9];
    OMcp3_29 = OMcp3_28+ROcp3_58*qd[9];
    OMcp3_39 = OMcp3_38+ROcp3_68*qd[9];
    ORcp3_19 = OMcp3_28*RLcp3_39-OMcp3_38*RLcp3_29;
    ORcp3_29 = -(OMcp3_18*RLcp3_39-OMcp3_38*RLcp3_19);
    ORcp3_39 = OMcp3_18*RLcp3_29-OMcp3_28*RLcp3_19;
    VIcp3_19 = ORcp3_18+ORcp3_19+qd[1];
    VIcp3_29 = ORcp3_28+ORcp3_29+qd[2];
    VIcp3_39 = ORcp3_38+ORcp3_39+qd[3];
    ACcp3_19 = qdd[1]+OMcp3_26*ORcp3_38+OMcp3_28*ORcp3_39-OMcp3_36*ORcp3_28-OMcp3_38*ORcp3_29+OPcp3_26*RLcp3_38+OPcp3_28*
 RLcp3_39-OPcp3_36*RLcp3_28-OPcp3_38*RLcp3_29;
    ACcp3_29 = qdd[2]-OMcp3_16*ORcp3_38-OMcp3_18*ORcp3_39+OMcp3_36*ORcp3_18+OMcp3_38*ORcp3_19-OPcp3_16*RLcp3_38-OPcp3_18*
 RLcp3_39+OPcp3_36*RLcp3_18+OPcp3_38*RLcp3_19;
    ACcp3_39 = qdd[3]+OMcp3_16*ORcp3_28+OMcp3_18*ORcp3_29-OMcp3_26*ORcp3_18-OMcp3_28*ORcp3_19+OPcp3_16*RLcp3_28+OPcp3_18*
 RLcp3_29-OPcp3_26*RLcp3_18-OPcp3_28*RLcp3_19;
    OMcp3_110 = OMcp3_19+ROcp3_19*qd[10];
    OMcp3_210 = OMcp3_29+ROcp3_29*qd[10];
    OMcp3_310 = OMcp3_39+ROcp3_39*qd[10];
    OMcp3_111 = OMcp3_110+ROcp3_710*qd[11];
    OMcp3_211 = OMcp3_210+ROcp3_810*qd[11];
    OMcp3_311 = OMcp3_310+ROcp3_910*qd[11];
    OPcp3_111 = OPcp3_18+ROcp3_19*qdd[10]+ROcp3_48*qdd[9]+ROcp3_710*qdd[11]+qd[10]*(OMcp3_29*ROcp3_39-OMcp3_39*ROcp3_29)+
 qd[11]*(OMcp3_210*ROcp3_910-OMcp3_310*ROcp3_810)+qd[9]*(OMcp3_28*ROcp3_68-OMcp3_38*ROcp3_58);
    OPcp3_211 = OPcp3_28+ROcp3_29*qdd[10]+ROcp3_58*qdd[9]+ROcp3_810*qdd[11]-qd[10]*(OMcp3_19*ROcp3_39-OMcp3_39*ROcp3_19)-
 qd[11]*(OMcp3_110*ROcp3_910-OMcp3_310*ROcp3_710)-qd[9]*(OMcp3_18*ROcp3_68-OMcp3_38*ROcp3_48);
    OPcp3_311 = OPcp3_38+ROcp3_39*qdd[10]+ROcp3_68*qdd[9]+ROcp3_910*qdd[11]+qd[10]*(OMcp3_19*ROcp3_29-OMcp3_29*ROcp3_19)+
 qd[11]*(OMcp3_110*ROcp3_810-OMcp3_210*ROcp3_710)+qd[9]*(OMcp3_18*ROcp3_58-OMcp3_28*ROcp3_48);

// = = Block_1_0_0_4_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp3_19;
    sens->P[2] = POcp3_29;
    sens->P[3] = POcp3_39;
    sens->R[1][1] = ROcp3_111;
    sens->R[1][2] = ROcp3_211;
    sens->R[1][3] = ROcp3_311;
    sens->R[2][1] = ROcp3_411;
    sens->R[2][2] = ROcp3_511;
    sens->R[2][3] = ROcp3_611;
    sens->R[3][1] = ROcp3_710;
    sens->R[3][2] = ROcp3_810;
    sens->R[3][3] = ROcp3_910;
    sens->V[1] = VIcp3_19;
    sens->V[2] = VIcp3_29;
    sens->V[3] = VIcp3_39;
    sens->OM[1] = OMcp3_111;
    sens->OM[2] = OMcp3_211;
    sens->OM[3] = OMcp3_311;
    sens->J[1][1] = (1.0);
    sens->J[1][4] = JTcp3_19_4;
    sens->J[1][5] = JTcp3_19_5;
    sens->J[1][6] = JTcp3_19_6;
    sens->J[1][8] = JTcp3_19_7;
    sens->J[2][2] = (1.0);
    sens->J[2][4] = JTcp3_29_4;
    sens->J[2][5] = JTcp3_29_5;
    sens->J[2][6] = JTcp3_29_6;
    sens->J[2][8] = JTcp3_29_7;
    sens->J[3][3] = (1.0);
    sens->J[3][5] = JTcp3_39_5;
    sens->J[3][6] = JTcp3_39_6;
    sens->J[3][8] = JTcp3_39_7;
    sens->J[4][5] = C4;
    sens->J[4][6] = ROcp3_45;
    sens->J[4][8] = ROcp3_16;
    sens->J[4][9] = ROcp3_48;
    sens->J[4][10] = ROcp3_19;
    sens->J[4][11] = ROcp3_710;
    sens->J[5][5] = S4;
    sens->J[5][6] = ROcp3_55;
    sens->J[5][8] = ROcp3_26;
    sens->J[5][9] = ROcp3_58;
    sens->J[5][10] = ROcp3_29;
    sens->J[5][11] = ROcp3_810;
    sens->J[6][4] = (1.0);
    sens->J[6][6] = S5;
    sens->J[6][8] = ROcp3_36;
    sens->J[6][9] = ROcp3_68;
    sens->J[6][10] = ROcp3_39;
    sens->J[6][11] = ROcp3_910;
    sens->A[1] = ACcp3_19;
    sens->A[2] = ACcp3_29;
    sens->A[3] = ACcp3_39;
    sens->OMP[1] = OPcp3_111;
    sens->OMP[2] = OPcp3_211;
    sens->OMP[3] = OPcp3_311;
 
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

// = = Block_1_0_0_5_0_3 = = 
 
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
    OMcp4_18 = OMcp4_16+ROcp4_16*qd[8];
    OMcp4_28 = OMcp4_26+ROcp4_26*qd[8];
    OMcp4_38 = OMcp4_36+ROcp4_36*qd[8];
    ORcp4_18 = OMcp4_26*RLcp4_38-OMcp4_36*RLcp4_28;
    ORcp4_28 = -(OMcp4_16*RLcp4_38-OMcp4_36*RLcp4_18);
    ORcp4_38 = OMcp4_16*RLcp4_28-OMcp4_26*RLcp4_18;
    OPcp4_18 = OPcp4_16+ROcp4_16*qdd[8]+qd[8]*(OMcp4_26*ROcp4_36-OMcp4_36*ROcp4_26);
    OPcp4_28 = OPcp4_26+ROcp4_26*qdd[8]-qd[8]*(OMcp4_16*ROcp4_36-OMcp4_36*ROcp4_16);
    OPcp4_38 = OPcp4_36+ROcp4_36*qdd[8]+qd[8]*(OMcp4_16*ROcp4_26-OMcp4_26*ROcp4_16);
    RLcp4_19 = ROcp4_48*s->dpt[2][19];
    RLcp4_29 = ROcp4_58*s->dpt[2][19];
    RLcp4_39 = ROcp4_68*s->dpt[2][19];
    OMcp4_19 = OMcp4_18+ROcp4_48*qd[9];
    OMcp4_29 = OMcp4_28+ROcp4_58*qd[9];
    OMcp4_39 = OMcp4_38+ROcp4_68*qd[9];
    ORcp4_19 = OMcp4_28*RLcp4_39-OMcp4_38*RLcp4_29;
    ORcp4_29 = -(OMcp4_18*RLcp4_39-OMcp4_38*RLcp4_19);
    ORcp4_39 = OMcp4_18*RLcp4_29-OMcp4_28*RLcp4_19;
    OMcp4_110 = OMcp4_19+ROcp4_19*qd[10];
    OMcp4_210 = OMcp4_29+ROcp4_29*qd[10];
    OMcp4_310 = OMcp4_39+ROcp4_39*qd[10];
    OMcp4_111 = OMcp4_110+ROcp4_710*qd[11];
    OMcp4_211 = OMcp4_210+ROcp4_810*qd[11];
    OMcp4_311 = OMcp4_310+ROcp4_910*qd[11];
    OPcp4_111 = OPcp4_18+ROcp4_19*qdd[10]+ROcp4_48*qdd[9]+ROcp4_710*qdd[11]+qd[10]*(OMcp4_29*ROcp4_39-OMcp4_39*ROcp4_29)+
 qd[11]*(OMcp4_210*ROcp4_910-OMcp4_310*ROcp4_810)+qd[9]*(OMcp4_28*ROcp4_68-OMcp4_38*ROcp4_58);
    OPcp4_211 = OPcp4_28+ROcp4_29*qdd[10]+ROcp4_58*qdd[9]+ROcp4_810*qdd[11]-qd[10]*(OMcp4_19*ROcp4_39-OMcp4_39*ROcp4_19)-
 qd[11]*(OMcp4_110*ROcp4_910-OMcp4_310*ROcp4_710)-qd[9]*(OMcp4_18*ROcp4_68-OMcp4_38*ROcp4_48);
    OPcp4_311 = OPcp4_38+ROcp4_39*qdd[10]+ROcp4_68*qdd[9]+ROcp4_910*qdd[11]+qd[10]*(OMcp4_19*ROcp4_29-OMcp4_29*ROcp4_19)+
 qd[11]*(OMcp4_110*ROcp4_810-OMcp4_210*ROcp4_710)+qd[9]*(OMcp4_18*ROcp4_58-OMcp4_28*ROcp4_48);
    RLcp4_112 = ROcp4_710*s->dpt[3][23];
    RLcp4_212 = ROcp4_810*s->dpt[3][23];
    RLcp4_312 = ROcp4_910*s->dpt[3][23];
    POcp4_112 = RLcp4_112+RLcp4_18+RLcp4_19+q[1];
    POcp4_212 = RLcp4_212+RLcp4_28+RLcp4_29+q[2];
    POcp4_312 = RLcp4_312+RLcp4_38+RLcp4_39+q[3];
    OMcp4_112 = OMcp4_111+ROcp4_411*qd[12];
    OMcp4_212 = OMcp4_211+ROcp4_511*qd[12];
    OMcp4_312 = OMcp4_311+ROcp4_611*qd[12];
    ORcp4_112 = OMcp4_211*RLcp4_312-OMcp4_311*RLcp4_212;
    ORcp4_212 = -(OMcp4_111*RLcp4_312-OMcp4_311*RLcp4_112);
    ORcp4_312 = OMcp4_111*RLcp4_212-OMcp4_211*RLcp4_112;
    VIcp4_112 = ORcp4_112+ORcp4_18+ORcp4_19+qd[1];
    VIcp4_212 = ORcp4_212+ORcp4_28+ORcp4_29+qd[2];
    VIcp4_312 = ORcp4_312+ORcp4_38+ORcp4_39+qd[3];
    OPcp4_112 = OPcp4_111+ROcp4_411*qdd[12]+qd[12]*(OMcp4_211*ROcp4_611-OMcp4_311*ROcp4_511);
    OPcp4_212 = OPcp4_211+ROcp4_511*qdd[12]-qd[12]*(OMcp4_111*ROcp4_611-OMcp4_311*ROcp4_411);
    OPcp4_312 = OPcp4_311+ROcp4_611*qdd[12]+qd[12]*(OMcp4_111*ROcp4_511-OMcp4_211*ROcp4_411);
    ACcp4_112 = qdd[1]+OMcp4_211*ORcp4_312+OMcp4_26*ORcp4_38+OMcp4_28*ORcp4_39-OMcp4_311*ORcp4_212-OMcp4_36*ORcp4_28-
 OMcp4_38*ORcp4_29+OPcp4_211*RLcp4_312+OPcp4_26*RLcp4_38+OPcp4_28*RLcp4_39-OPcp4_311*RLcp4_212-OPcp4_36*RLcp4_28-OPcp4_38*
 RLcp4_29;
    ACcp4_212 = qdd[2]-OMcp4_111*ORcp4_312-OMcp4_16*ORcp4_38-OMcp4_18*ORcp4_39+OMcp4_311*ORcp4_112+OMcp4_36*ORcp4_18+
 OMcp4_38*ORcp4_19-OPcp4_111*RLcp4_312-OPcp4_16*RLcp4_38-OPcp4_18*RLcp4_39+OPcp4_311*RLcp4_112+OPcp4_36*RLcp4_18+OPcp4_38*
 RLcp4_19;
    ACcp4_312 = qdd[3]+OMcp4_111*ORcp4_212+OMcp4_16*ORcp4_28+OMcp4_18*ORcp4_29-OMcp4_211*ORcp4_112-OMcp4_26*ORcp4_18-
 OMcp4_28*ORcp4_19+OPcp4_111*RLcp4_212+OPcp4_16*RLcp4_28+OPcp4_18*RLcp4_29-OPcp4_211*RLcp4_112-OPcp4_26*RLcp4_18-OPcp4_28*
 RLcp4_19;

// = = Block_1_0_0_5_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp4_112;
    sens->P[2] = POcp4_212;
    sens->P[3] = POcp4_312;
    sens->R[1][1] = ROcp4_112;
    sens->R[1][2] = ROcp4_212;
    sens->R[1][3] = ROcp4_312;
    sens->R[2][1] = ROcp4_411;
    sens->R[2][2] = ROcp4_511;
    sens->R[2][3] = ROcp4_611;
    sens->R[3][1] = ROcp4_712;
    sens->R[3][2] = ROcp4_812;
    sens->R[3][3] = ROcp4_912;
    sens->V[1] = VIcp4_112;
    sens->V[2] = VIcp4_212;
    sens->V[3] = VIcp4_312;
    sens->OM[1] = OMcp4_112;
    sens->OM[2] = OMcp4_212;
    sens->OM[3] = OMcp4_312;
    sens->A[1] = ACcp4_112;
    sens->A[2] = ACcp4_212;
    sens->A[3] = ACcp4_312;
    sens->OMP[1] = OPcp4_112;
    sens->OMP[2] = OPcp4_212;
    sens->OMP[3] = OPcp4_312;
 
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

// = = Block_1_0_0_6_0_4 = = 
 
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
    OMcp5_113 = OMcp5_16+ROcp5_16*qd[13];
    OMcp5_213 = OMcp5_26+ROcp5_26*qd[13];
    OMcp5_313 = OMcp5_36+ROcp5_36*qd[13];
    ORcp5_113 = OMcp5_26*RLcp5_313-OMcp5_36*RLcp5_213;
    ORcp5_213 = -(OMcp5_16*RLcp5_313-OMcp5_36*RLcp5_113);
    ORcp5_313 = OMcp5_16*RLcp5_213-OMcp5_26*RLcp5_113;
    OPcp5_113 = OPcp5_16+ROcp5_16*qdd[13]+qd[13]*(OMcp5_26*ROcp5_36-OMcp5_36*ROcp5_26);
    OPcp5_213 = OPcp5_26+ROcp5_26*qdd[13]-qd[13]*(OMcp5_16*ROcp5_36-OMcp5_36*ROcp5_16);
    OPcp5_313 = OPcp5_36+ROcp5_36*qdd[13]+qd[13]*(OMcp5_16*ROcp5_26-OMcp5_26*ROcp5_16);
    RLcp5_114 = ROcp5_413*s->dpt[2][26];
    RLcp5_214 = ROcp5_513*s->dpt[2][26];
    RLcp5_314 = ROcp5_613*s->dpt[2][26];
    OMcp5_114 = OMcp5_113+ROcp5_413*qd[14];
    OMcp5_214 = OMcp5_213+ROcp5_513*qd[14];
    OMcp5_314 = OMcp5_313+ROcp5_613*qd[14];
    ORcp5_114 = OMcp5_213*RLcp5_314-OMcp5_313*RLcp5_214;
    ORcp5_214 = -(OMcp5_113*RLcp5_314-OMcp5_313*RLcp5_114);
    ORcp5_314 = OMcp5_113*RLcp5_214-OMcp5_213*RLcp5_114;
    OMcp5_115 = OMcp5_114+ROcp5_114*qd[15];
    OMcp5_215 = OMcp5_214+ROcp5_214*qd[15];
    OMcp5_315 = OMcp5_314+ROcp5_314*qd[15];
    OMcp5_116 = OMcp5_115+ROcp5_715*qd[16];
    OMcp5_216 = OMcp5_215+ROcp5_815*qd[16];
    OMcp5_316 = OMcp5_315+ROcp5_915*qd[16];
    OPcp5_116 = OPcp5_113+ROcp5_114*qdd[15]+ROcp5_413*qdd[14]+ROcp5_715*qdd[16]+qd[14]*(OMcp5_213*ROcp5_613-OMcp5_313*
 ROcp5_513)+qd[15]*(OMcp5_214*ROcp5_314-OMcp5_314*ROcp5_214)+qd[16]*(OMcp5_215*ROcp5_915-OMcp5_315*ROcp5_815);
    OPcp5_216 = OPcp5_213+ROcp5_214*qdd[15]+ROcp5_513*qdd[14]+ROcp5_815*qdd[16]-qd[14]*(OMcp5_113*ROcp5_613-OMcp5_313*
 ROcp5_413)-qd[15]*(OMcp5_114*ROcp5_314-OMcp5_314*ROcp5_114)-qd[16]*(OMcp5_115*ROcp5_915-OMcp5_315*ROcp5_715);
    OPcp5_316 = OPcp5_313+ROcp5_314*qdd[15]+ROcp5_613*qdd[14]+ROcp5_915*qdd[16]+qd[14]*(OMcp5_113*ROcp5_513-OMcp5_213*
 ROcp5_413)+qd[15]*(OMcp5_114*ROcp5_214-OMcp5_214*ROcp5_114)+qd[16]*(OMcp5_115*ROcp5_815-OMcp5_215*ROcp5_715);
    RLcp5_117 = ROcp5_715*s->dpt[3][29];
    RLcp5_217 = ROcp5_815*s->dpt[3][29];
    RLcp5_317 = ROcp5_915*s->dpt[3][29];
    POcp5_117 = RLcp5_113+RLcp5_114+RLcp5_117+q[1];
    POcp5_217 = RLcp5_213+RLcp5_214+RLcp5_217+q[2];
    POcp5_317 = RLcp5_313+RLcp5_314+RLcp5_317+q[3];
    OMcp5_117 = OMcp5_116+ROcp5_416*qd[17];
    OMcp5_217 = OMcp5_216+ROcp5_516*qd[17];
    OMcp5_317 = OMcp5_316+ROcp5_616*qd[17];
    ORcp5_117 = OMcp5_216*RLcp5_317-OMcp5_316*RLcp5_217;
    ORcp5_217 = -(OMcp5_116*RLcp5_317-OMcp5_316*RLcp5_117);
    ORcp5_317 = OMcp5_116*RLcp5_217-OMcp5_216*RLcp5_117;
    VIcp5_117 = ORcp5_113+ORcp5_114+ORcp5_117+qd[1];
    VIcp5_217 = ORcp5_213+ORcp5_214+ORcp5_217+qd[2];
    VIcp5_317 = ORcp5_313+ORcp5_314+ORcp5_317+qd[3];
    OPcp5_117 = OPcp5_116+ROcp5_416*qdd[17]+qd[17]*(OMcp5_216*ROcp5_616-OMcp5_316*ROcp5_516);
    OPcp5_217 = OPcp5_216+ROcp5_516*qdd[17]-qd[17]*(OMcp5_116*ROcp5_616-OMcp5_316*ROcp5_416);
    OPcp5_317 = OPcp5_316+ROcp5_616*qdd[17]+qd[17]*(OMcp5_116*ROcp5_516-OMcp5_216*ROcp5_416);
    ACcp5_117 = qdd[1]+OMcp5_213*ORcp5_314+OMcp5_216*ORcp5_317+OMcp5_26*ORcp5_313-OMcp5_313*ORcp5_214-OMcp5_316*ORcp5_217-
 OMcp5_36*ORcp5_213+OPcp5_213*RLcp5_314+OPcp5_216*RLcp5_317+OPcp5_26*RLcp5_313-OPcp5_313*RLcp5_214-OPcp5_316*RLcp5_217-
 OPcp5_36*RLcp5_213;
    ACcp5_217 = qdd[2]-OMcp5_113*ORcp5_314-OMcp5_116*ORcp5_317-OMcp5_16*ORcp5_313+OMcp5_313*ORcp5_114+OMcp5_316*ORcp5_117+
 OMcp5_36*ORcp5_113-OPcp5_113*RLcp5_314-OPcp5_116*RLcp5_317-OPcp5_16*RLcp5_313+OPcp5_313*RLcp5_114+OPcp5_316*RLcp5_117+
 OPcp5_36*RLcp5_113;
    ACcp5_317 = qdd[3]+OMcp5_113*ORcp5_214+OMcp5_116*ORcp5_217+OMcp5_16*ORcp5_213-OMcp5_213*ORcp5_114-OMcp5_216*ORcp5_117-
 OMcp5_26*ORcp5_113+OPcp5_113*RLcp5_214+OPcp5_116*RLcp5_217+OPcp5_16*RLcp5_213-OPcp5_213*RLcp5_114-OPcp5_216*RLcp5_117-
 OPcp5_26*RLcp5_113;

// = = Block_1_0_0_6_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp5_117;
    sens->P[2] = POcp5_217;
    sens->P[3] = POcp5_317;
    sens->R[1][1] = ROcp5_117;
    sens->R[1][2] = ROcp5_217;
    sens->R[1][3] = ROcp5_317;
    sens->R[2][1] = ROcp5_416;
    sens->R[2][2] = ROcp5_516;
    sens->R[2][3] = ROcp5_616;
    sens->R[3][1] = ROcp5_717;
    sens->R[3][2] = ROcp5_817;
    sens->R[3][3] = ROcp5_917;
    sens->V[1] = VIcp5_117;
    sens->V[2] = VIcp5_217;
    sens->V[3] = VIcp5_317;
    sens->OM[1] = OMcp5_117;
    sens->OM[2] = OMcp5_217;
    sens->OM[3] = OMcp5_317;
    sens->A[1] = ACcp5_117;
    sens->A[2] = ACcp5_217;
    sens->A[3] = ACcp5_317;
    sens->OMP[1] = OPcp5_117;
    sens->OMP[2] = OPcp5_217;
    sens->OMP[3] = OPcp5_317;
 
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

// = = Block_1_0_0_7_0_16 = = 
 
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
    OMcp6_135 = OMcp6_16+ROcp6_16*qd[35];
    OMcp6_235 = OMcp6_26+ROcp6_26*qd[35];
    OMcp6_335 = OMcp6_36+ROcp6_36*qd[35];
    ORcp6_135 = OMcp6_26*RLcp6_335-OMcp6_36*RLcp6_235;
    ORcp6_235 = -(OMcp6_16*RLcp6_335-OMcp6_36*RLcp6_135);
    ORcp6_335 = OMcp6_16*RLcp6_235-OMcp6_26*RLcp6_135;
    OPcp6_135 = OPcp6_16+ROcp6_16*qdd[35]+qd[35]*(OMcp6_26*ROcp6_36-OMcp6_36*ROcp6_26);
    OPcp6_235 = OPcp6_26+ROcp6_26*qdd[35]-qd[35]*(OMcp6_16*ROcp6_36-OMcp6_36*ROcp6_16);
    OPcp6_335 = OPcp6_36+ROcp6_36*qdd[35]+qd[35]*(OMcp6_16*ROcp6_26-OMcp6_26*ROcp6_16);
    RLcp6_136 = ROcp6_435*s->dpt[2][50];
    RLcp6_236 = ROcp6_535*s->dpt[2][50];
    RLcp6_336 = ROcp6_635*s->dpt[2][50];
    OMcp6_136 = OMcp6_135+ROcp6_16*qd[36];
    OMcp6_236 = OMcp6_235+ROcp6_26*qd[36];
    OMcp6_336 = OMcp6_335+ROcp6_36*qd[36];
    ORcp6_136 = OMcp6_235*RLcp6_336-OMcp6_335*RLcp6_236;
    ORcp6_236 = -(OMcp6_135*RLcp6_336-OMcp6_335*RLcp6_136);
    ORcp6_336 = OMcp6_135*RLcp6_236-OMcp6_235*RLcp6_136;
    OPcp6_136 = OPcp6_135+ROcp6_16*qdd[36]+qd[36]*(OMcp6_235*ROcp6_36-OMcp6_335*ROcp6_26);
    OPcp6_236 = OPcp6_235+ROcp6_26*qdd[36]-qd[36]*(OMcp6_135*ROcp6_36-OMcp6_335*ROcp6_16);
    OPcp6_336 = OPcp6_335+ROcp6_36*qdd[36]+qd[36]*(OMcp6_135*ROcp6_26-OMcp6_235*ROcp6_16);
    RLcp6_137 = ROcp6_736*s->dpt[3][53];
    RLcp6_237 = ROcp6_836*s->dpt[3][53];
    RLcp6_337 = ROcp6_936*s->dpt[3][53];
    POcp6_137 = RLcp6_135+RLcp6_136+RLcp6_137+q[1];
    POcp6_237 = RLcp6_235+RLcp6_236+RLcp6_237+q[2];
    POcp6_337 = RLcp6_335+RLcp6_336+RLcp6_337+q[3];
    OMcp6_137 = OMcp6_136+ROcp6_436*qd[37];
    OMcp6_237 = OMcp6_236+ROcp6_536*qd[37];
    OMcp6_337 = OMcp6_336+ROcp6_636*qd[37];
    ORcp6_137 = OMcp6_236*RLcp6_337-OMcp6_336*RLcp6_237;
    ORcp6_237 = -(OMcp6_136*RLcp6_337-OMcp6_336*RLcp6_137);
    ORcp6_337 = OMcp6_136*RLcp6_237-OMcp6_236*RLcp6_137;
    VIcp6_137 = ORcp6_135+ORcp6_136+ORcp6_137+qd[1];
    VIcp6_237 = ORcp6_235+ORcp6_236+ORcp6_237+qd[2];
    VIcp6_337 = ORcp6_335+ORcp6_336+ORcp6_337+qd[3];
    OPcp6_137 = OPcp6_136+ROcp6_436*qdd[37]+qd[37]*(OMcp6_236*ROcp6_636-OMcp6_336*ROcp6_536);
    OPcp6_237 = OPcp6_236+ROcp6_536*qdd[37]-qd[37]*(OMcp6_136*ROcp6_636-OMcp6_336*ROcp6_436);
    OPcp6_337 = OPcp6_336+ROcp6_636*qdd[37]+qd[37]*(OMcp6_136*ROcp6_536-OMcp6_236*ROcp6_436);
    ACcp6_137 = qdd[1]+OMcp6_235*ORcp6_336+OMcp6_236*ORcp6_337+OMcp6_26*ORcp6_335-OMcp6_335*ORcp6_236-OMcp6_336*ORcp6_237-
 OMcp6_36*ORcp6_235+OPcp6_235*RLcp6_336+OPcp6_236*RLcp6_337+OPcp6_26*RLcp6_335-OPcp6_335*RLcp6_236-OPcp6_336*RLcp6_237-
 OPcp6_36*RLcp6_235;
    ACcp6_237 = qdd[2]-OMcp6_135*ORcp6_336-OMcp6_136*ORcp6_337-OMcp6_16*ORcp6_335+OMcp6_335*ORcp6_136+OMcp6_336*ORcp6_137+
 OMcp6_36*ORcp6_135-OPcp6_135*RLcp6_336-OPcp6_136*RLcp6_337-OPcp6_16*RLcp6_335+OPcp6_335*RLcp6_136+OPcp6_336*RLcp6_137+
 OPcp6_36*RLcp6_135;
    ACcp6_337 = qdd[3]+OMcp6_135*ORcp6_236+OMcp6_136*ORcp6_237+OMcp6_16*ORcp6_235-OMcp6_235*ORcp6_136-OMcp6_236*ORcp6_137-
 OMcp6_26*ORcp6_135+OPcp6_135*RLcp6_236+OPcp6_136*RLcp6_237+OPcp6_16*RLcp6_235-OPcp6_235*RLcp6_136-OPcp6_236*RLcp6_137-
 OPcp6_26*RLcp6_135;

// = = Block_1_0_0_7_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp6_137;
    sens->P[2] = POcp6_237;
    sens->P[3] = POcp6_337;
    sens->R[1][1] = ROcp6_137;
    sens->R[1][2] = ROcp6_237;
    sens->R[1][3] = ROcp6_337;
    sens->R[2][1] = ROcp6_436;
    sens->R[2][2] = ROcp6_536;
    sens->R[2][3] = ROcp6_636;
    sens->R[3][1] = ROcp6_737;
    sens->R[3][2] = ROcp6_837;
    sens->R[3][3] = ROcp6_937;
    sens->V[1] = VIcp6_137;
    sens->V[2] = VIcp6_237;
    sens->V[3] = VIcp6_337;
    sens->OM[1] = OMcp6_137;
    sens->OM[2] = OMcp6_237;
    sens->OM[3] = OMcp6_337;
    sens->A[1] = ACcp6_137;
    sens->A[2] = ACcp6_237;
    sens->A[3] = ACcp6_337;
    sens->OMP[1] = OPcp6_137;
    sens->OMP[2] = OPcp6_237;
    sens->OMP[3] = OPcp6_337;
 
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

// = = Block_1_0_0_8_0_17 = = 
 
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
    OMcp7_138 = OMcp7_16+ROcp7_16*qd[38];
    OMcp7_238 = OMcp7_26+ROcp7_26*qd[38];
    OMcp7_338 = OMcp7_36+ROcp7_36*qd[38];
    ORcp7_138 = OMcp7_26*RLcp7_338-OMcp7_36*RLcp7_238;
    ORcp7_238 = -(OMcp7_16*RLcp7_338-OMcp7_36*RLcp7_138);
    ORcp7_338 = OMcp7_16*RLcp7_238-OMcp7_26*RLcp7_138;
    OPcp7_138 = OPcp7_16+ROcp7_16*qdd[38]+qd[38]*(OMcp7_26*ROcp7_36-OMcp7_36*ROcp7_26);
    OPcp7_238 = OPcp7_26+ROcp7_26*qdd[38]-qd[38]*(OMcp7_16*ROcp7_36-OMcp7_36*ROcp7_16);
    OPcp7_338 = OPcp7_36+ROcp7_36*qdd[38]+qd[38]*(OMcp7_16*ROcp7_26-OMcp7_26*ROcp7_16);
    RLcp7_139 = ROcp7_438*s->dpt[2][55];
    RLcp7_239 = ROcp7_538*s->dpt[2][55];
    RLcp7_339 = ROcp7_638*s->dpt[2][55];
    OMcp7_139 = OMcp7_138+ROcp7_16*qd[39];
    OMcp7_239 = OMcp7_238+ROcp7_26*qd[39];
    OMcp7_339 = OMcp7_338+ROcp7_36*qd[39];
    ORcp7_139 = OMcp7_238*RLcp7_339-OMcp7_338*RLcp7_239;
    ORcp7_239 = -(OMcp7_138*RLcp7_339-OMcp7_338*RLcp7_139);
    ORcp7_339 = OMcp7_138*RLcp7_239-OMcp7_238*RLcp7_139;
    OPcp7_139 = OPcp7_138+ROcp7_16*qdd[39]+qd[39]*(OMcp7_238*ROcp7_36-OMcp7_338*ROcp7_26);
    OPcp7_239 = OPcp7_238+ROcp7_26*qdd[39]-qd[39]*(OMcp7_138*ROcp7_36-OMcp7_338*ROcp7_16);
    OPcp7_339 = OPcp7_338+ROcp7_36*qdd[39]+qd[39]*(OMcp7_138*ROcp7_26-OMcp7_238*ROcp7_16);
    RLcp7_140 = ROcp7_739*s->dpt[3][57];
    RLcp7_240 = ROcp7_839*s->dpt[3][57];
    RLcp7_340 = ROcp7_939*s->dpt[3][57];
    POcp7_140 = RLcp7_138+RLcp7_139+RLcp7_140+q[1];
    POcp7_240 = RLcp7_238+RLcp7_239+RLcp7_240+q[2];
    POcp7_340 = RLcp7_338+RLcp7_339+RLcp7_340+q[3];
    OMcp7_140 = OMcp7_139+ROcp7_439*qd[40];
    OMcp7_240 = OMcp7_239+ROcp7_539*qd[40];
    OMcp7_340 = OMcp7_339+ROcp7_639*qd[40];
    ORcp7_140 = OMcp7_239*RLcp7_340-OMcp7_339*RLcp7_240;
    ORcp7_240 = -(OMcp7_139*RLcp7_340-OMcp7_339*RLcp7_140);
    ORcp7_340 = OMcp7_139*RLcp7_240-OMcp7_239*RLcp7_140;
    VIcp7_140 = ORcp7_138+ORcp7_139+ORcp7_140+qd[1];
    VIcp7_240 = ORcp7_238+ORcp7_239+ORcp7_240+qd[2];
    VIcp7_340 = ORcp7_338+ORcp7_339+ORcp7_340+qd[3];
    OPcp7_140 = OPcp7_139+ROcp7_439*qdd[40]+qd[40]*(OMcp7_239*ROcp7_639-OMcp7_339*ROcp7_539);
    OPcp7_240 = OPcp7_239+ROcp7_539*qdd[40]-qd[40]*(OMcp7_139*ROcp7_639-OMcp7_339*ROcp7_439);
    OPcp7_340 = OPcp7_339+ROcp7_639*qdd[40]+qd[40]*(OMcp7_139*ROcp7_539-OMcp7_239*ROcp7_439);
    ACcp7_140 = qdd[1]+OMcp7_238*ORcp7_339+OMcp7_239*ORcp7_340+OMcp7_26*ORcp7_338-OMcp7_338*ORcp7_239-OMcp7_339*ORcp7_240-
 OMcp7_36*ORcp7_238+OPcp7_238*RLcp7_339+OPcp7_239*RLcp7_340+OPcp7_26*RLcp7_338-OPcp7_338*RLcp7_239-OPcp7_339*RLcp7_240-
 OPcp7_36*RLcp7_238;
    ACcp7_240 = qdd[2]-OMcp7_138*ORcp7_339-OMcp7_139*ORcp7_340-OMcp7_16*ORcp7_338+OMcp7_338*ORcp7_139+OMcp7_339*ORcp7_140+
 OMcp7_36*ORcp7_138-OPcp7_138*RLcp7_339-OPcp7_139*RLcp7_340-OPcp7_16*RLcp7_338+OPcp7_338*RLcp7_139+OPcp7_339*RLcp7_140+
 OPcp7_36*RLcp7_138;
    ACcp7_340 = qdd[3]+OMcp7_138*ORcp7_239+OMcp7_139*ORcp7_240+OMcp7_16*ORcp7_238-OMcp7_238*ORcp7_139-OMcp7_239*ORcp7_140-
 OMcp7_26*ORcp7_138+OPcp7_138*RLcp7_239+OPcp7_139*RLcp7_240+OPcp7_16*RLcp7_238-OPcp7_238*RLcp7_139-OPcp7_239*RLcp7_140-
 OPcp7_26*RLcp7_138;

// = = Block_1_0_0_8_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp7_140;
    sens->P[2] = POcp7_240;
    sens->P[3] = POcp7_340;
    sens->R[1][1] = ROcp7_140;
    sens->R[1][2] = ROcp7_240;
    sens->R[1][3] = ROcp7_340;
    sens->R[2][1] = ROcp7_439;
    sens->R[2][2] = ROcp7_539;
    sens->R[2][3] = ROcp7_639;
    sens->R[3][1] = ROcp7_740;
    sens->R[3][2] = ROcp7_840;
    sens->R[3][3] = ROcp7_940;
    sens->V[1] = VIcp7_140;
    sens->V[2] = VIcp7_240;
    sens->V[3] = VIcp7_340;
    sens->OM[1] = OMcp7_140;
    sens->OM[2] = OMcp7_240;
    sens->OM[3] = OMcp7_340;
    sens->A[1] = ACcp7_140;
    sens->A[2] = ACcp7_240;
    sens->A[3] = ACcp7_340;
    sens->OMP[1] = OPcp7_140;
    sens->OMP[2] = OPcp7_240;
    sens->OMP[3] = OPcp7_340;
 
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

// = = Block_1_0_0_9_0_22 = = 
 
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
    OMcp8_151 = OMcp8_16+ROcp8_76*qd[51];
    OMcp8_251 = OMcp8_26+ROcp8_86*qd[51];
    OMcp8_351 = OMcp8_36+ROcp8_96*qd[51];
    ORcp8_151 = OMcp8_26*RLcp8_351-OMcp8_36*RLcp8_251;
    ORcp8_251 = -(OMcp8_16*RLcp8_351-OMcp8_36*RLcp8_151);
    ORcp8_351 = OMcp8_16*RLcp8_251-OMcp8_26*RLcp8_151;
    OMcp8_152 = OMcp8_151+ROcp8_151*qd[52];
    OMcp8_252 = OMcp8_251+ROcp8_251*qd[52];
    OMcp8_352 = OMcp8_351+ROcp8_351*qd[52];
    OMcp8_153 = OMcp8_152+ROcp8_452*qd[53];
    OMcp8_253 = OMcp8_252+ROcp8_552*qd[53];
    OMcp8_353 = OMcp8_352+ROcp8_652*qd[53];
    OPcp8_153 = OPcp8_16+ROcp8_151*qdd[52]+ROcp8_452*qdd[53]+ROcp8_76*qdd[51]+qd[51]*(OMcp8_26*ROcp8_96-OMcp8_36*ROcp8_86)
 +qd[52]*(OMcp8_251*ROcp8_351-OMcp8_351*ROcp8_251)+qd[53]*(OMcp8_252*ROcp8_652-OMcp8_352*ROcp8_552);
    OPcp8_253 = OPcp8_26+ROcp8_251*qdd[52]+ROcp8_552*qdd[53]+ROcp8_86*qdd[51]-qd[51]*(OMcp8_16*ROcp8_96-OMcp8_36*ROcp8_76)
 -qd[52]*(OMcp8_151*ROcp8_351-OMcp8_351*ROcp8_151)-qd[53]*(OMcp8_152*ROcp8_652-OMcp8_352*ROcp8_452);
    OPcp8_353 = OPcp8_36+ROcp8_351*qdd[52]+ROcp8_652*qdd[53]+ROcp8_96*qdd[51]+qd[51]*(OMcp8_16*ROcp8_86-OMcp8_26*ROcp8_76)
 +qd[52]*(OMcp8_151*ROcp8_251-OMcp8_251*ROcp8_151)+qd[53]*(OMcp8_152*ROcp8_552-OMcp8_252*ROcp8_452);
    RLcp8_154 = ROcp8_153*q[54];
    RLcp8_254 = ROcp8_253*q[54];
    RLcp8_354 = ROcp8_353*q[54];
    ORcp8_154 = OMcp8_253*RLcp8_354-OMcp8_353*RLcp8_254;
    ORcp8_254 = -(OMcp8_153*RLcp8_354-OMcp8_353*RLcp8_154);
    ORcp8_354 = OMcp8_153*RLcp8_254-OMcp8_253*RLcp8_154;

// = = Block_1_0_0_9_0_23 = = 
 
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
    POcp8_155 = RLcp8_151+RLcp8_154+RLcp8_155+q[1];
    POcp8_255 = RLcp8_251+RLcp8_254+RLcp8_255+q[2];
    POcp8_355 = RLcp8_351+RLcp8_354+RLcp8_355+q[3];
    OMcp8_155 = OMcp8_153+ROcp8_452*qd[55];
    OMcp8_255 = OMcp8_253+ROcp8_552*qd[55];
    OMcp8_355 = OMcp8_353+ROcp8_652*qd[55];
    ORcp8_155 = OMcp8_253*RLcp8_355-OMcp8_353*RLcp8_255;
    ORcp8_255 = -(OMcp8_153*RLcp8_355-OMcp8_353*RLcp8_155);
    ORcp8_355 = OMcp8_153*RLcp8_255-OMcp8_253*RLcp8_155;
    VIcp8_155 = ORcp8_151+ORcp8_154+ORcp8_155+qd[1]+ROcp8_153*qd[54];
    VIcp8_255 = ORcp8_251+ORcp8_254+ORcp8_255+qd[2]+ROcp8_253*qd[54];
    VIcp8_355 = ORcp8_351+ORcp8_354+ORcp8_355+qd[3]+ROcp8_353*qd[54];
    OPcp8_155 = OPcp8_153+ROcp8_452*qdd[55]+qd[55]*(OMcp8_253*ROcp8_652-OMcp8_353*ROcp8_552);
    OPcp8_255 = OPcp8_253+ROcp8_552*qdd[55]-qd[55]*(OMcp8_153*ROcp8_652-OMcp8_353*ROcp8_452);
    OPcp8_355 = OPcp8_353+ROcp8_652*qdd[55]+qd[55]*(OMcp8_153*ROcp8_552-OMcp8_253*ROcp8_452);
    ACcp8_155 = qdd[1]+OMcp8_253*ORcp8_354+OMcp8_253*ORcp8_355+OMcp8_26*ORcp8_351-OMcp8_353*ORcp8_254-OMcp8_353*ORcp8_255-
 OMcp8_36*ORcp8_251+OPcp8_253*RLcp8_354+OPcp8_253*RLcp8_355+OPcp8_26*RLcp8_351-OPcp8_353*RLcp8_254-OPcp8_353*RLcp8_255-
 OPcp8_36*RLcp8_251+ROcp8_153*qdd[54]+(2.0)*qd[54]*(OMcp8_253*ROcp8_353-OMcp8_353*ROcp8_253);
    ACcp8_255 = qdd[2]-OMcp8_153*ORcp8_354-OMcp8_153*ORcp8_355-OMcp8_16*ORcp8_351+OMcp8_353*ORcp8_154+OMcp8_353*ORcp8_155+
 OMcp8_36*ORcp8_151-OPcp8_153*RLcp8_354-OPcp8_153*RLcp8_355-OPcp8_16*RLcp8_351+OPcp8_353*RLcp8_154+OPcp8_353*RLcp8_155+
 OPcp8_36*RLcp8_151+ROcp8_253*qdd[54]-(2.0)*qd[54]*(OMcp8_153*ROcp8_353-OMcp8_353*ROcp8_153);
    ACcp8_355 = qdd[3]+OMcp8_153*ORcp8_254+OMcp8_153*ORcp8_255+OMcp8_16*ORcp8_251-OMcp8_253*ORcp8_154-OMcp8_253*ORcp8_155-
 OMcp8_26*ORcp8_151+OPcp8_153*RLcp8_254+OPcp8_153*RLcp8_255+OPcp8_16*RLcp8_251-OPcp8_253*RLcp8_154-OPcp8_253*RLcp8_155-
 OPcp8_26*RLcp8_151+ROcp8_353*qdd[54]+(2.0)*qd[54]*(OMcp8_153*ROcp8_253-OMcp8_253*ROcp8_153);

// = = Block_1_0_0_9_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp8_155;
    sens->P[2] = POcp8_255;
    sens->P[3] = POcp8_355;
    sens->R[1][1] = ROcp8_155;
    sens->R[1][2] = ROcp8_255;
    sens->R[1][3] = ROcp8_355;
    sens->R[2][1] = ROcp8_452;
    sens->R[2][2] = ROcp8_552;
    sens->R[2][3] = ROcp8_652;
    sens->R[3][1] = ROcp8_755;
    sens->R[3][2] = ROcp8_855;
    sens->R[3][3] = ROcp8_955;
    sens->V[1] = VIcp8_155;
    sens->V[2] = VIcp8_255;
    sens->V[3] = VIcp8_355;
    sens->OM[1] = OMcp8_155;
    sens->OM[2] = OMcp8_255;
    sens->OM[3] = OMcp8_355;
    sens->A[1] = ACcp8_155;
    sens->A[2] = ACcp8_255;
    sens->A[3] = ACcp8_355;
    sens->OMP[1] = OPcp8_155;
    sens->OMP[2] = OPcp8_255;
    sens->OMP[3] = OPcp8_355;
 
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

// = = Block_1_0_0_10_0_22 = = 
 
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
    OMcp9_151 = OMcp9_16+ROcp9_76*qd[51];
    OMcp9_251 = OMcp9_26+ROcp9_86*qd[51];
    OMcp9_351 = OMcp9_36+ROcp9_96*qd[51];
    ORcp9_151 = OMcp9_26*RLcp9_351-OMcp9_36*RLcp9_251;
    ORcp9_251 = -(OMcp9_16*RLcp9_351-OMcp9_36*RLcp9_151);
    ORcp9_351 = OMcp9_16*RLcp9_251-OMcp9_26*RLcp9_151;
    OMcp9_152 = OMcp9_151+ROcp9_151*qd[52];
    OMcp9_252 = OMcp9_251+ROcp9_251*qd[52];
    OMcp9_352 = OMcp9_351+ROcp9_351*qd[52];
    OMcp9_153 = OMcp9_152+ROcp9_452*qd[53];
    OMcp9_253 = OMcp9_252+ROcp9_552*qd[53];
    OMcp9_353 = OMcp9_352+ROcp9_652*qd[53];
    OPcp9_153 = OPcp9_16+ROcp9_151*qdd[52]+ROcp9_452*qdd[53]+ROcp9_76*qdd[51]+qd[51]*(OMcp9_26*ROcp9_96-OMcp9_36*ROcp9_86)
 +qd[52]*(OMcp9_251*ROcp9_351-OMcp9_351*ROcp9_251)+qd[53]*(OMcp9_252*ROcp9_652-OMcp9_352*ROcp9_552);
    OPcp9_253 = OPcp9_26+ROcp9_251*qdd[52]+ROcp9_552*qdd[53]+ROcp9_86*qdd[51]-qd[51]*(OMcp9_16*ROcp9_96-OMcp9_36*ROcp9_76)
 -qd[52]*(OMcp9_151*ROcp9_351-OMcp9_351*ROcp9_151)-qd[53]*(OMcp9_152*ROcp9_652-OMcp9_352*ROcp9_452);
    OPcp9_353 = OPcp9_36+ROcp9_351*qdd[52]+ROcp9_652*qdd[53]+ROcp9_96*qdd[51]+qd[51]*(OMcp9_16*ROcp9_86-OMcp9_26*ROcp9_76)
 +qd[52]*(OMcp9_151*ROcp9_251-OMcp9_251*ROcp9_151)+qd[53]*(OMcp9_152*ROcp9_552-OMcp9_252*ROcp9_452);
    RLcp9_154 = ROcp9_153*q[54];
    RLcp9_254 = ROcp9_253*q[54];
    RLcp9_354 = ROcp9_353*q[54];
    ORcp9_154 = OMcp9_253*RLcp9_354-OMcp9_353*RLcp9_254;
    ORcp9_254 = -(OMcp9_153*RLcp9_354-OMcp9_353*RLcp9_154);
    ORcp9_354 = OMcp9_153*RLcp9_254-OMcp9_253*RLcp9_154;

// = = Block_1_0_0_10_0_24 = = 
 
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
    POcp9_156 = RLcp9_151+RLcp9_154+RLcp9_156+q[1];
    POcp9_256 = RLcp9_251+RLcp9_254+RLcp9_256+q[2];
    POcp9_356 = RLcp9_351+RLcp9_354+RLcp9_356+q[3];
    OMcp9_156 = OMcp9_153+ROcp9_452*qd[56];
    OMcp9_256 = OMcp9_253+ROcp9_552*qd[56];
    OMcp9_356 = OMcp9_353+ROcp9_652*qd[56];
    ORcp9_156 = OMcp9_253*RLcp9_356-OMcp9_353*RLcp9_256;
    ORcp9_256 = -(OMcp9_153*RLcp9_356-OMcp9_353*RLcp9_156);
    ORcp9_356 = OMcp9_153*RLcp9_256-OMcp9_253*RLcp9_156;
    VIcp9_156 = ORcp9_151+ORcp9_154+ORcp9_156+qd[1]+ROcp9_153*qd[54];
    VIcp9_256 = ORcp9_251+ORcp9_254+ORcp9_256+qd[2]+ROcp9_253*qd[54];
    VIcp9_356 = ORcp9_351+ORcp9_354+ORcp9_356+qd[3]+ROcp9_353*qd[54];
    OPcp9_156 = OPcp9_153+ROcp9_452*qdd[56]+qd[56]*(OMcp9_253*ROcp9_652-OMcp9_353*ROcp9_552);
    OPcp9_256 = OPcp9_253+ROcp9_552*qdd[56]-qd[56]*(OMcp9_153*ROcp9_652-OMcp9_353*ROcp9_452);
    OPcp9_356 = OPcp9_353+ROcp9_652*qdd[56]+qd[56]*(OMcp9_153*ROcp9_552-OMcp9_253*ROcp9_452);
    ACcp9_156 = qdd[1]+OMcp9_253*ORcp9_354+OMcp9_253*ORcp9_356+OMcp9_26*ORcp9_351-OMcp9_353*ORcp9_254-OMcp9_353*ORcp9_256-
 OMcp9_36*ORcp9_251+OPcp9_253*RLcp9_354+OPcp9_253*RLcp9_356+OPcp9_26*RLcp9_351-OPcp9_353*RLcp9_254-OPcp9_353*RLcp9_256-
 OPcp9_36*RLcp9_251+ROcp9_153*qdd[54]+(2.0)*qd[54]*(OMcp9_253*ROcp9_353-OMcp9_353*ROcp9_253);
    ACcp9_256 = qdd[2]-OMcp9_153*ORcp9_354-OMcp9_153*ORcp9_356-OMcp9_16*ORcp9_351+OMcp9_353*ORcp9_154+OMcp9_353*ORcp9_156+
 OMcp9_36*ORcp9_151-OPcp9_153*RLcp9_354-OPcp9_153*RLcp9_356-OPcp9_16*RLcp9_351+OPcp9_353*RLcp9_154+OPcp9_353*RLcp9_156+
 OPcp9_36*RLcp9_151+ROcp9_253*qdd[54]-(2.0)*qd[54]*(OMcp9_153*ROcp9_353-OMcp9_353*ROcp9_153);
    ACcp9_356 = qdd[3]+OMcp9_153*ORcp9_254+OMcp9_153*ORcp9_256+OMcp9_16*ORcp9_251-OMcp9_253*ORcp9_154-OMcp9_253*ORcp9_156-
 OMcp9_26*ORcp9_151+OPcp9_153*RLcp9_254+OPcp9_153*RLcp9_256+OPcp9_16*RLcp9_251-OPcp9_253*RLcp9_154-OPcp9_253*RLcp9_156-
 OPcp9_26*RLcp9_151+ROcp9_353*qdd[54]+(2.0)*qd[54]*(OMcp9_153*ROcp9_253-OMcp9_253*ROcp9_153);

// = = Block_1_0_0_10_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp9_156;
    sens->P[2] = POcp9_256;
    sens->P[3] = POcp9_356;
    sens->R[1][1] = ROcp9_156;
    sens->R[1][2] = ROcp9_256;
    sens->R[1][3] = ROcp9_356;
    sens->R[2][1] = ROcp9_452;
    sens->R[2][2] = ROcp9_552;
    sens->R[2][3] = ROcp9_652;
    sens->R[3][1] = ROcp9_756;
    sens->R[3][2] = ROcp9_856;
    sens->R[3][3] = ROcp9_956;
    sens->V[1] = VIcp9_156;
    sens->V[2] = VIcp9_256;
    sens->V[3] = VIcp9_356;
    sens->OM[1] = OMcp9_156;
    sens->OM[2] = OMcp9_256;
    sens->OM[3] = OMcp9_356;
    sens->A[1] = ACcp9_156;
    sens->A[2] = ACcp9_256;
    sens->A[3] = ACcp9_356;
    sens->OMP[1] = OPcp9_156;
    sens->OMP[2] = OPcp9_256;
    sens->OMP[3] = OPcp9_356;
 
// 
break;
case 11:
 


// = = Block_1_0_0_11_0_1 = = 
 
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
    OMcp10_16 = OMcp10_15+ROcp10_45*qd[6];
    OMcp10_26 = OMcp10_25+ROcp10_55*qd[6];
    OMcp10_36 = qd[4]+qd[6]*S5;
    OPcp10_16 = ROcp10_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp10_25*S5-ROcp10_55*qd[4]);
    OPcp10_26 = ROcp10_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp10_15*S5-ROcp10_45*qd[4]);
    OPcp10_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_11_0_22 = = 
 
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
    OMcp10_151 = OMcp10_16+ROcp10_76*qd[51];
    OMcp10_251 = OMcp10_26+ROcp10_86*qd[51];
    OMcp10_351 = OMcp10_36+ROcp10_96*qd[51];
    ORcp10_151 = OMcp10_26*RLcp10_351-OMcp10_36*RLcp10_251;
    ORcp10_251 = -(OMcp10_16*RLcp10_351-OMcp10_36*RLcp10_151);
    ORcp10_351 = OMcp10_16*RLcp10_251-OMcp10_26*RLcp10_151;
    OMcp10_152 = OMcp10_151+ROcp10_151*qd[52];
    OMcp10_252 = OMcp10_251+ROcp10_251*qd[52];
    OMcp10_352 = OMcp10_351+ROcp10_351*qd[52];
    OMcp10_153 = OMcp10_152+ROcp10_452*qd[53];
    OMcp10_253 = OMcp10_252+ROcp10_552*qd[53];
    OMcp10_353 = OMcp10_352+ROcp10_652*qd[53];
    OPcp10_153 = OPcp10_16+ROcp10_151*qdd[52]+ROcp10_452*qdd[53]+ROcp10_76*qdd[51]+qd[51]*(OMcp10_26*ROcp10_96-OMcp10_36*
 ROcp10_86)+qd[52]*(OMcp10_251*ROcp10_351-OMcp10_351*ROcp10_251)+qd[53]*(OMcp10_252*ROcp10_652-OMcp10_352*ROcp10_552);
    OPcp10_253 = OPcp10_26+ROcp10_251*qdd[52]+ROcp10_552*qdd[53]+ROcp10_86*qdd[51]-qd[51]*(OMcp10_16*ROcp10_96-OMcp10_36*
 ROcp10_76)-qd[52]*(OMcp10_151*ROcp10_351-OMcp10_351*ROcp10_151)-qd[53]*(OMcp10_152*ROcp10_652-OMcp10_352*ROcp10_452);
    OPcp10_353 = OPcp10_36+ROcp10_351*qdd[52]+ROcp10_652*qdd[53]+ROcp10_96*qdd[51]+qd[51]*(OMcp10_16*ROcp10_86-OMcp10_26*
 ROcp10_76)+qd[52]*(OMcp10_151*ROcp10_251-OMcp10_251*ROcp10_151)+qd[53]*(OMcp10_152*ROcp10_552-OMcp10_252*ROcp10_452);
    RLcp10_154 = ROcp10_153*q[54];
    RLcp10_254 = ROcp10_253*q[54];
    RLcp10_354 = ROcp10_353*q[54];
    ORcp10_154 = OMcp10_253*RLcp10_354-OMcp10_353*RLcp10_254;
    ORcp10_254 = -(OMcp10_153*RLcp10_354-OMcp10_353*RLcp10_154);
    ORcp10_354 = OMcp10_153*RLcp10_254-OMcp10_253*RLcp10_154;

// = = Block_1_0_0_11_0_25 = = 
 
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
    POcp10_157 = RLcp10_151+RLcp10_154+RLcp10_157+q[1];
    POcp10_257 = RLcp10_251+RLcp10_254+RLcp10_257+q[2];
    POcp10_357 = RLcp10_351+RLcp10_354+RLcp10_357+q[3];
    OMcp10_157 = OMcp10_153+ROcp10_452*qd[57];
    OMcp10_257 = OMcp10_253+ROcp10_552*qd[57];
    OMcp10_357 = OMcp10_353+ROcp10_652*qd[57];
    ORcp10_157 = OMcp10_253*RLcp10_357-OMcp10_353*RLcp10_257;
    ORcp10_257 = -(OMcp10_153*RLcp10_357-OMcp10_353*RLcp10_157);
    ORcp10_357 = OMcp10_153*RLcp10_257-OMcp10_253*RLcp10_157;
    VIcp10_157 = ORcp10_151+ORcp10_154+ORcp10_157+qd[1]+ROcp10_153*qd[54];
    VIcp10_257 = ORcp10_251+ORcp10_254+ORcp10_257+qd[2]+ROcp10_253*qd[54];
    VIcp10_357 = ORcp10_351+ORcp10_354+ORcp10_357+qd[3]+ROcp10_353*qd[54];
    OPcp10_157 = OPcp10_153+ROcp10_452*qdd[57]+qd[57]*(OMcp10_253*ROcp10_652-OMcp10_353*ROcp10_552);
    OPcp10_257 = OPcp10_253+ROcp10_552*qdd[57]-qd[57]*(OMcp10_153*ROcp10_652-OMcp10_353*ROcp10_452);
    OPcp10_357 = OPcp10_353+ROcp10_652*qdd[57]+qd[57]*(OMcp10_153*ROcp10_552-OMcp10_253*ROcp10_452);
    ACcp10_157 = qdd[1]+OMcp10_253*ORcp10_354+OMcp10_253*ORcp10_357+OMcp10_26*ORcp10_351-OMcp10_353*ORcp10_254-OMcp10_353*
 ORcp10_257-OMcp10_36*ORcp10_251+OPcp10_253*RLcp10_354+OPcp10_253*RLcp10_357+OPcp10_26*RLcp10_351-OPcp10_353*RLcp10_254-
 OPcp10_353*RLcp10_257-OPcp10_36*RLcp10_251+ROcp10_153*qdd[54]+(2.0)*qd[54]*(OMcp10_253*ROcp10_353-OMcp10_353*ROcp10_253);
    ACcp10_257 = qdd[2]-OMcp10_153*ORcp10_354-OMcp10_153*ORcp10_357-OMcp10_16*ORcp10_351+OMcp10_353*ORcp10_154+OMcp10_353*
 ORcp10_157+OMcp10_36*ORcp10_151-OPcp10_153*RLcp10_354-OPcp10_153*RLcp10_357-OPcp10_16*RLcp10_351+OPcp10_353*RLcp10_154+
 OPcp10_353*RLcp10_157+OPcp10_36*RLcp10_151+ROcp10_253*qdd[54]-(2.0)*qd[54]*(OMcp10_153*ROcp10_353-OMcp10_353*ROcp10_153);
    ACcp10_357 = qdd[3]+OMcp10_153*ORcp10_254+OMcp10_153*ORcp10_257+OMcp10_16*ORcp10_251-OMcp10_253*ORcp10_154-OMcp10_253*
 ORcp10_157-OMcp10_26*ORcp10_151+OPcp10_153*RLcp10_254+OPcp10_153*RLcp10_257+OPcp10_16*RLcp10_251-OPcp10_253*RLcp10_154-
 OPcp10_253*RLcp10_157-OPcp10_26*RLcp10_151+ROcp10_353*qdd[54]+(2.0)*qd[54]*(OMcp10_153*ROcp10_253-OMcp10_253*ROcp10_153);

// = = Block_1_0_0_11_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp10_157;
    sens->P[2] = POcp10_257;
    sens->P[3] = POcp10_357;
    sens->R[1][1] = ROcp10_157;
    sens->R[1][2] = ROcp10_257;
    sens->R[1][3] = ROcp10_357;
    sens->R[2][1] = ROcp10_452;
    sens->R[2][2] = ROcp10_552;
    sens->R[2][3] = ROcp10_652;
    sens->R[3][1] = ROcp10_757;
    sens->R[3][2] = ROcp10_857;
    sens->R[3][3] = ROcp10_957;
    sens->V[1] = VIcp10_157;
    sens->V[2] = VIcp10_257;
    sens->V[3] = VIcp10_357;
    sens->OM[1] = OMcp10_157;
    sens->OM[2] = OMcp10_257;
    sens->OM[3] = OMcp10_357;
    sens->A[1] = ACcp10_157;
    sens->A[2] = ACcp10_257;
    sens->A[3] = ACcp10_357;
    sens->OMP[1] = OPcp10_157;
    sens->OMP[2] = OPcp10_257;
    sens->OMP[3] = OPcp10_357;
 
// 
break;
case 12:
 


// = = Block_1_0_0_12_0_1 = = 
 
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
    OMcp11_16 = OMcp11_15+ROcp11_45*qd[6];
    OMcp11_26 = OMcp11_25+ROcp11_55*qd[6];
    OMcp11_36 = qd[4]+qd[6]*S5;
    OPcp11_16 = ROcp11_45*qdd[6]+qdd[5]*C4-qd[4]*qd[5]*S4+qd[6]*(OMcp11_25*S5-ROcp11_55*qd[4]);
    OPcp11_26 = ROcp11_55*qdd[6]+qdd[5]*S4+qd[4]*qd[5]*C4-qd[6]*(OMcp11_15*S5-ROcp11_45*qd[4]);
    OPcp11_36 = qdd[4]+qdd[6]*S5+qd[5]*qd[6]*C5;

// = = Block_1_0_0_12_0_22 = = 
 
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
    OMcp11_151 = OMcp11_16+ROcp11_76*qd[51];
    OMcp11_251 = OMcp11_26+ROcp11_86*qd[51];
    OMcp11_351 = OMcp11_36+ROcp11_96*qd[51];
    ORcp11_151 = OMcp11_26*RLcp11_351-OMcp11_36*RLcp11_251;
    ORcp11_251 = -(OMcp11_16*RLcp11_351-OMcp11_36*RLcp11_151);
    ORcp11_351 = OMcp11_16*RLcp11_251-OMcp11_26*RLcp11_151;
    OMcp11_152 = OMcp11_151+ROcp11_151*qd[52];
    OMcp11_252 = OMcp11_251+ROcp11_251*qd[52];
    OMcp11_352 = OMcp11_351+ROcp11_351*qd[52];
    OMcp11_153 = OMcp11_152+ROcp11_452*qd[53];
    OMcp11_253 = OMcp11_252+ROcp11_552*qd[53];
    OMcp11_353 = OMcp11_352+ROcp11_652*qd[53];
    OPcp11_153 = OPcp11_16+ROcp11_151*qdd[52]+ROcp11_452*qdd[53]+ROcp11_76*qdd[51]+qd[51]*(OMcp11_26*ROcp11_96-OMcp11_36*
 ROcp11_86)+qd[52]*(OMcp11_251*ROcp11_351-OMcp11_351*ROcp11_251)+qd[53]*(OMcp11_252*ROcp11_652-OMcp11_352*ROcp11_552);
    OPcp11_253 = OPcp11_26+ROcp11_251*qdd[52]+ROcp11_552*qdd[53]+ROcp11_86*qdd[51]-qd[51]*(OMcp11_16*ROcp11_96-OMcp11_36*
 ROcp11_76)-qd[52]*(OMcp11_151*ROcp11_351-OMcp11_351*ROcp11_151)-qd[53]*(OMcp11_152*ROcp11_652-OMcp11_352*ROcp11_452);
    OPcp11_353 = OPcp11_36+ROcp11_351*qdd[52]+ROcp11_652*qdd[53]+ROcp11_96*qdd[51]+qd[51]*(OMcp11_16*ROcp11_86-OMcp11_26*
 ROcp11_76)+qd[52]*(OMcp11_151*ROcp11_251-OMcp11_251*ROcp11_151)+qd[53]*(OMcp11_152*ROcp11_552-OMcp11_252*ROcp11_452);
    RLcp11_154 = ROcp11_153*q[54];
    RLcp11_254 = ROcp11_253*q[54];
    RLcp11_354 = ROcp11_353*q[54];
    ORcp11_154 = OMcp11_253*RLcp11_354-OMcp11_353*RLcp11_254;
    ORcp11_254 = -(OMcp11_153*RLcp11_354-OMcp11_353*RLcp11_154);
    ORcp11_354 = OMcp11_153*RLcp11_254-OMcp11_253*RLcp11_154;

// = = Block_1_0_0_12_0_26 = = 
 
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
    POcp11_158 = RLcp11_151+RLcp11_154+RLcp11_158+q[1];
    POcp11_258 = RLcp11_251+RLcp11_254+RLcp11_258+q[2];
    POcp11_358 = RLcp11_351+RLcp11_354+RLcp11_358+q[3];
    OMcp11_158 = OMcp11_153+ROcp11_452*qd[58];
    OMcp11_258 = OMcp11_253+ROcp11_552*qd[58];
    OMcp11_358 = OMcp11_353+ROcp11_652*qd[58];
    ORcp11_158 = OMcp11_253*RLcp11_358-OMcp11_353*RLcp11_258;
    ORcp11_258 = -(OMcp11_153*RLcp11_358-OMcp11_353*RLcp11_158);
    ORcp11_358 = OMcp11_153*RLcp11_258-OMcp11_253*RLcp11_158;
    VIcp11_158 = ORcp11_151+ORcp11_154+ORcp11_158+qd[1]+ROcp11_153*qd[54];
    VIcp11_258 = ORcp11_251+ORcp11_254+ORcp11_258+qd[2]+ROcp11_253*qd[54];
    VIcp11_358 = ORcp11_351+ORcp11_354+ORcp11_358+qd[3]+ROcp11_353*qd[54];
    OPcp11_158 = OPcp11_153+ROcp11_452*qdd[58]+qd[58]*(OMcp11_253*ROcp11_652-OMcp11_353*ROcp11_552);
    OPcp11_258 = OPcp11_253+ROcp11_552*qdd[58]-qd[58]*(OMcp11_153*ROcp11_652-OMcp11_353*ROcp11_452);
    OPcp11_358 = OPcp11_353+ROcp11_652*qdd[58]+qd[58]*(OMcp11_153*ROcp11_552-OMcp11_253*ROcp11_452);
    ACcp11_158 = qdd[1]+OMcp11_253*ORcp11_354+OMcp11_253*ORcp11_358+OMcp11_26*ORcp11_351-OMcp11_353*ORcp11_254-OMcp11_353*
 ORcp11_258-OMcp11_36*ORcp11_251+OPcp11_253*RLcp11_354+OPcp11_253*RLcp11_358+OPcp11_26*RLcp11_351-OPcp11_353*RLcp11_254-
 OPcp11_353*RLcp11_258-OPcp11_36*RLcp11_251+ROcp11_153*qdd[54]+(2.0)*qd[54]*(OMcp11_253*ROcp11_353-OMcp11_353*ROcp11_253);
    ACcp11_258 = qdd[2]-OMcp11_153*ORcp11_354-OMcp11_153*ORcp11_358-OMcp11_16*ORcp11_351+OMcp11_353*ORcp11_154+OMcp11_353*
 ORcp11_158+OMcp11_36*ORcp11_151-OPcp11_153*RLcp11_354-OPcp11_153*RLcp11_358-OPcp11_16*RLcp11_351+OPcp11_353*RLcp11_154+
 OPcp11_353*RLcp11_158+OPcp11_36*RLcp11_151+ROcp11_253*qdd[54]-(2.0)*qd[54]*(OMcp11_153*ROcp11_353-OMcp11_353*ROcp11_153);
    ACcp11_358 = qdd[3]+OMcp11_153*ORcp11_254+OMcp11_153*ORcp11_258+OMcp11_16*ORcp11_251-OMcp11_253*ORcp11_154-OMcp11_253*
 ORcp11_158-OMcp11_26*ORcp11_151+OPcp11_153*RLcp11_254+OPcp11_153*RLcp11_258+OPcp11_16*RLcp11_251-OPcp11_253*RLcp11_154-
 OPcp11_253*RLcp11_158-OPcp11_26*RLcp11_151+ROcp11_353*qdd[54]+(2.0)*qd[54]*(OMcp11_153*ROcp11_253-OMcp11_253*ROcp11_153);

// = = Block_1_0_0_12_1_0 = = 
 
// Symbolic Outputs  

    sens->P[1] = POcp11_158;
    sens->P[2] = POcp11_258;
    sens->P[3] = POcp11_358;
    sens->R[1][1] = ROcp11_158;
    sens->R[1][2] = ROcp11_258;
    sens->R[1][3] = ROcp11_358;
    sens->R[2][1] = ROcp11_452;
    sens->R[2][2] = ROcp11_552;
    sens->R[2][3] = ROcp11_652;
    sens->R[3][1] = ROcp11_758;
    sens->R[3][2] = ROcp11_858;
    sens->R[3][3] = ROcp11_958;
    sens->V[1] = VIcp11_158;
    sens->V[2] = VIcp11_258;
    sens->V[3] = VIcp11_358;
    sens->OM[1] = OMcp11_158;
    sens->OM[2] = OMcp11_258;
    sens->OM[3] = OMcp11_358;
    sens->A[1] = ACcp11_158;
    sens->A[2] = ACcp11_258;
    sens->A[3] = ACcp11_358;
    sens->OMP[1] = OPcp11_158;
    sens->OMP[2] = OPcp11_258;
    sens->OMP[3] = OPcp11_358;

break;
default:
break;
}


// ====== END Task 1 ====== 


}
 

