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
//	==> Function : F 1 : Direct Dynamics (Semi-Explicit formulation) : RNEA
//	==> Flops complexity : 9928
//
//	==> Generation Time :  0.140 seconds
//	==> Post-Processing :  0.210 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_dirdyna(double **M,double *c,
MbsData *s, double tsim)

// double M[53][53];
// double c[53];
{ 
 
#include "mbs_dirdyna_Car.h" 
#define q s->q 
#define qd s->qd 
#define qdd s->qdd 
 
 

// === begin imp_aux === 

// === end imp_aux === 

// ===== BEGIN task 0 ===== 

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

// = = Block_0_0_0_0_0_25 = = 
 
// Augmented Joint Position Vectors   

  Dz533 = q[53]+s->dpt[3][66];

// = = Block_0_1_0_0_0_1 = = 
 
// Forward Kinematics 

  OM35 = qd[4]*C5;
  OpF25 = qd[4]*qd[5]*C5;
  OpF35 = -qd[4]*qd[5]*S5;
  AlF25 = -s->g[3]*S5;
  AlF35 = -s->g[3]*C5;
  AlM25_1 = -S4*C5;
  AlM35_1 = S4*S5;
  AlM25_2 = C4*C5;
  AlM35_2 = -C4*S5;
  OM16 = qd[5]*C6-OM35*S6;
  OM26 = qd[6]+qd[4]*S5;
  OM36 = qd[5]*S6+OM35*C6;
  OpF16 = -(qd[6]*OM35*C6+S6*(OpF35+qd[5]*qd[6]));
  OpF36 = -(qd[6]*OM35*S6-C6*(OpF35+qd[5]*qd[6]));
  BS16 = -(OM26*OM26+OM36*OM36);
  BS26 = OM16*OM26;
  BS36 = OM16*OM36;
  BS56 = -(OM16*OM16+OM36*OM36);
  BS66 = OM26*OM36;
  BS96 = -(OM16*OM16+OM26*OM26);
  BeF26 = BS26-OpF36;
  BeF36 = BS36+OpF25;
  BeF46 = BS26+OpF36;
  BeF66 = BS66-OpF16;
  BeF76 = BS36-OpF25;
  BeF86 = BS66+OpF16;
  AlF16 = -AlF35*S6;
  AlF36 = AlF35*C6;
  AlM16_1 = -(AlM35_1*S6-C4*C6);
  AlM36_1 = AlM35_1*C6+C4*S6;
  AlM16_2 = -(AlM35_2*S6-S4*C6);
  AlM36_2 = AlM35_2*C6+S4*S6;
  AlM16_3 = -C5*S6;
  AlM36_3 = C5*C6;
  OpM16_4 = -C5*S6;
  OpM36_4 = C5*C6;

// = = Block_0_1_0_1_0_3 = = 
 
// Forward Kinematics 

  OM18 = qd[8]+OM16;
  OM28 = OM26*C8+OM36*S8;
  OM38 = -(OM26*S8-OM36*C8);
  OpF28 = C8*(OpF25+qd[8]*OM36)+S8*(OpF36-qd[8]*OM26);
  OpF38 = C8*(OpF36-qd[8]*OM26)-S8*(OpF25+qd[8]*OM36);
  BeF28 = OM18*OM28-OpF38;
  BeF88 = OpF16+OM28*OM38;
  AlF18 = AlF16+BS16*s->dpt[1][1]+BeF26*s->dpt[2][1]+BeF36*s->dpt[3][1];
  AlF38 = C8*(AlF36+BS96*s->dpt[3][1]+BeF76*s->dpt[1][1]+BeF86*s->dpt[2][1])-S8*(AlF25+BS56*s->dpt[2][1]+BeF46*
 s->dpt[1][1]+BeF66*s->dpt[3][1]);
  AlM28_1 = AlM25_1*C8+AlM36_1*S8;
  AlM38_1 = -(AlM25_1*S8-AlM36_1*C8);
  AlM28_2 = AlM25_2*C8+AlM36_2*S8;
  AlM38_2 = -(AlM25_2*S8-AlM36_2*C8);
  AlM28_3 = AlM36_3*S8+S5*C8;
  AlM38_3 = AlM36_3*C8-S5*S8;
  OpM28_4 = OpM36_4*S8+S5*C8;
  OpM38_4 = OpM36_4*C8-S5*S8;
  AlM18_4 = -(OpM36_4*s->dpt[2][1]-s->dpt[3][1]*S5);
  AlM28_4 = -(C8*(OpM16_4*s->dpt[3][1]-OpM36_4*s->dpt[1][1])-S8*(OpM16_4*s->dpt[2][1]-s->dpt[1][1]*S5));
  AlM38_4 = C8*(OpM16_4*s->dpt[2][1]-s->dpt[1][1]*S5)+S8*(OpM16_4*s->dpt[3][1]-OpM36_4*s->dpt[1][1]);
  OpM28_5 = S6*S8;
  OpM38_5 = S6*C8;
  AlM18_5 = -s->dpt[2][1]*S6;
  AlM28_5 = s->dpt[2][1]*C6*S8+C8*(s->dpt[1][1]*S6-s->dpt[3][1]*C6);
  AlM38_5 = s->dpt[2][1]*C6*C8-S8*(s->dpt[1][1]*S6-s->dpt[3][1]*C6);
  AlM28_6 = -s->dpt[1][1]*S8;
  AlM38_6 = -s->dpt[1][1]*C8;
  OM29 = qd[9]+OM28;
  OM39 = OM18*S9+OM38*C9;
  OpF19 = C9*(OpF16-qd[9]*OM38)-S9*(OpF38+qd[9]*OM18);
  OpF39 = C9*(OpF38+qd[9]*OM18)+S9*(OpF16-qd[9]*OM38);
  AlF19 = C9*(AlF18+BeF28*s->dpt[2][18])-S9*(AlF38+BeF88*s->dpt[2][18]);
  AlF29 = C8*(AlF25+BS56*s->dpt[2][1]+BeF46*s->dpt[1][1]+BeF66*s->dpt[3][1])+S8*(AlF36+BS96*s->dpt[3][1]+BeF76*
 s->dpt[1][1]+BeF86*s->dpt[2][1])-s->dpt[2][18]*(OM18*OM18+OM38*OM38);
  AlF39 = C9*(AlF38+BeF88*s->dpt[2][18])+S9*(AlF18+BeF28*s->dpt[2][18]);
  AlM19_1 = AlM16_1*C9-AlM38_1*S9;
  AlM39_1 = AlM16_1*S9+AlM38_1*C9;
  AlM19_2 = AlM16_2*C9-AlM38_2*S9;
  AlM39_2 = AlM16_2*S9+AlM38_2*C9;
  AlM19_3 = AlM16_3*C9-AlM38_3*S9;
  AlM39_3 = AlM16_3*S9+AlM38_3*C9;
  OpM19_4 = OpM16_4*C9-OpM38_4*S9;
  OpM39_4 = OpM16_4*S9+OpM38_4*C9;
  AlM19_4 = C9*(AlM18_4-OpM38_4*s->dpt[2][18])-S9*(AlM38_4+OpM16_4*s->dpt[2][18]);
  AlM39_4 = C9*(AlM38_4+OpM16_4*s->dpt[2][18])+S9*(AlM18_4-OpM38_4*s->dpt[2][18]);
  OpM19_5 = -(OpM38_5*S9-C6*C9);
  OpM39_5 = OpM38_5*C9+C6*S9;
  AlM19_5 = C9*(AlM18_5-OpM38_5*s->dpt[2][18])-S9*(AlM38_5+s->dpt[2][18]*C6);
  AlM39_5 = C9*(AlM38_5+s->dpt[2][18]*C6)+S9*(AlM18_5-OpM38_5*s->dpt[2][18]);
  OpM19_6 = S8*S9;
  OpM39_6 = -S8*C9;
  AlM19_6 = -(AlM38_6*S9-C9*(s->dpt[3][1]+s->dpt[2][18]*S8));
  AlM39_6 = AlM38_6*C9+S9*(s->dpt[3][1]+s->dpt[2][18]*S8);
  AlM19_8 = -s->dpt[2][18]*S9;
  AlM39_8 = s->dpt[2][18]*C9;
  OM110 = qd[10]+OM18*C9-OM38*S9;
  OM210 = OM29*C10+OM39*S10;
  OpF210 = C10*(OpF28+qd[10]*OM39)+S10*(OpF39-qd[10]*OM29);
  OpF310 = C10*(OpF39-qd[10]*OM29)-S10*(OpF28+qd[10]*OM39);
  AlF210 = AlF29*C10+AlF39*S10;
  AlF310 = -(AlF29*S10-AlF39*C10);
  AlM210_1 = AlM28_1*C10+AlM39_1*S10;
  AlM310_1 = -(AlM28_1*S10-AlM39_1*C10);
  AlM210_2 = AlM28_2*C10+AlM39_2*S10;
  AlM310_2 = -(AlM28_2*S10-AlM39_2*C10);
  AlM210_3 = AlM28_3*C10+AlM39_3*S10;
  AlM310_3 = -(AlM28_3*S10-AlM39_3*C10);
  OpM210_4 = OpM28_4*C10+OpM39_4*S10;
  OpM310_4 = -(OpM28_4*S10-OpM39_4*C10);
  AlM210_4 = AlM28_4*C10+AlM39_4*S10;
  AlM310_4 = -(AlM28_4*S10-AlM39_4*C10);
  OpM210_5 = OpM28_5*C10+OpM39_5*S10;
  OpM310_5 = -(OpM28_5*S10-OpM39_5*C10);
  AlM210_5 = AlM28_5*C10+AlM39_5*S10;
  AlM310_5 = -(AlM28_5*S10-AlM39_5*C10);
  OpM210_6 = OpM39_6*S10+C10*C8;
  OpM310_6 = OpM39_6*C10-S10*C8;
  AlM210_6 = AlM28_6*C10+AlM39_6*S10;
  AlM310_6 = -(AlM28_6*S10-AlM39_6*C10);
  OpM210_8 = S10*S9;
  OpM310_8 = C10*S9;
  AlM210_8 = AlM39_8*S10;
  AlM310_8 = AlM39_8*C10;
  OM111 = OM110*C11+OM210*S11;
  OM211 = -(OM110*S11-OM210*C11);
  OM311 = qd[11]-OM29*S10+OM39*C10;
  OpF111 = C11*(OpF19+qd[11]*OM210)+S11*(OpF210-qd[11]*OM110);
  OpF211 = C11*(OpF210-qd[11]*OM110)-S11*(OpF19+qd[11]*OM210);
  BS911 = -(OM111*OM111+OM211*OM211);
  BeF311 = OpF211+OM111*OM311;
  AlF111 = AlF19*C11+AlF210*S11;
  AlF211 = -(AlF19*S11-AlF210*C11);
  AlM111_1 = AlM19_1*C11+AlM210_1*S11;
  AlM211_1 = -(AlM19_1*S11-AlM210_1*C11);
  AlM111_2 = AlM19_2*C11+AlM210_2*S11;
  AlM211_2 = -(AlM19_2*S11-AlM210_2*C11);
  AlM111_3 = AlM19_3*C11+AlM210_3*S11;
  AlM211_3 = -(AlM19_3*S11-AlM210_3*C11);
  OpM111_4 = OpM19_4*C11+OpM210_4*S11;
  OpM211_4 = -(OpM19_4*S11-OpM210_4*C11);
  AlM111_4 = AlM19_4*C11+AlM210_4*S11;
  AlM211_4 = -(AlM19_4*S11-AlM210_4*C11);
  OpM111_5 = OpM19_5*C11+OpM210_5*S11;
  OpM211_5 = -(OpM19_5*S11-OpM210_5*C11);
  AlM111_5 = AlM19_5*C11+AlM210_5*S11;
  AlM211_5 = -(AlM19_5*S11-AlM210_5*C11);
  OpM111_6 = OpM19_6*C11+OpM210_6*S11;
  OpM211_6 = -(OpM19_6*S11-OpM210_6*C11);
  AlM111_6 = AlM19_6*C11+AlM210_6*S11;
  AlM211_6 = -(AlM19_6*S11-AlM210_6*C11);
  OpM111_8 = OpM210_8*S11+C11*C9;
  OpM211_8 = OpM210_8*C11-S11*C9;
  AlM111_8 = AlM19_8*C11+AlM210_8*S11;
  AlM211_8 = -(AlM19_8*S11-AlM210_8*C11);
  OpM111_9 = C10*S11;
  OpM211_9 = C10*C11;
  OM112 = OM111*C12-OM311*S12;
  OM212 = qd[12]+OM211;
  OM312 = OM111*S12+OM311*C12;

// = = Block_0_1_0_1_0_4 = = 
 
// Forward Kinematics 

  OM113 = qd[13]+OM16;
  OM213 = OM26*C13+OM36*S13;
  OM313 = -(OM26*S13-OM36*C13);
  OpF213 = C13*(OpF25+qd[13]*OM36)+S13*(OpF36-qd[13]*OM26);
  OpF313 = C13*(OpF36-qd[13]*OM26)-S13*(OpF25+qd[13]*OM36);
  BeF213 = OM113*OM213-OpF313;
  BeF813 = OpF16+OM213*OM313;
  AlF113 = AlF16+BS16*s->dpt[1][2]+BeF26*s->dpt[2][2]+BeF36*s->dpt[3][2];
  AlF313 = C13*(AlF36+BS96*s->dpt[3][2]+BeF76*s->dpt[1][2]+BeF86*s->dpt[2][2])-S13*(AlF25+BS56*s->dpt[2][2]+BeF46*
 s->dpt[1][2]+BeF66*s->dpt[3][2]);
  AlM213_1 = AlM25_1*C13+AlM36_1*S13;
  AlM313_1 = -(AlM25_1*S13-AlM36_1*C13);
  AlM213_2 = AlM25_2*C13+AlM36_2*S13;
  AlM313_2 = -(AlM25_2*S13-AlM36_2*C13);
  AlM213_3 = AlM36_3*S13+C13*S5;
  AlM313_3 = AlM36_3*C13-S13*S5;
  OpM213_4 = OpM36_4*S13+C13*S5;
  OpM313_4 = OpM36_4*C13-S13*S5;
  AlM113_4 = -(OpM36_4*s->dpt[2][2]-s->dpt[3][2]*S5);
  AlM213_4 = -(C13*(OpM16_4*s->dpt[3][2]-OpM36_4*s->dpt[1][2])-S13*(OpM16_4*s->dpt[2][2]-s->dpt[1][2]*S5));
  AlM313_4 = C13*(OpM16_4*s->dpt[2][2]-s->dpt[1][2]*S5)+S13*(OpM16_4*s->dpt[3][2]-OpM36_4*s->dpt[1][2]);
  OpM213_5 = S13*S6;
  OpM313_5 = C13*S6;
  AlM113_5 = -s->dpt[2][2]*S6;
  AlM213_5 = s->dpt[2][2]*S13*C6+C13*(s->dpt[1][2]*S6-s->dpt[3][2]*C6);
  AlM313_5 = s->dpt[2][2]*C13*C6-S13*(s->dpt[1][2]*S6-s->dpt[3][2]*C6);
  AlM213_6 = -s->dpt[1][2]*S13;
  AlM313_6 = -s->dpt[1][2]*C13;
  OM214 = qd[14]+OM213;
  OM314 = OM113*S14+OM313*C14;
  OpF114 = C14*(OpF16-qd[14]*OM313)-S14*(OpF313+qd[14]*OM113);
  OpF314 = C14*(OpF313+qd[14]*OM113)+S14*(OpF16-qd[14]*OM313);
  AlF114 = C14*(AlF113+BeF213*s->dpt[2][24])-S14*(AlF313+BeF813*s->dpt[2][24]);
  AlF214 = C13*(AlF25+BS56*s->dpt[2][2]+BeF46*s->dpt[1][2]+BeF66*s->dpt[3][2])+S13*(AlF36+BS96*s->dpt[3][2]+BeF76*
 s->dpt[1][2]+BeF86*s->dpt[2][2])-s->dpt[2][24]*(OM113*OM113+OM313*OM313);
  AlF314 = C14*(AlF313+BeF813*s->dpt[2][24])+S14*(AlF113+BeF213*s->dpt[2][24]);
  AlM114_1 = AlM16_1*C14-AlM313_1*S14;
  AlM314_1 = AlM16_1*S14+AlM313_1*C14;
  AlM114_2 = AlM16_2*C14-AlM313_2*S14;
  AlM314_2 = AlM16_2*S14+AlM313_2*C14;
  AlM114_3 = AlM16_3*C14-AlM313_3*S14;
  AlM314_3 = AlM16_3*S14+AlM313_3*C14;
  OpM114_4 = OpM16_4*C14-OpM313_4*S14;
  OpM314_4 = OpM16_4*S14+OpM313_4*C14;
  AlM114_4 = C14*(AlM113_4-OpM313_4*s->dpt[2][24])-S14*(AlM313_4+OpM16_4*s->dpt[2][24]);
  AlM314_4 = C14*(AlM313_4+OpM16_4*s->dpt[2][24])+S14*(AlM113_4-OpM313_4*s->dpt[2][24]);
  OpM114_5 = -(OpM313_5*S14-C14*C6);
  OpM314_5 = OpM313_5*C14+S14*C6;
  AlM114_5 = C14*(AlM113_5-OpM313_5*s->dpt[2][24])-S14*(AlM313_5+s->dpt[2][24]*C6);
  AlM314_5 = C14*(AlM313_5+s->dpt[2][24]*C6)+S14*(AlM113_5-OpM313_5*s->dpt[2][24]);
  OpM114_6 = S13*S14;
  OpM314_6 = -S13*C14;
  AlM114_6 = -(AlM313_6*S14-C14*(s->dpt[3][2]+s->dpt[2][24]*S13));
  AlM314_6 = AlM313_6*C14+S14*(s->dpt[3][2]+s->dpt[2][24]*S13);
  AlM114_13 = -s->dpt[2][24]*S14;
  AlM314_13 = s->dpt[2][24]*C14;
  OM115 = qd[15]+OM113*C14-OM313*S14;
  OM215 = OM214*C15+OM314*S15;
  OpF215 = C15*(OpF213+qd[15]*OM314)+S15*(OpF314-qd[15]*OM214);
  OpF315 = C15*(OpF314-qd[15]*OM214)-S15*(OpF213+qd[15]*OM314);
  AlF215 = AlF214*C15+AlF314*S15;
  AlF315 = -(AlF214*S15-AlF314*C15);
  AlM215_1 = AlM213_1*C15+AlM314_1*S15;
  AlM315_1 = -(AlM213_1*S15-AlM314_1*C15);
  AlM215_2 = AlM213_2*C15+AlM314_2*S15;
  AlM315_2 = -(AlM213_2*S15-AlM314_2*C15);
  AlM215_3 = AlM213_3*C15+AlM314_3*S15;
  AlM315_3 = -(AlM213_3*S15-AlM314_3*C15);
  OpM215_4 = OpM213_4*C15+OpM314_4*S15;
  OpM315_4 = -(OpM213_4*S15-OpM314_4*C15);
  AlM215_4 = AlM213_4*C15+AlM314_4*S15;
  AlM315_4 = -(AlM213_4*S15-AlM314_4*C15);
  OpM215_5 = OpM213_5*C15+OpM314_5*S15;
  OpM315_5 = -(OpM213_5*S15-OpM314_5*C15);
  AlM215_5 = AlM213_5*C15+AlM314_5*S15;
  AlM315_5 = -(AlM213_5*S15-AlM314_5*C15);
  OpM215_6 = OpM314_6*S15+C13*C15;
  OpM315_6 = OpM314_6*C15-C13*S15;
  AlM215_6 = AlM213_6*C15+AlM314_6*S15;
  AlM315_6 = -(AlM213_6*S15-AlM314_6*C15);
  OpM215_13 = S14*S15;
  OpM315_13 = S14*C15;
  AlM215_13 = AlM314_13*S15;
  AlM315_13 = AlM314_13*C15;
  OM116 = OM115*C16+OM215*S16;
  OM216 = -(OM115*S16-OM215*C16);
  OM316 = qd[16]-OM214*S15+OM314*C15;
  OpF116 = C16*(OpF114+qd[16]*OM215)+S16*(OpF215-qd[16]*OM115);
  OpF216 = C16*(OpF215-qd[16]*OM115)-S16*(OpF114+qd[16]*OM215);
  BS916 = -(OM116*OM116+OM216*OM216);
  BeF316 = OpF216+OM116*OM316;
  AlF116 = AlF114*C16+AlF215*S16;
  AlF216 = -(AlF114*S16-AlF215*C16);
  AlM116_1 = AlM114_1*C16+AlM215_1*S16;
  AlM216_1 = -(AlM114_1*S16-AlM215_1*C16);
  AlM116_2 = AlM114_2*C16+AlM215_2*S16;
  AlM216_2 = -(AlM114_2*S16-AlM215_2*C16);
  AlM116_3 = AlM114_3*C16+AlM215_3*S16;
  AlM216_3 = -(AlM114_3*S16-AlM215_3*C16);
  OpM116_4 = OpM114_4*C16+OpM215_4*S16;
  OpM216_4 = -(OpM114_4*S16-OpM215_4*C16);
  AlM116_4 = AlM114_4*C16+AlM215_4*S16;
  AlM216_4 = -(AlM114_4*S16-AlM215_4*C16);
  OpM116_5 = OpM114_5*C16+OpM215_5*S16;
  OpM216_5 = -(OpM114_5*S16-OpM215_5*C16);
  AlM116_5 = AlM114_5*C16+AlM215_5*S16;
  AlM216_5 = -(AlM114_5*S16-AlM215_5*C16);
  OpM116_6 = OpM114_6*C16+OpM215_6*S16;
  OpM216_6 = -(OpM114_6*S16-OpM215_6*C16);
  AlM116_6 = AlM114_6*C16+AlM215_6*S16;
  AlM216_6 = -(AlM114_6*S16-AlM215_6*C16);
  OpM116_13 = OpM215_13*S16+C14*C16;
  OpM216_13 = OpM215_13*C16-C14*S16;
  AlM116_13 = AlM114_13*C16+AlM215_13*S16;
  AlM216_13 = -(AlM114_13*S16-AlM215_13*C16);
  OpM116_14 = C15*S16;
  OpM216_14 = C15*C16;
  OM117 = OM116*C17-OM316*S17;
  OM217 = qd[17]+OM216;
  OM317 = OM116*S17+OM316*C17;

// = = Block_0_1_0_1_0_15 = = 
 
// Trigonometric Variables  

  S33p34 = C33*S34+S33*C34;
  C33p34 = C33*C34-S33*S34;
 
// Forward Kinematics 

  OM133 = qd[33]+OM16;
  OM233 = OM26*C33+OM36*S33;
  OM333 = -(OM26*S33-OM36*C33);
  OpF233 = C33*(OpF25+qd[33]*OM36)+S33*(OpF36-qd[33]*OM26);
  OpF333 = C33*(OpF36-qd[33]*OM26)-S33*(OpF25+qd[33]*OM36);
  BS533 = -(OM133*OM133+OM333*OM333);
  BeF833 = OpF16+OM233*OM333;
  AlF233 = C33*(AlF25+BS56*s->dpt[2][12]+BeF46*s->dpt[1][12]+BeF66*s->dpt[3][12])+S33*(AlF36+BS96*s->dpt[3][12]+BeF76*
 s->dpt[1][12]+BeF86*s->dpt[2][12]);
  AlF333 = C33*(AlF36+BS96*s->dpt[3][12]+BeF76*s->dpt[1][12]+BeF86*s->dpt[2][12])-S33*(AlF25+BS56*s->dpt[2][12]+BeF46*
 s->dpt[1][12]+BeF66*s->dpt[3][12]);
  AlM233_1 = AlM25_1*C33+AlM36_1*S33;
  AlM333_1 = -(AlM25_1*S33-AlM36_1*C33);
  AlM233_2 = AlM25_2*C33+AlM36_2*S33;
  AlM333_2 = -(AlM25_2*S33-AlM36_2*C33);
  AlM233_3 = AlM36_3*S33+C33*S5;
  AlM333_3 = AlM36_3*C33-S33*S5;
  OpM233_4 = OpM36_4*S33+C33*S5;
  OpM333_4 = OpM36_4*C33-S33*S5;
  AlM233_4 = -(C33*(OpM16_4*s->dpt[3][12]-OpM36_4*s->dpt[1][12])-S33*(OpM16_4*s->dpt[2][12]-s->dpt[1][12]*S5));
  AlM333_4 = C33*(OpM16_4*s->dpt[2][12]-s->dpt[1][12]*S5)+S33*(OpM16_4*s->dpt[3][12]-OpM36_4*s->dpt[1][12]);
  AlM233_5 = s->dpt[2][12]*S33*C6+C33*(s->dpt[1][12]*S6-s->dpt[3][12]*C6);
  AlM333_5 = s->dpt[2][12]*C33*C6-S33*(s->dpt[1][12]*S6-s->dpt[3][12]*C6);
  OM134 = qd[34]+OM133;
  OM234 = OM233*C34+OM333*S34;
  OM334 = -(OM233*S34-OM333*C34);
  OpF234 = C34*(OpF233+qd[34]*OM333)+S34*(OpF333-qd[34]*OM233);
  OpF334 = C34*(OpF333-qd[34]*OM233)-S34*(OpF233+qd[34]*OM333);
  BS934 = -(OM134*OM134+OM234*OM234);
  BeF334 = OpF234+OM134*OM334;
  AlF134 = AlF16+BS16*s->dpt[1][12]+BeF26*s->dpt[2][12]+BeF36*s->dpt[3][12]-s->dpt[2][47]*(OpF333-OM133*OM233);
  AlF234 = C34*(AlF233+BS533*s->dpt[2][47])+S34*(AlF333+BeF833*s->dpt[2][47]);
  AlF334 = C34*(AlF333+BeF833*s->dpt[2][47])-S34*(AlF233+BS533*s->dpt[2][47]);
  AlM234_1 = AlM233_1*C34+AlM333_1*S34;
  AlM334_1 = -(AlM233_1*S34-AlM333_1*C34);
  AlM234_2 = AlM233_2*C34+AlM333_2*S34;
  AlM334_2 = -(AlM233_2*S34-AlM333_2*C34);
  AlM234_3 = AlM233_3*C34+AlM333_3*S34;
  AlM334_3 = -(AlM233_3*S34-AlM333_3*C34);
  OpM234_4 = OpM233_4*C34+OpM333_4*S34;
  OpM334_4 = -(OpM233_4*S34-OpM333_4*C34);
  AlM134_4 = -(OpM333_4*s->dpt[2][47]+OpM36_4*s->dpt[2][12]-s->dpt[3][12]*S5);
  AlM234_4 = AlM233_4*C34+S34*(AlM333_4+OpM16_4*s->dpt[2][47]);
  AlM334_4 = -(AlM233_4*S34-C34*(AlM333_4+OpM16_4*s->dpt[2][47]));
  OpM234_5 = S33p34*S6;
  OpM334_5 = C33p34*S6;
  AlM134_5 = -S6*(s->dpt[2][12]+s->dpt[2][47]*C33);
  AlM234_5 = AlM233_5*C34+S34*(AlM333_5+s->dpt[2][47]*C6);
  AlM334_5 = -(AlM233_5*S34-C34*(AlM333_5+s->dpt[2][47]*C6));
  AlM134_6 = s->dpt[3][12]+s->dpt[2][47]*S33;
  AlM234_6 = -s->dpt[1][12]*S33p34;
  AlM334_6 = -s->dpt[1][12]*C33p34;
  AlM234_33 = s->dpt[2][47]*S34;
  AlM334_33 = s->dpt[2][47]*C34;
  OM135 = OM134*C35-OM334*S35;
  OM235 = qd[35]+OM234;
  OM335 = OM134*S35+OM334*C35;

// = = Block_0_1_0_1_0_16 = = 
 
// Trigonometric Variables  

  S36p37 = C36*S37+S36*C37;
  C36p37 = C36*C37-S36*S37;
 
// Forward Kinematics 

  OM136 = qd[36]+OM16;
  OM236 = OM26*C36+OM36*S36;
  OM336 = -(OM26*S36-OM36*C36);
  OpF236 = C36*(OpF25+qd[36]*OM36)+S36*(OpF36-qd[36]*OM26);
  OpF336 = C36*(OpF36-qd[36]*OM26)-S36*(OpF25+qd[36]*OM36);
  BS536 = -(OM136*OM136+OM336*OM336);
  BeF836 = OpF16+OM236*OM336;
  AlF236 = C36*(AlF25+BS56*s->dpt[2][13]+BeF46*s->dpt[1][13]+BeF66*s->dpt[3][13])+S36*(AlF36+BS96*s->dpt[3][13]+BeF76*
 s->dpt[1][13]+BeF86*s->dpt[2][13]);
  AlF336 = C36*(AlF36+BS96*s->dpt[3][13]+BeF76*s->dpt[1][13]+BeF86*s->dpt[2][13])-S36*(AlF25+BS56*s->dpt[2][13]+BeF46*
 s->dpt[1][13]+BeF66*s->dpt[3][13]);
  AlM236_1 = AlM25_1*C36+AlM36_1*S36;
  AlM336_1 = -(AlM25_1*S36-AlM36_1*C36);
  AlM236_2 = AlM25_2*C36+AlM36_2*S36;
  AlM336_2 = -(AlM25_2*S36-AlM36_2*C36);
  AlM236_3 = AlM36_3*S36+C36*S5;
  AlM336_3 = AlM36_3*C36-S36*S5;
  OpM236_4 = OpM36_4*S36+C36*S5;
  OpM336_4 = OpM36_4*C36-S36*S5;
  AlM236_4 = -(C36*(OpM16_4*s->dpt[3][13]-OpM36_4*s->dpt[1][13])-S36*(OpM16_4*s->dpt[2][13]-s->dpt[1][13]*S5));
  AlM336_4 = C36*(OpM16_4*s->dpt[2][13]-s->dpt[1][13]*S5)+S36*(OpM16_4*s->dpt[3][13]-OpM36_4*s->dpt[1][13]);
  AlM236_5 = s->dpt[2][13]*S36*C6+C36*(s->dpt[1][13]*S6-s->dpt[3][13]*C6);
  AlM336_5 = s->dpt[2][13]*C36*C6-S36*(s->dpt[1][13]*S6-s->dpt[3][13]*C6);
  OM137 = qd[37]+OM136;
  OM237 = OM236*C37+OM336*S37;
  OM337 = -(OM236*S37-OM336*C37);
  OpF237 = C37*(OpF236+qd[37]*OM336)+S37*(OpF336-qd[37]*OM236);
  OpF337 = C37*(OpF336-qd[37]*OM236)-S37*(OpF236+qd[37]*OM336);
  BS937 = -(OM137*OM137+OM237*OM237);
  BeF337 = OpF237+OM137*OM337;
  AlF137 = AlF16+BS16*s->dpt[1][13]+BeF26*s->dpt[2][13]+BeF36*s->dpt[3][13]-s->dpt[2][52]*(OpF336-OM136*OM236);
  AlF237 = C37*(AlF236+BS536*s->dpt[2][52])+S37*(AlF336+BeF836*s->dpt[2][52]);
  AlF337 = C37*(AlF336+BeF836*s->dpt[2][52])-S37*(AlF236+BS536*s->dpt[2][52]);
  AlM237_1 = AlM236_1*C37+AlM336_1*S37;
  AlM337_1 = -(AlM236_1*S37-AlM336_1*C37);
  AlM237_2 = AlM236_2*C37+AlM336_2*S37;
  AlM337_2 = -(AlM236_2*S37-AlM336_2*C37);
  AlM237_3 = AlM236_3*C37+AlM336_3*S37;
  AlM337_3 = -(AlM236_3*S37-AlM336_3*C37);
  OpM237_4 = OpM236_4*C37+OpM336_4*S37;
  OpM337_4 = -(OpM236_4*S37-OpM336_4*C37);
  AlM137_4 = -(OpM336_4*s->dpt[2][52]+OpM36_4*s->dpt[2][13]-s->dpt[3][13]*S5);
  AlM237_4 = AlM236_4*C37+S37*(AlM336_4+OpM16_4*s->dpt[2][52]);
  AlM337_4 = -(AlM236_4*S37-C37*(AlM336_4+OpM16_4*s->dpt[2][52]));
  OpM237_5 = S36p37*S6;
  OpM337_5 = C36p37*S6;
  AlM137_5 = -S6*(s->dpt[2][13]+s->dpt[2][52]*C36);
  AlM237_5 = AlM236_5*C37+S37*(AlM336_5+s->dpt[2][52]*C6);
  AlM337_5 = -(AlM236_5*S37-C37*(AlM336_5+s->dpt[2][52]*C6));
  AlM137_6 = s->dpt[3][13]+s->dpt[2][52]*S36;
  AlM237_6 = -s->dpt[1][13]*S36p37;
  AlM337_6 = -s->dpt[1][13]*C36p37;
  AlM237_36 = s->dpt[2][52]*S37;
  AlM337_36 = s->dpt[2][52]*C37;
  OM138 = OM137*C38-OM337*S38;
  OM238 = qd[38]+OM237;
  OM338 = OM137*S38+OM337*C38;

// = = Block_0_1_0_1_0_20 = = 
 
// Forward Kinematics 

  OM245 = -(OM16*S45-OM26*C45);
  OM345 = qd[45]+OM36;
  OpF145 = C45*(OpF16+qd[45]*OM26)+S45*(OpF25-qd[45]*OM16);
  OpF245 = C45*(OpF25-qd[45]*OM16)-S45*(OpF16+qd[45]*OM26);
  AlF145 = C45*(AlF16+BS16*s->dpt[1][17]+BeF36*s->dpt[3][17])+S45*(AlF25+BeF46*s->dpt[1][17]+BeF66*s->dpt[3][17]);
  AlF245 = C45*(AlF25+BeF46*s->dpt[1][17]+BeF66*s->dpt[3][17])-S45*(AlF16+BS16*s->dpt[1][17]+BeF36*s->dpt[3][17]);
  AlF345 = AlF36+BS96*s->dpt[3][17]+BeF76*s->dpt[1][17];
  AlM145_1 = AlM16_1*C45+AlM25_1*S45;
  AlM245_1 = -(AlM16_1*S45-AlM25_1*C45);
  AlM145_2 = AlM16_2*C45+AlM25_2*S45;
  AlM245_2 = -(AlM16_2*S45-AlM25_2*C45);
  AlM145_3 = AlM16_3*C45+S45*S5;
  AlM245_3 = -(AlM16_3*S45-C45*S5);
  OpM145_4 = OpM16_4*C45+S45*S5;
  OpM245_4 = -(OpM16_4*S45-C45*S5);
  AlM145_4 = s->dpt[3][17]*C45*S5-S45*(OpM16_4*s->dpt[3][17]-OpM36_4*s->dpt[1][17]);
  AlM245_4 = -(s->dpt[3][17]*S45*S5+C45*(OpM16_4*s->dpt[3][17]-OpM36_4*s->dpt[1][17]));
  AlM345_4 = -s->dpt[1][17]*S5;
  OpM145_5 = C45*C6;
  OpM245_5 = -S45*C6;
  AlM145_5 = S45*(s->dpt[1][17]*S6-s->dpt[3][17]*C6);
  AlM245_5 = C45*(s->dpt[1][17]*S6-s->dpt[3][17]*C6);
  AlM145_6 = s->dpt[3][17]*C45;
  AlM245_6 = -s->dpt[3][17]*S45;
  OM146 = qd[46]+OM16*C45+OM26*S45;
  OM346 = -(OM245*S46-OM345*C46);
  OpF246 = C46*(OpF245+qd[46]*OM345)+S46*(OpF36-qd[46]*OM245);
  OpF346 = C46*(OpF36-qd[46]*OM245)-S46*(OpF245+qd[46]*OM345);
  AlF346 = -(AlF245*S46-AlF345*C46);
  AlM246_1 = AlM245_1*C46+AlM36_1*S46;
  AlM346_1 = -(AlM245_1*S46-AlM36_1*C46);
  AlM246_2 = AlM245_2*C46+AlM36_2*S46;
  AlM346_2 = -(AlM245_2*S46-AlM36_2*C46);
  AlM246_3 = AlM245_3*C46+AlM36_3*S46;
  AlM346_3 = -(AlM245_3*S46-AlM36_3*C46);
  OpM246_4 = OpM245_4*C46+OpM36_4*S46;
  OpM346_4 = -(OpM245_4*S46-OpM36_4*C46);
  AlM346_4 = -(AlM245_4*S46-AlM345_4*C46);
  OpM246_5 = OpM245_5*C46+S46*S6;
  OpM346_5 = -(OpM245_5*S46-C46*S6);
  AlM346_5 = -AlM245_5*S46;
  OpM246_6 = C45*C46;
  OpM346_6 = -C45*S46;
  AlM346_6 = -(AlM245_6*S46+s->dpt[1][17]*C46);
  OM147 = OM146*C47-OM346*S47;
  OM247 = qd[47]+OM245*C46+OM345*S46;
  OM347 = OM146*S47+OM346*C47;
  OpF147 = C47*(OpF145-qd[47]*OM346)-S47*(OpF346+qd[47]*OM146);
  OpF347 = C47*(OpF346+qd[47]*OM146)+S47*(OpF145-qd[47]*OM346);
  AlM147_1 = AlM145_1*C47-AlM346_1*S47;
  AlM347_1 = AlM145_1*S47+AlM346_1*C47;
  AlM147_2 = AlM145_2*C47-AlM346_2*S47;
  AlM347_2 = AlM145_2*S47+AlM346_2*C47;
  AlM147_3 = AlM145_3*C47-AlM346_3*S47;
  AlM347_3 = AlM145_3*S47+AlM346_3*C47;
  OpM147_4 = OpM145_4*C47-OpM346_4*S47;
  OpM347_4 = OpM145_4*S47+OpM346_4*C47;
  AlM147_4 = AlM145_4*C47-AlM346_4*S47;
  OpM147_5 = OpM145_5*C47-OpM346_5*S47;
  OpM347_5 = OpM145_5*S47+OpM346_5*C47;
  AlM147_5 = AlM145_5*C47-AlM346_5*S47;
  OpM147_6 = -(OpM346_6*S47-S45*C47);
  OpM347_6 = OpM346_6*C47+S45*S47;
  AlM147_6 = AlM145_6*C47-AlM346_6*S47;
  OpM147_45 = -C46*S47;
  OpM347_45 = C46*C47;
  BS148 = -(OM247*OM247+OM347*OM347);
  BS248 = OM147*OM247;
  BS348 = OM147*OM347;
  BS548 = -(OM147*OM147+OM347*OM347);
  BS648 = OM247*OM347;
  BS948 = -(OM147*OM147+OM247*OM247);
  BeF248 = BS248-OpF347;
  BeF348 = BS348+OpF246;
  BeF448 = BS248+OpF347;
  BeF648 = BS648-OpF147;
  BeF748 = BS348-OpF246;
  BeF848 = BS648+OpF147;
  AlF148 = AlF145*C47-AlF346*S47-q[48]*(OM247*OM247+OM347*OM347);
  AlF248 = q[48]*(OpF347+OM147*OM247)+(2.0)*qd[48]*OM347+AlF245*C46+AlF345*S46;
  AlF348 = AlF145*S47+AlF346*C47-q[48]*(OpF246-OM147*OM347)-(2.0)*qd[48]*OM247;
  AlM248_4 = q[48]*OpM347_4+AlM245_4*C46+AlM345_4*S46;
  AlM348_4 = AlM145_4*S47+AlM346_4*C47-q[48]*OpM246_4;
  AlM248_5 = q[48]*OpM347_5+AlM245_5*C46;
  AlM348_5 = AlM145_5*S47+AlM346_5*C47-q[48]*OpM246_5;
  AlM248_6 = q[48]*OpM347_6+AlM245_6*C46-s->dpt[1][17]*S46;
  AlM348_6 = AlM145_6*S47+AlM346_6*C47-q[48]*OpM246_6;
  AlM248_45 = q[48]*OpM347_45;
  AlM348_45 = -q[48]*S46;
  AlM248_46 = q[48]*S47;

// = = Block_0_1_0_2_0_21 = = 
 
// Trigonometric Variables  

  S47p49 = C47*S49+S47*C49;
  C47p49 = C47*C49-S47*S49;
 
// Forward Kinematics 

  OM149 = OM147*C49-OM347*S49;
  OM249 = qd[49]+OM247;
  OM349 = OM147*S49+OM347*C49;

// = = Block_0_1_0_2_0_22 = = 
 
// Trigonometric Variables  

  S47p50 = C47*S50+S47*C50;
  C47p50 = C47*C50-S47*S50;
 
// Forward Kinematics 

  OM150 = OM147*C50-OM347*S50;
  OM250 = qd[50]+OM247;
  OM350 = OM147*S50+OM347*C50;

// = = Block_0_1_0_2_0_23 = = 
 
// Trigonometric Variables  

  S47p51 = C47*S51+S47*C51;
  C47p51 = C47*C51-S47*S51;
 
// Forward Kinematics 

  OM151 = OM147*C51-OM347*S51;
  OM251 = qd[51]+OM247;
  OM351 = OM147*S51+OM347*C51;

// = = Block_0_1_0_2_0_24 = = 
 
// Trigonometric Variables  

  S47p52 = C47*S52+S47*C52;
  C47p52 = C47*C52-S47*S52;
 
// Forward Kinematics 

  OM152 = OM147*C52-OM347*S52;
  OM252 = qd[52]+OM247;
  OM352 = OM147*S52+OM347*C52;

// = = Block_0_2_0_1_0_10 = = 
 
// Backward Dynamics 

  FF23_124 = -(s->frc[1][24]*C24+s->frc[3][24]*S24);
  FF23_324 = s->frc[1][24]*S24-s->frc[3][24]*C24;
  CF23_124 = -(s->trq[1][24]*C24+s->trq[3][24]*S24);
  CF23_324 = s->trq[1][24]*S24-s->trq[3][24]*C24;

// = = Block_0_2_0_1_0_11 = = 
 
// Backward Dynamics 

  FF26_127 = -(s->frc[1][27]*C27+s->frc[3][27]*S27);
  FF26_327 = s->frc[1][27]*S27-s->frc[3][27]*C27;
  CF26_127 = -(s->trq[1][27]*C27+s->trq[3][27]*S27);
  CF26_327 = s->trq[1][27]*S27-s->trq[3][27]*C27;
  FF125 = -(s->frc[1][25]-FF26_127);
  FF325 = -(s->frc[3][25]+s->frc[2][27]*S26-FF26_327*C26);
  CF125 = -(s->trq[1][25]-CF26_127+s->dpt[2][41]*(s->frc[2][27]*S26-FF26_327*C26));
  CF225 = -(s->trq[2][25]+s->trq[2][27]*C26+CF26_327*S26-s->dpt[1][41]*(s->frc[2][27]*S26-FF26_327*C26));
  CF325 = -(s->trq[3][25]+s->trq[2][27]*S26-CF26_327*C26+FF26_127*s->dpt[2][41]+s->dpt[1][41]*(s->frc[2][27]*C26+
 FF26_327*S26));

// = = Block_0_2_0_1_0_13 = = 
 
// Backward Dynamics 

  FF29_130 = -(s->frc[1][30]*C30-s->frc[2][30]*S30);
  FF29_230 = -(s->frc[1][30]*S30+s->frc[2][30]*C30);
  CF29_130 = -(s->trq[1][30]*C30-s->trq[2][30]*S30);
  CF29_230 = -(s->trq[1][30]*S30+s->trq[2][30]*C30);

// = = Block_0_2_0_1_0_14 = = 
 
// Backward Dynamics 

  FF31_132 = -(s->frc[1][32]*C32-s->frc[2][32]*S32);
  FF31_232 = -(s->frc[1][32]*S32+s->frc[2][32]*C32);
  CF31_132 = -(s->trq[1][32]*C32-s->trq[2][32]*S32);
  CF31_232 = -(s->trq[1][32]*S32+s->trq[2][32]*C32);

// = = Block_0_2_0_1_0_18 = = 
 
// Backward Dynamics 

  FF41_142 = -(s->frc[1][42]*C42+s->frc[3][42]*S42);
  FF41_342 = s->frc[1][42]*S42-s->frc[3][42]*C42;
  CF41_142 = -(s->trq[1][42]*C42+s->trq[3][42]*S42);
  CF41_342 = s->trq[1][42]*S42-s->trq[3][42]*C42;
  FF140 = -(s->frc[1][40]-FF41_142);
  FF340 = -(s->frc[3][40]+s->frc[2][42]*S41-FF41_342*C41);
  CF140 = -(s->trq[1][40]-CF41_142+s->dpt[2][59]*(s->frc[2][42]*S41-FF41_342*C41));
  CF240 = -(s->trq[2][40]+s->trq[2][42]*C41+CF41_342*S41-s->dpt[1][59]*(s->frc[2][42]*S41-FF41_342*C41));
  CF340 = -(s->trq[3][40]+s->trq[2][42]*S41-CF41_342*C41+FF41_142*s->dpt[2][59]+s->dpt[1][59]*(s->frc[2][42]*C41+
 FF41_342*S41));

// = = Block_0_2_0_1_0_19 = = 
 
// Backward Dynamics 

  FF43_144 = -(s->frc[1][44]*C44+s->frc[3][44]*S44);
  FF43_344 = s->frc[1][44]*S44-s->frc[3][44]*C44;
  CF43_144 = -(s->trq[1][44]*C44+s->trq[3][44]*S44);
  CF43_344 = s->trq[1][44]*S44-s->trq[3][44]*C44;

// = = Block_0_2_0_1_0_21 = = 
 
// Backward Dynamics 

  FA149 = -(s->frc[1][49]-s->m[49]*(C49*(AlF148+BS148*s->dpt[1][62]+BeF248*s->dpt[2][62]+BeF348*s->dpt[3][62])-S49*(
 AlF348+BS948*s->dpt[3][62]+BeF748*s->dpt[1][62]+BeF848*s->dpt[2][62])));
  FA249 = -(s->frc[2][49]-s->m[49]*(AlF248+BS548*s->dpt[2][62]+BeF448*s->dpt[1][62]+BeF648*s->dpt[3][62]));
  FA349 = -(s->frc[3][49]-s->m[49]*(C49*(AlF348+BS948*s->dpt[3][62]+BeF748*s->dpt[1][62]+BeF848*s->dpt[2][62])+S49*(
 AlF148+BS148*s->dpt[1][62]+BeF248*s->dpt[2][62]+BeF348*s->dpt[3][62])));
  CF149 = -(s->trq[1][49]-s->In[1][49]*(C49*(OpF147-qd[49]*OM347)-S49*(OpF347+qd[49]*OM147))+OM249*OM349*(s->In[5][49]-
 s->In[9][49]));
  CF249 = -(s->trq[2][49]-s->In[5][49]*OpF246-OM149*OM349*(s->In[1][49]-s->In[9][49]));
  CF349 = -(s->trq[3][49]-s->In[9][49]*(C49*(OpF347+qd[49]*OM147)+S49*(OpF147-qd[49]*OM347))+OM149*OM249*(s->In[1][49]-
 s->In[5][49]));
  FB149_1 = s->m[49]*(AlM147_1*C49-AlM347_1*S49);
  FB249_1 = s->m[49]*AlM246_1;
  FB349_1 = s->m[49]*(AlM147_1*S49+AlM347_1*C49);
  FB149_2 = s->m[49]*(AlM147_2*C49-AlM347_2*S49);
  FB249_2 = s->m[49]*AlM246_2;
  FB349_2 = s->m[49]*(AlM147_2*S49+AlM347_2*C49);
  FB149_3 = s->m[49]*(AlM147_3*C49-AlM347_3*S49);
  FB249_3 = s->m[49]*AlM246_3;
  FB349_3 = s->m[49]*(AlM147_3*S49+AlM347_3*C49);
  FB149_4 = s->m[49]*(C49*(AlM147_4+OpM246_4*s->dpt[3][62]-OpM347_4*s->dpt[2][62])-S49*(AlM348_4+OpM147_4*s->dpt[2][62]-
 OpM246_4*s->dpt[1][62]));
  FB249_4 = s->m[49]*(AlM248_4-OpM147_4*s->dpt[3][62]+OpM347_4*s->dpt[1][62]);
  FB349_4 = s->m[49]*(C49*(AlM348_4+OpM147_4*s->dpt[2][62]-OpM246_4*s->dpt[1][62])+S49*(AlM147_4+OpM246_4*s->dpt[3][62]-
 OpM347_4*s->dpt[2][62]));
  CM149_4 = s->In[1][49]*(OpM147_4*C49-OpM347_4*S49);
  CM249_4 = s->In[5][49]*OpM246_4;
  CM349_4 = s->In[9][49]*(OpM147_4*S49+OpM347_4*C49);
  FB149_5 = s->m[49]*(C49*(AlM147_5+OpM246_5*s->dpt[3][62]-OpM347_5*s->dpt[2][62])-S49*(AlM348_5+OpM147_5*s->dpt[2][62]-
 OpM246_5*s->dpt[1][62]));
  FB249_5 = s->m[49]*(AlM248_5-OpM147_5*s->dpt[3][62]+OpM347_5*s->dpt[1][62]);
  FB349_5 = s->m[49]*(C49*(AlM348_5+OpM147_5*s->dpt[2][62]-OpM246_5*s->dpt[1][62])+S49*(AlM147_5+OpM246_5*s->dpt[3][62]-
 OpM347_5*s->dpt[2][62]));
  CM149_5 = s->In[1][49]*(OpM147_5*C49-OpM347_5*S49);
  CM249_5 = s->In[5][49]*OpM246_5;
  CM349_5 = s->In[9][49]*(OpM147_5*S49+OpM347_5*C49);
  FB149_6 = s->m[49]*(C49*(AlM147_6+OpM246_6*s->dpt[3][62]-OpM347_6*s->dpt[2][62])-S49*(AlM348_6+OpM147_6*s->dpt[2][62]-
 OpM246_6*s->dpt[1][62]));
  FB249_6 = s->m[49]*(AlM248_6-OpM147_6*s->dpt[3][62]+OpM347_6*s->dpt[1][62]);
  FB349_6 = s->m[49]*(C49*(AlM348_6+OpM147_6*s->dpt[2][62]-OpM246_6*s->dpt[1][62])+S49*(AlM147_6+OpM246_6*s->dpt[3][62]-
 OpM347_6*s->dpt[2][62]));
  CM149_6 = s->In[1][49]*(OpM147_6*C49-OpM347_6*S49);
  CM249_6 = s->In[5][49]*OpM246_6;
  CM349_6 = s->In[9][49]*(OpM147_6*S49+OpM347_6*C49);
  FB149_45 = -s->m[49]*(C49*(OpM347_45*s->dpt[2][62]-s->dpt[3][62]*S46)+S49*(AlM348_45+OpM147_45*s->dpt[2][62]-
 s->dpt[1][62]*S46));
  FB249_45 = s->m[49]*(AlM248_45-OpM147_45*s->dpt[3][62]+OpM347_45*s->dpt[1][62]);
  FB349_45 = s->m[49]*(C49*(AlM348_45+OpM147_45*s->dpt[2][62]-s->dpt[1][62]*S46)-S49*(OpM347_45*s->dpt[2][62]-
 s->dpt[3][62]*S46));
  CM149_45 = -s->In[1][49]*S47p49*C46;
  CM249_45 = s->In[5][49]*S46;
  CM349_45 = s->In[9][49]*C47p49*C46;
  FB149_46 = -s->m[49]*s->dpt[2][62]*S47p49;
  FB249_46 = s->m[49]*(AlM248_46+s->dpt[1][62]*S47-s->dpt[3][62]*C47);
  FB349_46 = s->m[49]*s->dpt[2][62]*C47p49;
  CM149_46 = s->In[1][49]*C47p49;
  CM349_46 = s->In[9][49]*S47p49;
  FB149_47 = s->m[49]*(s->dpt[3][62]*C49+S49*(q[48]+s->dpt[1][62]));
  FB349_47 = s->m[49]*(s->dpt[3][62]*S49-C49*(q[48]+s->dpt[1][62]));

// = = Block_0_2_0_1_0_22 = = 
 
// Backward Dynamics 

  FA150 = -(s->frc[1][50]-s->m[50]*(C50*(AlF148+BS148*s->dpt[1][63]+BeF248*s->dpt[2][63]+BeF348*s->dpt[3][63])-S50*(
 AlF348+BS948*s->dpt[3][63]+BeF748*s->dpt[1][63]+BeF848*s->dpt[2][63])));
  FA250 = -(s->frc[2][50]-s->m[50]*(AlF248+BS548*s->dpt[2][63]+BeF448*s->dpt[1][63]+BeF648*s->dpt[3][63]));
  FA350 = -(s->frc[3][50]-s->m[50]*(C50*(AlF348+BS948*s->dpt[3][63]+BeF748*s->dpt[1][63]+BeF848*s->dpt[2][63])+S50*(
 AlF148+BS148*s->dpt[1][63]+BeF248*s->dpt[2][63]+BeF348*s->dpt[3][63])));
  CF150 = -(s->trq[1][50]-s->In[1][50]*(C50*(OpF147-qd[50]*OM347)-S50*(OpF347+qd[50]*OM147))+OM250*OM350*(s->In[5][50]-
 s->In[9][50]));
  CF250 = -(s->trq[2][50]-s->In[5][50]*OpF246-OM150*OM350*(s->In[1][50]-s->In[9][50]));
  CF350 = -(s->trq[3][50]-s->In[9][50]*(C50*(OpF347+qd[50]*OM147)+S50*(OpF147-qd[50]*OM347))+OM150*OM250*(s->In[1][50]-
 s->In[5][50]));
  FB150_1 = s->m[50]*(AlM147_1*C50-AlM347_1*S50);
  FB250_1 = s->m[50]*AlM246_1;
  FB350_1 = s->m[50]*(AlM147_1*S50+AlM347_1*C50);
  FB150_2 = s->m[50]*(AlM147_2*C50-AlM347_2*S50);
  FB250_2 = s->m[50]*AlM246_2;
  FB350_2 = s->m[50]*(AlM147_2*S50+AlM347_2*C50);
  FB150_3 = s->m[50]*(AlM147_3*C50-AlM347_3*S50);
  FB250_3 = s->m[50]*AlM246_3;
  FB350_3 = s->m[50]*(AlM147_3*S50+AlM347_3*C50);
  FB150_4 = s->m[50]*(C50*(AlM147_4+OpM246_4*s->dpt[3][63]-OpM347_4*s->dpt[2][63])-S50*(AlM348_4+OpM147_4*s->dpt[2][63]-
 OpM246_4*s->dpt[1][63]));
  FB250_4 = s->m[50]*(AlM248_4-OpM147_4*s->dpt[3][63]+OpM347_4*s->dpt[1][63]);
  FB350_4 = s->m[50]*(C50*(AlM348_4+OpM147_4*s->dpt[2][63]-OpM246_4*s->dpt[1][63])+S50*(AlM147_4+OpM246_4*s->dpt[3][63]-
 OpM347_4*s->dpt[2][63]));
  CM150_4 = s->In[1][50]*(OpM147_4*C50-OpM347_4*S50);
  CM250_4 = s->In[5][50]*OpM246_4;
  CM350_4 = s->In[9][50]*(OpM147_4*S50+OpM347_4*C50);
  FB150_5 = s->m[50]*(C50*(AlM147_5+OpM246_5*s->dpt[3][63]-OpM347_5*s->dpt[2][63])-S50*(AlM348_5+OpM147_5*s->dpt[2][63]-
 OpM246_5*s->dpt[1][63]));
  FB250_5 = s->m[50]*(AlM248_5-OpM147_5*s->dpt[3][63]+OpM347_5*s->dpt[1][63]);
  FB350_5 = s->m[50]*(C50*(AlM348_5+OpM147_5*s->dpt[2][63]-OpM246_5*s->dpt[1][63])+S50*(AlM147_5+OpM246_5*s->dpt[3][63]-
 OpM347_5*s->dpt[2][63]));
  CM150_5 = s->In[1][50]*(OpM147_5*C50-OpM347_5*S50);
  CM250_5 = s->In[5][50]*OpM246_5;
  CM350_5 = s->In[9][50]*(OpM147_5*S50+OpM347_5*C50);
  FB150_6 = s->m[50]*(C50*(AlM147_6+OpM246_6*s->dpt[3][63]-OpM347_6*s->dpt[2][63])-S50*(AlM348_6+OpM147_6*s->dpt[2][63]-
 OpM246_6*s->dpt[1][63]));
  FB250_6 = s->m[50]*(AlM248_6-OpM147_6*s->dpt[3][63]+OpM347_6*s->dpt[1][63]);
  FB350_6 = s->m[50]*(C50*(AlM348_6+OpM147_6*s->dpt[2][63]-OpM246_6*s->dpt[1][63])+S50*(AlM147_6+OpM246_6*s->dpt[3][63]-
 OpM347_6*s->dpt[2][63]));
  CM150_6 = s->In[1][50]*(OpM147_6*C50-OpM347_6*S50);
  CM250_6 = s->In[5][50]*OpM246_6;
  CM350_6 = s->In[9][50]*(OpM147_6*S50+OpM347_6*C50);
  FB150_45 = -s->m[50]*(C50*(OpM347_45*s->dpt[2][63]-s->dpt[3][63]*S46)+S50*(AlM348_45+OpM147_45*s->dpt[2][63]-
 s->dpt[1][63]*S46));
  FB250_45 = s->m[50]*(AlM248_45-OpM147_45*s->dpt[3][63]+OpM347_45*s->dpt[1][63]);
  FB350_45 = s->m[50]*(C50*(AlM348_45+OpM147_45*s->dpt[2][63]-s->dpt[1][63]*S46)-S50*(OpM347_45*s->dpt[2][63]-
 s->dpt[3][63]*S46));
  CM150_45 = -s->In[1][50]*S47p50*C46;
  CM250_45 = s->In[5][50]*S46;
  CM350_45 = s->In[9][50]*C47p50*C46;
  FB150_46 = -s->m[50]*s->dpt[2][63]*S47p50;
  FB250_46 = s->m[50]*(AlM248_46+s->dpt[1][63]*S47-s->dpt[3][63]*C47);
  FB350_46 = s->m[50]*s->dpt[2][63]*C47p50;
  CM150_46 = s->In[1][50]*C47p50;
  CM350_46 = s->In[9][50]*S47p50;
  FB150_47 = s->m[50]*(s->dpt[3][63]*C50+S50*(q[48]+s->dpt[1][63]));
  FB350_47 = s->m[50]*(s->dpt[3][63]*S50-C50*(q[48]+s->dpt[1][63]));

// = = Block_0_2_0_1_0_23 = = 
 
// Backward Dynamics 

  FA151 = -(s->frc[1][51]-s->m[51]*(C51*(AlF148+BS148*s->dpt[1][64]+BeF248*s->dpt[2][64]+BeF348*s->dpt[3][64])-S51*(
 AlF348+BS948*s->dpt[3][64]+BeF748*s->dpt[1][64]+BeF848*s->dpt[2][64])));
  FA251 = -(s->frc[2][51]-s->m[51]*(AlF248+BS548*s->dpt[2][64]+BeF448*s->dpt[1][64]+BeF648*s->dpt[3][64]));
  FA351 = -(s->frc[3][51]-s->m[51]*(C51*(AlF348+BS948*s->dpt[3][64]+BeF748*s->dpt[1][64]+BeF848*s->dpt[2][64])+S51*(
 AlF148+BS148*s->dpt[1][64]+BeF248*s->dpt[2][64]+BeF348*s->dpt[3][64])));
  CF151 = -(s->trq[1][51]-s->In[1][51]*(C51*(OpF147-qd[51]*OM347)-S51*(OpF347+qd[51]*OM147))+OM251*OM351*(s->In[5][51]-
 s->In[9][51]));
  CF251 = -(s->trq[2][51]-s->In[5][51]*OpF246-OM151*OM351*(s->In[1][51]-s->In[9][51]));
  CF351 = -(s->trq[3][51]-s->In[9][51]*(C51*(OpF347+qd[51]*OM147)+S51*(OpF147-qd[51]*OM347))+OM151*OM251*(s->In[1][51]-
 s->In[5][51]));
  FB151_1 = s->m[51]*(AlM147_1*C51-AlM347_1*S51);
  FB251_1 = s->m[51]*AlM246_1;
  FB351_1 = s->m[51]*(AlM147_1*S51+AlM347_1*C51);
  FB151_2 = s->m[51]*(AlM147_2*C51-AlM347_2*S51);
  FB251_2 = s->m[51]*AlM246_2;
  FB351_2 = s->m[51]*(AlM147_2*S51+AlM347_2*C51);
  FB151_3 = s->m[51]*(AlM147_3*C51-AlM347_3*S51);
  FB251_3 = s->m[51]*AlM246_3;
  FB351_3 = s->m[51]*(AlM147_3*S51+AlM347_3*C51);
  FB151_4 = s->m[51]*(C51*(AlM147_4+OpM246_4*s->dpt[3][64]-OpM347_4*s->dpt[2][64])-S51*(AlM348_4+OpM147_4*s->dpt[2][64]-
 OpM246_4*s->dpt[1][64]));
  FB251_4 = s->m[51]*(AlM248_4-OpM147_4*s->dpt[3][64]+OpM347_4*s->dpt[1][64]);
  FB351_4 = s->m[51]*(C51*(AlM348_4+OpM147_4*s->dpt[2][64]-OpM246_4*s->dpt[1][64])+S51*(AlM147_4+OpM246_4*s->dpt[3][64]-
 OpM347_4*s->dpt[2][64]));
  CM151_4 = s->In[1][51]*(OpM147_4*C51-OpM347_4*S51);
  CM251_4 = s->In[5][51]*OpM246_4;
  CM351_4 = s->In[9][51]*(OpM147_4*S51+OpM347_4*C51);
  FB151_5 = s->m[51]*(C51*(AlM147_5+OpM246_5*s->dpt[3][64]-OpM347_5*s->dpt[2][64])-S51*(AlM348_5+OpM147_5*s->dpt[2][64]-
 OpM246_5*s->dpt[1][64]));
  FB251_5 = s->m[51]*(AlM248_5-OpM147_5*s->dpt[3][64]+OpM347_5*s->dpt[1][64]);
  FB351_5 = s->m[51]*(C51*(AlM348_5+OpM147_5*s->dpt[2][64]-OpM246_5*s->dpt[1][64])+S51*(AlM147_5+OpM246_5*s->dpt[3][64]-
 OpM347_5*s->dpt[2][64]));
  CM151_5 = s->In[1][51]*(OpM147_5*C51-OpM347_5*S51);
  CM251_5 = s->In[5][51]*OpM246_5;
  CM351_5 = s->In[9][51]*(OpM147_5*S51+OpM347_5*C51);
  FB151_6 = s->m[51]*(C51*(AlM147_6+OpM246_6*s->dpt[3][64]-OpM347_6*s->dpt[2][64])-S51*(AlM348_6+OpM147_6*s->dpt[2][64]-
 OpM246_6*s->dpt[1][64]));
  FB251_6 = s->m[51]*(AlM248_6-OpM147_6*s->dpt[3][64]+OpM347_6*s->dpt[1][64]);
  FB351_6 = s->m[51]*(C51*(AlM348_6+OpM147_6*s->dpt[2][64]-OpM246_6*s->dpt[1][64])+S51*(AlM147_6+OpM246_6*s->dpt[3][64]-
 OpM347_6*s->dpt[2][64]));
  CM151_6 = s->In[1][51]*(OpM147_6*C51-OpM347_6*S51);
  CM251_6 = s->In[5][51]*OpM246_6;
  CM351_6 = s->In[9][51]*(OpM147_6*S51+OpM347_6*C51);
  FB151_45 = -s->m[51]*(C51*(OpM347_45*s->dpt[2][64]-s->dpt[3][64]*S46)+S51*(AlM348_45+OpM147_45*s->dpt[2][64]-
 s->dpt[1][64]*S46));
  FB251_45 = s->m[51]*(AlM248_45-OpM147_45*s->dpt[3][64]+OpM347_45*s->dpt[1][64]);
  FB351_45 = s->m[51]*(C51*(AlM348_45+OpM147_45*s->dpt[2][64]-s->dpt[1][64]*S46)-S51*(OpM347_45*s->dpt[2][64]-
 s->dpt[3][64]*S46));
  CM151_45 = -s->In[1][51]*S47p51*C46;
  CM251_45 = s->In[5][51]*S46;
  CM351_45 = s->In[9][51]*C47p51*C46;
  FB151_46 = -s->m[51]*s->dpt[2][64]*S47p51;
  FB251_46 = s->m[51]*(AlM248_46+s->dpt[1][64]*S47-s->dpt[3][64]*C47);
  FB351_46 = s->m[51]*s->dpt[2][64]*C47p51;
  CM151_46 = s->In[1][51]*C47p51;
  CM351_46 = s->In[9][51]*S47p51;
  FB151_47 = s->m[51]*(s->dpt[3][64]*C51+S51*(q[48]+s->dpt[1][64]));
  FB351_47 = s->m[51]*(s->dpt[3][64]*S51-C51*(q[48]+s->dpt[1][64]));

// = = Block_0_2_0_1_0_24 = = 
 
// Backward Dynamics 

  FA152 = -(s->frc[1][52]-s->m[52]*(C52*(AlF148+BS148*s->dpt[1][65]+BeF248*s->dpt[2][65]+BeF348*s->dpt[3][65])-S52*(
 AlF348+BS948*s->dpt[3][65]+BeF748*s->dpt[1][65]+BeF848*s->dpt[2][65])));
  FA252 = -(s->frc[2][52]-s->m[52]*(AlF248+BS548*s->dpt[2][65]+BeF448*s->dpt[1][65]+BeF648*s->dpt[3][65]));
  FA352 = -(s->frc[3][52]-s->m[52]*(C52*(AlF348+BS948*s->dpt[3][65]+BeF748*s->dpt[1][65]+BeF848*s->dpt[2][65])+S52*(
 AlF148+BS148*s->dpt[1][65]+BeF248*s->dpt[2][65]+BeF348*s->dpt[3][65])));
  CF152 = -(s->trq[1][52]-s->In[1][52]*(C52*(OpF147-qd[52]*OM347)-S52*(OpF347+qd[52]*OM147))+OM252*OM352*(s->In[5][52]-
 s->In[9][52]));
  CF252 = -(s->trq[2][52]-s->In[5][52]*OpF246-OM152*OM352*(s->In[1][52]-s->In[9][52]));
  CF352 = -(s->trq[3][52]-s->In[9][52]*(C52*(OpF347+qd[52]*OM147)+S52*(OpF147-qd[52]*OM347))+OM152*OM252*(s->In[1][52]-
 s->In[5][52]));
  FB152_1 = s->m[52]*(AlM147_1*C52-AlM347_1*S52);
  FB252_1 = s->m[52]*AlM246_1;
  FB352_1 = s->m[52]*(AlM147_1*S52+AlM347_1*C52);
  FB152_2 = s->m[52]*(AlM147_2*C52-AlM347_2*S52);
  FB252_2 = s->m[52]*AlM246_2;
  FB352_2 = s->m[52]*(AlM147_2*S52+AlM347_2*C52);
  FB152_3 = s->m[52]*(AlM147_3*C52-AlM347_3*S52);
  FB252_3 = s->m[52]*AlM246_3;
  FB352_3 = s->m[52]*(AlM147_3*S52+AlM347_3*C52);
  FB152_4 = s->m[52]*(C52*(AlM147_4+OpM246_4*s->dpt[3][65]-OpM347_4*s->dpt[2][65])-S52*(AlM348_4+OpM147_4*s->dpt[2][65]-
 OpM246_4*s->dpt[1][65]));
  FB252_4 = s->m[52]*(AlM248_4-OpM147_4*s->dpt[3][65]+OpM347_4*s->dpt[1][65]);
  FB352_4 = s->m[52]*(C52*(AlM348_4+OpM147_4*s->dpt[2][65]-OpM246_4*s->dpt[1][65])+S52*(AlM147_4+OpM246_4*s->dpt[3][65]-
 OpM347_4*s->dpt[2][65]));
  CM152_4 = s->In[1][52]*(OpM147_4*C52-OpM347_4*S52);
  CM252_4 = s->In[5][52]*OpM246_4;
  CM352_4 = s->In[9][52]*(OpM147_4*S52+OpM347_4*C52);
  FB152_5 = s->m[52]*(C52*(AlM147_5+OpM246_5*s->dpt[3][65]-OpM347_5*s->dpt[2][65])-S52*(AlM348_5+OpM147_5*s->dpt[2][65]-
 OpM246_5*s->dpt[1][65]));
  FB252_5 = s->m[52]*(AlM248_5-OpM147_5*s->dpt[3][65]+OpM347_5*s->dpt[1][65]);
  FB352_5 = s->m[52]*(C52*(AlM348_5+OpM147_5*s->dpt[2][65]-OpM246_5*s->dpt[1][65])+S52*(AlM147_5+OpM246_5*s->dpt[3][65]-
 OpM347_5*s->dpt[2][65]));
  CM152_5 = s->In[1][52]*(OpM147_5*C52-OpM347_5*S52);
  CM252_5 = s->In[5][52]*OpM246_5;
  CM352_5 = s->In[9][52]*(OpM147_5*S52+OpM347_5*C52);
  FB152_6 = s->m[52]*(C52*(AlM147_6+OpM246_6*s->dpt[3][65]-OpM347_6*s->dpt[2][65])-S52*(AlM348_6+OpM147_6*s->dpt[2][65]-
 OpM246_6*s->dpt[1][65]));
  FB252_6 = s->m[52]*(AlM248_6-OpM147_6*s->dpt[3][65]+OpM347_6*s->dpt[1][65]);
  FB352_6 = s->m[52]*(C52*(AlM348_6+OpM147_6*s->dpt[2][65]-OpM246_6*s->dpt[1][65])+S52*(AlM147_6+OpM246_6*s->dpt[3][65]-
 OpM347_6*s->dpt[2][65]));
  CM152_6 = s->In[1][52]*(OpM147_6*C52-OpM347_6*S52);
  CM252_6 = s->In[5][52]*OpM246_6;
  CM352_6 = s->In[9][52]*(OpM147_6*S52+OpM347_6*C52);
  FB152_45 = -s->m[52]*(C52*(OpM347_45*s->dpt[2][65]-s->dpt[3][65]*S46)+S52*(AlM348_45+OpM147_45*s->dpt[2][65]-
 s->dpt[1][65]*S46));
  FB252_45 = s->m[52]*(AlM248_45-OpM147_45*s->dpt[3][65]+OpM347_45*s->dpt[1][65]);
  FB352_45 = s->m[52]*(C52*(AlM348_45+OpM147_45*s->dpt[2][65]-s->dpt[1][65]*S46)-S52*(OpM347_45*s->dpt[2][65]-
 s->dpt[3][65]*S46));
  CM152_45 = -s->In[1][52]*S47p52*C46;
  CM252_45 = s->In[5][52]*S46;
  CM352_45 = s->In[9][52]*C47p52*C46;
  FB152_46 = -s->m[52]*s->dpt[2][65]*S47p52;
  FB252_46 = s->m[52]*(AlM248_46+s->dpt[1][65]*S47-s->dpt[3][65]*C47);
  FB352_46 = s->m[52]*s->dpt[2][65]*C47p52;
  CM152_46 = s->In[1][52]*C47p52;
  CM352_46 = s->In[9][52]*S47p52;
  FB152_47 = s->m[52]*(s->dpt[3][65]*C52+S52*(q[48]+s->dpt[1][65]));
  FB352_47 = s->m[52]*(s->dpt[3][65]*S52-C52*(q[48]+s->dpt[1][65]));

// = = Block_0_2_0_1_0_25 = = 
 
// Backward Dynamics 

  FA153 = -(s->frc[1][53]-s->m[53]*(AlF148+(2.0)*qd[53]*OM247+BS148*s->dpt[1][66]+BeF348*Dz533));
  FA253 = -(s->frc[2][53]-s->m[53]*(AlF248-(2.0)*qd[53]*OM147+BeF448*s->dpt[1][66]+BeF648*Dz533));
  FA353 = -(s->frc[3][53]-s->m[53]*(AlF348+BS948*Dz533+BeF748*s->dpt[1][66]));
  FB153_1 = s->m[53]*AlM147_1;
  FB253_1 = s->m[53]*AlM246_1;
  FB353_1 = s->m[53]*AlM347_1;
  FB153_2 = s->m[53]*AlM147_2;
  FB253_2 = s->m[53]*AlM246_2;
  FB353_2 = s->m[53]*AlM347_2;
  FB153_3 = s->m[53]*AlM147_3;
  FB253_3 = s->m[53]*AlM246_3;
  FB353_3 = s->m[53]*AlM347_3;
  FB153_4 = s->m[53]*(AlM147_4+Dz533*OpM246_4);
  FB253_4 = s->m[53]*(AlM248_4-Dz533*OpM147_4+OpM347_4*s->dpt[1][66]);
  FB353_4 = s->m[53]*(AlM348_4-OpM246_4*s->dpt[1][66]);
  FB153_5 = s->m[53]*(AlM147_5+Dz533*OpM246_5);
  FB253_5 = s->m[53]*(AlM248_5-Dz533*OpM147_5+OpM347_5*s->dpt[1][66]);
  FB353_5 = s->m[53]*(AlM348_5-OpM246_5*s->dpt[1][66]);
  FB153_6 = s->m[53]*(AlM147_6+Dz533*OpM246_6);
  FB253_6 = s->m[53]*(AlM248_6-Dz533*OpM147_6+OpM347_6*s->dpt[1][66]);
  FB353_6 = s->m[53]*(AlM348_6-OpM246_6*s->dpt[1][66]);
  FB153_45 = s->m[53]*Dz533*S46;
  FB253_45 = s->m[53]*(AlM248_45-Dz533*OpM147_45+OpM347_45*s->dpt[1][66]);
  FB353_45 = s->m[53]*(AlM348_45-s->dpt[1][66]*S46);
  FB253_46 = s->m[53]*(AlM248_46-Dz533*C47+s->dpt[1][66]*S47);
  FB153_47 = s->m[53]*Dz533;
  FB353_47 = -s->m[53]*(q[48]+s->dpt[1][66]);

// = = Block_0_2_0_2_0_2 = = 
 
// Backward Dynamics 

  FA17 = -(s->frc[1][7]-s->m[7]*(AlF16+q[7]*BS16));
  FA27 = -(s->frc[2][7]-s->m[7]*(AlF25+q[7]*BeF46+(2.0)*qd[7]*OM36));
  FA37 = -(s->frc[3][7]-s->m[7]*(AlF36+q[7]*BeF76-(2.0)*qd[7]*OM26));
  FB17_1 = s->m[7]*AlM16_1;
  FB27_1 = s->m[7]*AlM25_1;
  FB37_1 = s->m[7]*AlM36_1;
  FB17_2 = s->m[7]*AlM16_2;
  FB27_2 = s->m[7]*AlM25_2;
  FB37_2 = s->m[7]*AlM36_2;
  FB17_3 = s->m[7]*AlM16_3;
  FB27_3 = s->m[7]*S5;
  FB37_3 = s->m[7]*AlM36_3;

// = = Block_0_2_0_2_0_3 = = 
 
// Backward Dynamics 

  FA112 = -(s->frc[1][12]-s->m[12]*(C12*(AlF111+BeF311*s->dpt[3][22])-S12*(AlF310+BS911*s->dpt[3][22])));
  FA212 = -(s->frc[2][12]-s->m[12]*(AlF211-s->dpt[3][22]*(OpF111-OM211*OM311)));
  FA312 = -(s->frc[3][12]-s->m[12]*(C12*(AlF310+BS911*s->dpt[3][22])+S12*(AlF111+BeF311*s->dpt[3][22])));
  CF112 = -(s->trq[1][12]-s->In[1][12]*(C12*(OpF111-qd[12]*OM311)-S12*(OpF310+qd[12]*OM111))+OM212*OM312*(s->In[5][12]-
 s->In[9][12]));
  CF212 = -(s->trq[2][12]-s->In[5][12]*OpF211-OM112*OM312*(s->In[1][12]-s->In[9][12]));
  CF312 = -(s->trq[3][12]-s->In[9][12]*(C12*(OpF310+qd[12]*OM111)+S12*(OpF111-qd[12]*OM311))+OM112*OM212*(s->In[1][12]-
 s->In[5][12]));
  FB112_1 = s->m[12]*(AlM111_1*C12-AlM310_1*S12);
  FB212_1 = s->m[12]*AlM211_1;
  FB312_1 = s->m[12]*(AlM111_1*S12+AlM310_1*C12);
  FB112_2 = s->m[12]*(AlM111_2*C12-AlM310_2*S12);
  FB212_2 = s->m[12]*AlM211_2;
  FB312_2 = s->m[12]*(AlM111_2*S12+AlM310_2*C12);
  FB112_3 = s->m[12]*(AlM111_3*C12-AlM310_3*S12);
  FB212_3 = s->m[12]*AlM211_3;
  FB312_3 = s->m[12]*(AlM111_3*S12+AlM310_3*C12);
  FB112_4 = -s->m[12]*(AlM310_4*S12-C12*(AlM111_4+OpM211_4*s->dpt[3][22]));
  FB212_4 = s->m[12]*(AlM211_4-OpM111_4*s->dpt[3][22]);
  FB312_4 = s->m[12]*(AlM310_4*C12+S12*(AlM111_4+OpM211_4*s->dpt[3][22]));
  CM112_4 = s->In[1][12]*(OpM111_4*C12-OpM310_4*S12);
  CM212_4 = s->In[5][12]*OpM211_4;
  CM312_4 = s->In[9][12]*(OpM111_4*S12+OpM310_4*C12);
  FB112_5 = -s->m[12]*(AlM310_5*S12-C12*(AlM111_5+OpM211_5*s->dpt[3][22]));
  FB212_5 = s->m[12]*(AlM211_5-OpM111_5*s->dpt[3][22]);
  FB312_5 = s->m[12]*(AlM310_5*C12+S12*(AlM111_5+OpM211_5*s->dpt[3][22]));
  CM112_5 = s->In[1][12]*(OpM111_5*C12-OpM310_5*S12);
  CM212_5 = s->In[5][12]*OpM211_5;
  CM312_5 = s->In[9][12]*(OpM111_5*S12+OpM310_5*C12);
  FB112_6 = -s->m[12]*(AlM310_6*S12-C12*(AlM111_6+OpM211_6*s->dpt[3][22]));
  FB212_6 = s->m[12]*(AlM211_6-OpM111_6*s->dpt[3][22]);
  FB312_6 = s->m[12]*(AlM310_6*C12+S12*(AlM111_6+OpM211_6*s->dpt[3][22]));
  CM112_6 = s->In[1][12]*(OpM111_6*C12-OpM310_6*S12);
  CM212_6 = s->In[5][12]*OpM211_6;
  CM312_6 = s->In[9][12]*(OpM111_6*S12+OpM310_6*C12);
  FB112_8 = -s->m[12]*(AlM310_8*S12-C12*(AlM111_8+OpM211_8*s->dpt[3][22]));
  FB212_8 = s->m[12]*(AlM211_8-OpM111_8*s->dpt[3][22]);
  FB312_8 = s->m[12]*(AlM310_8*C12+S12*(AlM111_8+OpM211_8*s->dpt[3][22]));
  CM112_8 = s->In[1][12]*(OpM111_8*C12-OpM310_8*S12);
  CM212_8 = s->In[5][12]*OpM211_8;
  CM312_8 = s->In[9][12]*(OpM111_8*S12+OpM310_8*C12);
  CM112_9 = s->In[1][12]*(OpM111_9*C12+S10*S12);
  CM212_9 = s->In[5][12]*OpM211_9;
  CM312_9 = s->In[9][12]*(OpM111_9*S12-S10*C12);
  CM112_10 = s->In[1][12]*C11*C12;
  CM212_10 = -s->In[5][12]*S11;
  CM312_10 = s->In[9][12]*C11*S12;
  FF111 = -(s->frc[1][11]-s->m[11]*AlF111-FA112*C12-FA312*S12);
  FF211 = -(s->frc[2][11]-FA212-s->m[11]*AlF211);
  FF311 = -(s->frc[3][11]-s->m[11]*AlF310+FA112*S12-FA312*C12);
  CF111 = -(s->trq[1][11]-CF112*C12-CF312*S12+FA212*s->dpt[3][22]);
  CF211 = -(s->trq[2][11]-CF212-s->dpt[3][22]*(FA112*C12+FA312*S12));
  CF311 = -(s->trq[3][11]+CF112*S12-CF312*C12);
  FM111_1 = s->m[11]*AlM111_1+FB112_1*C12+FB312_1*S12;
  FM211_1 = FB212_1+s->m[11]*AlM211_1;
  FM311_1 = s->m[11]*AlM310_1-FB112_1*S12+FB312_1*C12;
  CM111_112 = -FB212_1*s->dpt[3][22];
  CM111_212 = s->dpt[3][22]*(FB112_1*C12+FB312_1*S12);
  FM111_2 = s->m[11]*AlM111_2+FB112_2*C12+FB312_2*S12;
  FM211_2 = FB212_2+s->m[11]*AlM211_2;
  FM311_2 = s->m[11]*AlM310_2-FB112_2*S12+FB312_2*C12;
  CM112_112 = -FB212_2*s->dpt[3][22];
  CM112_212 = s->dpt[3][22]*(FB112_2*C12+FB312_2*S12);
  FM111_3 = s->m[11]*AlM111_3+FB112_3*C12+FB312_3*S12;
  FM211_3 = FB212_3+s->m[11]*AlM211_3;
  FM311_3 = s->m[11]*AlM310_3-FB112_3*S12+FB312_3*C12;
  CM113_112 = -FB212_3*s->dpt[3][22];
  CM113_212 = s->dpt[3][22]*(FB112_3*C12+FB312_3*S12);
  FM111_4 = s->m[11]*AlM111_4+FB112_4*C12+FB312_4*S12;
  FM211_4 = FB212_4+s->m[11]*AlM211_4;
  FM311_4 = s->m[11]*AlM310_4-FB112_4*S12+FB312_4*C12;
  CM114_112 = CM112_4*C12+CM312_4*S12-FB212_4*s->dpt[3][22];
  CM114_212 = CM212_4+s->dpt[3][22]*(FB112_4*C12+FB312_4*S12);
  CM114_312 = -(CM112_4*S12-CM312_4*C12);
  FM111_5 = s->m[11]*AlM111_5+FB112_5*C12+FB312_5*S12;
  FM211_5 = FB212_5+s->m[11]*AlM211_5;
  FM311_5 = s->m[11]*AlM310_5-FB112_5*S12+FB312_5*C12;
  CM115_112 = CM112_5*C12+CM312_5*S12-FB212_5*s->dpt[3][22];
  CM115_212 = CM212_5+s->dpt[3][22]*(FB112_5*C12+FB312_5*S12);
  CM115_312 = -(CM112_5*S12-CM312_5*C12);
  FM111_6 = s->m[11]*AlM111_6+FB112_6*C12+FB312_6*S12;
  FM211_6 = FB212_6+s->m[11]*AlM211_6;
  FM311_6 = s->m[11]*AlM310_6-FB112_6*S12+FB312_6*C12;
  CM116_112 = CM112_6*C12+CM312_6*S12-FB212_6*s->dpt[3][22];
  CM116_212 = CM212_6+s->dpt[3][22]*(FB112_6*C12+FB312_6*S12);
  CM116_312 = -(CM112_6*S12-CM312_6*C12);
  FM111_8 = s->m[11]*AlM111_8+FB112_8*C12+FB312_8*S12;
  FM211_8 = FB212_8+s->m[11]*AlM211_8;
  CM118_112 = CM112_8*C12+CM312_8*S12-FB212_8*s->dpt[3][22];
  CM118_212 = CM212_8+s->dpt[3][22]*(FB112_8*C12+FB312_8*S12);
  CM118_312 = -(CM112_8*S12-CM312_8*C12);
  CM119_112 = s->m[12]*OpM111_9*s->dpt[3][22]*s->dpt[3][22]+CM112_9*C12+CM312_9*S12;
  CM119_212 = CM212_9+s->m[12]*OpM211_9*s->dpt[3][22]*s->dpt[3][22];
  CM119_312 = -(CM112_9*S12-CM312_9*C12);
  CM1110_312 = -(CM112_10*S12-CM312_10*C12);
  CM1111_312 = s->In[1][12]*S12*S12+s->In[9][12]*C12*C12;
  FF10_111 = FF111*C11-FF211*S11;
  FF10_211 = FF111*S11+FF211*C11;
  CF10_111 = CF111*C11-CF211*S11;
  CF10_211 = CF111*S11+CF211*C11;
  FM101_111 = FM111_1*C11-FM211_1*S11;
  FM101_211 = FM111_1*S11+FM211_1*C11;
  CM101_111 = CM111_112*C11-CM111_212*S11;
  CM101_211 = CM111_112*S11+CM111_212*C11;
  FM102_111 = FM111_2*C11-FM211_2*S11;
  FM102_211 = FM111_2*S11+FM211_2*C11;
  CM102_111 = CM112_112*C11-CM112_212*S11;
  CM102_211 = CM112_112*S11+CM112_212*C11;
  FM103_111 = FM111_3*C11-FM211_3*S11;
  FM103_211 = FM111_3*S11+FM211_3*C11;
  CM103_111 = CM113_112*C11-CM113_212*S11;
  CM103_211 = CM113_112*S11+CM113_212*C11;
  FM104_111 = FM111_4*C11-FM211_4*S11;
  FM104_211 = FM111_4*S11+FM211_4*C11;
  CM104_111 = CM114_112*C11-CM114_212*S11;
  CM104_211 = CM114_112*S11+CM114_212*C11;
  FM105_111 = FM111_5*C11-FM211_5*S11;
  FM105_211 = FM111_5*S11+FM211_5*C11;
  CM105_111 = CM115_112*C11-CM115_212*S11;
  CM105_211 = CM115_112*S11+CM115_212*C11;
  FM106_111 = FM111_6*C11-FM211_6*S11;
  FM106_211 = FM111_6*S11+FM211_6*C11;
  CM106_111 = CM116_112*C11-CM116_212*S11;
  CM106_211 = CM116_112*S11+CM116_212*C11;
  CM108_111 = CM118_112*C11-CM118_212*S11;
  CM108_211 = CM118_112*S11+CM118_212*C11;
  CM109_111 = CM119_112*C11-CM119_212*S11;
  CM1010_111 = C11*(s->m[12]*s->dpt[3][22]*s->dpt[3][22]*C11+CM112_10*C12+CM312_10*S12)+S11*S11*(s->In[5][12]+s->m[12]*
 s->dpt[3][22]*s->dpt[3][22]);
  FF9_310 = FF10_211*S10+FF311*C10;
  CF9_210 = CF10_211*C10-CF311*S10;
  CF9_310 = CF10_211*S10+CF311*C10;
  FM91_210 = FM101_211*C10-FM311_1*S10;
  FM91_310 = FM101_211*S10+FM311_1*C10;
  CM91_210 = CM101_211*C10;
  CM91_310 = CM101_211*S10;
  FM92_210 = FM102_211*C10-FM311_2*S10;
  FM92_310 = FM102_211*S10+FM311_2*C10;
  CM92_210 = CM102_211*C10;
  CM92_310 = CM102_211*S10;
  FM93_210 = FM103_211*C10-FM311_3*S10;
  FM93_310 = FM103_211*S10+FM311_3*C10;
  CM93_210 = CM103_211*C10;
  CM93_310 = CM103_211*S10;
  FM94_210 = FM104_211*C10-FM311_4*S10;
  FM94_310 = FM104_211*S10+FM311_4*C10;
  CM94_210 = CM104_211*C10-CM114_312*S10;
  CM94_310 = CM104_211*S10+CM114_312*C10;
  FM95_210 = FM105_211*C10-FM311_5*S10;
  FM95_310 = FM105_211*S10+FM311_5*C10;
  CM95_210 = CM105_211*C10-CM115_312*S10;
  CM95_310 = CM105_211*S10+CM115_312*C10;
  FM96_310 = FM106_211*S10+FM311_6*C10;
  CM96_210 = CM106_211*C10-CM116_312*S10;
  CM96_310 = CM106_211*S10+CM116_312*C10;
  CM98_210 = CM108_211*C10-CM118_312*S10;
  CM99_210 = (CM119_112*S11+CM119_212*C11)*C10-CM119_312*S10;
  FF18 = -(s->frc[1][8]-FF10_111*C9-FF9_310*S9);
  FF28 = -(s->frc[2][8]-FF10_211*C10+FF311*S10);
  FF38 = -(s->frc[3][8]+FF10_111*S9-FF9_310*C9);
  CF18 = -(s->trq[1][8]-CF10_111*C9-CF9_310*S9+s->dpt[2][18]*(FF10_111*S9-FF9_310*C9));
  CF28 = -(s->trq[2][8]-CF9_210);
  CF38 = -(s->trq[3][8]+CF10_111*S9-CF9_310*C9+s->dpt[2][18]*(FF10_111*C9+FF9_310*S9));
  FM81_19 = FM101_111*C9+FM91_310*S9;
  FM81_39 = -(FM101_111*S9-FM91_310*C9);
  CM81_19 = CM101_111*C9+CM91_310*S9-s->dpt[2][18]*(FM101_111*S9-FM91_310*C9);
  CM81_39 = -(CM101_111*S9-CM91_310*C9+s->dpt[2][18]*(FM101_111*C9+FM91_310*S9));
  FM82_19 = FM102_111*C9+FM92_310*S9;
  FM82_39 = -(FM102_111*S9-FM92_310*C9);
  CM82_19 = CM102_111*C9+CM92_310*S9-s->dpt[2][18]*(FM102_111*S9-FM92_310*C9);
  CM82_39 = -(CM102_111*S9-CM92_310*C9+s->dpt[2][18]*(FM102_111*C9+FM92_310*S9));
  FM83_19 = FM103_111*C9+FM93_310*S9;
  FM83_39 = -(FM103_111*S9-FM93_310*C9);
  CM83_19 = CM103_111*C9+CM93_310*S9-s->dpt[2][18]*(FM103_111*S9-FM93_310*C9);
  CM83_39 = -(CM103_111*S9-CM93_310*C9+s->dpt[2][18]*(FM103_111*C9+FM93_310*S9));
  FM84_19 = FM104_111*C9+FM94_310*S9;
  FM84_39 = -(FM104_111*S9-FM94_310*C9);
  CM84_19 = CM104_111*C9+CM94_310*S9-s->dpt[2][18]*(FM104_111*S9-FM94_310*C9);
  CM84_39 = -(CM104_111*S9-CM94_310*C9+s->dpt[2][18]*(FM104_111*C9+FM94_310*S9));
  FM85_19 = FM105_111*C9+FM95_310*S9;
  FM85_39 = -(FM105_111*S9-FM95_310*C9);
  CM85_19 = CM105_111*C9+CM95_310*S9-s->dpt[2][18]*(FM105_111*S9-FM95_310*C9);
  CM85_39 = -(CM105_111*S9-CM95_310*C9+s->dpt[2][18]*(FM105_111*C9+FM95_310*S9));
  CM86_19 = CM106_111*C9+CM96_310*S9-s->dpt[2][18]*(FM106_111*S9-FM96_310*C9);
  CM88_19 = CM108_111*C9+s->dpt[2][18]*(C9*(C10*(s->m[11]*AlM310_8-FB112_8*S12+FB312_8*C12)+S10*(FM111_8*S11+FM211_8*C11
 ))-S9*(FM111_8*C11-FM211_8*S11))+S9*(CM108_211*S10+CM118_312*C10);

// = = Block_0_2_0_2_0_4 = = 
 
// Backward Dynamics 

  FA117 = -(s->frc[1][17]-s->m[17]*(C17*(AlF116+BeF316*s->dpt[3][27])-S17*(AlF315+BS916*s->dpt[3][27])));
  FA217 = -(s->frc[2][17]-s->m[17]*(AlF216-s->dpt[3][27]*(OpF116-OM216*OM316)));
  FA317 = -(s->frc[3][17]-s->m[17]*(C17*(AlF315+BS916*s->dpt[3][27])+S17*(AlF116+BeF316*s->dpt[3][27])));
  CF117 = -(s->trq[1][17]-s->In[1][17]*(C17*(OpF116-qd[17]*OM316)-S17*(OpF315+qd[17]*OM116))+OM217*OM317*(s->In[5][17]-
 s->In[9][17]));
  CF217 = -(s->trq[2][17]-s->In[5][17]*OpF216-OM117*OM317*(s->In[1][17]-s->In[9][17]));
  CF317 = -(s->trq[3][17]-s->In[9][17]*(C17*(OpF315+qd[17]*OM116)+S17*(OpF116-qd[17]*OM316))+OM117*OM217*(s->In[1][17]-
 s->In[5][17]));
  FB117_1 = s->m[17]*(AlM116_1*C17-AlM315_1*S17);
  FB217_1 = s->m[17]*AlM216_1;
  FB317_1 = s->m[17]*(AlM116_1*S17+AlM315_1*C17);
  FB117_2 = s->m[17]*(AlM116_2*C17-AlM315_2*S17);
  FB217_2 = s->m[17]*AlM216_2;
  FB317_2 = s->m[17]*(AlM116_2*S17+AlM315_2*C17);
  FB117_3 = s->m[17]*(AlM116_3*C17-AlM315_3*S17);
  FB217_3 = s->m[17]*AlM216_3;
  FB317_3 = s->m[17]*(AlM116_3*S17+AlM315_3*C17);
  FB117_4 = -s->m[17]*(AlM315_4*S17-C17*(AlM116_4+OpM216_4*s->dpt[3][27]));
  FB217_4 = s->m[17]*(AlM216_4-OpM116_4*s->dpt[3][27]);
  FB317_4 = s->m[17]*(AlM315_4*C17+S17*(AlM116_4+OpM216_4*s->dpt[3][27]));
  CM117_4 = s->In[1][17]*(OpM116_4*C17-OpM315_4*S17);
  CM217_4 = s->In[5][17]*OpM216_4;
  CM317_4 = s->In[9][17]*(OpM116_4*S17+OpM315_4*C17);
  FB117_5 = -s->m[17]*(AlM315_5*S17-C17*(AlM116_5+OpM216_5*s->dpt[3][27]));
  FB217_5 = s->m[17]*(AlM216_5-OpM116_5*s->dpt[3][27]);
  FB317_5 = s->m[17]*(AlM315_5*C17+S17*(AlM116_5+OpM216_5*s->dpt[3][27]));
  CM117_5 = s->In[1][17]*(OpM116_5*C17-OpM315_5*S17);
  CM217_5 = s->In[5][17]*OpM216_5;
  CM317_5 = s->In[9][17]*(OpM116_5*S17+OpM315_5*C17);
  FB117_6 = -s->m[17]*(AlM315_6*S17-C17*(AlM116_6+OpM216_6*s->dpt[3][27]));
  FB217_6 = s->m[17]*(AlM216_6-OpM116_6*s->dpt[3][27]);
  FB317_6 = s->m[17]*(AlM315_6*C17+S17*(AlM116_6+OpM216_6*s->dpt[3][27]));
  CM117_6 = s->In[1][17]*(OpM116_6*C17-OpM315_6*S17);
  CM217_6 = s->In[5][17]*OpM216_6;
  CM317_6 = s->In[9][17]*(OpM116_6*S17+OpM315_6*C17);
  FB117_13 = -s->m[17]*(AlM315_13*S17-C17*(AlM116_13+OpM216_13*s->dpt[3][27]));
  FB217_13 = s->m[17]*(AlM216_13-OpM116_13*s->dpt[3][27]);
  FB317_13 = s->m[17]*(AlM315_13*C17+S17*(AlM116_13+OpM216_13*s->dpt[3][27]));
  CM117_13 = s->In[1][17]*(OpM116_13*C17-OpM315_13*S17);
  CM217_13 = s->In[5][17]*OpM216_13;
  CM317_13 = s->In[9][17]*(OpM116_13*S17+OpM315_13*C17);
  CM117_14 = s->In[1][17]*(OpM116_14*C17+S15*S17);
  CM217_14 = s->In[5][17]*OpM216_14;
  CM317_14 = s->In[9][17]*(OpM116_14*S17-S15*C17);
  CM117_15 = s->In[1][17]*C16*C17;
  CM217_15 = -s->In[5][17]*S16;
  CM317_15 = s->In[9][17]*C16*S17;
  FF116 = -(s->frc[1][16]-s->m[16]*AlF116-FA117*C17-FA317*S17);
  FF216 = -(s->frc[2][16]-FA217-s->m[16]*AlF216);
  FF316 = -(s->frc[3][16]-s->m[16]*AlF315+FA117*S17-FA317*C17);
  CF116 = -(s->trq[1][16]-CF117*C17-CF317*S17+FA217*s->dpt[3][27]);
  CF216 = -(s->trq[2][16]-CF217-s->dpt[3][27]*(FA117*C17+FA317*S17));
  CF316 = -(s->trq[3][16]+CF117*S17-CF317*C17);
  FM116_1 = s->m[16]*AlM116_1+FB117_1*C17+FB317_1*S17;
  FM216_1 = FB217_1+s->m[16]*AlM216_1;
  FM316_1 = s->m[16]*AlM315_1-FB117_1*S17+FB317_1*C17;
  CM161_117 = -FB217_1*s->dpt[3][27];
  CM161_217 = s->dpt[3][27]*(FB117_1*C17+FB317_1*S17);
  FM116_2 = s->m[16]*AlM116_2+FB117_2*C17+FB317_2*S17;
  FM216_2 = FB217_2+s->m[16]*AlM216_2;
  FM316_2 = s->m[16]*AlM315_2-FB117_2*S17+FB317_2*C17;
  CM162_117 = -FB217_2*s->dpt[3][27];
  CM162_217 = s->dpt[3][27]*(FB117_2*C17+FB317_2*S17);
  FM116_3 = s->m[16]*AlM116_3+FB117_3*C17+FB317_3*S17;
  FM216_3 = FB217_3+s->m[16]*AlM216_3;
  FM316_3 = s->m[16]*AlM315_3-FB117_3*S17+FB317_3*C17;
  CM163_117 = -FB217_3*s->dpt[3][27];
  CM163_217 = s->dpt[3][27]*(FB117_3*C17+FB317_3*S17);
  FM116_4 = s->m[16]*AlM116_4+FB117_4*C17+FB317_4*S17;
  FM216_4 = FB217_4+s->m[16]*AlM216_4;
  FM316_4 = s->m[16]*AlM315_4-FB117_4*S17+FB317_4*C17;
  CM164_117 = CM117_4*C17+CM317_4*S17-FB217_4*s->dpt[3][27];
  CM164_217 = CM217_4+s->dpt[3][27]*(FB117_4*C17+FB317_4*S17);
  CM164_317 = -(CM117_4*S17-CM317_4*C17);
  FM116_5 = s->m[16]*AlM116_5+FB117_5*C17+FB317_5*S17;
  FM216_5 = FB217_5+s->m[16]*AlM216_5;
  FM316_5 = s->m[16]*AlM315_5-FB117_5*S17+FB317_5*C17;
  CM165_117 = CM117_5*C17+CM317_5*S17-FB217_5*s->dpt[3][27];
  CM165_217 = CM217_5+s->dpt[3][27]*(FB117_5*C17+FB317_5*S17);
  CM165_317 = -(CM117_5*S17-CM317_5*C17);
  FM116_6 = s->m[16]*AlM116_6+FB117_6*C17+FB317_6*S17;
  FM216_6 = FB217_6+s->m[16]*AlM216_6;
  FM316_6 = s->m[16]*AlM315_6-FB117_6*S17+FB317_6*C17;
  CM166_117 = CM117_6*C17+CM317_6*S17-FB217_6*s->dpt[3][27];
  CM166_217 = CM217_6+s->dpt[3][27]*(FB117_6*C17+FB317_6*S17);
  CM166_317 = -(CM117_6*S17-CM317_6*C17);
  FM116_13 = s->m[16]*AlM116_13+FB117_13*C17+FB317_13*S17;
  FM216_13 = FB217_13+s->m[16]*AlM216_13;
  CM1613_117 = CM117_13*C17+CM317_13*S17-FB217_13*s->dpt[3][27];
  CM1613_217 = CM217_13+s->dpt[3][27]*(FB117_13*C17+FB317_13*S17);
  CM1613_317 = -(CM117_13*S17-CM317_13*C17);
  CM1614_117 = s->m[17]*OpM116_14*s->dpt[3][27]*s->dpt[3][27]+CM117_14*C17+CM317_14*S17;
  CM1614_217 = CM217_14+s->m[17]*OpM216_14*s->dpt[3][27]*s->dpt[3][27];
  CM1614_317 = -(CM117_14*S17-CM317_14*C17);
  CM1615_317 = -(CM117_15*S17-CM317_15*C17);
  CM1616_317 = s->In[1][17]*S17*S17+s->In[9][17]*C17*C17;
  FF15_116 = FF116*C16-FF216*S16;
  FF15_216 = FF116*S16+FF216*C16;
  CF15_116 = CF116*C16-CF216*S16;
  CF15_216 = CF116*S16+CF216*C16;
  FM151_116 = FM116_1*C16-FM216_1*S16;
  FM151_216 = FM116_1*S16+FM216_1*C16;
  CM151_116 = CM161_117*C16-CM161_217*S16;
  CM151_216 = CM161_117*S16+CM161_217*C16;
  FM152_116 = FM116_2*C16-FM216_2*S16;
  FM152_216 = FM116_2*S16+FM216_2*C16;
  CM152_116 = CM162_117*C16-CM162_217*S16;
  CM152_216 = CM162_117*S16+CM162_217*C16;
  FM153_116 = FM116_3*C16-FM216_3*S16;
  FM153_216 = FM116_3*S16+FM216_3*C16;
  CM153_116 = CM163_117*C16-CM163_217*S16;
  CM153_216 = CM163_117*S16+CM163_217*C16;
  FM154_116 = FM116_4*C16-FM216_4*S16;
  FM154_216 = FM116_4*S16+FM216_4*C16;
  CM154_116 = CM164_117*C16-CM164_217*S16;
  CM154_216 = CM164_117*S16+CM164_217*C16;
  FM155_116 = FM116_5*C16-FM216_5*S16;
  FM155_216 = FM116_5*S16+FM216_5*C16;
  CM155_116 = CM165_117*C16-CM165_217*S16;
  CM155_216 = CM165_117*S16+CM165_217*C16;
  FM156_116 = FM116_6*C16-FM216_6*S16;
  FM156_216 = FM116_6*S16+FM216_6*C16;
  CM156_116 = CM166_117*C16-CM166_217*S16;
  CM156_216 = CM166_117*S16+CM166_217*C16;
  CM1513_116 = CM1613_117*C16-CM1613_217*S16;
  CM1513_216 = CM1613_117*S16+CM1613_217*C16;
  CM1514_116 = CM1614_117*C16-CM1614_217*S16;
  CM1515_116 = C16*(s->m[17]*s->dpt[3][27]*s->dpt[3][27]*C16+CM117_15*C17+CM317_15*S17)+S16*S16*(s->In[5][17]+s->m[17]*
 s->dpt[3][27]*s->dpt[3][27]);
  FF14_315 = FF15_216*S15+FF316*C15;
  CF14_215 = CF15_216*C15-CF316*S15;
  CF14_315 = CF15_216*S15+CF316*C15;
  FM141_215 = FM151_216*C15-FM316_1*S15;
  FM141_315 = FM151_216*S15+FM316_1*C15;
  CM141_215 = CM151_216*C15;
  CM141_315 = CM151_216*S15;
  FM142_215 = FM152_216*C15-FM316_2*S15;
  FM142_315 = FM152_216*S15+FM316_2*C15;
  CM142_215 = CM152_216*C15;
  CM142_315 = CM152_216*S15;
  FM143_215 = FM153_216*C15-FM316_3*S15;
  FM143_315 = FM153_216*S15+FM316_3*C15;
  CM143_215 = CM153_216*C15;
  CM143_315 = CM153_216*S15;
  FM144_215 = FM154_216*C15-FM316_4*S15;
  FM144_315 = FM154_216*S15+FM316_4*C15;
  CM144_215 = CM154_216*C15-CM164_317*S15;
  CM144_315 = CM154_216*S15+CM164_317*C15;
  FM145_215 = FM155_216*C15-FM316_5*S15;
  FM145_315 = FM155_216*S15+FM316_5*C15;
  CM145_215 = CM155_216*C15-CM165_317*S15;
  CM145_315 = CM155_216*S15+CM165_317*C15;
  FM146_315 = FM156_216*S15+FM316_6*C15;
  CM146_215 = CM156_216*C15-CM166_317*S15;
  CM146_315 = CM156_216*S15+CM166_317*C15;
  CM1413_215 = CM1513_216*C15-CM1613_317*S15;
  CM1414_215 = (CM1614_117*S16+CM1614_217*C16)*C15-CM1614_317*S15;
  FF113 = -(s->frc[1][13]-FF14_315*S14-FF15_116*C14);
  FF213 = -(s->frc[2][13]-FF15_216*C15+FF316*S15);
  FF313 = -(s->frc[3][13]-FF14_315*C14+FF15_116*S14);
  CF113 = -(s->trq[1][13]-CF14_315*S14-CF15_116*C14-s->dpt[2][24]*(FF14_315*C14-FF15_116*S14));
  CF213 = -(s->trq[2][13]-CF14_215);
  CF313 = -(s->trq[3][13]-CF14_315*C14+CF15_116*S14+s->dpt[2][24]*(FF14_315*S14+FF15_116*C14));
  FM131_114 = FM141_315*S14+FM151_116*C14;
  FM131_314 = FM141_315*C14-FM151_116*S14;
  CM131_114 = CM141_315*S14+CM151_116*C14+s->dpt[2][24]*(FM141_315*C14-FM151_116*S14);
  CM131_314 = CM141_315*C14-CM151_116*S14-s->dpt[2][24]*(FM141_315*S14+FM151_116*C14);
  FM132_114 = FM142_315*S14+FM152_116*C14;
  FM132_314 = FM142_315*C14-FM152_116*S14;
  CM132_114 = CM142_315*S14+CM152_116*C14+s->dpt[2][24]*(FM142_315*C14-FM152_116*S14);
  CM132_314 = CM142_315*C14-CM152_116*S14-s->dpt[2][24]*(FM142_315*S14+FM152_116*C14);
  FM133_114 = FM143_315*S14+FM153_116*C14;
  FM133_314 = FM143_315*C14-FM153_116*S14;
  CM133_114 = CM143_315*S14+CM153_116*C14+s->dpt[2][24]*(FM143_315*C14-FM153_116*S14);
  CM133_314 = CM143_315*C14-CM153_116*S14-s->dpt[2][24]*(FM143_315*S14+FM153_116*C14);
  FM134_114 = FM144_315*S14+FM154_116*C14;
  FM134_314 = FM144_315*C14-FM154_116*S14;
  CM134_114 = CM144_315*S14+CM154_116*C14+s->dpt[2][24]*(FM144_315*C14-FM154_116*S14);
  CM134_314 = CM144_315*C14-CM154_116*S14-s->dpt[2][24]*(FM144_315*S14+FM154_116*C14);
  FM135_114 = FM145_315*S14+FM155_116*C14;
  FM135_314 = FM145_315*C14-FM155_116*S14;
  CM135_114 = CM145_315*S14+CM155_116*C14+s->dpt[2][24]*(FM145_315*C14-FM155_116*S14);
  CM135_314 = CM145_315*C14-CM155_116*S14-s->dpt[2][24]*(FM145_315*S14+FM155_116*C14);
  CM136_114 = CM146_315*S14+CM156_116*C14+s->dpt[2][24]*(FM146_315*C14-FM156_116*S14);
  CM1313_114 = CM1513_116*C14+s->dpt[2][24]*(C14*(C15*(s->m[16]*AlM315_13-FB117_13*S17+FB317_13*C17)+S15*(FM116_13*S16+
 FM216_13*C16))-S14*(FM116_13*C16-FM216_13*S16))+S14*(CM1513_216*S15+CM1613_317*C15);

// = = Block_0_2_0_2_0_9 = = 
 
// Backward Dynamics 

  FF122 = -(s->frc[1][22]-FF23_124-FF125*C25-FF325*S25);
  FF222 = -(s->frc[2][22]+s->frc[2][25]+s->frc[2][24]*C23+s->frc[2][27]*C26+FF23_324*S23+FF26_327*S26);
  FF322 = -(s->frc[3][22]+s->frc[2][24]*S23+FF125*S25-FF23_324*C23-FF325*C25);
  CF122 = -(s->trq[1][22]-CF23_124-CF125*C25-CF325*S25+s->dpt[2][38]*(s->frc[2][24]*S23-FF23_324*C23));
  CF222 = -(s->trq[2][22]-CF225+s->trq[2][24]*C23+CF23_324*S23-s->dpt[1][38]*(s->frc[2][24]*S23-FF23_324*C23));
  CF322 = -(s->trq[3][22]+s->trq[2][24]*S23+CF125*S25-CF23_324*C23-CF325*C25+FF23_124*s->dpt[2][38]+s->dpt[1][38]*(
 s->frc[2][24]*C23+FF23_324*S23));

// = = Block_0_2_0_2_0_12 = = 
 
// Backward Dynamics 

  FF128 = -(s->frc[1][28]-FF29_130-FF31_132-s->m[28]*(AlF16+q[28]*BeF26-(2.0)*qd[28]*OM36+BS16*s->dpt[1][9]+BeF36*
 s->dpt[3][9]));
  FF228 = -(s->frc[2][28]-s->frc[3][30]*S29-s->frc[3][32]*S31-s->m[28]*(AlF25+q[28]*BS56+BeF46*s->dpt[1][9]+BeF66*
 s->dpt[3][9])-FF29_230*C29-FF31_232*C31);
  FF328 = -(s->frc[3][28]+s->frc[3][30]*C29+s->frc[3][32]*C31-s->m[28]*(AlF36+q[28]*BeF86+(2.0)*qd[28]*OM16+BS96*s->dpt[3][9]
 +BeF76*s->dpt[1][9])-FF29_230*S29-FF31_232*S31);
  FB128_1 = s->m[28]*AlM16_1;
  FB228_1 = s->m[28]*AlM25_1;
  FB328_1 = s->m[28]*AlM36_1;
  FB128_2 = s->m[28]*AlM16_2;
  FB228_2 = s->m[28]*AlM25_2;
  FB328_2 = s->m[28]*AlM36_2;
  FB128_3 = s->m[28]*AlM16_3;
  FB228_3 = s->m[28]*S5;
  FB328_3 = s->m[28]*AlM36_3;
  FB128_4 = -s->m[28]*(q[28]*OpM36_4-s->dpt[3][9]*S5);
  FB228_4 = -s->m[28]*(OpM16_4*s->dpt[3][9]-OpM36_4*s->dpt[1][9]);
  FB328_4 = s->m[28]*(q[28]*OpM16_4-s->dpt[1][9]*S5);
  FB128_5 = -q[28]*s->m[28]*S6;
  FB228_5 = s->m[28]*(s->dpt[1][9]*S6-s->dpt[3][9]*C6);
  FB328_5 = q[28]*s->m[28]*C6;

// = = Block_0_2_0_2_0_15 = = 
 
// Backward Dynamics 

  FA135 = -(s->frc[1][35]-s->m[35]*(C35*(AlF134+BeF334*s->dpt[3][50])-S35*(AlF334+BS934*s->dpt[3][50])));
  FA235 = -(s->frc[2][35]-s->m[35]*(AlF234-s->dpt[3][50]*(OpF16-OM234*OM334)));
  FA335 = -(s->frc[3][35]-s->m[35]*(C35*(AlF334+BS934*s->dpt[3][50])+S35*(AlF134+BeF334*s->dpt[3][50])));
  CF135 = -(s->trq[1][35]-s->In[1][35]*(C35*(OpF16-qd[35]*OM334)-S35*(OpF334+qd[35]*OM134))+OM235*OM335*(s->In[5][35]-
 s->In[9][35]));
  CF235 = -(s->trq[2][35]-s->In[5][35]*OpF234-OM135*OM335*(s->In[1][35]-s->In[9][35]));
  CF335 = -(s->trq[3][35]-s->In[9][35]*(C35*(OpF334+qd[35]*OM134)+S35*(OpF16-qd[35]*OM334))+OM135*OM235*(s->In[1][35]-
 s->In[5][35]));
  FB135_1 = s->m[35]*(AlM16_1*C35-AlM334_1*S35);
  FB235_1 = s->m[35]*AlM234_1;
  FB335_1 = s->m[35]*(AlM16_1*S35+AlM334_1*C35);
  FB135_2 = s->m[35]*(AlM16_2*C35-AlM334_2*S35);
  FB235_2 = s->m[35]*AlM234_2;
  FB335_2 = s->m[35]*(AlM16_2*S35+AlM334_2*C35);
  FB135_3 = s->m[35]*(AlM16_3*C35-AlM334_3*S35);
  FB235_3 = s->m[35]*AlM234_3;
  FB335_3 = s->m[35]*(AlM16_3*S35+AlM334_3*C35);
  FB135_4 = -s->m[35]*(AlM334_4*S35-C35*(AlM134_4+OpM234_4*s->dpt[3][50]));
  FB235_4 = s->m[35]*(AlM234_4-OpM16_4*s->dpt[3][50]);
  FB335_4 = s->m[35]*(AlM334_4*C35+S35*(AlM134_4+OpM234_4*s->dpt[3][50]));
  CM135_4 = s->In[1][35]*(OpM16_4*C35-OpM334_4*S35);
  CM235_4 = s->In[5][35]*OpM234_4;
  CM335_4 = s->In[9][35]*(OpM16_4*S35+OpM334_4*C35);
  FB135_5 = -s->m[35]*(AlM334_5*S35-C35*(AlM134_5+OpM234_5*s->dpt[3][50]));
  FB235_5 = s->m[35]*(AlM234_5-s->dpt[3][50]*C6);
  FB335_5 = s->m[35]*(AlM334_5*C35+S35*(AlM134_5+OpM234_5*s->dpt[3][50]));
  CM135_5 = -s->In[1][35]*(OpM334_5*S35-C35*C6);
  CM235_5 = s->In[5][35]*OpM234_5;
  CM335_5 = s->In[9][35]*(OpM334_5*C35+S35*C6);
  FB135_6 = -s->m[35]*(AlM334_6*S35-C35*(AlM134_6+s->dpt[3][50]*C33p34));
  FB235_6 = s->m[35]*AlM234_6;
  FB335_6 = s->m[35]*(AlM334_6*C35+S35*(AlM134_6+s->dpt[3][50]*C33p34));
  CM135_6 = s->In[1][35]*S33p34*S35;
  CM235_6 = s->In[5][35]*C33p34;
  CM335_6 = -s->In[9][35]*S33p34*C35;
  FB235_33 = s->m[35]*(AlM234_33-s->dpt[3][50]);
  FF134 = -(s->frc[1][34]-s->m[34]*AlF134-FA135*C35-FA335*S35);
  FF234 = -(s->frc[2][34]-FA235-s->m[34]*AlF234);
  FF334 = -(s->frc[3][34]-s->m[34]*AlF334+FA135*S35-FA335*C35);
  CF134 = -(s->trq[1][34]-CF135*C35-CF335*S35+FA235*s->dpt[3][50]);
  CF234 = -(s->trq[2][34]-CF235-s->dpt[3][50]*(FA135*C35+FA335*S35));
  CF334 = -(s->trq[3][34]+CF135*S35-CF335*C35);
  FM134_1 = s->m[34]*AlM16_1+FB135_1*C35+FB335_1*S35;
  FM234_1 = FB235_1+s->m[34]*AlM234_1;
  FM334_1 = s->m[34]*AlM334_1-FB135_1*S35+FB335_1*C35;
  CM341_135 = -FB235_1*s->dpt[3][50];
  CM341_235 = s->dpt[3][50]*(FB135_1*C35+FB335_1*S35);
  FM134_2 = s->m[34]*AlM16_2+FB135_2*C35+FB335_2*S35;
  FM234_2 = FB235_2+s->m[34]*AlM234_2;
  FM334_2 = s->m[34]*AlM334_2-FB135_2*S35+FB335_2*C35;
  CM342_135 = -FB235_2*s->dpt[3][50];
  CM342_235 = s->dpt[3][50]*(FB135_2*C35+FB335_2*S35);
  FM134_3 = s->m[34]*AlM16_3+FB135_3*C35+FB335_3*S35;
  FM234_3 = FB235_3+s->m[34]*AlM234_3;
  FM334_3 = s->m[34]*AlM334_3-FB135_3*S35+FB335_3*C35;
  CM343_135 = -FB235_3*s->dpt[3][50];
  CM343_235 = s->dpt[3][50]*(FB135_3*C35+FB335_3*S35);
  FM134_4 = s->m[34]*AlM134_4+FB135_4*C35+FB335_4*S35;
  FM234_4 = FB235_4+s->m[34]*AlM234_4;
  FM334_4 = s->m[34]*AlM334_4-FB135_4*S35+FB335_4*C35;
  CM344_135 = CM135_4*C35+CM335_4*S35-FB235_4*s->dpt[3][50];
  CM344_235 = CM235_4+s->dpt[3][50]*(FB135_4*C35+FB335_4*S35);
  CM344_335 = -(CM135_4*S35-CM335_4*C35);
  FM134_5 = s->m[34]*AlM134_5+FB135_5*C35+FB335_5*S35;
  FM234_5 = FB235_5+s->m[34]*AlM234_5;
  FM334_5 = s->m[34]*AlM334_5-FB135_5*S35+FB335_5*C35;
  CM345_135 = CM135_5*C35+CM335_5*S35-FB235_5*s->dpt[3][50];
  CM345_235 = CM235_5+s->dpt[3][50]*(FB135_5*C35+FB335_5*S35);
  CM345_335 = -(CM135_5*S35-CM335_5*C35);
  FM134_6 = s->m[34]*AlM134_6+FB135_6*C35+FB335_6*S35;
  FM234_6 = FB235_6+s->m[34]*AlM234_6;
  FM334_6 = s->m[34]*AlM334_6-FB135_6*S35+FB335_6*C35;
  CM346_135 = CM135_6*C35+CM335_6*S35-FB235_6*s->dpt[3][50];
  CM346_235 = CM235_6+s->dpt[3][50]*(FB135_6*C35+FB335_6*S35);
  CM346_335 = -(CM135_6*S35-CM335_6*C35);
  CM3433_135 = s->In[1][35]*C35*C35+s->In[9][35]*S35*S35-FB235_33*s->dpt[3][50];
  CM3434_135 = s->In[1][35]*C35*C35+s->In[9][35]*S35*S35+s->m[35]*s->dpt[3][50]*s->dpt[3][50];
  FF133 = -(s->frc[1][33]-FF134);
  FF233 = -(s->frc[2][33]-FF234*C34+FF334*S34);
  FF333 = -(s->frc[3][33]-FF234*S34-FF334*C34);
  CF133 = -(s->trq[1][33]-CF134-s->dpt[2][47]*(FF234*S34+FF334*C34));
  CF233 = -(s->trq[2][33]-CF234*C34+CF334*S34);
  CF333 = -(s->trq[3][33]-CF234*S34-CF334*C34+FF134*s->dpt[2][47]);
  FM331_234 = FM234_1*C34-FM334_1*S34;
  FM331_334 = FM234_1*S34+FM334_1*C34;
  CM331_134 = CM341_135+s->dpt[2][47]*(FM234_1*S34+FM334_1*C34);
  CM331_234 = CM341_235*C34;
  CM331_334 = CM341_235*S34-FM134_1*s->dpt[2][47];
  FM332_234 = FM234_2*C34-FM334_2*S34;
  FM332_334 = FM234_2*S34+FM334_2*C34;
  CM332_134 = CM342_135+s->dpt[2][47]*(FM234_2*S34+FM334_2*C34);
  CM332_234 = CM342_235*C34;
  CM332_334 = CM342_235*S34-FM134_2*s->dpt[2][47];
  FM333_234 = FM234_3*C34-FM334_3*S34;
  FM333_334 = FM234_3*S34+FM334_3*C34;
  CM333_134 = CM343_135+s->dpt[2][47]*(FM234_3*S34+FM334_3*C34);
  CM333_234 = CM343_235*C34;
  CM333_334 = CM343_235*S34-FM134_3*s->dpt[2][47];
  FM334_234 = FM234_4*C34-FM334_4*S34;
  FM334_334 = FM234_4*S34+FM334_4*C34;
  CM334_134 = CM344_135+s->dpt[2][47]*(FM234_4*S34+FM334_4*C34);
  CM334_234 = CM344_235*C34-CM344_335*S34;
  CM334_334 = CM344_235*S34+CM344_335*C34-FM134_4*s->dpt[2][47];
  FM335_234 = FM234_5*C34-FM334_5*S34;
  FM335_334 = FM234_5*S34+FM334_5*C34;
  CM335_134 = CM345_135+s->dpt[2][47]*(FM234_5*S34+FM334_5*C34);
  CM335_234 = CM345_235*C34-CM345_335*S34;
  CM335_334 = CM345_235*S34+CM345_335*C34-FM134_5*s->dpt[2][47];
  CM336_134 = CM346_135+s->dpt[2][47]*(FM234_6*S34+FM334_6*C34);
  CM3333_134 = CM3433_135+s->dpt[2][47]*(AlM334_33*C34*(s->m[34]+s->m[35])+S34*(FB235_33+s->m[34]*AlM234_33));

// = = Block_0_2_0_2_0_16 = = 
 
// Backward Dynamics 

  FA138 = -(s->frc[1][38]-s->m[38]*(C38*(AlF137+BeF337*s->dpt[3][54])-S38*(AlF337+BS937*s->dpt[3][54])));
  FA238 = -(s->frc[2][38]-s->m[38]*(AlF237-s->dpt[3][54]*(OpF16-OM237*OM337)));
  FA338 = -(s->frc[3][38]-s->m[38]*(C38*(AlF337+BS937*s->dpt[3][54])+S38*(AlF137+BeF337*s->dpt[3][54])));
  CF138 = -(s->trq[1][38]-s->In[1][38]*(C38*(OpF16-qd[38]*OM337)-S38*(OpF337+qd[38]*OM137))+OM238*OM338*(s->In[5][38]-
 s->In[9][38]));
  CF238 = -(s->trq[2][38]-s->In[5][38]*OpF237-OM138*OM338*(s->In[1][38]-s->In[9][38]));
  CF338 = -(s->trq[3][38]-s->In[9][38]*(C38*(OpF337+qd[38]*OM137)+S38*(OpF16-qd[38]*OM337))+OM138*OM238*(s->In[1][38]-
 s->In[5][38]));
  FB138_1 = s->m[38]*(AlM16_1*C38-AlM337_1*S38);
  FB238_1 = s->m[38]*AlM237_1;
  FB338_1 = s->m[38]*(AlM16_1*S38+AlM337_1*C38);
  FB138_2 = s->m[38]*(AlM16_2*C38-AlM337_2*S38);
  FB238_2 = s->m[38]*AlM237_2;
  FB338_2 = s->m[38]*(AlM16_2*S38+AlM337_2*C38);
  FB138_3 = s->m[38]*(AlM16_3*C38-AlM337_3*S38);
  FB238_3 = s->m[38]*AlM237_3;
  FB338_3 = s->m[38]*(AlM16_3*S38+AlM337_3*C38);
  FB138_4 = -s->m[38]*(AlM337_4*S38-C38*(AlM137_4+OpM237_4*s->dpt[3][54]));
  FB238_4 = s->m[38]*(AlM237_4-OpM16_4*s->dpt[3][54]);
  FB338_4 = s->m[38]*(AlM337_4*C38+S38*(AlM137_4+OpM237_4*s->dpt[3][54]));
  CM138_4 = s->In[1][38]*(OpM16_4*C38-OpM337_4*S38);
  CM238_4 = s->In[5][38]*OpM237_4;
  CM338_4 = s->In[9][38]*(OpM16_4*S38+OpM337_4*C38);
  FB138_5 = -s->m[38]*(AlM337_5*S38-C38*(AlM137_5+OpM237_5*s->dpt[3][54]));
  FB238_5 = s->m[38]*(AlM237_5-s->dpt[3][54]*C6);
  FB338_5 = s->m[38]*(AlM337_5*C38+S38*(AlM137_5+OpM237_5*s->dpt[3][54]));
  CM138_5 = -s->In[1][38]*(OpM337_5*S38-C38*C6);
  CM238_5 = s->In[5][38]*OpM237_5;
  CM338_5 = s->In[9][38]*(OpM337_5*C38+S38*C6);
  FB138_6 = -s->m[38]*(AlM337_6*S38-C38*(AlM137_6+s->dpt[3][54]*C36p37));
  FB238_6 = s->m[38]*AlM237_6;
  FB338_6 = s->m[38]*(AlM337_6*C38+S38*(AlM137_6+s->dpt[3][54]*C36p37));
  CM138_6 = s->In[1][38]*S36p37*S38;
  CM238_6 = s->In[5][38]*C36p37;
  CM338_6 = -s->In[9][38]*S36p37*C38;
  FB238_36 = s->m[38]*(AlM237_36-s->dpt[3][54]);
  FF137 = -(s->frc[1][37]-s->m[37]*AlF137-FA138*C38-FA338*S38);
  FF237 = -(s->frc[2][37]-FA238-s->m[37]*AlF237);
  FF337 = -(s->frc[3][37]-s->m[37]*AlF337+FA138*S38-FA338*C38);
  CF137 = -(s->trq[1][37]-CF138*C38-CF338*S38+FA238*s->dpt[3][54]);
  CF237 = -(s->trq[2][37]-CF238-s->dpt[3][54]*(FA138*C38+FA338*S38));
  CF337 = -(s->trq[3][37]+CF138*S38-CF338*C38);
  FM137_1 = s->m[37]*AlM16_1+FB138_1*C38+FB338_1*S38;
  FM237_1 = FB238_1+s->m[37]*AlM237_1;
  FM337_1 = s->m[37]*AlM337_1-FB138_1*S38+FB338_1*C38;
  CM371_138 = -FB238_1*s->dpt[3][54];
  CM371_238 = s->dpt[3][54]*(FB138_1*C38+FB338_1*S38);
  FM137_2 = s->m[37]*AlM16_2+FB138_2*C38+FB338_2*S38;
  FM237_2 = FB238_2+s->m[37]*AlM237_2;
  FM337_2 = s->m[37]*AlM337_2-FB138_2*S38+FB338_2*C38;
  CM372_138 = -FB238_2*s->dpt[3][54];
  CM372_238 = s->dpt[3][54]*(FB138_2*C38+FB338_2*S38);
  FM137_3 = s->m[37]*AlM16_3+FB138_3*C38+FB338_3*S38;
  FM237_3 = FB238_3+s->m[37]*AlM237_3;
  FM337_3 = s->m[37]*AlM337_3-FB138_3*S38+FB338_3*C38;
  CM373_138 = -FB238_3*s->dpt[3][54];
  CM373_238 = s->dpt[3][54]*(FB138_3*C38+FB338_3*S38);
  FM137_4 = s->m[37]*AlM137_4+FB138_4*C38+FB338_4*S38;
  FM237_4 = FB238_4+s->m[37]*AlM237_4;
  FM337_4 = s->m[37]*AlM337_4-FB138_4*S38+FB338_4*C38;
  CM374_138 = CM138_4*C38+CM338_4*S38-FB238_4*s->dpt[3][54];
  CM374_238 = CM238_4+s->dpt[3][54]*(FB138_4*C38+FB338_4*S38);
  CM374_338 = -(CM138_4*S38-CM338_4*C38);
  FM137_5 = s->m[37]*AlM137_5+FB138_5*C38+FB338_5*S38;
  FM237_5 = FB238_5+s->m[37]*AlM237_5;
  FM337_5 = s->m[37]*AlM337_5-FB138_5*S38+FB338_5*C38;
  CM375_138 = CM138_5*C38+CM338_5*S38-FB238_5*s->dpt[3][54];
  CM375_238 = CM238_5+s->dpt[3][54]*(FB138_5*C38+FB338_5*S38);
  CM375_338 = -(CM138_5*S38-CM338_5*C38);
  FM137_6 = s->m[37]*AlM137_6+FB138_6*C38+FB338_6*S38;
  FM237_6 = FB238_6+s->m[37]*AlM237_6;
  FM337_6 = s->m[37]*AlM337_6-FB138_6*S38+FB338_6*C38;
  CM376_138 = CM138_6*C38+CM338_6*S38-FB238_6*s->dpt[3][54];
  CM376_238 = CM238_6+s->dpt[3][54]*(FB138_6*C38+FB338_6*S38);
  CM376_338 = -(CM138_6*S38-CM338_6*C38);
  CM3736_138 = s->In[1][38]*C38*C38+s->In[9][38]*S38*S38-FB238_36*s->dpt[3][54];
  CM3737_138 = s->In[1][38]*C38*C38+s->In[9][38]*S38*S38+s->m[38]*s->dpt[3][54]*s->dpt[3][54];
  FF136 = -(s->frc[1][36]-FF137);
  FF236 = -(s->frc[2][36]-FF237*C37+FF337*S37);
  FF336 = -(s->frc[3][36]-FF237*S37-FF337*C37);
  CF136 = -(s->trq[1][36]-CF137-s->dpt[2][52]*(FF237*S37+FF337*C37));
  CF236 = -(s->trq[2][36]-CF237*C37+CF337*S37);
  CF336 = -(s->trq[3][36]-CF237*S37-CF337*C37+FF137*s->dpt[2][52]);
  FM361_237 = FM237_1*C37-FM337_1*S37;
  FM361_337 = FM237_1*S37+FM337_1*C37;
  CM361_137 = CM371_138+s->dpt[2][52]*(FM237_1*S37+FM337_1*C37);
  CM361_237 = CM371_238*C37;
  CM361_337 = CM371_238*S37-FM137_1*s->dpt[2][52];
  FM362_237 = FM237_2*C37-FM337_2*S37;
  FM362_337 = FM237_2*S37+FM337_2*C37;
  CM362_137 = CM372_138+s->dpt[2][52]*(FM237_2*S37+FM337_2*C37);
  CM362_237 = CM372_238*C37;
  CM362_337 = CM372_238*S37-FM137_2*s->dpt[2][52];
  FM363_237 = FM237_3*C37-FM337_3*S37;
  FM363_337 = FM237_3*S37+FM337_3*C37;
  CM363_137 = CM373_138+s->dpt[2][52]*(FM237_3*S37+FM337_3*C37);
  CM363_237 = CM373_238*C37;
  CM363_337 = CM373_238*S37-FM137_3*s->dpt[2][52];
  FM364_237 = FM237_4*C37-FM337_4*S37;
  FM364_337 = FM237_4*S37+FM337_4*C37;
  CM364_137 = CM374_138+s->dpt[2][52]*(FM237_4*S37+FM337_4*C37);
  CM364_237 = CM374_238*C37-CM374_338*S37;
  CM364_337 = CM374_238*S37+CM374_338*C37-FM137_4*s->dpt[2][52];
  FM365_237 = FM237_5*C37-FM337_5*S37;
  FM365_337 = FM237_5*S37+FM337_5*C37;
  CM365_137 = CM375_138+s->dpt[2][52]*(FM237_5*S37+FM337_5*C37);
  CM365_237 = CM375_238*C37-CM375_338*S37;
  CM365_337 = CM375_238*S37+CM375_338*C37-FM137_5*s->dpt[2][52];
  CM366_137 = CM376_138+s->dpt[2][52]*(FM237_6*S37+FM337_6*C37);
  CM3636_137 = CM3736_138+s->dpt[2][52]*(AlM337_36*C37*(s->m[37]+s->m[38])+S37*(FB238_36+s->m[37]*AlM237_36));

// = = Block_0_2_0_2_0_17 = = 
 
// Backward Dynamics 

  FF139 = -(s->frc[1][39]-FF43_144-FF140*C40-FF340*S40);
  FF239 = -(s->frc[2][39]+s->frc[2][40]+s->frc[2][42]*C41+s->frc[2][44]*C43+FF41_342*S41+FF43_344*S43);
  FF339 = -(s->frc[3][39]+s->frc[2][44]*S43+FF140*S40-FF340*C40-FF43_344*C43);
  CF139 = -(s->trq[1][39]-CF43_144-CF140*C40-CF340*S40+s->dpt[2][58]*(s->frc[2][44]*S43-FF43_344*C43));
  CF239 = -(s->trq[2][39]-CF240+s->trq[2][44]*C43+CF43_344*S43-s->dpt[1][58]*(s->frc[2][44]*S43-FF43_344*C43));
  CF339 = -(s->trq[3][39]+s->trq[2][44]*S43+CF140*S40-CF340*C40-CF43_344*C43+FF43_144*s->dpt[2][58]+s->dpt[1][58]*(
 s->frc[2][44]*C43+FF43_344*S43));

// = = Block_0_2_0_2_0_20 = = 
 
// Backward Dynamics 

  FA148 = -(s->frc[1][48]-s->m[48]*(AlF148+BS148*s->l[1][48]+BeF348*s->l[3][48]));
  FA248 = -(s->frc[2][48]-s->m[48]*(AlF248+BeF448*s->l[1][48]+BeF648*s->l[3][48]));
  FA348 = -(s->frc[3][48]-s->m[48]*(AlF348+BS948*s->l[3][48]+BeF748*s->l[1][48]));
  FF148 = FA148+FA153+FA149*C49+FA150*C50+FA151*C51+FA152*C52+FA349*S49+FA350*S50+FA351*S51+FA352*S52;
  FF248 = FA248+FA249+FA250+FA251+FA252+FA253;
  FF348 = FA348+FA353-FA149*S49-FA150*S50-FA151*S51-FA152*S52+FA349*C49+FA350*C50+FA351*C51+FA352*C52;
  CF148 = -(s->trq[1][48]+s->trq[1][53]-s->In[1][48]*OpF147-s->In[1][53]*OpF147-CF149*C49-CF150*C50-CF151*C51-CF152*C52-
 CF349*S49-CF350*S50-CF351*S51-CF352*S52+Dz533*FA253+FA248*s->l[3][48]+FA249*s->dpt[3][62]+FA250*s->dpt[3][63]+FA251*
 s->dpt[3][64]+FA252*s->dpt[3][65]+OM247*OM347*(s->In[5][48]-s->In[9][48])+OM247*OM347*(s->In[5][53]-s->In[9][53])+
 s->dpt[2][62]*(FA149*S49-FA349*C49)+s->dpt[2][63]*(FA150*S50-FA350*C50)+s->dpt[2][64]*(FA151*S51-FA351*C51)+s->dpt[2][65]*(
 FA152*S52-FA352*C52));
  FB148_1 = s->m[48]*AlM147_1;
  FB248_1 = s->m[48]*AlM246_1;
  FB348_1 = s->m[48]*AlM347_1;
  FM148_1 = FB148_1+FB153_1+FB149_1*C49+FB150_1*C50+FB151_1*C51+FB152_1*C52+FB349_1*S49+FB350_1*S50+FB351_1*S51+FB352_1*
 S52;
  FM248_1 = FB248_1+FB249_1+FB250_1+FB251_1+FB252_1+FB253_1;
  FM348_1 = FB348_1+FB353_1-FB149_1*S49-FB150_1*S50-FB151_1*S51-FB152_1*S52+FB349_1*C49+FB350_1*C50+FB351_1*C51+FB352_1*
 C52;
  CM148_1 = -(Dz533*FB253_1+FB248_1*s->l[3][48]+FB249_1*s->dpt[3][62]+FB250_1*s->dpt[3][63]+FB251_1*s->dpt[3][64]+
 FB252_1*s->dpt[3][65]+s->dpt[2][62]*(FB149_1*S49-FB349_1*C49)+s->dpt[2][63]*(FB150_1*S50-FB350_1*C50)+s->dpt[2][64]*(FB151_1
 *S51-FB351_1*C51)+s->dpt[2][65]*(FB152_1*S52-FB352_1*C52));
  FB148_2 = s->m[48]*AlM147_2;
  FB248_2 = s->m[48]*AlM246_2;
  FB348_2 = s->m[48]*AlM347_2;
  FM148_2 = FB148_2+FB153_2+FB149_2*C49+FB150_2*C50+FB151_2*C51+FB152_2*C52+FB349_2*S49+FB350_2*S50+FB351_2*S51+FB352_2*
 S52;
  FM248_2 = FB248_2+FB249_2+FB250_2+FB251_2+FB252_2+FB253_2;
  FM348_2 = FB348_2+FB353_2-FB149_2*S49-FB150_2*S50-FB151_2*S51-FB152_2*S52+FB349_2*C49+FB350_2*C50+FB351_2*C51+FB352_2*
 C52;
  CM148_2 = -(Dz533*FB253_2+FB248_2*s->l[3][48]+FB249_2*s->dpt[3][62]+FB250_2*s->dpt[3][63]+FB251_2*s->dpt[3][64]+
 FB252_2*s->dpt[3][65]+s->dpt[2][62]*(FB149_2*S49-FB349_2*C49)+s->dpt[2][63]*(FB150_2*S50-FB350_2*C50)+s->dpt[2][64]*(FB151_2
 *S51-FB351_2*C51)+s->dpt[2][65]*(FB152_2*S52-FB352_2*C52));
  FB148_3 = s->m[48]*AlM147_3;
  FB248_3 = s->m[48]*AlM246_3;
  FB348_3 = s->m[48]*AlM347_3;
  FM148_3 = FB148_3+FB153_3+FB149_3*C49+FB150_3*C50+FB151_3*C51+FB152_3*C52+FB349_3*S49+FB350_3*S50+FB351_3*S51+FB352_3*
 S52;
  FM248_3 = FB248_3+FB249_3+FB250_3+FB251_3+FB252_3+FB253_3;
  FM348_3 = FB348_3+FB353_3-FB149_3*S49-FB150_3*S50-FB151_3*S51-FB152_3*S52+FB349_3*C49+FB350_3*C50+FB351_3*C51+FB352_3*
 C52;
  CM148_3 = -(Dz533*FB253_3+FB248_3*s->l[3][48]+FB249_3*s->dpt[3][62]+FB250_3*s->dpt[3][63]+FB251_3*s->dpt[3][64]+
 FB252_3*s->dpt[3][65]+s->dpt[2][62]*(FB149_3*S49-FB349_3*C49)+s->dpt[2][63]*(FB150_3*S50-FB350_3*C50)+s->dpt[2][64]*(FB151_3
 *S51-FB351_3*C51)+s->dpt[2][65]*(FB152_3*S52-FB352_3*C52));
  FB148_4 = s->m[48]*(AlM147_4+OpM246_4*s->l[3][48]);
  FB248_4 = s->m[48]*(AlM248_4-OpM147_4*s->l[3][48]+OpM347_4*s->l[1][48]);
  FB348_4 = s->m[48]*(AlM348_4-OpM246_4*s->l[1][48]);
  FM148_4 = FB148_4+FB153_4+FB149_4*C49+FB150_4*C50+FB151_4*C51+FB152_4*C52+FB349_4*S49+FB350_4*S50+FB351_4*S51+FB352_4*
 S52;
  FM248_4 = FB248_4+FB249_4+FB250_4+FB251_4+FB252_4+FB253_4;
  FM348_4 = FB348_4+FB353_4-FB149_4*S49-FB150_4*S50-FB151_4*S51-FB152_4*S52+FB349_4*C49+FB350_4*C50+FB351_4*C51+FB352_4*
 C52;
  CM148_4 = CM149_4*C49+CM150_4*C50+CM151_4*C51+CM152_4*C52+CM349_4*S49+CM350_4*S50+CM351_4*S51+CM352_4*S52-Dz533*
 FB253_4-FB248_4*s->l[3][48]-FB249_4*s->dpt[3][62]-FB250_4*s->dpt[3][63]-FB251_4*s->dpt[3][64]-FB252_4*s->dpt[3][65]+OpM147_4
 *(s->In[1][48]+s->In[1][53])-s->dpt[2][62]*(FB149_4*S49-FB349_4*C49)-s->dpt[2][63]*(FB150_4*S50-FB350_4*C50)-s->dpt[2][64]*(
 FB151_4*S51-FB351_4*C51)-s->dpt[2][65]*(FB152_4*S52-FB352_4*C52);
  FB148_5 = s->m[48]*(AlM147_5+OpM246_5*s->l[3][48]);
  FB248_5 = s->m[48]*(AlM248_5-OpM147_5*s->l[3][48]+OpM347_5*s->l[1][48]);
  FB348_5 = s->m[48]*(AlM348_5-OpM246_5*s->l[1][48]);
  FM148_5 = FB148_5+FB153_5+FB149_5*C49+FB150_5*C50+FB151_5*C51+FB152_5*C52+FB349_5*S49+FB350_5*S50+FB351_5*S51+FB352_5*
 S52;
  FM248_5 = FB248_5+FB249_5+FB250_5+FB251_5+FB252_5+FB253_5;
  FM348_5 = FB348_5+FB353_5-FB149_5*S49-FB150_5*S50-FB151_5*S51-FB152_5*S52+FB349_5*C49+FB350_5*C50+FB351_5*C51+FB352_5*
 C52;
  CM148_5 = CM149_5*C49+CM150_5*C50+CM151_5*C51+CM152_5*C52+CM349_5*S49+CM350_5*S50+CM351_5*S51+CM352_5*S52-Dz533*
 FB253_5-FB248_5*s->l[3][48]-FB249_5*s->dpt[3][62]-FB250_5*s->dpt[3][63]-FB251_5*s->dpt[3][64]-FB252_5*s->dpt[3][65]+OpM147_5
 *(s->In[1][48]+s->In[1][53])-s->dpt[2][62]*(FB149_5*S49-FB349_5*C49)-s->dpt[2][63]*(FB150_5*S50-FB350_5*C50)-s->dpt[2][64]*(
 FB151_5*S51-FB351_5*C51)-s->dpt[2][65]*(FB152_5*S52-FB352_5*C52);
  FB148_6 = s->m[48]*(AlM147_6+OpM246_6*s->l[3][48]);
  FB248_6 = s->m[48]*(AlM248_6-OpM147_6*s->l[3][48]+OpM347_6*s->l[1][48]);
  FB348_6 = s->m[48]*(AlM348_6-OpM246_6*s->l[1][48]);
  FM148_6 = FB148_6+FB153_6+FB149_6*C49+FB150_6*C50+FB151_6*C51+FB152_6*C52+FB349_6*S49+FB350_6*S50+FB351_6*S51+FB352_6*
 S52;
  FM248_6 = FB248_6+FB249_6+FB250_6+FB251_6+FB252_6+FB253_6;
  FM348_6 = FB348_6+FB353_6-FB149_6*S49-FB150_6*S50-FB151_6*S51-FB152_6*S52+FB349_6*C49+FB350_6*C50+FB351_6*C51+FB352_6*
 C52;
  CM148_6 = CM149_6*C49+CM150_6*C50+CM151_6*C51+CM152_6*C52+CM349_6*S49+CM350_6*S50+CM351_6*S51+CM352_6*S52-Dz533*
 FB253_6-FB248_6*s->l[3][48]-FB249_6*s->dpt[3][62]-FB250_6*s->dpt[3][63]-FB251_6*s->dpt[3][64]-FB252_6*s->dpt[3][65]+OpM147_6
 *(s->In[1][48]+s->In[1][53])-s->dpt[2][62]*(FB149_6*S49-FB349_6*C49)-s->dpt[2][63]*(FB150_6*S50-FB350_6*C50)-s->dpt[2][64]*(
 FB151_6*S51-FB351_6*C51)-s->dpt[2][65]*(FB152_6*S52-FB352_6*C52);
  FB148_45 = s->m[48]*s->l[3][48]*S46;
  FB248_45 = s->m[48]*(AlM248_45-OpM147_45*s->l[3][48]+OpM347_45*s->l[1][48]);
  FB348_45 = s->m[48]*(AlM348_45-s->l[1][48]*S46);
  FM148_45 = FB148_45+FB153_45+FB149_45*C49+FB150_45*C50+FB151_45*C51+FB152_45*C52+FB349_45*S49+FB350_45*S50+FB351_45*
 S51+FB352_45*S52;
  CM148_45 = CM149_45*C49+CM150_45*C50+CM151_45*C51+CM152_45*C52+CM349_45*S49+CM350_45*S50+CM351_45*S51+CM352_45*S52-
 Dz533*FB253_45-FB248_45*s->l[3][48]-FB249_45*s->dpt[3][62]-FB250_45*s->dpt[3][63]-FB251_45*s->dpt[3][64]-FB252_45*
 s->dpt[3][65]+OpM147_45*(s->In[1][48]+s->In[1][53])-s->dpt[2][62]*(FB149_45*S49-FB349_45*C49)-s->dpt[2][63]*(FB150_45*S50-
 FB350_45*C50)-s->dpt[2][64]*(FB151_45*S51-FB351_45*C51)-s->dpt[2][65]*(FB152_45*S52-FB352_45*C52);
  FB248_46 = s->m[48]*(AlM248_46+s->l[1][48]*S47-s->l[3][48]*C47);
  FM148_46 = -S47*(s->m[49]*s->dpt[2][62]+s->m[50]*s->dpt[2][63]+s->m[51]*s->dpt[2][64]+s->m[52]*s->dpt[2][65]);
  FB148_47 = s->m[48]*s->l[3][48];
  FB348_47 = -s->m[48]*(q[48]+s->l[1][48]);
  FM148_47 = FB148_47+FB153_47+FB149_47*C49+FB150_47*C50+FB151_47*C51+FB152_47*C52+FB349_47*S49+FB350_47*S50+FB351_47*
 S51+FB352_47*S52;
  FM148_48 = s->m[48]+s->m[49]+s->m[50]+s->m[51]+s->m[52]+s->m[53];
  CF47_248 = -(s->trq[2][48]+s->trq[2][53]-CF249-CF250-CF251-CF252+q[48]*FF348-s->In[5][48]*OpF246-s->In[5][53]*OpF246-
 Dz533*FA153-FA148*s->l[3][48]+FA348*s->l[1][48]+FA353*s->dpt[1][66]-OM147*OM347*(s->In[1][48]-s->In[9][48])-OM147*OM347*(
 s->In[1][53]-s->In[9][53])-s->dpt[1][62]*(FA149*S49-FA349*C49)-s->dpt[1][63]*(FA150*S50-FA350*C50)-s->dpt[1][64]*(FA151*S51-
 FA351*C51)-s->dpt[1][65]*(FA152*S52-FA352*C52)-s->dpt[3][62]*(FA149*C49+FA349*S49)-s->dpt[3][63]*(FA150*C50+FA350*S50)-
 s->dpt[3][64]*(FA151*C51+FA351*S51)-s->dpt[3][65]*(FA152*C52+FA352*S52));
  CF47_348 = -(s->trq[3][48]+s->trq[3][53]-q[48]*FF248-s->In[9][48]*OpF347-s->In[9][53]*OpF347+CF149*S49+CF150*S50+CF151
 *S51+CF152*S52-CF349*C49-CF350*C50-CF351*C51-CF352*C52-FA248*s->l[1][48]-FA249*s->dpt[1][62]-FA250*s->dpt[1][63]-FA251*
 s->dpt[1][64]-FA252*s->dpt[1][65]-FA253*s->dpt[1][66]+OM147*OM247*(s->In[1][48]-s->In[5][48])+OM147*OM247*(s->In[1][53]-
 s->In[5][53])+s->dpt[2][62]*(FA149*C49+FA349*S49)+s->dpt[2][63]*(FA150*C50+FA350*S50)+s->dpt[2][64]*(FA151*C51+FA351*S51)+
 s->dpt[2][65]*(FA152*C52+FA352*S52));
  CM471_248 = Dz533*FB153_1+FB148_1*s->l[3][48]-FB348_1*s->l[1][48]-FB353_1*s->dpt[1][66]+s->dpt[1][62]*(FB149_1*S49-
 FB349_1*C49)+s->dpt[1][63]*(FB150_1*S50-FB350_1*C50)+s->dpt[1][64]*(FB151_1*S51-FB351_1*C51)+s->dpt[1][65]*(FB152_1*S52-
 FB352_1*C52)+s->dpt[3][62]*(FB149_1*C49+FB349_1*S49)+s->dpt[3][63]*(FB150_1*C50+FB350_1*S50)+s->dpt[3][64]*(FB151_1*C51+
 FB351_1*S51)+s->dpt[3][65]*(FB152_1*C52+FB352_1*S52)-q[48]*FM348_1;
  CM471_348 = q[48]*FM248_1+FB248_1*s->l[1][48]+FB249_1*s->dpt[1][62]+FB250_1*s->dpt[1][63]+FB251_1*s->dpt[1][64]+
 FB252_1*s->dpt[1][65]+FB253_1*s->dpt[1][66]-s->dpt[2][62]*(FB149_1*C49+FB349_1*S49)-s->dpt[2][63]*(FB150_1*C50+FB350_1*S50)-
 s->dpt[2][64]*(FB151_1*C51+FB351_1*S51)-s->dpt[2][65]*(FB152_1*C52+FB352_1*S52);
  CM472_248 = Dz533*FB153_2+FB148_2*s->l[3][48]-FB348_2*s->l[1][48]-FB353_2*s->dpt[1][66]+s->dpt[1][62]*(FB149_2*S49-
 FB349_2*C49)+s->dpt[1][63]*(FB150_2*S50-FB350_2*C50)+s->dpt[1][64]*(FB151_2*S51-FB351_2*C51)+s->dpt[1][65]*(FB152_2*S52-
 FB352_2*C52)+s->dpt[3][62]*(FB149_2*C49+FB349_2*S49)+s->dpt[3][63]*(FB150_2*C50+FB350_2*S50)+s->dpt[3][64]*(FB151_2*C51+
 FB351_2*S51)+s->dpt[3][65]*(FB152_2*C52+FB352_2*S52)-q[48]*FM348_2;
  CM472_348 = q[48]*FM248_2+FB248_2*s->l[1][48]+FB249_2*s->dpt[1][62]+FB250_2*s->dpt[1][63]+FB251_2*s->dpt[1][64]+
 FB252_2*s->dpt[1][65]+FB253_2*s->dpt[1][66]-s->dpt[2][62]*(FB149_2*C49+FB349_2*S49)-s->dpt[2][63]*(FB150_2*C50+FB350_2*S50)-
 s->dpt[2][64]*(FB151_2*C51+FB351_2*S51)-s->dpt[2][65]*(FB152_2*C52+FB352_2*S52);
  CM473_248 = Dz533*FB153_3+FB148_3*s->l[3][48]-FB348_3*s->l[1][48]-FB353_3*s->dpt[1][66]+s->dpt[1][62]*(FB149_3*S49-
 FB349_3*C49)+s->dpt[1][63]*(FB150_3*S50-FB350_3*C50)+s->dpt[1][64]*(FB151_3*S51-FB351_3*C51)+s->dpt[1][65]*(FB152_3*S52-
 FB352_3*C52)+s->dpt[3][62]*(FB149_3*C49+FB349_3*S49)+s->dpt[3][63]*(FB150_3*C50+FB350_3*S50)+s->dpt[3][64]*(FB151_3*C51+
 FB351_3*S51)+s->dpt[3][65]*(FB152_3*C52+FB352_3*S52)-q[48]*FM348_3;
  CM473_348 = q[48]*FM248_3+FB248_3*s->l[1][48]+FB249_3*s->dpt[1][62]+FB250_3*s->dpt[1][63]+FB251_3*s->dpt[1][64]+
 FB252_3*s->dpt[1][65]+FB253_3*s->dpt[1][66]-s->dpt[2][62]*(FB149_3*C49+FB349_3*S49)-s->dpt[2][63]*(FB150_3*C50+FB350_3*S50)-
 s->dpt[2][64]*(FB151_3*C51+FB351_3*S51)-s->dpt[2][65]*(FB152_3*C52+FB352_3*S52);
  CM474_248 = CM249_4+CM250_4+CM251_4+CM252_4-q[48]*FM348_4+s->In[5][48]*OpM246_4+s->In[5][53]*OpM246_4+Dz533*FB153_4+
 FB148_4*s->l[3][48]-FB348_4*s->l[1][48]-FB353_4*s->dpt[1][66]+s->dpt[1][62]*(FB149_4*S49-FB349_4*C49)+s->dpt[1][63]*(FB150_4
 *S50-FB350_4*C50)+s->dpt[1][64]*(FB151_4*S51-FB351_4*C51)+s->dpt[1][65]*(FB152_4*S52-FB352_4*C52)+s->dpt[3][62]*(FB149_4*C49
 +FB349_4*S49)+s->dpt[3][63]*(FB150_4*C50+FB350_4*S50)+s->dpt[3][64]*(FB151_4*C51+FB351_4*S51)+s->dpt[3][65]*(FB152_4*C52+
 FB352_4*S52);
  CM474_348 = q[48]*FM248_4-CM149_4*S49-CM150_4*S50-CM151_4*S51-CM152_4*S52+CM349_4*C49+CM350_4*C50+CM351_4*C51+CM352_4*
 C52+FB248_4*s->l[1][48]+FB249_4*s->dpt[1][62]+FB250_4*s->dpt[1][63]+FB251_4*s->dpt[1][64]+FB252_4*s->dpt[1][65]+FB253_4*
 s->dpt[1][66]+OpM347_4*(s->In[9][48]+s->In[9][53])-s->dpt[2][62]*(FB149_4*C49+FB349_4*S49)-s->dpt[2][63]*(FB150_4*C50+
 FB350_4*S50)-s->dpt[2][64]*(FB151_4*C51+FB351_4*S51)-s->dpt[2][65]*(FB152_4*C52+FB352_4*S52);
  CM475_248 = CM249_5+CM250_5+CM251_5+CM252_5-q[48]*FM348_5+s->In[5][48]*OpM246_5+s->In[5][53]*OpM246_5+Dz533*FB153_5+
 FB148_5*s->l[3][48]-FB348_5*s->l[1][48]-FB353_5*s->dpt[1][66]+s->dpt[1][62]*(FB149_5*S49-FB349_5*C49)+s->dpt[1][63]*(FB150_5
 *S50-FB350_5*C50)+s->dpt[1][64]*(FB151_5*S51-FB351_5*C51)+s->dpt[1][65]*(FB152_5*S52-FB352_5*C52)+s->dpt[3][62]*(FB149_5*C49
 +FB349_5*S49)+s->dpt[3][63]*(FB150_5*C50+FB350_5*S50)+s->dpt[3][64]*(FB151_5*C51+FB351_5*S51)+s->dpt[3][65]*(FB152_5*C52+
 FB352_5*S52);
  CM475_348 = q[48]*FM248_5-CM149_5*S49-CM150_5*S50-CM151_5*S51-CM152_5*S52+CM349_5*C49+CM350_5*C50+CM351_5*C51+CM352_5*
 C52+FB248_5*s->l[1][48]+FB249_5*s->dpt[1][62]+FB250_5*s->dpt[1][63]+FB251_5*s->dpt[1][64]+FB252_5*s->dpt[1][65]+FB253_5*
 s->dpt[1][66]+OpM347_5*(s->In[9][48]+s->In[9][53])-s->dpt[2][62]*(FB149_5*C49+FB349_5*S49)-s->dpt[2][63]*(FB150_5*C50+
 FB350_5*S50)-s->dpt[2][64]*(FB151_5*C51+FB351_5*S51)-s->dpt[2][65]*(FB152_5*C52+FB352_5*S52);
  CM476_248 = CM249_6+CM250_6+CM251_6+CM252_6-q[48]*FM348_6+s->In[5][48]*OpM246_6+s->In[5][53]*OpM246_6+Dz533*FB153_6+
 FB148_6*s->l[3][48]-FB348_6*s->l[1][48]-FB353_6*s->dpt[1][66]+s->dpt[1][62]*(FB149_6*S49-FB349_6*C49)+s->dpt[1][63]*(FB150_6
 *S50-FB350_6*C50)+s->dpt[1][64]*(FB151_6*S51-FB351_6*C51)+s->dpt[1][65]*(FB152_6*S52-FB352_6*C52)+s->dpt[3][62]*(FB149_6*C49
 +FB349_6*S49)+s->dpt[3][63]*(FB150_6*C50+FB350_6*S50)+s->dpt[3][64]*(FB151_6*C51+FB351_6*S51)+s->dpt[3][65]*(FB152_6*C52+
 FB352_6*S52);
  CM476_348 = q[48]*FM248_6-CM149_6*S49-CM150_6*S50-CM151_6*S51-CM152_6*S52+CM349_6*C49+CM350_6*C50+CM351_6*C51+CM352_6*
 C52+FB248_6*s->l[1][48]+FB249_6*s->dpt[1][62]+FB250_6*s->dpt[1][63]+FB251_6*s->dpt[1][64]+FB252_6*s->dpt[1][65]+FB253_6*
 s->dpt[1][66]+OpM347_6*(s->In[9][48]+s->In[9][53])-s->dpt[2][62]*(FB149_6*C49+FB349_6*S49)-s->dpt[2][63]*(FB150_6*C50+
 FB350_6*S50)-s->dpt[2][64]*(FB151_6*C51+FB351_6*S51)-s->dpt[2][65]*(FB152_6*C52+FB352_6*S52);
  CM4745_248 = CM249_45+CM250_45+CM251_45+CM252_45-q[48]*(FB348_45+FB353_45-FB149_45*S49-FB150_45*S50-FB151_45*S51-
 FB152_45*S52+FB349_45*C49+FB350_45*C50+FB351_45*C51+FB352_45*C52)+s->In[5][48]*S46+s->In[5][53]*S46+Dz533*FB153_45+FB148_45*
 s->l[3][48]-FB348_45*s->l[1][48]-FB353_45*s->dpt[1][66]+s->dpt[1][62]*(FB149_45*S49-FB349_45*C49)+s->dpt[1][63]*(FB150_45*
 S50-FB350_45*C50)+s->dpt[1][64]*(FB151_45*S51-FB351_45*C51)+s->dpt[1][65]*(FB152_45*S52-FB352_45*C52)+s->dpt[3][62]*(
 FB149_45*C49+FB349_45*S49)+s->dpt[3][63]*(FB150_45*C50+FB350_45*S50)+s->dpt[3][64]*(FB151_45*C51+FB351_45*S51)+s->dpt[3][65]
 *(FB152_45*C52+FB352_45*S52);
  CM4745_348 = q[48]*(FB248_45+FB249_45+FB250_45+FB251_45+FB252_45+FB253_45)-CM149_45*S49-CM150_45*S50-CM151_45*S51-
 CM152_45*S52+CM349_45*C49+CM350_45*C50+CM351_45*C51+CM352_45*C52+FB248_45*s->l[1][48]+FB249_45*s->dpt[1][62]+FB250_45*
 s->dpt[1][63]+FB251_45*s->dpt[1][64]+FB252_45*s->dpt[1][65]+FB253_45*s->dpt[1][66]+OpM347_45*(s->In[9][48]+s->In[9][53])-
 s->dpt[2][62]*(FB149_45*C49+FB349_45*S49)-s->dpt[2][63]*(FB150_45*C50+FB350_45*S50)-s->dpt[2][64]*(FB151_45*C51+FB351_45*S51
 )-s->dpt[2][65]*(FB152_45*C52+FB352_45*S52);
  CM4746_248 = -(q[48]*C47*(s->m[49]*s->dpt[2][62]+s->m[50]*s->dpt[2][63]+s->m[51]*s->dpt[2][64]+s->m[52]*s->dpt[2][65])
 +s->m[49]*s->dpt[2][62]*(s->dpt[1][62]*C47+s->dpt[3][62]*S47)+s->m[50]*s->dpt[2][63]*(s->dpt[1][63]*C47+s->dpt[3][63]*S47)+
 s->m[51]*s->dpt[2][64]*(s->dpt[1][64]*C47+s->dpt[3][64]*S47)+s->m[52]*s->dpt[2][65]*(s->dpt[1][65]*C47+s->dpt[3][65]*S47));
  CM4747_248 = s->In[5][48]+s->In[5][49]+s->In[5][50]+s->In[5][51]+s->In[5][52]+s->In[5][53]-q[48]*(FB348_47+FB353_47-
 FB149_47*S49-FB150_47*S50-FB151_47*S51-FB152_47*S52+FB349_47*C49+FB350_47*C50+FB351_47*C51+FB352_47*C52)+Dz533*FB153_47+
 FB148_47*s->l[3][48]-FB348_47*s->l[1][48]-FB353_47*s->dpt[1][66]+s->dpt[1][62]*(FB149_47*S49-FB349_47*C49)+s->dpt[1][63]*(
 FB150_47*S50-FB350_47*C50)+s->dpt[1][64]*(FB151_47*S51-FB351_47*C51)+s->dpt[1][65]*(FB152_47*S52-FB352_47*C52)+s->dpt[3][62]
 *(FB149_47*C49+FB349_47*S49)+s->dpt[3][63]*(FB150_47*C50+FB350_47*S50)+s->dpt[3][64]*(FB151_47*C51+FB351_47*S51)+
 s->dpt[3][65]*(FB152_47*C52+FB352_47*S52);
  FF46_147 = FF148*C47+FF348*S47;
  FF46_347 = -(FF148*S47-FF348*C47);
  CF46_147 = CF148*C47+CF47_348*S47;
  CF46_347 = -(CF148*S47-CF47_348*C47);
  FM461_147 = FM148_1*C47+FM348_1*S47;
  FM461_347 = -(FM148_1*S47-FM348_1*C47);
  CM461_147 = CM148_1*C47+CM471_348*S47;
  CM461_347 = -(CM148_1*S47-CM471_348*C47);
  FM462_147 = FM148_2*C47+FM348_2*S47;
  FM462_347 = -(FM148_2*S47-FM348_2*C47);
  CM462_147 = CM148_2*C47+CM472_348*S47;
  CM462_347 = -(CM148_2*S47-CM472_348*C47);
  FM463_147 = FM148_3*C47+FM348_3*S47;
  FM463_347 = -(FM148_3*S47-FM348_3*C47);
  CM463_147 = CM148_3*C47+CM473_348*S47;
  CM463_347 = -(CM148_3*S47-CM473_348*C47);
  FM464_147 = FM148_4*C47+FM348_4*S47;
  FM464_347 = -(FM148_4*S47-FM348_4*C47);
  CM464_147 = CM148_4*C47+CM474_348*S47;
  CM464_347 = -(CM148_4*S47-CM474_348*C47);
  FM465_147 = FM148_5*C47+FM348_5*S47;
  FM465_347 = -(FM148_5*S47-FM348_5*C47);
  CM465_147 = CM148_5*C47+CM475_348*S47;
  CM465_347 = -(CM148_5*S47-CM475_348*C47);
  FM466_347 = -(FM148_6*S47-FM348_6*C47);
  CM466_147 = CM148_6*C47+CM476_348*S47;
  CM466_347 = -(CM148_6*S47-CM476_348*C47);
  CM4645_147 = CM148_45*C47+CM4745_348*S47;
  CM4646_147 = C47*(CM149_46*C49+CM150_46*C50+CM151_46*C51+CM152_46*C52+CM349_46*S49+CM350_46*S50+CM351_46*S51+CM352_46*
 S52-Dz533*FB253_46-FB248_46*s->l[3][48]-FB249_46*s->dpt[3][62]-FB250_46*s->dpt[3][63]-FB251_46*s->dpt[3][64]-FB252_46*
 s->dpt[3][65]-s->dpt[2][62]*(FB149_46*S49-FB349_46*C49)-s->dpt[2][63]*(FB150_46*S50-FB350_46*C50)-s->dpt[2][64]*(FB151_46*
 S51-FB351_46*C51)-s->dpt[2][65]*(FB152_46*S52-FB352_46*C52)+C47*(s->In[1][48]+s->In[1][53]))+S47*(q[48]*(FB248_46+FB249_46+
 FB250_46+FB251_46+FB252_46+FB253_46)-CM149_46*S49-CM150_46*S50-CM151_46*S51-CM152_46*S52+CM349_46*C49+CM350_46*C50+CM351_46*
 C51+CM352_46*C52+FB248_46*s->l[1][48]+FB249_46*s->dpt[1][62]+FB250_46*s->dpt[1][63]+FB251_46*s->dpt[1][64]+FB252_46*
 s->dpt[1][65]+FB253_46*s->dpt[1][66]-s->dpt[2][62]*(FB149_46*C49+FB349_46*S49)-s->dpt[2][63]*(FB150_46*C50+FB350_46*S50)-
 s->dpt[2][64]*(FB151_46*C51+FB351_46*S51)-s->dpt[2][65]*(FB152_46*C52+FB352_46*S52)+S47*(s->In[9][48]+s->In[9][53]));
  FF45_246 = FF248*C46-FF46_347*S46;
  FF45_346 = FF248*S46+FF46_347*C46;
  CF45_246 = -(CF46_347*S46-CF47_248*C46);
  CF45_346 = CF46_347*C46+CF47_248*S46;
  FM451_246 = FM248_1*C46-FM461_347*S46;
  FM451_346 = FM248_1*S46+FM461_347*C46;
  CM451_246 = -(CM461_347*S46-CM471_248*C46);
  CM451_346 = CM461_347*C46+CM471_248*S46;
  FM452_246 = FM248_2*C46-FM462_347*S46;
  FM452_346 = FM248_2*S46+FM462_347*C46;
  CM452_246 = -(CM462_347*S46-CM472_248*C46);
  CM452_346 = CM462_347*C46+CM472_248*S46;
  FM453_246 = FM248_3*C46-FM463_347*S46;
  FM453_346 = FM248_3*S46+FM463_347*C46;
  CM453_246 = -(CM463_347*S46-CM473_248*C46);
  CM453_346 = CM463_347*C46+CM473_248*S46;
  FM454_246 = FM248_4*C46-FM464_347*S46;
  CM454_246 = -(CM464_347*S46-CM474_248*C46);
  CM454_346 = CM464_347*C46+CM474_248*S46;
  FM455_246 = FM248_5*C46-FM465_347*S46;
  CM455_246 = -(CM465_347*S46-CM475_248*C46);
  CM455_346 = CM465_347*C46+CM475_248*S46;
  CM456_346 = CM466_347*C46+CM476_248*S46;
  CM4545_346 = CM4745_248*S46-C46*(CM148_45*S47-CM4745_348*C47);

// = = Block_0_2_0_3_0_1 = = 
 
// Backward Dynamics 

  FA16 = -(s->frc[1][6]-s->m[6]*(AlF16+BS16*s->l[1][6]+BeF36*s->l[3][6]));
  FA26 = -(s->frc[2][6]-s->m[6]*(AlF25+BeF46*s->l[1][6]+BeF66*s->l[3][6]));
  FA36 = -(s->frc[3][6]-s->m[6]*(AlF36+BS96*s->l[3][6]+BeF76*s->l[1][6]));
  FF16 = -(s->frc[1][18]+s->frc[1][19]+s->frc[1][20]+s->frc[1][21]-FA16-FA17-FF113-FF128-FF133-FF136-FF18-FF122*C22-
 FF139*C39-FF322*S22-FF339*S39+FF45_246*S45-FF46_147*C45);
  FF26 = FA26+FA27+FF222+FF228+FF239-s->frc[2][18]*C18-s->frc[2][19]*C19-s->frc[2][20]*C20-s->frc[2][21]*C21+
 s->frc[3][18]*S18+s->frc[3][19]*S19+s->frc[3][20]*S20+s->frc[3][21]*S21+FF213*C13+FF233*C33+FF236*C36+FF28*C8-FF313*S13-
 FF333*S33-FF336*S36-FF38*S8+FF45_246*C45+FF46_147*S45;
  FF36 = FA36+FA37+FF328+FF45_346-s->frc[2][18]*S18-s->frc[2][19]*S19-s->frc[2][20]*S20-s->frc[2][21]*S21-s->frc[3][18]*
 C18-s->frc[3][19]*C19-s->frc[3][20]*C20-s->frc[3][21]*C21-FF122*S22-FF139*S39+FF213*S13+FF233*S33+FF236*S36+FF28*S8+FF313*
 C13+FF322*C22+FF333*C33+FF336*C36+FF339*C39+FF38*C8;
  CF16 = -(s->trq[1][18]+s->trq[1][19]+s->trq[1][20]+s->trq[1][21]+s->trq[1][28]+s->trq[1][6]+s->trq[1][7]-CF113-CF133-
 CF136-CF18-CF29_130-CF31_132-q[28]*FF328-s->In[1][6]*OpF16-CF122*C22-CF139*C39-CF322*S22-CF339*S39+CF45_246*S45-CF46_147*C45
 +FA26*s->l[3][6]+FF222*s->dpt[3][8]+FF228*s->dpt[3][9]+FF239*s->dpt[3][15]+OM26*OM36*(s->In[5][6]-s->In[9][6])-s->dpt[2][12]
 *(FF233*S33+FF333*C33)-s->dpt[2][13]*(FF236*S36+FF336*C36)-s->dpt[2][1]*(FF28*S8+FF38*C8)-s->dpt[2][2]*(FF213*S13+FF313*C13)
 +s->dpt[2][3]*(s->frc[2][18]*S18+s->frc[3][18]*C18)+s->dpt[2][43]*(s->frc[3][30]*C29-FF29_230*S29)+s->dpt[2][44]*(
 s->frc[3][32]*C31-FF31_232*S31)+s->dpt[2][4]*(s->frc[2][19]*S19+s->frc[3][19]*C19)+s->dpt[2][5]*(s->frc[2][20]*S20+
 s->frc[3][20]*C20)+s->dpt[2][7]*(s->frc[2][21]*S21+s->frc[3][21]*C21)+s->dpt[3][12]*(FF233*C33-FF333*S33)+s->dpt[3][13]*(
 FF236*C36-FF336*S36)+s->dpt[3][17]*(FF45_246*C45+FF46_147*S45)+s->dpt[3][1]*(FF28*C8-FF38*S8)+s->dpt[3][2]*(FF213*C13-FF313*
 S13));
  CF26 = -(s->trq[2][28]+s->trq[2][6]+s->trq[2][7]-CF222-CF239+q[7]*FA37-s->In[5][6]*OpF25+s->trq[2][18]*C18+
 s->trq[2][19]*C19+s->trq[2][20]*C20+s->trq[2][21]*C21-s->trq[3][18]*S18-s->trq[3][19]*S19-s->trq[3][20]*S20-s->trq[3][21]*
 S21-s->trq[3][30]*S29-s->trq[3][32]*S31-CF213*C13-CF233*C33-CF236*C36-CF28*C8-CF29_230*C29+CF313*S13-CF31_232*C31+CF333*S33+
 CF336*S36+CF38*S8-CF45_246*C45-CF46_147*S45-FA16*s->l[3][6]+FA36*s->l[1][6]-FF113*s->dpt[3][2]-FF128*s->dpt[3][9]-FF133*
 s->dpt[3][12]-FF136*s->dpt[3][13]-FF18*s->dpt[3][1]+FF328*s->dpt[1][9]+FF45_346*s->dpt[1][17]-OM16*OM36*(s->In[1][6]-
 s->In[9][6])+s->dpt[1][12]*(FF233*S33+FF333*C33)+s->dpt[1][13]*(FF236*S36+FF336*C36)-s->dpt[1][15]*(FF139*S39-FF339*C39)+
 s->dpt[1][1]*(FF28*S8+FF38*C8)+s->dpt[1][2]*(FF213*S13+FF313*C13)-s->dpt[1][5]*(s->frc[2][20]*S20+s->frc[3][20]*C20)-
 s->dpt[1][7]*(s->frc[2][21]*S21+s->frc[3][21]*C21)-s->dpt[1][8]*(FF122*S22-FF322*C22)-s->dpt[3][15]*(FF139*C39+FF339*S39)+
 s->dpt[3][17]*(FF45_246*S45-FF46_147*C45)-s->dpt[3][8]*(FF122*C22+FF322*S22));
  CF36 = -(s->trq[3][28]+s->trq[3][6]+s->trq[3][7]-CF45_346+q[28]*FF128-q[7]*FA27-s->In[9][6]*OpF36-s->frc[1][18]*
 s->dpt[2][3]-s->frc[1][19]*s->dpt[2][4]-s->frc[1][20]*s->dpt[2][5]-s->frc[1][21]*s->dpt[2][7]+s->trq[2][18]*S18+
 s->trq[2][19]*S19+s->trq[2][20]*S20+s->trq[2][21]*S21+s->trq[3][18]*C18+s->trq[3][19]*C19+s->trq[3][20]*C20+s->trq[3][21]*
 C21+s->trq[3][30]*C29+s->trq[3][32]*C31+CF122*S22+CF139*S39-CF213*S13-CF233*S33-CF236*S36-CF28*S8-CF29_230*S29-CF313*C13-
 CF31_232*S31-CF322*C22-CF333*C33-CF336*C36-CF339*C39-CF38*C8-FA26*s->l[1][6]+FF113*s->dpt[2][2]+FF133*s->dpt[2][12]+FF136*
 s->dpt[2][13]+FF18*s->dpt[2][1]-FF222*s->dpt[1][8]-FF228*s->dpt[1][9]-FF239*s->dpt[1][15]+FF29_130*s->dpt[2][43]+FF31_132*
 s->dpt[2][44]+OM16*OM26*(s->In[1][6]-s->In[5][6])-s->dpt[1][12]*(FF233*C33-FF333*S33)-s->dpt[1][13]*(FF236*C36-FF336*S36)-
 s->dpt[1][17]*(FF45_246*C45+FF46_147*S45)-s->dpt[1][1]*(FF28*C8-FF38*S8)-s->dpt[1][2]*(FF213*C13-FF313*S13)+s->dpt[1][5]*(
 s->frc[2][20]*C20-s->frc[3][20]*S20)+s->dpt[1][7]*(s->frc[2][21]*C21-s->frc[3][21]*S21));
  FB16_1 = s->m[6]*AlM16_1;
  FB26_1 = s->m[6]*AlM25_1;
  FB36_1 = s->m[6]*AlM36_1;
  FM16_1 = FB128_1+FB16_1+FB17_1+FM131_114+FM134_1+FM137_1+FM81_19-FM451_246*S45+FM461_147*C45;
  FM26_1 = FB228_1+FB26_1+FB27_1-FM131_314*S13+FM141_215*C13+FM331_234*C33-FM331_334*S33+FM361_237*C36-FM361_337*S36+
 FM451_246*C45+FM461_147*S45-FM81_39*S8+FM91_210*C8;
  FM36_1 = FB328_1+FB36_1+FB37_1+FM451_346+FM131_314*C13+FM141_215*S13+FM331_234*S33+FM331_334*C33+FM361_237*S36+
 FM361_337*C36+FM81_39*C8+FM91_210*S8;
  CM16_1 = CM131_114+CM331_134+CM361_137+CM81_19+q[28]*FB328_1-CM451_246*S45+CM461_147*C45-FB228_1*s->dpt[3][9]-FB26_1*
 s->l[3][6]+s->dpt[2][12]*(FM331_234*S33+FM331_334*C33)+s->dpt[2][13]*(FM361_237*S36+FM361_337*C36)+s->dpt[2][1]*(FM81_39*C8+
 FM91_210*S8)+s->dpt[2][2]*(FM131_314*C13+FM141_215*S13)-s->dpt[3][12]*(FM331_234*C33-FM331_334*S33)-s->dpt[3][13]*(FM361_237
 *C36-FM361_337*S36)-s->dpt[3][17]*(FM451_246*C45+FM461_147*S45)+s->dpt[3][1]*(FM81_39*S8-FM91_210*C8)+s->dpt[3][2]*(
 FM131_314*S13-FM141_215*C13);
  CM26_1 = CM451_246*C45+CM461_147*S45-FM451_346*s->dpt[1][17]-s->dpt[3][17]*(FM451_246*S45-FM461_147*C45)-q[7]*FB37_1-
 CM131_314*S13+CM141_215*C13+CM331_234*C33-CM331_334*S33+CM361_237*C36-CM361_337*S36-CM81_39*S8+CM91_210*C8+FB128_1*
 s->dpt[3][9]+FB16_1*s->l[3][6]-FB328_1*s->dpt[1][9]-FB36_1*s->l[1][6]+FM131_114*s->dpt[3][2]+FM134_1*s->dpt[3][12]+FM137_1*
 s->dpt[3][13]+FM81_19*s->dpt[3][1]-s->dpt[1][12]*(FM331_234*S33+FM331_334*C33)-s->dpt[1][13]*(FM361_237*S36+FM361_337*C36)-
 s->dpt[1][1]*(FM81_39*C8+FM91_210*S8)-s->dpt[1][2]*(FM131_314*C13+FM141_215*S13);
  CM36_1 = CM451_346-q[28]*FB128_1+q[7]*FB27_1+CM131_314*C13+CM141_215*S13+CM331_234*S33+CM331_334*C33+CM361_237*S36+
 CM361_337*C36+CM81_39*C8+CM91_210*S8+FB228_1*s->dpt[1][9]+FB26_1*s->l[1][6]-FM131_114*s->dpt[2][2]-FM134_1*s->dpt[2][12]-
 FM137_1*s->dpt[2][13]-FM81_19*s->dpt[2][1]+s->dpt[1][12]*(FM331_234*C33-FM331_334*S33)+s->dpt[1][13]*(FM361_237*C36-
 FM361_337*S36)+s->dpt[1][17]*(FM451_246*C45+FM461_147*S45)-s->dpt[1][1]*(FM81_39*S8-FM91_210*C8)-s->dpt[1][2]*(FM131_314*S13
 -FM141_215*C13);
  FB16_2 = s->m[6]*AlM16_2;
  FB26_2 = s->m[6]*AlM25_2;
  FB36_2 = s->m[6]*AlM36_2;
  FM16_2 = FB128_2+FB16_2+FB17_2+FM132_114+FM134_2+FM137_2+FM82_19-FM452_246*S45+FM462_147*C45;
  FM26_2 = FB228_2+FB26_2+FB27_2-FM132_314*S13+FM142_215*C13+FM332_234*C33-FM332_334*S33+FM362_237*C36-FM362_337*S36+
 FM452_246*C45+FM462_147*S45-FM82_39*S8+FM92_210*C8;
  FM36_2 = FB328_2+FB36_2+FB37_2+FM452_346+FM132_314*C13+FM142_215*S13+FM332_234*S33+FM332_334*C33+FM362_237*S36+
 FM362_337*C36+FM82_39*C8+FM92_210*S8;
  CM16_2 = CM132_114+CM332_134+CM362_137+CM82_19+q[28]*FB328_2-CM452_246*S45+CM462_147*C45-FB228_2*s->dpt[3][9]-FB26_2*
 s->l[3][6]+s->dpt[2][12]*(FM332_234*S33+FM332_334*C33)+s->dpt[2][13]*(FM362_237*S36+FM362_337*C36)+s->dpt[2][1]*(FM82_39*C8+
 FM92_210*S8)+s->dpt[2][2]*(FM132_314*C13+FM142_215*S13)-s->dpt[3][12]*(FM332_234*C33-FM332_334*S33)-s->dpt[3][13]*(FM362_237
 *C36-FM362_337*S36)-s->dpt[3][17]*(FM452_246*C45+FM462_147*S45)+s->dpt[3][1]*(FM82_39*S8-FM92_210*C8)+s->dpt[3][2]*(
 FM132_314*S13-FM142_215*C13);
  CM26_2 = CM452_246*C45+CM462_147*S45-FM452_346*s->dpt[1][17]-s->dpt[3][17]*(FM452_246*S45-FM462_147*C45)-q[7]*FB37_2-
 CM132_314*S13+CM142_215*C13+CM332_234*C33-CM332_334*S33+CM362_237*C36-CM362_337*S36-CM82_39*S8+CM92_210*C8+FB128_2*
 s->dpt[3][9]+FB16_2*s->l[3][6]-FB328_2*s->dpt[1][9]-FB36_2*s->l[1][6]+FM132_114*s->dpt[3][2]+FM134_2*s->dpt[3][12]+FM137_2*
 s->dpt[3][13]+FM82_19*s->dpt[3][1]-s->dpt[1][12]*(FM332_234*S33+FM332_334*C33)-s->dpt[1][13]*(FM362_237*S36+FM362_337*C36)-
 s->dpt[1][1]*(FM82_39*C8+FM92_210*S8)-s->dpt[1][2]*(FM132_314*C13+FM142_215*S13);
  CM36_2 = CM452_346-q[28]*FB128_2+q[7]*FB27_2+CM132_314*C13+CM142_215*S13+CM332_234*S33+CM332_334*C33+CM362_237*S36+
 CM362_337*C36+CM82_39*C8+CM92_210*S8+FB228_2*s->dpt[1][9]+FB26_2*s->l[1][6]-FM132_114*s->dpt[2][2]-FM134_2*s->dpt[2][12]-
 FM137_2*s->dpt[2][13]-FM82_19*s->dpt[2][1]+s->dpt[1][12]*(FM332_234*C33-FM332_334*S33)+s->dpt[1][13]*(FM362_237*C36-
 FM362_337*S36)+s->dpt[1][17]*(FM452_246*C45+FM462_147*S45)-s->dpt[1][1]*(FM82_39*S8-FM92_210*C8)-s->dpt[1][2]*(FM132_314*S13
 -FM142_215*C13);
  FB16_3 = s->m[6]*AlM16_3;
  FB26_3 = s->m[6]*S5;
  FB36_3 = s->m[6]*AlM36_3;
  CM16_3 = CM133_114+CM333_134+CM363_137+CM83_19+q[28]*FB328_3-CM453_246*S45+CM463_147*C45-FB228_3*s->dpt[3][9]-FB26_3*
 s->l[3][6]+s->dpt[2][12]*(FM333_234*S33+FM333_334*C33)+s->dpt[2][13]*(FM363_237*S36+FM363_337*C36)+s->dpt[2][1]*(FM83_39*C8+
 FM93_210*S8)+s->dpt[2][2]*(FM133_314*C13+FM143_215*S13)-s->dpt[3][12]*(FM333_234*C33-FM333_334*S33)-s->dpt[3][13]*(FM363_237
 *C36-FM363_337*S36)-s->dpt[3][17]*(FM453_246*C45+FM463_147*S45)+s->dpt[3][1]*(FM83_39*S8-FM93_210*C8)+s->dpt[3][2]*(
 FM133_314*S13-FM143_215*C13);
  CM26_3 = CM453_246*C45+CM463_147*S45-FM453_346*s->dpt[1][17]-s->dpt[3][17]*(FM453_246*S45-FM463_147*C45)-q[7]*FB37_3-
 CM133_314*S13+CM143_215*C13+CM333_234*C33-CM333_334*S33+CM363_237*C36-CM363_337*S36-CM83_39*S8+CM93_210*C8+FB128_3*
 s->dpt[3][9]+FB16_3*s->l[3][6]-FB328_3*s->dpt[1][9]-FB36_3*s->l[1][6]+FM133_114*s->dpt[3][2]+FM134_3*s->dpt[3][12]+FM137_3*
 s->dpt[3][13]+FM83_19*s->dpt[3][1]-s->dpt[1][12]*(FM333_234*S33+FM333_334*C33)-s->dpt[1][13]*(FM363_237*S36+FM363_337*C36)-
 s->dpt[1][1]*(FM83_39*C8+FM93_210*S8)-s->dpt[1][2]*(FM133_314*C13+FM143_215*S13);
  CM36_3 = CM453_346-q[28]*FB128_3+q[7]*FB27_3+CM133_314*C13+CM143_215*S13+CM333_234*S33+CM333_334*C33+CM363_237*S36+
 CM363_337*C36+CM83_39*C8+CM93_210*S8+FB228_3*s->dpt[1][9]+FB26_3*s->l[1][6]-FM133_114*s->dpt[2][2]-FM134_3*s->dpt[2][12]-
 FM137_3*s->dpt[2][13]-FM83_19*s->dpt[2][1]+s->dpt[1][12]*(FM333_234*C33-FM333_334*S33)+s->dpt[1][13]*(FM363_237*C36-
 FM363_337*S36)+s->dpt[1][17]*(FM453_246*C45+FM463_147*S45)-s->dpt[1][1]*(FM83_39*S8-FM93_210*C8)-s->dpt[1][2]*(FM133_314*S13
 -FM143_215*C13);
  FB26_4 = -s->m[6]*(OpM16_4*s->l[3][6]-OpM36_4*s->l[1][6]);
  CM16_4 = CM134_114+CM334_134+CM364_137+CM84_19+q[28]*FB328_4+s->In[1][6]*OpM16_4-CM454_246*S45+CM464_147*C45-FB228_4*
 s->dpt[3][9]-FB26_4*s->l[3][6]+s->dpt[2][12]*(FM334_234*S33+FM334_334*C33)+s->dpt[2][13]*(FM364_237*S36+FM364_337*C36)+
 s->dpt[2][1]*(FM84_39*C8+FM94_210*S8)+s->dpt[2][2]*(FM134_314*C13+FM144_215*S13)-s->dpt[3][12]*(FM334_234*C33-FM334_334*S33)
 -s->dpt[3][13]*(FM364_237*C36-FM364_337*S36)-s->dpt[3][17]*(FM454_246*C45+FM464_147*S45)+s->dpt[3][1]*(FM84_39*S8-FM94_210*
 C8)+s->dpt[3][2]*(FM134_314*S13-FM144_215*C13);
  CM26_4 = s->m[6]*S5*(s->l[1][6]*s->l[1][6]+s->l[3][6]*s->l[3][6])-CM134_314*S13+CM144_215*C13+CM334_234*C33-CM334_334*
 S33+CM364_237*C36-CM364_337*S36+CM454_246*C45+CM464_147*S45-CM84_39*S8+CM94_210*C8+FB128_4*s->dpt[3][9]-FB328_4*s->dpt[1][9]
 +FM134_114*s->dpt[3][2]+FM134_4*s->dpt[3][12]+FM137_4*s->dpt[3][13]+FM84_19*s->dpt[3][1]-s->dpt[1][12]*(FM334_234*S33+
 FM334_334*C33)-s->dpt[1][13]*(FM364_237*S36+FM364_337*C36)-s->dpt[1][17]*(FM248_4*S46+FM464_347*C46)-s->dpt[1][1]*(FM84_39*
 C8+FM94_210*S8)-s->dpt[1][2]*(FM134_314*C13+FM144_215*S13)-s->dpt[3][17]*(FM454_246*S45-FM464_147*C45)+S5*(s->In[5][6]+q[7]*
 q[7]*s->m[7]);
  CM36_4 = CM454_346-q[28]*FB128_4+q[7]*q[7]*s->m[7]*OpM36_4+s->In[9][6]*OpM36_4+CM134_314*C13+CM144_215*S13+CM334_234*
 S33+CM334_334*C33+CM364_237*S36+CM364_337*C36+CM84_39*C8+CM94_210*S8+FB228_4*s->dpt[1][9]+FB26_4*s->l[1][6]-FM134_114*
 s->dpt[2][2]-FM134_4*s->dpt[2][12]-FM137_4*s->dpt[2][13]-FM84_19*s->dpt[2][1]+s->dpt[1][12]*(FM334_234*C33-FM334_334*S33)+
 s->dpt[1][13]*(FM364_237*C36-FM364_337*S36)+s->dpt[1][17]*(FM454_246*C45+FM464_147*S45)-s->dpt[1][1]*(FM84_39*S8-FM94_210*C8
 )-s->dpt[1][2]*(FM134_314*S13-FM144_215*C13);
  FB26_5 = s->m[6]*(s->l[1][6]*S6-s->l[3][6]*C6);
  CM26_5 = CM455_246*C45+CM465_147*S45-s->dpt[1][17]*(FM248_5*S46+FM465_347*C46)-s->dpt[3][17]*(FM455_246*S45-FM465_147*
 C45)-CM135_314*S13+CM145_215*C13+CM335_234*C33-CM335_334*S33+CM365_237*C36-CM365_337*S36-CM85_39*S8+CM95_210*C8+FB128_5*
 s->dpt[3][9]-FB328_5*s->dpt[1][9]+FM134_5*s->dpt[3][12]+FM135_114*s->dpt[3][2]+FM137_5*s->dpt[3][13]+FM85_19*s->dpt[3][1]-
 s->dpt[1][12]*(FM335_234*S33+FM335_334*C33)-s->dpt[1][13]*(FM365_237*S36+FM365_337*C36)-s->dpt[1][1]*(FM85_39*C8+FM95_210*S8
 )-s->dpt[1][2]*(FM135_314*C13+FM145_215*S13);
  CM26_6 = s->In[5][6]+q[7]*q[7]*s->m[7]+s->m[28]*(s->dpt[1][9]*s->dpt[1][9]+s->dpt[3][9]*s->dpt[3][9])+s->m[6]*
 s->l[1][6]*s->l[1][6]+s->m[6]*s->l[3][6]*s->l[3][6]+CM146_215*C13+CM466_147*S45+CM96_210*C8+FM134_6*s->dpt[3][12]+FM137_6*
 s->dpt[3][13]-s->dpt[1][12]*(C33*(FM234_6*S34+FM334_6*C34)+S33*(FM234_6*C34-FM334_6*S34))-s->dpt[1][13]*(C36*(FM237_6*S37+
 FM337_6*C37)+S36*(FM237_6*C37-FM337_6*S37))-s->dpt[1][17]*(FM248_6*S46+FM466_347*C46)+s->dpt[1][1]*(C8*(FM106_111*S9-
 FM96_310*C9)-S8*(FM106_211*C10-FM311_6*S10))-s->dpt[1][2]*(C13*(FM146_315*C14-FM156_116*S14)+S13*(FM156_216*C15-FM316_6*S15)
 )+s->dpt[3][17]*(C45*(FM148_6*C47+FM348_6*S47)-S45*(FM248_6*C46-FM466_347*S46))+s->dpt[3][1]*(FM106_111*C9+FM96_310*S9)+
 s->dpt[3][2]*(FM146_315*S14+FM156_116*C14)-S13*(CM146_315*C14-CM156_116*S14-s->dpt[2][24]*(FM146_315*S14+FM156_116*C14))+C33
 *(CM346_235*C34-CM346_335*S34)-S33*(CM346_235*S34+CM346_335*C34-FM134_6*s->dpt[2][47])+C36*(CM376_238*C37-CM376_338*S37)-S36
 *(CM376_238*S37+CM376_338*C37-FM137_6*s->dpt[2][52])-C45*(CM466_347*S46-CM476_248*C46)+S8*(CM106_111*S9-CM96_310*C9+
 s->dpt[2][18]*(FM106_111*C9+FM96_310*S9));
  FF5_16 = FF16*C6+FF36*S6;
  FF5_36 = -(FF16*S6-FF36*C6);
  CF5_16 = CF16*C6+CF36*S6;
  FM51_16 = FM16_1*C6+FM36_1*S6;
  FM51_36 = -(FM16_1*S6-FM36_1*C6);
  CM51_16 = CM16_1*C6+CM36_1*S6;
  FM52_36 = -(FM16_2*S6-FM36_2*C6);
  CM52_16 = CM16_2*C6+CM36_2*S6;
  CM53_16 = CM16_3*C6+CM36_3*S6;
  CM54_16 = CM16_4*C6+CM36_4*S6;
  CM55_16 = C6*(CM135_114+CM335_134+CM365_137+CM85_19+q[28]*FB328_5+s->In[1][6]*C6-CM455_246*S45+CM465_147*C45-FB228_5*
 s->dpt[3][9]-FB26_5*s->l[3][6]+s->dpt[2][12]*(FM335_234*S33+FM335_334*C33)+s->dpt[2][13]*(FM365_237*S36+FM365_337*C36)+
 s->dpt[2][1]*(FM85_39*C8+FM95_210*S8)+s->dpt[2][2]*(FM135_314*C13+FM145_215*S13)-s->dpt[3][12]*(FM335_234*C33-FM335_334*S33)
 -s->dpt[3][13]*(FM365_237*C36-FM365_337*S36)-s->dpt[3][17]*(FM455_246*C45+FM465_147*S45)+s->dpt[3][1]*(FM85_39*S8-FM95_210*
 C8)+s->dpt[3][2]*(FM135_314*S13-FM145_215*C13))+S6*(CM455_346-q[28]*FB128_5+q[7]*q[7]*s->m[7]*S6+s->In[9][6]*S6+CM135_314*
 C13+CM145_215*S13+CM335_234*S33+CM335_334*C33+CM365_237*S36+CM365_337*C36+CM85_39*C8+CM95_210*S8+FB228_5*s->dpt[1][9]+FB26_5
 *s->l[1][6]-FM134_5*s->dpt[2][12]-FM135_114*s->dpt[2][2]-FM137_5*s->dpt[2][13]-FM85_19*s->dpt[2][1]+s->dpt[1][12]*(FM335_234
 *C33-FM335_334*S33)+s->dpt[1][13]*(FM365_237*C36-FM365_337*S36)+s->dpt[1][17]*(FM455_246*C45+FM465_147*S45)-s->dpt[1][1]*(
 FM85_39*S8-FM95_210*C8)-s->dpt[1][2]*(FM135_314*S13-FM145_215*C13));
  FF4_25 = FF26*C5-FF5_36*S5;
  FF4_35 = FF26*S5+FF5_36*C5;
  CF4_35 = CF26*S5-C5*(CF16*S6-CF36*C6);
  FM41_25 = FM26_1*C5-FM51_36*S5;
  FM41_35 = FM26_1*S5+FM51_36*C5;
  CM41_35 = CM26_1*S5-C5*(CM16_1*S6-CM36_1*C6);
  FM42_35 = FM26_2*S5+FM52_36*C5;
  CM42_35 = CM26_2*S5-C5*(CM16_2*S6-CM36_2*C6);
  FM43_35 = C5*(C6*(FB328_3+FB36_3+FB37_3+FM453_346+FM133_314*C13+FM143_215*S13+FM333_234*S33+FM333_334*C33+FM363_237*
 S36+FM363_337*C36+FM83_39*C8+FM93_210*S8)-S6*(FB128_3+FB16_3+FB17_3+FM133_114+FM134_3+FM137_3+FM83_19-FM453_246*S45+
 FM463_147*C45))+S5*(FB228_3+FB26_3+FB27_3-FM133_314*S13+FM143_215*C13+FM333_234*C33-FM333_334*S33+FM363_237*C36-FM363_337*
 S36+FM453_246*C45+FM463_147*S45-FM83_39*S8+FM93_210*C8);
  CM43_35 = CM26_3*S5-C5*(CM16_3*S6-CM36_3*C6);
  CM44_35 = CM26_4*S5-C5*(CM16_4*S6-CM36_4*C6);
  FF3_14 = -(FF4_25*S4-FF5_16*C4);
  FF3_24 = FF4_25*C4+FF5_16*S4;
  FM31_14 = -(FM41_25*S4-FM51_16*C4);
  FM31_24 = FM41_25*C4+FM51_16*S4;
  FM32_24 = C4*(FM26_2*C5-FM52_36*S5)+S4*(FM16_2*C6+FM36_2*S6);

// = = Block_0_3_0_0_0_0 = = 
 
// Symbolic Outputs  

  M[1][1] = FM31_14;
  M[1][2] = FM31_24;
  M[1][3] = FM41_35;
  M[1][4] = CM41_35;
  M[1][5] = CM51_16;
  M[1][6] = CM26_1;
  M[1][7] = FB17_1;
  M[1][8] = CM81_19;
  M[1][9] = CM91_210;
  M[1][10] = CM101_111;
  M[1][13] = CM131_114;
  M[1][14] = CM141_215;
  M[1][15] = CM151_116;
  M[1][28] = FB228_1;
  M[1][33] = CM331_134;
  M[1][34] = CM341_135;
  M[1][36] = CM361_137;
  M[1][37] = CM371_138;
  M[1][45] = CM451_346;
  M[1][46] = CM461_147;
  M[1][47] = CM471_248;
  M[1][48] = FM148_1;
  M[1][53] = FB353_1;
  M[2][1] = FM31_24;
  M[2][2] = FM32_24;
  M[2][3] = FM42_35;
  M[2][4] = CM42_35;
  M[2][5] = CM52_16;
  M[2][6] = CM26_2;
  M[2][7] = FB17_2;
  M[2][8] = CM82_19;
  M[2][9] = CM92_210;
  M[2][10] = CM102_111;
  M[2][13] = CM132_114;
  M[2][14] = CM142_215;
  M[2][15] = CM152_116;
  M[2][28] = FB228_2;
  M[2][33] = CM332_134;
  M[2][34] = CM342_135;
  M[2][36] = CM362_137;
  M[2][37] = CM372_138;
  M[2][45] = CM452_346;
  M[2][46] = CM462_147;
  M[2][47] = CM472_248;
  M[2][48] = FM148_2;
  M[2][53] = FB353_2;
  M[3][1] = FM41_35;
  M[3][2] = FM42_35;
  M[3][3] = FM43_35;
  M[3][4] = CM43_35;
  M[3][5] = CM53_16;
  M[3][6] = CM26_3;
  M[3][7] = FB17_3;
  M[3][8] = CM83_19;
  M[3][9] = CM93_210;
  M[3][10] = CM103_111;
  M[3][13] = CM133_114;
  M[3][14] = CM143_215;
  M[3][15] = CM153_116;
  M[3][28] = FB228_3;
  M[3][33] = CM333_134;
  M[3][34] = CM343_135;
  M[3][36] = CM363_137;
  M[3][37] = CM373_138;
  M[3][45] = CM453_346;
  M[3][46] = CM463_147;
  M[3][47] = CM473_248;
  M[3][48] = FM148_3;
  M[3][53] = FB353_3;
  M[4][1] = CM41_35;
  M[4][2] = CM42_35;
  M[4][3] = CM43_35;
  M[4][4] = CM44_35;
  M[4][5] = CM54_16;
  M[4][6] = CM26_4;
  M[4][8] = CM84_19;
  M[4][9] = CM94_210;
  M[4][10] = CM104_111;
  M[4][11] = CM114_312;
  M[4][12] = CM212_4;
  M[4][13] = CM134_114;
  M[4][14] = CM144_215;
  M[4][15] = CM154_116;
  M[4][16] = CM164_317;
  M[4][17] = CM217_4;
  M[4][28] = FB228_4;
  M[4][33] = CM334_134;
  M[4][34] = CM344_135;
  M[4][35] = CM235_4;
  M[4][36] = CM364_137;
  M[4][37] = CM374_138;
  M[4][38] = CM238_4;
  M[4][45] = CM454_346;
  M[4][46] = CM464_147;
  M[4][47] = CM474_248;
  M[4][48] = FM148_4;
  M[4][49] = CM249_4;
  M[4][50] = CM250_4;
  M[4][51] = CM251_4;
  M[4][52] = CM252_4;
  M[4][53] = FB353_4;
  M[5][1] = CM51_16;
  M[5][2] = CM52_16;
  M[5][3] = CM53_16;
  M[5][4] = CM54_16;
  M[5][5] = CM55_16;
  M[5][6] = CM26_5;
  M[5][8] = CM85_19;
  M[5][9] = CM95_210;
  M[5][10] = CM105_111;
  M[5][11] = CM115_312;
  M[5][12] = CM212_5;
  M[5][13] = CM135_114;
  M[5][14] = CM145_215;
  M[5][15] = CM155_116;
  M[5][16] = CM165_317;
  M[5][17] = CM217_5;
  M[5][28] = FB228_5;
  M[5][33] = CM335_134;
  M[5][34] = CM345_135;
  M[5][35] = CM235_5;
  M[5][36] = CM365_137;
  M[5][37] = CM375_138;
  M[5][38] = CM238_5;
  M[5][45] = CM455_346;
  M[5][46] = CM465_147;
  M[5][47] = CM475_248;
  M[5][48] = FM148_5;
  M[5][49] = CM249_5;
  M[5][50] = CM250_5;
  M[5][51] = CM251_5;
  M[5][52] = CM252_5;
  M[5][53] = FB353_5;
  M[6][1] = CM26_1;
  M[6][2] = CM26_2;
  M[6][3] = CM26_3;
  M[6][4] = CM26_4;
  M[6][5] = CM26_5;
  M[6][6] = CM26_6;
  M[6][8] = CM86_19;
  M[6][9] = CM96_210;
  M[6][10] = CM106_111;
  M[6][11] = CM116_312;
  M[6][12] = CM212_6;
  M[6][13] = CM136_114;
  M[6][14] = CM146_215;
  M[6][15] = CM156_116;
  M[6][16] = CM166_317;
  M[6][17] = CM217_6;
  M[6][33] = CM336_134;
  M[6][34] = CM346_135;
  M[6][35] = CM235_6;
  M[6][36] = CM366_137;
  M[6][37] = CM376_138;
  M[6][38] = CM238_6;
  M[6][45] = CM456_346;
  M[6][46] = CM466_147;
  M[6][47] = CM476_248;
  M[6][48] = FM148_6;
  M[6][49] = CM249_6;
  M[6][50] = CM250_6;
  M[6][51] = CM251_6;
  M[6][52] = CM252_6;
  M[6][53] = FB353_6;
  M[7][1] = FB17_1;
  M[7][2] = FB17_2;
  M[7][3] = FB17_3;
  M[7][7] = s->m[7];
  M[8][1] = CM81_19;
  M[8][2] = CM82_19;
  M[8][3] = CM83_19;
  M[8][4] = CM84_19;
  M[8][5] = CM85_19;
  M[8][6] = CM86_19;
  M[8][8] = CM88_19;
  M[8][9] = CM98_210;
  M[8][10] = CM108_111;
  M[8][11] = CM118_312;
  M[8][12] = CM212_8;
  M[9][1] = CM91_210;
  M[9][2] = CM92_210;
  M[9][3] = CM93_210;
  M[9][4] = CM94_210;
  M[9][5] = CM95_210;
  M[9][6] = CM96_210;
  M[9][8] = CM98_210;
  M[9][9] = CM99_210;
  M[9][10] = CM109_111;
  M[9][11] = CM119_312;
  M[9][12] = CM212_9;
  M[10][1] = CM101_111;
  M[10][2] = CM102_111;
  M[10][3] = CM103_111;
  M[10][4] = CM104_111;
  M[10][5] = CM105_111;
  M[10][6] = CM106_111;
  M[10][8] = CM108_111;
  M[10][9] = CM109_111;
  M[10][10] = CM1010_111;
  M[10][11] = CM1110_312;
  M[10][12] = CM212_10;
  M[11][4] = CM114_312;
  M[11][5] = CM115_312;
  M[11][6] = CM116_312;
  M[11][8] = CM118_312;
  M[11][9] = CM119_312;
  M[11][10] = CM1110_312;
  M[11][11] = CM1111_312;
  M[12][4] = CM212_4;
  M[12][5] = CM212_5;
  M[12][6] = CM212_6;
  M[12][8] = CM212_8;
  M[12][9] = CM212_9;
  M[12][10] = CM212_10;
  M[12][12] = s->In[5][12];
  M[13][1] = CM131_114;
  M[13][2] = CM132_114;
  M[13][3] = CM133_114;
  M[13][4] = CM134_114;
  M[13][5] = CM135_114;
  M[13][6] = CM136_114;
  M[13][13] = CM1313_114;
  M[13][14] = CM1413_215;
  M[13][15] = CM1513_116;
  M[13][16] = CM1613_317;
  M[13][17] = CM217_13;
  M[14][1] = CM141_215;
  M[14][2] = CM142_215;
  M[14][3] = CM143_215;
  M[14][4] = CM144_215;
  M[14][5] = CM145_215;
  M[14][6] = CM146_215;
  M[14][13] = CM1413_215;
  M[14][14] = CM1414_215;
  M[14][15] = CM1514_116;
  M[14][16] = CM1614_317;
  M[14][17] = CM217_14;
  M[15][1] = CM151_116;
  M[15][2] = CM152_116;
  M[15][3] = CM153_116;
  M[15][4] = CM154_116;
  M[15][5] = CM155_116;
  M[15][6] = CM156_116;
  M[15][13] = CM1513_116;
  M[15][14] = CM1514_116;
  M[15][15] = CM1515_116;
  M[15][16] = CM1615_317;
  M[15][17] = CM217_15;
  M[16][4] = CM164_317;
  M[16][5] = CM165_317;
  M[16][6] = CM166_317;
  M[16][13] = CM1613_317;
  M[16][14] = CM1614_317;
  M[16][15] = CM1615_317;
  M[16][16] = CM1616_317;
  M[17][4] = CM217_4;
  M[17][5] = CM217_5;
  M[17][6] = CM217_6;
  M[17][13] = CM217_13;
  M[17][14] = CM217_14;
  M[17][15] = CM217_15;
  M[17][17] = s->In[5][17];
  M[28][1] = FB228_1;
  M[28][2] = FB228_2;
  M[28][3] = FB228_3;
  M[28][4] = FB228_4;
  M[28][5] = FB228_5;
  M[28][28] = s->m[28];
  M[33][1] = CM331_134;
  M[33][2] = CM332_134;
  M[33][3] = CM333_134;
  M[33][4] = CM334_134;
  M[33][5] = CM335_134;
  M[33][6] = CM336_134;
  M[33][33] = CM3333_134;
  M[33][34] = CM3433_135;
  M[34][1] = CM341_135;
  M[34][2] = CM342_135;
  M[34][3] = CM343_135;
  M[34][4] = CM344_135;
  M[34][5] = CM345_135;
  M[34][6] = CM346_135;
  M[34][33] = CM3433_135;
  M[34][34] = CM3434_135;
  M[35][4] = CM235_4;
  M[35][5] = CM235_5;
  M[35][6] = CM235_6;
  M[35][35] = s->In[5][35];
  M[36][1] = CM361_137;
  M[36][2] = CM362_137;
  M[36][3] = CM363_137;
  M[36][4] = CM364_137;
  M[36][5] = CM365_137;
  M[36][6] = CM366_137;
  M[36][36] = CM3636_137;
  M[36][37] = CM3736_138;
  M[37][1] = CM371_138;
  M[37][2] = CM372_138;
  M[37][3] = CM373_138;
  M[37][4] = CM374_138;
  M[37][5] = CM375_138;
  M[37][6] = CM376_138;
  M[37][36] = CM3736_138;
  M[37][37] = CM3737_138;
  M[38][4] = CM238_4;
  M[38][5] = CM238_5;
  M[38][6] = CM238_6;
  M[38][38] = s->In[5][38];
  M[45][1] = CM451_346;
  M[45][2] = CM452_346;
  M[45][3] = CM453_346;
  M[45][4] = CM454_346;
  M[45][5] = CM455_346;
  M[45][6] = CM456_346;
  M[45][45] = CM4545_346;
  M[45][46] = CM4645_147;
  M[45][47] = CM4745_248;
  M[45][48] = FM148_45;
  M[45][49] = CM249_45;
  M[45][50] = CM250_45;
  M[45][51] = CM251_45;
  M[45][52] = CM252_45;
  M[45][53] = FB353_45;
  M[46][1] = CM461_147;
  M[46][2] = CM462_147;
  M[46][3] = CM463_147;
  M[46][4] = CM464_147;
  M[46][5] = CM465_147;
  M[46][6] = CM466_147;
  M[46][45] = CM4645_147;
  M[46][46] = CM4646_147;
  M[46][47] = CM4746_248;
  M[46][48] = FM148_46;
  M[47][1] = CM471_248;
  M[47][2] = CM472_248;
  M[47][3] = CM473_248;
  M[47][4] = CM474_248;
  M[47][5] = CM475_248;
  M[47][6] = CM476_248;
  M[47][45] = CM4745_248;
  M[47][46] = CM4746_248;
  M[47][47] = CM4747_248;
  M[47][48] = FM148_47;
  M[47][49] = s->In[5][49];
  M[47][50] = s->In[5][50];
  M[47][51] = s->In[5][51];
  M[47][52] = s->In[5][52];
  M[47][53] = FB353_47;
  M[48][1] = FM148_1;
  M[48][2] = FM148_2;
  M[48][3] = FM148_3;
  M[48][4] = FM148_4;
  M[48][5] = FM148_5;
  M[48][6] = FM148_6;
  M[48][45] = FM148_45;
  M[48][46] = FM148_46;
  M[48][47] = FM148_47;
  M[48][48] = FM148_48;
  M[49][4] = CM249_4;
  M[49][5] = CM249_5;
  M[49][6] = CM249_6;
  M[49][45] = CM249_45;
  M[49][47] = s->In[5][49];
  M[49][49] = s->In[5][49];
  M[50][4] = CM250_4;
  M[50][5] = CM250_5;
  M[50][6] = CM250_6;
  M[50][45] = CM250_45;
  M[50][47] = s->In[5][50];
  M[50][50] = s->In[5][50];
  M[51][4] = CM251_4;
  M[51][5] = CM251_5;
  M[51][6] = CM251_6;
  M[51][45] = CM251_45;
  M[51][47] = s->In[5][51];
  M[51][51] = s->In[5][51];
  M[52][4] = CM252_4;
  M[52][5] = CM252_5;
  M[52][6] = CM252_6;
  M[52][45] = CM252_45;
  M[52][47] = s->In[5][52];
  M[52][52] = s->In[5][52];
  M[53][1] = FB353_1;
  M[53][2] = FB353_2;
  M[53][3] = FB353_3;
  M[53][4] = FB353_4;
  M[53][5] = FB353_5;
  M[53][6] = FB353_6;
  M[53][45] = FB353_45;
  M[53][47] = FB353_47;
  M[53][53] = s->m[53];
  c[1] = FF3_14;
  c[2] = FF3_24;
  c[3] = FF4_35;
  c[4] = CF4_35;
  c[5] = CF5_16;
  c[6] = CF26;
  c[7] = FA17;
  c[8] = CF18;
  c[9] = CF9_210;
  c[10] = CF10_111;
  c[11] = CF311;
  c[12] = CF212;
  c[13] = CF113;
  c[14] = CF14_215;
  c[15] = CF15_116;
  c[16] = CF316;
  c[17] = CF217;
  c[18] = -s->trq[1][18];
  c[19] = -s->trq[1][19];
  c[20] = -s->trq[1][20];
  c[21] = -s->trq[1][21];
  c[22] = CF222;
  c[23] = CF23_124;
  c[24] = -s->trq[2][24];
  c[25] = CF225;
  c[26] = CF26_127;
  c[27] = -s->trq[2][27];
  c[28] = FF228;
  c[29] = CF29_130;
  c[30] = -s->trq[3][30];
  c[31] = CF31_132;
  c[32] = -s->trq[3][32];
  c[33] = CF133;
  c[34] = CF134;
  c[35] = CF235;
  c[36] = CF136;
  c[37] = CF137;
  c[38] = CF238;
  c[39] = CF239;
  c[40] = CF240;
  c[41] = CF41_142;
  c[42] = -s->trq[2][42];
  c[43] = CF43_144;
  c[44] = -s->trq[2][44];
  c[45] = CF45_346;
  c[46] = CF46_147;
  c[47] = CF47_248;
  c[48] = FF148;
  c[49] = CF249;
  c[50] = CF250;
  c[51] = CF251;
  c[52] = CF252;
  c[53] = FA353;

// ====== END Task 0 ====== 


}
 

