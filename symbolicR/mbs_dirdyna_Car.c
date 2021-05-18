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
//	==> Function : F 1 : Direct Dynamics (Semi-Explicit formulation) : RNEA
//	==> Flops complexity : 11010
//
//	==> Generation Time :  0.160 seconds
//	==> Post-Processing :  0.240 seconds
//
//-------------------------------------------------------------
//
 
#include <math.h> 

#include "mbs_data.h"
#include "mbs_project_interface.h"
 
void mbs_dirdyna(double **M,double *c,
MbsData *s, double tsim)

// double M[59][59];
// double c[59];
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
 
// Augmented Joint Position Vectors   

  Dz332 = q[33]+s->dpt[2][47];
 
// Trigonometric Variables  

  C34 = cos(q[34]);
  S34 = sin(q[34]);

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

// = = Block_0_0_0_0_0_21 = = 
 
// Trigonometric Variables  

  C47 = cos(q[47]);
  S47 = sin(q[47]);
  C48 = cos(q[48]);
  S48 = sin(q[48]);
  C49 = cos(q[49]);
  S49 = sin(q[49]);
  C50 = cos(q[50]);
  S50 = sin(q[50]);

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

// = = Block_0_0_0_0_0_27 = = 
 
// Augmented Joint Position Vectors   

  Dz593 = q[59]+s->dpt[3][69];

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
  AlF19 = C9*(AlF18+BeF28*s->dpt[2][19])-S9*(AlF38+BeF88*s->dpt[2][19]);
  AlF29 = C8*(AlF25+BS56*s->dpt[2][1]+BeF46*s->dpt[1][1]+BeF66*s->dpt[3][1])+S8*(AlF36+BS96*s->dpt[3][1]+BeF76*
 s->dpt[1][1]+BeF86*s->dpt[2][1])-s->dpt[2][19]*(OM18*OM18+OM38*OM38);
  AlF39 = C9*(AlF38+BeF88*s->dpt[2][19])+S9*(AlF18+BeF28*s->dpt[2][19]);
  AlM19_1 = AlM16_1*C9-AlM38_1*S9;
  AlM39_1 = AlM16_1*S9+AlM38_1*C9;
  AlM19_2 = AlM16_2*C9-AlM38_2*S9;
  AlM39_2 = AlM16_2*S9+AlM38_2*C9;
  AlM19_3 = AlM16_3*C9-AlM38_3*S9;
  AlM39_3 = AlM16_3*S9+AlM38_3*C9;
  OpM19_4 = OpM16_4*C9-OpM38_4*S9;
  OpM39_4 = OpM16_4*S9+OpM38_4*C9;
  AlM19_4 = C9*(AlM18_4-OpM38_4*s->dpt[2][19])-S9*(AlM38_4+OpM16_4*s->dpt[2][19]);
  AlM39_4 = C9*(AlM38_4+OpM16_4*s->dpt[2][19])+S9*(AlM18_4-OpM38_4*s->dpt[2][19]);
  OpM19_5 = -(OpM38_5*S9-C6*C9);
  OpM39_5 = OpM38_5*C9+C6*S9;
  AlM19_5 = C9*(AlM18_5-OpM38_5*s->dpt[2][19])-S9*(AlM38_5+s->dpt[2][19]*C6);
  AlM39_5 = C9*(AlM38_5+s->dpt[2][19]*C6)+S9*(AlM18_5-OpM38_5*s->dpt[2][19]);
  OpM19_6 = S8*S9;
  OpM39_6 = -S8*C9;
  AlM19_6 = -(AlM38_6*S9-C9*(s->dpt[3][1]+s->dpt[2][19]*S8));
  AlM39_6 = AlM38_6*C9+S9*(s->dpt[3][1]+s->dpt[2][19]*S8);
  AlM19_8 = -s->dpt[2][19]*S9;
  AlM39_8 = s->dpt[2][19]*C9;
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
  AlF114 = C14*(AlF113+BeF213*s->dpt[2][26])-S14*(AlF313+BeF813*s->dpt[2][26]);
  AlF214 = C13*(AlF25+BS56*s->dpt[2][2]+BeF46*s->dpt[1][2]+BeF66*s->dpt[3][2])+S13*(AlF36+BS96*s->dpt[3][2]+BeF76*
 s->dpt[1][2]+BeF86*s->dpt[2][2])-s->dpt[2][26]*(OM113*OM113+OM313*OM313);
  AlF314 = C14*(AlF313+BeF813*s->dpt[2][26])+S14*(AlF113+BeF213*s->dpt[2][26]);
  AlM114_1 = AlM16_1*C14-AlM313_1*S14;
  AlM314_1 = AlM16_1*S14+AlM313_1*C14;
  AlM114_2 = AlM16_2*C14-AlM313_2*S14;
  AlM314_2 = AlM16_2*S14+AlM313_2*C14;
  AlM114_3 = AlM16_3*C14-AlM313_3*S14;
  AlM314_3 = AlM16_3*S14+AlM313_3*C14;
  OpM114_4 = OpM16_4*C14-OpM313_4*S14;
  OpM314_4 = OpM16_4*S14+OpM313_4*C14;
  AlM114_4 = C14*(AlM113_4-OpM313_4*s->dpt[2][26])-S14*(AlM313_4+OpM16_4*s->dpt[2][26]);
  AlM314_4 = C14*(AlM313_4+OpM16_4*s->dpt[2][26])+S14*(AlM113_4-OpM313_4*s->dpt[2][26]);
  OpM114_5 = -(OpM313_5*S14-C14*C6);
  OpM314_5 = OpM313_5*C14+S14*C6;
  AlM114_5 = C14*(AlM113_5-OpM313_5*s->dpt[2][26])-S14*(AlM313_5+s->dpt[2][26]*C6);
  AlM314_5 = C14*(AlM313_5+s->dpt[2][26]*C6)+S14*(AlM113_5-OpM313_5*s->dpt[2][26]);
  OpM114_6 = S13*S14;
  OpM314_6 = -S13*C14;
  AlM114_6 = -(AlM313_6*S14-C14*(s->dpt[3][2]+s->dpt[2][26]*S13));
  AlM314_6 = AlM313_6*C14+S14*(s->dpt[3][2]+s->dpt[2][26]*S13);
  AlM114_13 = -s->dpt[2][26]*S14;
  AlM314_13 = s->dpt[2][26]*C14;
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

// = = Block_0_1_0_1_0_12 = = 
 
// Forward Kinematics 

  AlF128 = AlF16+q[28]*BeF26-(2.0)*qd[28]*OM36+BS16*s->dpt[1][9]+BeF36*s->dpt[3][9];
  AlF228 = AlF25+q[28]*BS56+BeF46*s->dpt[1][9]+BeF66*s->dpt[3][9];
  AlF328 = AlF36+q[28]*BeF86+(2.0)*qd[28]*OM16+BS96*s->dpt[3][9]+BeF76*s->dpt[1][9];
  AlM128_4 = -(q[28]*OpM36_4-s->dpt[3][9]*S5);
  AlM228_4 = -(OpM16_4*s->dpt[3][9]-OpM36_4*s->dpt[1][9]);
  AlM328_4 = q[28]*OpM16_4-s->dpt[1][9]*S5;
  AlM128_5 = -q[28]*S6;
  AlM228_5 = s->dpt[1][9]*S6-s->dpt[3][9]*C6;
  AlM328_5 = q[28]*C6;

// = = Block_0_1_0_1_0_16 = = 
 
// Trigonometric Variables  

  S35p36 = C35*S36+S35*C36;
  C35p36 = C35*C36-S35*S36;
 
// Forward Kinematics 

  OM135 = qd[35]+OM16;
  OM235 = OM26*C35+OM36*S35;
  OM335 = -(OM26*S35-OM36*C35);
  OpF235 = C35*(OpF25+qd[35]*OM36)+S35*(OpF36-qd[35]*OM26);
  OpF335 = C35*(OpF36-qd[35]*OM26)-S35*(OpF25+qd[35]*OM36);
  BS535 = -(OM135*OM135+OM335*OM335);
  BeF835 = OpF16+OM235*OM335;
  AlF235 = C35*(AlF25+BS56*s->dpt[2][12]+BeF46*s->dpt[1][12]+BeF66*s->dpt[3][12])+S35*(AlF36+BS96*s->dpt[3][12]+BeF76*
 s->dpt[1][12]+BeF86*s->dpt[2][12]);
  AlF335 = C35*(AlF36+BS96*s->dpt[3][12]+BeF76*s->dpt[1][12]+BeF86*s->dpt[2][12])-S35*(AlF25+BS56*s->dpt[2][12]+BeF46*
 s->dpt[1][12]+BeF66*s->dpt[3][12]);
  AlM235_1 = AlM25_1*C35+AlM36_1*S35;
  AlM335_1 = -(AlM25_1*S35-AlM36_1*C35);
  AlM235_2 = AlM25_2*C35+AlM36_2*S35;
  AlM335_2 = -(AlM25_2*S35-AlM36_2*C35);
  AlM235_3 = AlM36_3*S35+C35*S5;
  AlM335_3 = AlM36_3*C35-S35*S5;
  OpM235_4 = OpM36_4*S35+C35*S5;
  OpM335_4 = OpM36_4*C35-S35*S5;
  AlM235_4 = -(C35*(OpM16_4*s->dpt[3][12]-OpM36_4*s->dpt[1][12])-S35*(OpM16_4*s->dpt[2][12]-s->dpt[1][12]*S5));
  AlM335_4 = C35*(OpM16_4*s->dpt[2][12]-s->dpt[1][12]*S5)+S35*(OpM16_4*s->dpt[3][12]-OpM36_4*s->dpt[1][12]);
  AlM235_5 = s->dpt[2][12]*S35*C6+C35*(s->dpt[1][12]*S6-s->dpt[3][12]*C6);
  AlM335_5 = s->dpt[2][12]*C35*C6-S35*(s->dpt[1][12]*S6-s->dpt[3][12]*C6);
  OM136 = qd[36]+OM135;
  OM236 = OM235*C36+OM335*S36;
  OM336 = -(OM235*S36-OM335*C36);
  OpF236 = C36*(OpF235+qd[36]*OM335)+S36*(OpF335-qd[36]*OM235);
  OpF336 = C36*(OpF335-qd[36]*OM235)-S36*(OpF235+qd[36]*OM335);
  BS936 = -(OM136*OM136+OM236*OM236);
  BeF336 = OpF236+OM136*OM336;
  AlF136 = AlF16+BS16*s->dpt[1][12]+BeF26*s->dpt[2][12]+BeF36*s->dpt[3][12]-s->dpt[2][50]*(OpF335-OM135*OM235);
  AlF236 = C36*(AlF235+BS535*s->dpt[2][50])+S36*(AlF335+BeF835*s->dpt[2][50]);
  AlF336 = C36*(AlF335+BeF835*s->dpt[2][50])-S36*(AlF235+BS535*s->dpt[2][50]);
  AlM236_1 = AlM235_1*C36+AlM335_1*S36;
  AlM336_1 = -(AlM235_1*S36-AlM335_1*C36);
  AlM236_2 = AlM235_2*C36+AlM335_2*S36;
  AlM336_2 = -(AlM235_2*S36-AlM335_2*C36);
  AlM236_3 = AlM235_3*C36+AlM335_3*S36;
  AlM336_3 = -(AlM235_3*S36-AlM335_3*C36);
  OpM236_4 = OpM235_4*C36+OpM335_4*S36;
  OpM336_4 = -(OpM235_4*S36-OpM335_4*C36);
  AlM136_4 = -(OpM335_4*s->dpt[2][50]+OpM36_4*s->dpt[2][12]-s->dpt[3][12]*S5);
  AlM236_4 = AlM235_4*C36+S36*(AlM335_4+OpM16_4*s->dpt[2][50]);
  AlM336_4 = -(AlM235_4*S36-C36*(AlM335_4+OpM16_4*s->dpt[2][50]));
  OpM236_5 = S35p36*S6;
  OpM336_5 = C35p36*S6;
  AlM136_5 = -S6*(s->dpt[2][12]+s->dpt[2][50]*C35);
  AlM236_5 = AlM235_5*C36+S36*(AlM335_5+s->dpt[2][50]*C6);
  AlM336_5 = -(AlM235_5*S36-C36*(AlM335_5+s->dpt[2][50]*C6));
  AlM136_6 = s->dpt[3][12]+s->dpt[2][50]*S35;
  AlM236_6 = -s->dpt[1][12]*S35p36;
  AlM336_6 = -s->dpt[1][12]*C35p36;
  AlM236_35 = s->dpt[2][50]*S36;
  AlM336_35 = s->dpt[2][50]*C36;
  OM137 = OM136*C37-OM336*S37;
  OM237 = qd[37]+OM236;
  OM337 = OM136*S37+OM336*C37;

// = = Block_0_1_0_1_0_17 = = 
 
// Trigonometric Variables  

  S38p39 = C38*S39+S38*C39;
  C38p39 = C38*C39-S38*S39;
 
// Forward Kinematics 

  OM138 = qd[38]+OM16;
  OM238 = OM26*C38+OM36*S38;
  OM338 = -(OM26*S38-OM36*C38);
  OpF238 = C38*(OpF25+qd[38]*OM36)+S38*(OpF36-qd[38]*OM26);
  OpF338 = C38*(OpF36-qd[38]*OM26)-S38*(OpF25+qd[38]*OM36);
  BS538 = -(OM138*OM138+OM338*OM338);
  BeF838 = OpF16+OM238*OM338;
  AlF238 = C38*(AlF25+BS56*s->dpt[2][13]+BeF46*s->dpt[1][13]+BeF66*s->dpt[3][13])+S38*(AlF36+BS96*s->dpt[3][13]+BeF76*
 s->dpt[1][13]+BeF86*s->dpt[2][13]);
  AlF338 = C38*(AlF36+BS96*s->dpt[3][13]+BeF76*s->dpt[1][13]+BeF86*s->dpt[2][13])-S38*(AlF25+BS56*s->dpt[2][13]+BeF46*
 s->dpt[1][13]+BeF66*s->dpt[3][13]);
  AlM238_1 = AlM25_1*C38+AlM36_1*S38;
  AlM338_1 = -(AlM25_1*S38-AlM36_1*C38);
  AlM238_2 = AlM25_2*C38+AlM36_2*S38;
  AlM338_2 = -(AlM25_2*S38-AlM36_2*C38);
  AlM238_3 = AlM36_3*S38+C38*S5;
  AlM338_3 = AlM36_3*C38-S38*S5;
  OpM238_4 = OpM36_4*S38+C38*S5;
  OpM338_4 = OpM36_4*C38-S38*S5;
  AlM238_4 = -(C38*(OpM16_4*s->dpt[3][13]-OpM36_4*s->dpt[1][13])-S38*(OpM16_4*s->dpt[2][13]-s->dpt[1][13]*S5));
  AlM338_4 = C38*(OpM16_4*s->dpt[2][13]-s->dpt[1][13]*S5)+S38*(OpM16_4*s->dpt[3][13]-OpM36_4*s->dpt[1][13]);
  AlM238_5 = s->dpt[2][13]*S38*C6+C38*(s->dpt[1][13]*S6-s->dpt[3][13]*C6);
  AlM338_5 = s->dpt[2][13]*C38*C6-S38*(s->dpt[1][13]*S6-s->dpt[3][13]*C6);
  OM139 = qd[39]+OM138;
  OM239 = OM238*C39+OM338*S39;
  OM339 = -(OM238*S39-OM338*C39);
  OpF239 = C39*(OpF238+qd[39]*OM338)+S39*(OpF338-qd[39]*OM238);
  OpF339 = C39*(OpF338-qd[39]*OM238)-S39*(OpF238+qd[39]*OM338);
  BS939 = -(OM139*OM139+OM239*OM239);
  BeF339 = OpF239+OM139*OM339;
  AlF139 = AlF16+BS16*s->dpt[1][13]+BeF26*s->dpt[2][13]+BeF36*s->dpt[3][13]-s->dpt[2][55]*(OpF338-OM138*OM238);
  AlF239 = C39*(AlF238+BS538*s->dpt[2][55])+S39*(AlF338+BeF838*s->dpt[2][55]);
  AlF339 = C39*(AlF338+BeF838*s->dpt[2][55])-S39*(AlF238+BS538*s->dpt[2][55]);
  AlM239_1 = AlM238_1*C39+AlM338_1*S39;
  AlM339_1 = -(AlM238_1*S39-AlM338_1*C39);
  AlM239_2 = AlM238_2*C39+AlM338_2*S39;
  AlM339_2 = -(AlM238_2*S39-AlM338_2*C39);
  AlM239_3 = AlM238_3*C39+AlM338_3*S39;
  AlM339_3 = -(AlM238_3*S39-AlM338_3*C39);
  OpM239_4 = OpM238_4*C39+OpM338_4*S39;
  OpM339_4 = -(OpM238_4*S39-OpM338_4*C39);
  AlM139_4 = -(OpM338_4*s->dpt[2][55]+OpM36_4*s->dpt[2][13]-s->dpt[3][13]*S5);
  AlM239_4 = AlM238_4*C39+S39*(AlM338_4+OpM16_4*s->dpt[2][55]);
  AlM339_4 = -(AlM238_4*S39-C39*(AlM338_4+OpM16_4*s->dpt[2][55]));
  OpM239_5 = S38p39*S6;
  OpM339_5 = C38p39*S6;
  AlM139_5 = -S6*(s->dpt[2][13]+s->dpt[2][55]*C38);
  AlM239_5 = AlM238_5*C39+S39*(AlM338_5+s->dpt[2][55]*C6);
  AlM339_5 = -(AlM238_5*S39-C39*(AlM338_5+s->dpt[2][55]*C6));
  AlM139_6 = s->dpt[3][13]+s->dpt[2][55]*S38;
  AlM239_6 = -s->dpt[1][13]*S38p39;
  AlM339_6 = -s->dpt[1][13]*C38p39;
  AlM239_38 = s->dpt[2][55]*S39;
  AlM339_38 = s->dpt[2][55]*C39;
  OM140 = OM139*C40-OM339*S40;
  OM240 = qd[40]+OM239;
  OM340 = OM139*S40+OM339*C40;

// = = Block_0_1_0_1_0_21 = = 
 
// Trigonometric Variables  

  S47p48 = C47*S48+S47*C48;
  C47p48 = C47*C48-S47*S48;
  S47p48p49 = C47p48*S49+S47p48*C49;
  C47p48p49 = C47p48*C49-S47p48*S49;
  S47p48p49p50 = C47p48p49*S50+S47p48p49*C50;
  C47p48p49p50 = C47p48p49*C50-S47p48p49*S50;
 
// Forward Kinematics 

  OM247 = OM26*C47+OM36*S47;
  OM347 = -(OM26*S47-OM36*C47);
  OpF247 = C47*(OpF25+qd[47]*OM36)+S47*(OpF36-qd[47]*OM26);
  OpF347 = C47*(OpF36-qd[47]*OM26)-S47*(OpF25+qd[47]*OM36);
  AlF247 = C47*(AlF25+BS56*s->dpt[2][17]+BeF46*s->dpt[1][17]+BeF66*s->dpt[3][17])+S47*(AlF36+BS96*s->dpt[3][17]+BeF76*
 s->dpt[1][17]+BeF86*s->dpt[2][17]);
  AlF347 = C47*(AlF36+BS96*s->dpt[3][17]+BeF76*s->dpt[1][17]+BeF86*s->dpt[2][17])-S47*(AlF25+BS56*s->dpt[2][17]+BeF46*
 s->dpt[1][17]+BeF66*s->dpt[3][17]);
  AlM247_1 = AlM25_1*C47+AlM36_1*S47;
  AlM347_1 = -(AlM25_1*S47-AlM36_1*C47);
  AlM247_2 = AlM25_2*C47+AlM36_2*S47;
  AlM347_2 = -(AlM25_2*S47-AlM36_2*C47);
  AlM247_3 = AlM36_3*S47+C47*S5;
  AlM347_3 = AlM36_3*C47-S47*S5;
  OpM247_4 = OpM36_4*S47+C47*S5;
  OpM347_4 = OpM36_4*C47-S47*S5;
  AlM247_4 = -(C47*(OpM16_4*s->dpt[3][17]-OpM36_4*s->dpt[1][17])-S47*(OpM16_4*s->dpt[2][17]-s->dpt[1][17]*S5));
  AlM347_4 = C47*(OpM16_4*s->dpt[2][17]-s->dpt[1][17]*S5)+S47*(OpM16_4*s->dpt[3][17]-OpM36_4*s->dpt[1][17]);
  AlM247_5 = s->dpt[2][17]*S47*C6+C47*(s->dpt[1][17]*S6-s->dpt[3][17]*C6);
  AlM347_5 = s->dpt[2][17]*C47*C6-S47*(s->dpt[1][17]*S6-s->dpt[3][17]*C6);
  OM248 = OM247*C48+OM347*S48;
  OM348 = -(OM247*S48-OM347*C48);
  OpF248 = C48*(OpF247+qd[48]*OM347)+S48*(OpF347-qd[48]*OM247);
  OpF348 = C48*(OpF347-qd[48]*OM247)-S48*(OpF247+qd[48]*OM347);
  AlF248 = AlF247*C48+AlF347*S48;
  AlF348 = -(AlF247*S48-AlF347*C48);
  AlM248_1 = AlM247_1*C48+AlM347_1*S48;
  AlM348_1 = -(AlM247_1*S48-AlM347_1*C48);
  AlM248_2 = AlM247_2*C48+AlM347_2*S48;
  AlM348_2 = -(AlM247_2*S48-AlM347_2*C48);
  AlM248_3 = AlM247_3*C48+AlM347_3*S48;
  AlM348_3 = -(AlM247_3*S48-AlM347_3*C48);
  OpM248_4 = OpM247_4*C48+OpM347_4*S48;
  OpM348_4 = -(OpM247_4*S48-OpM347_4*C48);
  AlM248_4 = AlM247_4*C48+AlM347_4*S48;
  AlM348_4 = -(AlM247_4*S48-AlM347_4*C48);
  AlM248_5 = AlM247_5*C48+AlM347_5*S48;
  AlM348_5 = -(AlM247_5*S48-AlM347_5*C48);
  OM249 = OM248*C49+OM348*S49;
  OM349 = -(OM248*S49-OM348*C49);
  OpF249 = C49*(OpF248+qd[49]*OM348)+S49*(OpF348-qd[49]*OM248);
  OpF349 = C49*(OpF348-qd[49]*OM248)-S49*(OpF248+qd[49]*OM348);
  AlF249 = AlF248*C49+AlF348*S49;
  AlF349 = -(AlF248*S49-AlF348*C49);
  AlM249_1 = AlM248_1*C49+AlM348_1*S49;
  AlM349_1 = -(AlM248_1*S49-AlM348_1*C49);
  AlM249_2 = AlM248_2*C49+AlM348_2*S49;
  AlM349_2 = -(AlM248_2*S49-AlM348_2*C49);
  AlM249_3 = AlM248_3*C49+AlM348_3*S49;
  AlM349_3 = -(AlM248_3*S49-AlM348_3*C49);
  OpM249_4 = OpM248_4*C49+OpM348_4*S49;
  OpM349_4 = -(OpM248_4*S49-OpM348_4*C49);
  AlM249_4 = AlM248_4*C49+AlM348_4*S49;
  AlM349_4 = -(AlM248_4*S49-AlM348_4*C49);
  AlM249_5 = AlM248_5*C49+AlM348_5*S49;
  AlM349_5 = -(AlM248_5*S49-AlM348_5*C49);
  OM150 = qd[47]+qd[48]+qd[49]+qd[50]+OM16;
  OM250 = OM249*C50+OM349*S50;
  OM350 = -(OM249*S50-OM349*C50);

// = = Block_0_1_0_1_0_22 = = 
 
// Forward Kinematics 

  OM251 = -(OM16*S51-OM26*C51);
  OM351 = qd[51]+OM36;
  OpF151 = C51*(OpF16+qd[51]*OM26)+S51*(OpF25-qd[51]*OM16);
  OpF251 = C51*(OpF25-qd[51]*OM16)-S51*(OpF16+qd[51]*OM26);
  AlF151 = C51*(AlF16+BS16*s->dpt[1][18]+BeF36*s->dpt[3][18])+S51*(AlF25+BeF46*s->dpt[1][18]+BeF66*s->dpt[3][18]);
  AlF251 = C51*(AlF25+BeF46*s->dpt[1][18]+BeF66*s->dpt[3][18])-S51*(AlF16+BS16*s->dpt[1][18]+BeF36*s->dpt[3][18]);
  AlF351 = AlF36+BS96*s->dpt[3][18]+BeF76*s->dpt[1][18];
  AlM151_1 = AlM16_1*C51+AlM25_1*S51;
  AlM251_1 = -(AlM16_1*S51-AlM25_1*C51);
  AlM151_2 = AlM16_2*C51+AlM25_2*S51;
  AlM251_2 = -(AlM16_2*S51-AlM25_2*C51);
  AlM151_3 = AlM16_3*C51+S51*S5;
  AlM251_3 = -(AlM16_3*S51-C51*S5);
  OpM151_4 = OpM16_4*C51+S51*S5;
  OpM251_4 = -(OpM16_4*S51-C51*S5);
  AlM151_4 = s->dpt[3][18]*C51*S5-S51*(OpM16_4*s->dpt[3][18]-OpM36_4*s->dpt[1][18]);
  AlM251_4 = -(s->dpt[3][18]*S51*S5+C51*(OpM16_4*s->dpt[3][18]-OpM36_4*s->dpt[1][18]));
  AlM351_4 = -s->dpt[1][18]*S5;
  OpM151_5 = C51*C6;
  OpM251_5 = -S51*C6;
  AlM151_5 = S51*(s->dpt[1][18]*S6-s->dpt[3][18]*C6);
  AlM251_5 = C51*(s->dpt[1][18]*S6-s->dpt[3][18]*C6);
  AlM151_6 = s->dpt[3][18]*C51;
  AlM251_6 = -s->dpt[3][18]*S51;
  OM152 = qd[52]+OM16*C51+OM26*S51;
  OM352 = -(OM251*S52-OM351*C52);
  OpF252 = C52*(OpF251+qd[52]*OM351)+S52*(OpF36-qd[52]*OM251);
  OpF352 = C52*(OpF36-qd[52]*OM251)-S52*(OpF251+qd[52]*OM351);
  AlF352 = -(AlF251*S52-AlF351*C52);
  AlM252_1 = AlM251_1*C52+AlM36_1*S52;
  AlM352_1 = -(AlM251_1*S52-AlM36_1*C52);
  AlM252_2 = AlM251_2*C52+AlM36_2*S52;
  AlM352_2 = -(AlM251_2*S52-AlM36_2*C52);
  AlM252_3 = AlM251_3*C52+AlM36_3*S52;
  AlM352_3 = -(AlM251_3*S52-AlM36_3*C52);
  OpM252_4 = OpM251_4*C52+OpM36_4*S52;
  OpM352_4 = -(OpM251_4*S52-OpM36_4*C52);
  AlM352_4 = -(AlM251_4*S52-AlM351_4*C52);
  OpM252_5 = OpM251_5*C52+S52*S6;
  OpM352_5 = -(OpM251_5*S52-C52*S6);
  AlM352_5 = -AlM251_5*S52;
  OpM252_6 = C51*C52;
  OpM352_6 = -C51*S52;
  AlM352_6 = -(AlM251_6*S52+s->dpt[1][18]*C52);
  OM153 = OM152*C53-OM352*S53;
  OM253 = qd[53]+OM251*C52+OM351*S52;
  OM353 = OM152*S53+OM352*C53;
  OpF153 = C53*(OpF151-qd[53]*OM352)-S53*(OpF352+qd[53]*OM152);
  OpF353 = C53*(OpF352+qd[53]*OM152)+S53*(OpF151-qd[53]*OM352);
  AlM153_1 = AlM151_1*C53-AlM352_1*S53;
  AlM353_1 = AlM151_1*S53+AlM352_1*C53;
  AlM153_2 = AlM151_2*C53-AlM352_2*S53;
  AlM353_2 = AlM151_2*S53+AlM352_2*C53;
  AlM153_3 = AlM151_3*C53-AlM352_3*S53;
  AlM353_3 = AlM151_3*S53+AlM352_3*C53;
  OpM153_4 = OpM151_4*C53-OpM352_4*S53;
  OpM353_4 = OpM151_4*S53+OpM352_4*C53;
  AlM153_4 = AlM151_4*C53-AlM352_4*S53;
  OpM153_5 = OpM151_5*C53-OpM352_5*S53;
  OpM353_5 = OpM151_5*S53+OpM352_5*C53;
  AlM153_5 = AlM151_5*C53-AlM352_5*S53;
  OpM153_6 = -(OpM352_6*S53-S51*C53);
  OpM353_6 = OpM352_6*C53+S51*S53;
  AlM153_6 = AlM151_6*C53-AlM352_6*S53;
  OpM153_51 = -C52*S53;
  OpM353_51 = C52*C53;
  BS154 = -(OM253*OM253+OM353*OM353);
  BS254 = OM153*OM253;
  BS354 = OM153*OM353;
  BS554 = -(OM153*OM153+OM353*OM353);
  BS654 = OM253*OM353;
  BS954 = -(OM153*OM153+OM253*OM253);
  BeF254 = BS254-OpF353;
  BeF354 = BS354+OpF252;
  BeF454 = BS254+OpF353;
  BeF654 = BS654-OpF153;
  BeF754 = BS354-OpF252;
  BeF854 = BS654+OpF153;
  AlF154 = AlF151*C53-AlF352*S53-q[54]*(OM253*OM253+OM353*OM353);
  AlF254 = q[54]*(OpF353+OM153*OM253)+(2.0)*qd[54]*OM353+AlF251*C52+AlF351*S52;
  AlF354 = AlF151*S53+AlF352*C53-q[54]*(OpF252-OM153*OM353)-(2.0)*qd[54]*OM253;
  AlM254_4 = q[54]*OpM353_4+AlM251_4*C52+AlM351_4*S52;
  AlM354_4 = AlM151_4*S53+AlM352_4*C53-q[54]*OpM252_4;
  AlM254_5 = q[54]*OpM353_5+AlM251_5*C52;
  AlM354_5 = AlM151_5*S53+AlM352_5*C53-q[54]*OpM252_5;
  AlM254_6 = q[54]*OpM353_6+AlM251_6*C52-s->dpt[1][18]*S52;
  AlM354_6 = AlM151_6*S53+AlM352_6*C53-q[54]*OpM252_6;
  AlM254_51 = q[54]*OpM353_51;
  AlM354_51 = -q[54]*S52;
  AlM254_52 = q[54]*S53;

// = = Block_0_1_0_2_0_15 = = 
 
// Forward Kinematics 

  AlF133 = AlF128-(2.0)*qd[33]*OM36-Dz332*(OpF36-OM16*OM26);
  AlF233 = AlF228-Dz332*(OM16*OM16+OM36*OM36);
  AlM133_4 = AlM128_4-Dz332*OpM36_4;
  AlM133_5 = AlM128_5-Dz332*S6;
  OM134 = OM16*C34+OM26*S34;
  OM234 = -(OM16*S34-OM26*C34);
  OM334 = qd[34]+OM36;

// = = Block_0_1_0_2_0_23 = = 
 
// Trigonometric Variables  

  S53p55 = C53*S55+S53*C55;
  C53p55 = C53*C55-S53*S55;
 
// Forward Kinematics 

  OM155 = OM153*C55-OM353*S55;
  OM255 = qd[55]+OM253;
  OM355 = OM153*S55+OM353*C55;

// = = Block_0_1_0_2_0_24 = = 
 
// Trigonometric Variables  

  S53p56 = C53*S56+S53*C56;
  C53p56 = C53*C56-S53*S56;
 
// Forward Kinematics 

  OM156 = OM153*C56-OM353*S56;
  OM256 = qd[56]+OM253;
  OM356 = OM153*S56+OM353*C56;

// = = Block_0_1_0_2_0_25 = = 
 
// Trigonometric Variables  

  S53p57 = C53*S57+S53*C57;
  C53p57 = C53*C57-S53*S57;
 
// Forward Kinematics 

  OM157 = OM153*C57-OM353*S57;
  OM257 = qd[57]+OM253;
  OM357 = OM153*S57+OM353*C57;

// = = Block_0_1_0_2_0_26 = = 
 
// Trigonometric Variables  

  S53p58 = C53*S58+S53*C58;
  C53p58 = C53*C58-S53*S58;
 
// Forward Kinematics 

  OM158 = OM153*C58-OM353*S58;
  OM258 = qd[58]+OM253;
  OM358 = OM153*S58+OM353*C58;

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
  CF125 = -(s->trq[1][25]-CF26_127+s->dpt[2][43]*(s->frc[2][27]*S26-FF26_327*C26));
  CF225 = -(s->trq[2][25]+s->trq[2][27]*C26+CF26_327*S26-s->dpt[1][43]*(s->frc[2][27]*S26-FF26_327*C26));
  CF325 = -(s->trq[3][25]+s->trq[2][27]*S26-CF26_327*C26+FF26_127*s->dpt[2][43]+s->dpt[1][43]*(s->frc[2][27]*C26+
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

// = = Block_0_2_0_1_0_15 = = 
 
// Backward Dynamics 

  FA134 = -(s->frc[1][34]-s->m[34]*(AlF133*C34+AlF233*S34));
  FA234 = -(s->frc[2][34]+s->m[34]*(AlF133*S34-AlF233*C34));
  FA334 = -(s->frc[3][34]-s->m[34]*(AlF328+(2.0)*qd[33]*OM16+Dz332*(OpF16+OM26*OM36)));
  CF134 = -(s->trq[1][34]-s->In[1][34]*(C34*(OpF16+qd[34]*OM26)+S34*(OpF25-qd[34]*OM16))+OM234*OM334*(s->In[5][34]-
 s->In[9][34]));
  CF234 = -(s->trq[2][34]-s->In[5][34]*(C34*(OpF25-qd[34]*OM16)-S34*(OpF16+qd[34]*OM26))-OM134*OM334*(s->In[1][34]-
 s->In[9][34]));
  CF334 = -(s->trq[3][34]-s->In[9][34]*OpF36+OM134*OM234*(s->In[1][34]-s->In[5][34]));
  FB134_1 = s->m[34]*(AlM16_1*C34+AlM25_1*S34);
  FB234_1 = -s->m[34]*(AlM16_1*S34-AlM25_1*C34);
  FB334_1 = s->m[34]*AlM36_1;
  FB134_2 = s->m[34]*(AlM16_2*C34+AlM25_2*S34);
  FB234_2 = -s->m[34]*(AlM16_2*S34-AlM25_2*C34);
  FB334_2 = s->m[34]*AlM36_2;
  FB134_3 = s->m[34]*(AlM16_3*C34+S34*S5);
  FB234_3 = -s->m[34]*(AlM16_3*S34-C34*S5);
  FB334_3 = s->m[34]*AlM36_3;
  FB134_4 = s->m[34]*(AlM133_4*C34+AlM228_4*S34);
  FB234_4 = -s->m[34]*(AlM133_4*S34-AlM228_4*C34);
  FB334_4 = s->m[34]*(AlM328_4+Dz332*OpM16_4);
  CM134_4 = s->In[1][34]*(OpM16_4*C34+S34*S5);
  CM234_4 = -s->In[5][34]*(OpM16_4*S34-C34*S5);
  CM334_4 = s->In[9][34]*OpM36_4;
  FB134_5 = s->m[34]*(AlM133_5*C34+AlM228_5*S34);
  FB234_5 = -s->m[34]*(AlM133_5*S34-AlM228_5*C34);
  FB334_5 = s->m[34]*(AlM328_5+Dz332*C6);
  CM134_5 = s->In[1][34]*C34*C6;
  CM234_5 = -s->In[5][34]*S34*C6;
  CM334_5 = s->In[9][34]*S6;
  FF33_134 = FA134*C34-FA234*S34;
  FF33_234 = FA134*S34+FA234*C34;
  FM331_134 = FB134_1*C34-FB234_1*S34;
  FM331_234 = FB134_1*S34+FB234_1*C34;
  FM332_134 = FB134_2*C34-FB234_2*S34;
  FM332_234 = FB134_2*S34+FB234_2*C34;
  FM333_134 = FB134_3*C34-FB234_3*S34;
  FM333_234 = FB134_3*S34+FB234_3*C34;
  FM334_134 = FB134_4*C34-FB234_4*S34;
  FM334_234 = FB134_4*S34+FB234_4*C34;
  FM335_134 = FB134_5*C34-FB234_5*S34;
  FM335_234 = FB134_5*S34+FB234_5*C34;

// = = Block_0_2_0_1_0_19 = = 
 
// Backward Dynamics 

  FF43_144 = -(s->frc[1][44]*C44+s->frc[3][44]*S44);
  FF43_344 = s->frc[1][44]*S44-s->frc[3][44]*C44;
  CF43_144 = -(s->trq[1][44]*C44+s->trq[3][44]*S44);
  CF43_344 = s->trq[1][44]*S44-s->trq[3][44]*C44;
  FF142 = -(s->frc[1][42]-FF43_144);
  FF342 = -(s->frc[3][42]+s->frc[2][44]*S43-FF43_344*C43);
  CF142 = -(s->trq[1][42]-CF43_144+s->dpt[2][62]*(s->frc[2][44]*S43-FF43_344*C43));
  CF242 = -(s->trq[2][42]+s->trq[2][44]*C43+CF43_344*S43-s->dpt[1][62]*(s->frc[2][44]*S43-FF43_344*C43));
  CF342 = -(s->trq[3][42]+s->trq[2][44]*S43-CF43_344*C43+FF43_144*s->dpt[2][62]+s->dpt[1][62]*(s->frc[2][44]*C43+
 FF43_344*S43));

// = = Block_0_2_0_1_0_20 = = 
 
// Backward Dynamics 

  FF45_146 = -(s->frc[1][46]*C46+s->frc[3][46]*S46);
  FF45_346 = s->frc[1][46]*S46-s->frc[3][46]*C46;
  CF45_146 = -(s->trq[1][46]*C46+s->trq[3][46]*S46);
  CF45_346 = s->trq[1][46]*S46-s->trq[3][46]*C46;

// = = Block_0_2_0_1_0_23 = = 
 
// Backward Dynamics 

  FA155 = -(s->frc[1][55]-s->m[55]*(C55*(AlF154+BS154*s->dpt[1][65]+BeF254*s->dpt[2][65]+BeF354*s->dpt[3][65])-S55*(
 AlF354+BS954*s->dpt[3][65]+BeF754*s->dpt[1][65]+BeF854*s->dpt[2][65])));
  FA255 = -(s->frc[2][55]-s->m[55]*(AlF254+BS554*s->dpt[2][65]+BeF454*s->dpt[1][65]+BeF654*s->dpt[3][65]));
  FA355 = -(s->frc[3][55]-s->m[55]*(C55*(AlF354+BS954*s->dpt[3][65]+BeF754*s->dpt[1][65]+BeF854*s->dpt[2][65])+S55*(
 AlF154+BS154*s->dpt[1][65]+BeF254*s->dpt[2][65]+BeF354*s->dpt[3][65])));
  CF155 = -(s->trq[1][55]-s->In[1][55]*(C55*(OpF153-qd[55]*OM353)-S55*(OpF353+qd[55]*OM153))+OM255*OM355*(s->In[5][55]-
 s->In[9][55]));
  CF255 = -(s->trq[2][55]-s->In[5][55]*OpF252-OM155*OM355*(s->In[1][55]-s->In[9][55]));
  CF355 = -(s->trq[3][55]-s->In[9][55]*(C55*(OpF353+qd[55]*OM153)+S55*(OpF153-qd[55]*OM353))+OM155*OM255*(s->In[1][55]-
 s->In[5][55]));
  FB155_1 = s->m[55]*(AlM153_1*C55-AlM353_1*S55);
  FB255_1 = s->m[55]*AlM252_1;
  FB355_1 = s->m[55]*(AlM153_1*S55+AlM353_1*C55);
  FB155_2 = s->m[55]*(AlM153_2*C55-AlM353_2*S55);
  FB255_2 = s->m[55]*AlM252_2;
  FB355_2 = s->m[55]*(AlM153_2*S55+AlM353_2*C55);
  FB155_3 = s->m[55]*(AlM153_3*C55-AlM353_3*S55);
  FB255_3 = s->m[55]*AlM252_3;
  FB355_3 = s->m[55]*(AlM153_3*S55+AlM353_3*C55);
  FB155_4 = s->m[55]*(C55*(AlM153_4+OpM252_4*s->dpt[3][65]-OpM353_4*s->dpt[2][65])-S55*(AlM354_4+OpM153_4*s->dpt[2][65]-
 OpM252_4*s->dpt[1][65]));
  FB255_4 = s->m[55]*(AlM254_4-OpM153_4*s->dpt[3][65]+OpM353_4*s->dpt[1][65]);
  FB355_4 = s->m[55]*(C55*(AlM354_4+OpM153_4*s->dpt[2][65]-OpM252_4*s->dpt[1][65])+S55*(AlM153_4+OpM252_4*s->dpt[3][65]-
 OpM353_4*s->dpt[2][65]));
  CM155_4 = s->In[1][55]*(OpM153_4*C55-OpM353_4*S55);
  CM255_4 = s->In[5][55]*OpM252_4;
  CM355_4 = s->In[9][55]*(OpM153_4*S55+OpM353_4*C55);
  FB155_5 = s->m[55]*(C55*(AlM153_5+OpM252_5*s->dpt[3][65]-OpM353_5*s->dpt[2][65])-S55*(AlM354_5+OpM153_5*s->dpt[2][65]-
 OpM252_5*s->dpt[1][65]));
  FB255_5 = s->m[55]*(AlM254_5-OpM153_5*s->dpt[3][65]+OpM353_5*s->dpt[1][65]);
  FB355_5 = s->m[55]*(C55*(AlM354_5+OpM153_5*s->dpt[2][65]-OpM252_5*s->dpt[1][65])+S55*(AlM153_5+OpM252_5*s->dpt[3][65]-
 OpM353_5*s->dpt[2][65]));
  CM155_5 = s->In[1][55]*(OpM153_5*C55-OpM353_5*S55);
  CM255_5 = s->In[5][55]*OpM252_5;
  CM355_5 = s->In[9][55]*(OpM153_5*S55+OpM353_5*C55);
  FB155_6 = s->m[55]*(C55*(AlM153_6+OpM252_6*s->dpt[3][65]-OpM353_6*s->dpt[2][65])-S55*(AlM354_6+OpM153_6*s->dpt[2][65]-
 OpM252_6*s->dpt[1][65]));
  FB255_6 = s->m[55]*(AlM254_6-OpM153_6*s->dpt[3][65]+OpM353_6*s->dpt[1][65]);
  FB355_6 = s->m[55]*(C55*(AlM354_6+OpM153_6*s->dpt[2][65]-OpM252_6*s->dpt[1][65])+S55*(AlM153_6+OpM252_6*s->dpt[3][65]-
 OpM353_6*s->dpt[2][65]));
  CM155_6 = s->In[1][55]*(OpM153_6*C55-OpM353_6*S55);
  CM255_6 = s->In[5][55]*OpM252_6;
  CM355_6 = s->In[9][55]*(OpM153_6*S55+OpM353_6*C55);
  FB155_51 = -s->m[55]*(C55*(OpM353_51*s->dpt[2][65]-s->dpt[3][65]*S52)+S55*(AlM354_51+OpM153_51*s->dpt[2][65]-
 s->dpt[1][65]*S52));
  FB255_51 = s->m[55]*(AlM254_51-OpM153_51*s->dpt[3][65]+OpM353_51*s->dpt[1][65]);
  FB355_51 = s->m[55]*(C55*(AlM354_51+OpM153_51*s->dpt[2][65]-s->dpt[1][65]*S52)-S55*(OpM353_51*s->dpt[2][65]-
 s->dpt[3][65]*S52));
  CM155_51 = -s->In[1][55]*S53p55*C52;
  CM255_51 = s->In[5][55]*S52;
  CM355_51 = s->In[9][55]*C53p55*C52;
  FB155_52 = -s->m[55]*s->dpt[2][65]*S53p55;
  FB255_52 = s->m[55]*(AlM254_52+s->dpt[1][65]*S53-s->dpt[3][65]*C53);
  FB355_52 = s->m[55]*s->dpt[2][65]*C53p55;
  CM155_52 = s->In[1][55]*C53p55;
  CM355_52 = s->In[9][55]*S53p55;
  FB155_53 = s->m[55]*(s->dpt[3][65]*C55+S55*(q[54]+s->dpt[1][65]));
  FB355_53 = s->m[55]*(s->dpt[3][65]*S55-C55*(q[54]+s->dpt[1][65]));

// = = Block_0_2_0_1_0_24 = = 
 
// Backward Dynamics 

  FA156 = -(s->frc[1][56]-s->m[56]*(C56*(AlF154+BS154*s->dpt[1][66]+BeF254*s->dpt[2][66]+BeF354*s->dpt[3][66])-S56*(
 AlF354+BS954*s->dpt[3][66]+BeF754*s->dpt[1][66]+BeF854*s->dpt[2][66])));
  FA256 = -(s->frc[2][56]-s->m[56]*(AlF254+BS554*s->dpt[2][66]+BeF454*s->dpt[1][66]+BeF654*s->dpt[3][66]));
  FA356 = -(s->frc[3][56]-s->m[56]*(C56*(AlF354+BS954*s->dpt[3][66]+BeF754*s->dpt[1][66]+BeF854*s->dpt[2][66])+S56*(
 AlF154+BS154*s->dpt[1][66]+BeF254*s->dpt[2][66]+BeF354*s->dpt[3][66])));
  CF156 = -(s->trq[1][56]-s->In[1][56]*(C56*(OpF153-qd[56]*OM353)-S56*(OpF353+qd[56]*OM153))+OM256*OM356*(s->In[5][56]-
 s->In[9][56]));
  CF256 = -(s->trq[2][56]-s->In[5][56]*OpF252-OM156*OM356*(s->In[1][56]-s->In[9][56]));
  CF356 = -(s->trq[3][56]-s->In[9][56]*(C56*(OpF353+qd[56]*OM153)+S56*(OpF153-qd[56]*OM353))+OM156*OM256*(s->In[1][56]-
 s->In[5][56]));
  FB156_1 = s->m[56]*(AlM153_1*C56-AlM353_1*S56);
  FB256_1 = s->m[56]*AlM252_1;
  FB356_1 = s->m[56]*(AlM153_1*S56+AlM353_1*C56);
  FB156_2 = s->m[56]*(AlM153_2*C56-AlM353_2*S56);
  FB256_2 = s->m[56]*AlM252_2;
  FB356_2 = s->m[56]*(AlM153_2*S56+AlM353_2*C56);
  FB156_3 = s->m[56]*(AlM153_3*C56-AlM353_3*S56);
  FB256_3 = s->m[56]*AlM252_3;
  FB356_3 = s->m[56]*(AlM153_3*S56+AlM353_3*C56);
  FB156_4 = s->m[56]*(C56*(AlM153_4+OpM252_4*s->dpt[3][66]-OpM353_4*s->dpt[2][66])-S56*(AlM354_4+OpM153_4*s->dpt[2][66]-
 OpM252_4*s->dpt[1][66]));
  FB256_4 = s->m[56]*(AlM254_4-OpM153_4*s->dpt[3][66]+OpM353_4*s->dpt[1][66]);
  FB356_4 = s->m[56]*(C56*(AlM354_4+OpM153_4*s->dpt[2][66]-OpM252_4*s->dpt[1][66])+S56*(AlM153_4+OpM252_4*s->dpt[3][66]-
 OpM353_4*s->dpt[2][66]));
  CM156_4 = s->In[1][56]*(OpM153_4*C56-OpM353_4*S56);
  CM256_4 = s->In[5][56]*OpM252_4;
  CM356_4 = s->In[9][56]*(OpM153_4*S56+OpM353_4*C56);
  FB156_5 = s->m[56]*(C56*(AlM153_5+OpM252_5*s->dpt[3][66]-OpM353_5*s->dpt[2][66])-S56*(AlM354_5+OpM153_5*s->dpt[2][66]-
 OpM252_5*s->dpt[1][66]));
  FB256_5 = s->m[56]*(AlM254_5-OpM153_5*s->dpt[3][66]+OpM353_5*s->dpt[1][66]);
  FB356_5 = s->m[56]*(C56*(AlM354_5+OpM153_5*s->dpt[2][66]-OpM252_5*s->dpt[1][66])+S56*(AlM153_5+OpM252_5*s->dpt[3][66]-
 OpM353_5*s->dpt[2][66]));
  CM156_5 = s->In[1][56]*(OpM153_5*C56-OpM353_5*S56);
  CM256_5 = s->In[5][56]*OpM252_5;
  CM356_5 = s->In[9][56]*(OpM153_5*S56+OpM353_5*C56);
  FB156_6 = s->m[56]*(C56*(AlM153_6+OpM252_6*s->dpt[3][66]-OpM353_6*s->dpt[2][66])-S56*(AlM354_6+OpM153_6*s->dpt[2][66]-
 OpM252_6*s->dpt[1][66]));
  FB256_6 = s->m[56]*(AlM254_6-OpM153_6*s->dpt[3][66]+OpM353_6*s->dpt[1][66]);
  FB356_6 = s->m[56]*(C56*(AlM354_6+OpM153_6*s->dpt[2][66]-OpM252_6*s->dpt[1][66])+S56*(AlM153_6+OpM252_6*s->dpt[3][66]-
 OpM353_6*s->dpt[2][66]));
  CM156_6 = s->In[1][56]*(OpM153_6*C56-OpM353_6*S56);
  CM256_6 = s->In[5][56]*OpM252_6;
  CM356_6 = s->In[9][56]*(OpM153_6*S56+OpM353_6*C56);
  FB156_51 = -s->m[56]*(C56*(OpM353_51*s->dpt[2][66]-s->dpt[3][66]*S52)+S56*(AlM354_51+OpM153_51*s->dpt[2][66]-
 s->dpt[1][66]*S52));
  FB256_51 = s->m[56]*(AlM254_51-OpM153_51*s->dpt[3][66]+OpM353_51*s->dpt[1][66]);
  FB356_51 = s->m[56]*(C56*(AlM354_51+OpM153_51*s->dpt[2][66]-s->dpt[1][66]*S52)-S56*(OpM353_51*s->dpt[2][66]-
 s->dpt[3][66]*S52));
  CM156_51 = -s->In[1][56]*S53p56*C52;
  CM256_51 = s->In[5][56]*S52;
  CM356_51 = s->In[9][56]*C53p56*C52;
  FB156_52 = -s->m[56]*s->dpt[2][66]*S53p56;
  FB256_52 = s->m[56]*(AlM254_52+s->dpt[1][66]*S53-s->dpt[3][66]*C53);
  FB356_52 = s->m[56]*s->dpt[2][66]*C53p56;
  CM156_52 = s->In[1][56]*C53p56;
  CM356_52 = s->In[9][56]*S53p56;
  FB156_53 = s->m[56]*(s->dpt[3][66]*C56+S56*(q[54]+s->dpt[1][66]));
  FB356_53 = s->m[56]*(s->dpt[3][66]*S56-C56*(q[54]+s->dpt[1][66]));

// = = Block_0_2_0_1_0_25 = = 
 
// Backward Dynamics 

  FA157 = -(s->frc[1][57]-s->m[57]*(C57*(AlF154+BS154*s->dpt[1][67]+BeF254*s->dpt[2][67]+BeF354*s->dpt[3][67])-S57*(
 AlF354+BS954*s->dpt[3][67]+BeF754*s->dpt[1][67]+BeF854*s->dpt[2][67])));
  FA257 = -(s->frc[2][57]-s->m[57]*(AlF254+BS554*s->dpt[2][67]+BeF454*s->dpt[1][67]+BeF654*s->dpt[3][67]));
  FA357 = -(s->frc[3][57]-s->m[57]*(C57*(AlF354+BS954*s->dpt[3][67]+BeF754*s->dpt[1][67]+BeF854*s->dpt[2][67])+S57*(
 AlF154+BS154*s->dpt[1][67]+BeF254*s->dpt[2][67]+BeF354*s->dpt[3][67])));
  CF157 = -(s->trq[1][57]-s->In[1][57]*(C57*(OpF153-qd[57]*OM353)-S57*(OpF353+qd[57]*OM153))+OM257*OM357*(s->In[5][57]-
 s->In[9][57]));
  CF257 = -(s->trq[2][57]-s->In[5][57]*OpF252-OM157*OM357*(s->In[1][57]-s->In[9][57]));
  CF357 = -(s->trq[3][57]-s->In[9][57]*(C57*(OpF353+qd[57]*OM153)+S57*(OpF153-qd[57]*OM353))+OM157*OM257*(s->In[1][57]-
 s->In[5][57]));
  FB157_1 = s->m[57]*(AlM153_1*C57-AlM353_1*S57);
  FB257_1 = s->m[57]*AlM252_1;
  FB357_1 = s->m[57]*(AlM153_1*S57+AlM353_1*C57);
  FB157_2 = s->m[57]*(AlM153_2*C57-AlM353_2*S57);
  FB257_2 = s->m[57]*AlM252_2;
  FB357_2 = s->m[57]*(AlM153_2*S57+AlM353_2*C57);
  FB157_3 = s->m[57]*(AlM153_3*C57-AlM353_3*S57);
  FB257_3 = s->m[57]*AlM252_3;
  FB357_3 = s->m[57]*(AlM153_3*S57+AlM353_3*C57);
  FB157_4 = s->m[57]*(C57*(AlM153_4+OpM252_4*s->dpt[3][67]-OpM353_4*s->dpt[2][67])-S57*(AlM354_4+OpM153_4*s->dpt[2][67]-
 OpM252_4*s->dpt[1][67]));
  FB257_4 = s->m[57]*(AlM254_4-OpM153_4*s->dpt[3][67]+OpM353_4*s->dpt[1][67]);
  FB357_4 = s->m[57]*(C57*(AlM354_4+OpM153_4*s->dpt[2][67]-OpM252_4*s->dpt[1][67])+S57*(AlM153_4+OpM252_4*s->dpt[3][67]-
 OpM353_4*s->dpt[2][67]));
  CM157_4 = s->In[1][57]*(OpM153_4*C57-OpM353_4*S57);
  CM257_4 = s->In[5][57]*OpM252_4;
  CM357_4 = s->In[9][57]*(OpM153_4*S57+OpM353_4*C57);
  FB157_5 = s->m[57]*(C57*(AlM153_5+OpM252_5*s->dpt[3][67]-OpM353_5*s->dpt[2][67])-S57*(AlM354_5+OpM153_5*s->dpt[2][67]-
 OpM252_5*s->dpt[1][67]));
  FB257_5 = s->m[57]*(AlM254_5-OpM153_5*s->dpt[3][67]+OpM353_5*s->dpt[1][67]);
  FB357_5 = s->m[57]*(C57*(AlM354_5+OpM153_5*s->dpt[2][67]-OpM252_5*s->dpt[1][67])+S57*(AlM153_5+OpM252_5*s->dpt[3][67]-
 OpM353_5*s->dpt[2][67]));
  CM157_5 = s->In[1][57]*(OpM153_5*C57-OpM353_5*S57);
  CM257_5 = s->In[5][57]*OpM252_5;
  CM357_5 = s->In[9][57]*(OpM153_5*S57+OpM353_5*C57);
  FB157_6 = s->m[57]*(C57*(AlM153_6+OpM252_6*s->dpt[3][67]-OpM353_6*s->dpt[2][67])-S57*(AlM354_6+OpM153_6*s->dpt[2][67]-
 OpM252_6*s->dpt[1][67]));
  FB257_6 = s->m[57]*(AlM254_6-OpM153_6*s->dpt[3][67]+OpM353_6*s->dpt[1][67]);
  FB357_6 = s->m[57]*(C57*(AlM354_6+OpM153_6*s->dpt[2][67]-OpM252_6*s->dpt[1][67])+S57*(AlM153_6+OpM252_6*s->dpt[3][67]-
 OpM353_6*s->dpt[2][67]));
  CM157_6 = s->In[1][57]*(OpM153_6*C57-OpM353_6*S57);
  CM257_6 = s->In[5][57]*OpM252_6;
  CM357_6 = s->In[9][57]*(OpM153_6*S57+OpM353_6*C57);
  FB157_51 = -s->m[57]*(C57*(OpM353_51*s->dpt[2][67]-s->dpt[3][67]*S52)+S57*(AlM354_51+OpM153_51*s->dpt[2][67]-
 s->dpt[1][67]*S52));
  FB257_51 = s->m[57]*(AlM254_51-OpM153_51*s->dpt[3][67]+OpM353_51*s->dpt[1][67]);
  FB357_51 = s->m[57]*(C57*(AlM354_51+OpM153_51*s->dpt[2][67]-s->dpt[1][67]*S52)-S57*(OpM353_51*s->dpt[2][67]-
 s->dpt[3][67]*S52));
  CM157_51 = -s->In[1][57]*S53p57*C52;
  CM257_51 = s->In[5][57]*S52;
  CM357_51 = s->In[9][57]*C53p57*C52;
  FB157_52 = -s->m[57]*s->dpt[2][67]*S53p57;
  FB257_52 = s->m[57]*(AlM254_52+s->dpt[1][67]*S53-s->dpt[3][67]*C53);
  FB357_52 = s->m[57]*s->dpt[2][67]*C53p57;
  CM157_52 = s->In[1][57]*C53p57;
  CM357_52 = s->In[9][57]*S53p57;
  FB157_53 = s->m[57]*(s->dpt[3][67]*C57+S57*(q[54]+s->dpt[1][67]));
  FB357_53 = s->m[57]*(s->dpt[3][67]*S57-C57*(q[54]+s->dpt[1][67]));

// = = Block_0_2_0_1_0_26 = = 
 
// Backward Dynamics 

  FA158 = -(s->frc[1][58]-s->m[58]*(C58*(AlF154+BS154*s->dpt[1][68]+BeF254*s->dpt[2][68]+BeF354*s->dpt[3][68])-S58*(
 AlF354+BS954*s->dpt[3][68]+BeF754*s->dpt[1][68]+BeF854*s->dpt[2][68])));
  FA258 = -(s->frc[2][58]-s->m[58]*(AlF254+BS554*s->dpt[2][68]+BeF454*s->dpt[1][68]+BeF654*s->dpt[3][68]));
  FA358 = -(s->frc[3][58]-s->m[58]*(C58*(AlF354+BS954*s->dpt[3][68]+BeF754*s->dpt[1][68]+BeF854*s->dpt[2][68])+S58*(
 AlF154+BS154*s->dpt[1][68]+BeF254*s->dpt[2][68]+BeF354*s->dpt[3][68])));
  CF158 = -(s->trq[1][58]-s->In[1][58]*(C58*(OpF153-qd[58]*OM353)-S58*(OpF353+qd[58]*OM153))+OM258*OM358*(s->In[5][58]-
 s->In[9][58]));
  CF258 = -(s->trq[2][58]-s->In[5][58]*OpF252-OM158*OM358*(s->In[1][58]-s->In[9][58]));
  CF358 = -(s->trq[3][58]-s->In[9][58]*(C58*(OpF353+qd[58]*OM153)+S58*(OpF153-qd[58]*OM353))+OM158*OM258*(s->In[1][58]-
 s->In[5][58]));
  FB158_1 = s->m[58]*(AlM153_1*C58-AlM353_1*S58);
  FB258_1 = s->m[58]*AlM252_1;
  FB358_1 = s->m[58]*(AlM153_1*S58+AlM353_1*C58);
  FB158_2 = s->m[58]*(AlM153_2*C58-AlM353_2*S58);
  FB258_2 = s->m[58]*AlM252_2;
  FB358_2 = s->m[58]*(AlM153_2*S58+AlM353_2*C58);
  FB158_3 = s->m[58]*(AlM153_3*C58-AlM353_3*S58);
  FB258_3 = s->m[58]*AlM252_3;
  FB358_3 = s->m[58]*(AlM153_3*S58+AlM353_3*C58);
  FB158_4 = s->m[58]*(C58*(AlM153_4+OpM252_4*s->dpt[3][68]-OpM353_4*s->dpt[2][68])-S58*(AlM354_4+OpM153_4*s->dpt[2][68]-
 OpM252_4*s->dpt[1][68]));
  FB258_4 = s->m[58]*(AlM254_4-OpM153_4*s->dpt[3][68]+OpM353_4*s->dpt[1][68]);
  FB358_4 = s->m[58]*(C58*(AlM354_4+OpM153_4*s->dpt[2][68]-OpM252_4*s->dpt[1][68])+S58*(AlM153_4+OpM252_4*s->dpt[3][68]-
 OpM353_4*s->dpt[2][68]));
  CM158_4 = s->In[1][58]*(OpM153_4*C58-OpM353_4*S58);
  CM258_4 = s->In[5][58]*OpM252_4;
  CM358_4 = s->In[9][58]*(OpM153_4*S58+OpM353_4*C58);
  FB158_5 = s->m[58]*(C58*(AlM153_5+OpM252_5*s->dpt[3][68]-OpM353_5*s->dpt[2][68])-S58*(AlM354_5+OpM153_5*s->dpt[2][68]-
 OpM252_5*s->dpt[1][68]));
  FB258_5 = s->m[58]*(AlM254_5-OpM153_5*s->dpt[3][68]+OpM353_5*s->dpt[1][68]);
  FB358_5 = s->m[58]*(C58*(AlM354_5+OpM153_5*s->dpt[2][68]-OpM252_5*s->dpt[1][68])+S58*(AlM153_5+OpM252_5*s->dpt[3][68]-
 OpM353_5*s->dpt[2][68]));
  CM158_5 = s->In[1][58]*(OpM153_5*C58-OpM353_5*S58);
  CM258_5 = s->In[5][58]*OpM252_5;
  CM358_5 = s->In[9][58]*(OpM153_5*S58+OpM353_5*C58);
  FB158_6 = s->m[58]*(C58*(AlM153_6+OpM252_6*s->dpt[3][68]-OpM353_6*s->dpt[2][68])-S58*(AlM354_6+OpM153_6*s->dpt[2][68]-
 OpM252_6*s->dpt[1][68]));
  FB258_6 = s->m[58]*(AlM254_6-OpM153_6*s->dpt[3][68]+OpM353_6*s->dpt[1][68]);
  FB358_6 = s->m[58]*(C58*(AlM354_6+OpM153_6*s->dpt[2][68]-OpM252_6*s->dpt[1][68])+S58*(AlM153_6+OpM252_6*s->dpt[3][68]-
 OpM353_6*s->dpt[2][68]));
  CM158_6 = s->In[1][58]*(OpM153_6*C58-OpM353_6*S58);
  CM258_6 = s->In[5][58]*OpM252_6;
  CM358_6 = s->In[9][58]*(OpM153_6*S58+OpM353_6*C58);
  FB158_51 = -s->m[58]*(C58*(OpM353_51*s->dpt[2][68]-s->dpt[3][68]*S52)+S58*(AlM354_51+OpM153_51*s->dpt[2][68]-
 s->dpt[1][68]*S52));
  FB258_51 = s->m[58]*(AlM254_51-OpM153_51*s->dpt[3][68]+OpM353_51*s->dpt[1][68]);
  FB358_51 = s->m[58]*(C58*(AlM354_51+OpM153_51*s->dpt[2][68]-s->dpt[1][68]*S52)-S58*(OpM353_51*s->dpt[2][68]-
 s->dpt[3][68]*S52));
  CM158_51 = -s->In[1][58]*S53p58*C52;
  CM258_51 = s->In[5][58]*S52;
  CM358_51 = s->In[9][58]*C53p58*C52;
  FB158_52 = -s->m[58]*s->dpt[2][68]*S53p58;
  FB258_52 = s->m[58]*(AlM254_52+s->dpt[1][68]*S53-s->dpt[3][68]*C53);
  FB358_52 = s->m[58]*s->dpt[2][68]*C53p58;
  CM158_52 = s->In[1][58]*C53p58;
  CM358_52 = s->In[9][58]*S53p58;
  FB158_53 = s->m[58]*(s->dpt[3][68]*C58+S58*(q[54]+s->dpt[1][68]));
  FB358_53 = s->m[58]*(s->dpt[3][68]*S58-C58*(q[54]+s->dpt[1][68]));

// = = Block_0_2_0_1_0_27 = = 
 
// Backward Dynamics 

  FA159 = -(s->frc[1][59]-s->m[59]*(AlF154+(2.0)*qd[59]*OM253+BS154*s->dpt[1][69]+BeF354*Dz593));
  FA259 = -(s->frc[2][59]-s->m[59]*(AlF254-(2.0)*qd[59]*OM153+BeF454*s->dpt[1][69]+BeF654*Dz593));
  FA359 = -(s->frc[3][59]-s->m[59]*(AlF354+BS954*Dz593+BeF754*s->dpt[1][69]));
  FB159_1 = s->m[59]*AlM153_1;
  FB259_1 = s->m[59]*AlM252_1;
  FB359_1 = s->m[59]*AlM353_1;
  FB159_2 = s->m[59]*AlM153_2;
  FB259_2 = s->m[59]*AlM252_2;
  FB359_2 = s->m[59]*AlM353_2;
  FB159_3 = s->m[59]*AlM153_3;
  FB259_3 = s->m[59]*AlM252_3;
  FB359_3 = s->m[59]*AlM353_3;
  FB159_4 = s->m[59]*(AlM153_4+Dz593*OpM252_4);
  FB259_4 = s->m[59]*(AlM254_4-Dz593*OpM153_4+OpM353_4*s->dpt[1][69]);
  FB359_4 = s->m[59]*(AlM354_4-OpM252_4*s->dpt[1][69]);
  FB159_5 = s->m[59]*(AlM153_5+Dz593*OpM252_5);
  FB259_5 = s->m[59]*(AlM254_5-Dz593*OpM153_5+OpM353_5*s->dpt[1][69]);
  FB359_5 = s->m[59]*(AlM354_5-OpM252_5*s->dpt[1][69]);
  FB159_6 = s->m[59]*(AlM153_6+Dz593*OpM252_6);
  FB259_6 = s->m[59]*(AlM254_6-Dz593*OpM153_6+OpM353_6*s->dpt[1][69]);
  FB359_6 = s->m[59]*(AlM354_6-OpM252_6*s->dpt[1][69]);
  FB159_51 = s->m[59]*Dz593*S52;
  FB259_51 = s->m[59]*(AlM254_51-Dz593*OpM153_51+OpM353_51*s->dpt[1][69]);
  FB359_51 = s->m[59]*(AlM354_51-s->dpt[1][69]*S52);
  FB259_52 = s->m[59]*(AlM254_52-Dz593*C53+s->dpt[1][69]*S53);
  FB159_53 = s->m[59]*Dz593;
  FB359_53 = -s->m[59]*(q[54]+s->dpt[1][69]);

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

  FA112 = -(s->frc[1][12]-s->m[12]*(C12*(AlF111+BeF311*s->dpt[3][23])-S12*(AlF310+BS911*s->dpt[3][23])));
  FA212 = -(s->frc[2][12]-s->m[12]*(AlF211-s->dpt[3][23]*(OpF111-OM211*OM311)));
  FA312 = -(s->frc[3][12]-s->m[12]*(C12*(AlF310+BS911*s->dpt[3][23])+S12*(AlF111+BeF311*s->dpt[3][23])));
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
  FB112_4 = -s->m[12]*(AlM310_4*S12-C12*(AlM111_4+OpM211_4*s->dpt[3][23]));
  FB212_4 = s->m[12]*(AlM211_4-OpM111_4*s->dpt[3][23]);
  FB312_4 = s->m[12]*(AlM310_4*C12+S12*(AlM111_4+OpM211_4*s->dpt[3][23]));
  CM112_4 = s->In[1][12]*(OpM111_4*C12-OpM310_4*S12);
  CM212_4 = s->In[5][12]*OpM211_4;
  CM312_4 = s->In[9][12]*(OpM111_4*S12+OpM310_4*C12);
  FB112_5 = -s->m[12]*(AlM310_5*S12-C12*(AlM111_5+OpM211_5*s->dpt[3][23]));
  FB212_5 = s->m[12]*(AlM211_5-OpM111_5*s->dpt[3][23]);
  FB312_5 = s->m[12]*(AlM310_5*C12+S12*(AlM111_5+OpM211_5*s->dpt[3][23]));
  CM112_5 = s->In[1][12]*(OpM111_5*C12-OpM310_5*S12);
  CM212_5 = s->In[5][12]*OpM211_5;
  CM312_5 = s->In[9][12]*(OpM111_5*S12+OpM310_5*C12);
  FB112_6 = -s->m[12]*(AlM310_6*S12-C12*(AlM111_6+OpM211_6*s->dpt[3][23]));
  FB212_6 = s->m[12]*(AlM211_6-OpM111_6*s->dpt[3][23]);
  FB312_6 = s->m[12]*(AlM310_6*C12+S12*(AlM111_6+OpM211_6*s->dpt[3][23]));
  CM112_6 = s->In[1][12]*(OpM111_6*C12-OpM310_6*S12);
  CM212_6 = s->In[5][12]*OpM211_6;
  CM312_6 = s->In[9][12]*(OpM111_6*S12+OpM310_6*C12);
  FB112_8 = -s->m[12]*(AlM310_8*S12-C12*(AlM111_8+OpM211_8*s->dpt[3][23]));
  FB212_8 = s->m[12]*(AlM211_8-OpM111_8*s->dpt[3][23]);
  FB312_8 = s->m[12]*(AlM310_8*C12+S12*(AlM111_8+OpM211_8*s->dpt[3][23]));
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
  CF111 = -(s->trq[1][11]-CF112*C12-CF312*S12+FA212*s->dpt[3][23]);
  CF211 = -(s->trq[2][11]-CF212-s->dpt[3][23]*(FA112*C12+FA312*S12));
  CF311 = -(s->trq[3][11]+CF112*S12-CF312*C12);
  FM111_1 = s->m[11]*AlM111_1+FB112_1*C12+FB312_1*S12;
  FM211_1 = FB212_1+s->m[11]*AlM211_1;
  FM311_1 = s->m[11]*AlM310_1-FB112_1*S12+FB312_1*C12;
  CM111_112 = -FB212_1*s->dpt[3][23];
  CM111_212 = s->dpt[3][23]*(FB112_1*C12+FB312_1*S12);
  FM111_2 = s->m[11]*AlM111_2+FB112_2*C12+FB312_2*S12;
  FM211_2 = FB212_2+s->m[11]*AlM211_2;
  FM311_2 = s->m[11]*AlM310_2-FB112_2*S12+FB312_2*C12;
  CM112_112 = -FB212_2*s->dpt[3][23];
  CM112_212 = s->dpt[3][23]*(FB112_2*C12+FB312_2*S12);
  FM111_3 = s->m[11]*AlM111_3+FB112_3*C12+FB312_3*S12;
  FM211_3 = FB212_3+s->m[11]*AlM211_3;
  FM311_3 = s->m[11]*AlM310_3-FB112_3*S12+FB312_3*C12;
  CM113_112 = -FB212_3*s->dpt[3][23];
  CM113_212 = s->dpt[3][23]*(FB112_3*C12+FB312_3*S12);
  FM111_4 = s->m[11]*AlM111_4+FB112_4*C12+FB312_4*S12;
  FM211_4 = FB212_4+s->m[11]*AlM211_4;
  FM311_4 = s->m[11]*AlM310_4-FB112_4*S12+FB312_4*C12;
  CM114_112 = CM112_4*C12+CM312_4*S12-FB212_4*s->dpt[3][23];
  CM114_212 = CM212_4+s->dpt[3][23]*(FB112_4*C12+FB312_4*S12);
  CM114_312 = -(CM112_4*S12-CM312_4*C12);
  FM111_5 = s->m[11]*AlM111_5+FB112_5*C12+FB312_5*S12;
  FM211_5 = FB212_5+s->m[11]*AlM211_5;
  FM311_5 = s->m[11]*AlM310_5-FB112_5*S12+FB312_5*C12;
  CM115_112 = CM112_5*C12+CM312_5*S12-FB212_5*s->dpt[3][23];
  CM115_212 = CM212_5+s->dpt[3][23]*(FB112_5*C12+FB312_5*S12);
  CM115_312 = -(CM112_5*S12-CM312_5*C12);
  FM111_6 = s->m[11]*AlM111_6+FB112_6*C12+FB312_6*S12;
  FM211_6 = FB212_6+s->m[11]*AlM211_6;
  FM311_6 = s->m[11]*AlM310_6-FB112_6*S12+FB312_6*C12;
  CM116_112 = CM112_6*C12+CM312_6*S12-FB212_6*s->dpt[3][23];
  CM116_212 = CM212_6+s->dpt[3][23]*(FB112_6*C12+FB312_6*S12);
  CM116_312 = -(CM112_6*S12-CM312_6*C12);
  FM111_8 = s->m[11]*AlM111_8+FB112_8*C12+FB312_8*S12;
  FM211_8 = FB212_8+s->m[11]*AlM211_8;
  CM118_112 = CM112_8*C12+CM312_8*S12-FB212_8*s->dpt[3][23];
  CM118_212 = CM212_8+s->dpt[3][23]*(FB112_8*C12+FB312_8*S12);
  CM118_312 = -(CM112_8*S12-CM312_8*C12);
  CM119_112 = s->m[12]*OpM111_9*s->dpt[3][23]*s->dpt[3][23]+CM112_9*C12+CM312_9*S12;
  CM119_212 = CM212_9+s->m[12]*OpM211_9*s->dpt[3][23]*s->dpt[3][23];
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
  CM1010_111 = C11*(s->m[12]*s->dpt[3][23]*s->dpt[3][23]*C11+CM112_10*C12+CM312_10*S12)+S11*S11*(s->In[5][12]+s->m[12]*
 s->dpt[3][23]*s->dpt[3][23]);
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
  CF18 = -(s->trq[1][8]-CF10_111*C9-CF9_310*S9+s->dpt[2][19]*(FF10_111*S9-FF9_310*C9));
  CF28 = -(s->trq[2][8]-CF9_210);
  CF38 = -(s->trq[3][8]+CF10_111*S9-CF9_310*C9+s->dpt[2][19]*(FF10_111*C9+FF9_310*S9));
  FM81_19 = FM101_111*C9+FM91_310*S9;
  FM81_39 = -(FM101_111*S9-FM91_310*C9);
  CM81_19 = CM101_111*C9+CM91_310*S9-s->dpt[2][19]*(FM101_111*S9-FM91_310*C9);
  CM81_39 = -(CM101_111*S9-CM91_310*C9+s->dpt[2][19]*(FM101_111*C9+FM91_310*S9));
  FM82_19 = FM102_111*C9+FM92_310*S9;
  FM82_39 = -(FM102_111*S9-FM92_310*C9);
  CM82_19 = CM102_111*C9+CM92_310*S9-s->dpt[2][19]*(FM102_111*S9-FM92_310*C9);
  CM82_39 = -(CM102_111*S9-CM92_310*C9+s->dpt[2][19]*(FM102_111*C9+FM92_310*S9));
  FM83_19 = FM103_111*C9+FM93_310*S9;
  FM83_39 = -(FM103_111*S9-FM93_310*C9);
  CM83_19 = CM103_111*C9+CM93_310*S9-s->dpt[2][19]*(FM103_111*S9-FM93_310*C9);
  CM83_39 = -(CM103_111*S9-CM93_310*C9+s->dpt[2][19]*(FM103_111*C9+FM93_310*S9));
  FM84_19 = FM104_111*C9+FM94_310*S9;
  FM84_39 = -(FM104_111*S9-FM94_310*C9);
  CM84_19 = CM104_111*C9+CM94_310*S9-s->dpt[2][19]*(FM104_111*S9-FM94_310*C9);
  CM84_39 = -(CM104_111*S9-CM94_310*C9+s->dpt[2][19]*(FM104_111*C9+FM94_310*S9));
  FM85_19 = FM105_111*C9+FM95_310*S9;
  FM85_39 = -(FM105_111*S9-FM95_310*C9);
  CM85_19 = CM105_111*C9+CM95_310*S9-s->dpt[2][19]*(FM105_111*S9-FM95_310*C9);
  CM85_39 = -(CM105_111*S9-CM95_310*C9+s->dpt[2][19]*(FM105_111*C9+FM95_310*S9));
  CM86_19 = CM106_111*C9+CM96_310*S9-s->dpt[2][19]*(FM106_111*S9-FM96_310*C9);
  CM88_19 = CM108_111*C9+s->dpt[2][19]*(C9*(C10*(s->m[11]*AlM310_8-FB112_8*S12+FB312_8*C12)+S10*(FM111_8*S11+FM211_8*C11
 ))-S9*(FM111_8*C11-FM211_8*S11))+S9*(CM108_211*S10+CM118_312*C10);

// = = Block_0_2_0_2_0_4 = = 
 
// Backward Dynamics 

  FA117 = -(s->frc[1][17]-s->m[17]*(C17*(AlF116+BeF316*s->dpt[3][29])-S17*(AlF315+BS916*s->dpt[3][29])));
  FA217 = -(s->frc[2][17]-s->m[17]*(AlF216-s->dpt[3][29]*(OpF116-OM216*OM316)));
  FA317 = -(s->frc[3][17]-s->m[17]*(C17*(AlF315+BS916*s->dpt[3][29])+S17*(AlF116+BeF316*s->dpt[3][29])));
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
  FB117_4 = -s->m[17]*(AlM315_4*S17-C17*(AlM116_4+OpM216_4*s->dpt[3][29]));
  FB217_4 = s->m[17]*(AlM216_4-OpM116_4*s->dpt[3][29]);
  FB317_4 = s->m[17]*(AlM315_4*C17+S17*(AlM116_4+OpM216_4*s->dpt[3][29]));
  CM117_4 = s->In[1][17]*(OpM116_4*C17-OpM315_4*S17);
  CM217_4 = s->In[5][17]*OpM216_4;
  CM317_4 = s->In[9][17]*(OpM116_4*S17+OpM315_4*C17);
  FB117_5 = -s->m[17]*(AlM315_5*S17-C17*(AlM116_5+OpM216_5*s->dpt[3][29]));
  FB217_5 = s->m[17]*(AlM216_5-OpM116_5*s->dpt[3][29]);
  FB317_5 = s->m[17]*(AlM315_5*C17+S17*(AlM116_5+OpM216_5*s->dpt[3][29]));
  CM117_5 = s->In[1][17]*(OpM116_5*C17-OpM315_5*S17);
  CM217_5 = s->In[5][17]*OpM216_5;
  CM317_5 = s->In[9][17]*(OpM116_5*S17+OpM315_5*C17);
  FB117_6 = -s->m[17]*(AlM315_6*S17-C17*(AlM116_6+OpM216_6*s->dpt[3][29]));
  FB217_6 = s->m[17]*(AlM216_6-OpM116_6*s->dpt[3][29]);
  FB317_6 = s->m[17]*(AlM315_6*C17+S17*(AlM116_6+OpM216_6*s->dpt[3][29]));
  CM117_6 = s->In[1][17]*(OpM116_6*C17-OpM315_6*S17);
  CM217_6 = s->In[5][17]*OpM216_6;
  CM317_6 = s->In[9][17]*(OpM116_6*S17+OpM315_6*C17);
  FB117_13 = -s->m[17]*(AlM315_13*S17-C17*(AlM116_13+OpM216_13*s->dpt[3][29]));
  FB217_13 = s->m[17]*(AlM216_13-OpM116_13*s->dpt[3][29]);
  FB317_13 = s->m[17]*(AlM315_13*C17+S17*(AlM116_13+OpM216_13*s->dpt[3][29]));
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
  CF116 = -(s->trq[1][16]-CF117*C17-CF317*S17+FA217*s->dpt[3][29]);
  CF216 = -(s->trq[2][16]-CF217-s->dpt[3][29]*(FA117*C17+FA317*S17));
  CF316 = -(s->trq[3][16]+CF117*S17-CF317*C17);
  FM116_1 = s->m[16]*AlM116_1+FB117_1*C17+FB317_1*S17;
  FM216_1 = FB217_1+s->m[16]*AlM216_1;
  FM316_1 = s->m[16]*AlM315_1-FB117_1*S17+FB317_1*C17;
  CM161_117 = -FB217_1*s->dpt[3][29];
  CM161_217 = s->dpt[3][29]*(FB117_1*C17+FB317_1*S17);
  FM116_2 = s->m[16]*AlM116_2+FB117_2*C17+FB317_2*S17;
  FM216_2 = FB217_2+s->m[16]*AlM216_2;
  FM316_2 = s->m[16]*AlM315_2-FB117_2*S17+FB317_2*C17;
  CM162_117 = -FB217_2*s->dpt[3][29];
  CM162_217 = s->dpt[3][29]*(FB117_2*C17+FB317_2*S17);
  FM116_3 = s->m[16]*AlM116_3+FB117_3*C17+FB317_3*S17;
  FM216_3 = FB217_3+s->m[16]*AlM216_3;
  FM316_3 = s->m[16]*AlM315_3-FB117_3*S17+FB317_3*C17;
  CM163_117 = -FB217_3*s->dpt[3][29];
  CM163_217 = s->dpt[3][29]*(FB117_3*C17+FB317_3*S17);
  FM116_4 = s->m[16]*AlM116_4+FB117_4*C17+FB317_4*S17;
  FM216_4 = FB217_4+s->m[16]*AlM216_4;
  FM316_4 = s->m[16]*AlM315_4-FB117_4*S17+FB317_4*C17;
  CM164_117 = CM117_4*C17+CM317_4*S17-FB217_4*s->dpt[3][29];
  CM164_217 = CM217_4+s->dpt[3][29]*(FB117_4*C17+FB317_4*S17);
  CM164_317 = -(CM117_4*S17-CM317_4*C17);
  FM116_5 = s->m[16]*AlM116_5+FB117_5*C17+FB317_5*S17;
  FM216_5 = FB217_5+s->m[16]*AlM216_5;
  FM316_5 = s->m[16]*AlM315_5-FB117_5*S17+FB317_5*C17;
  CM165_117 = CM117_5*C17+CM317_5*S17-FB217_5*s->dpt[3][29];
  CM165_217 = CM217_5+s->dpt[3][29]*(FB117_5*C17+FB317_5*S17);
  CM165_317 = -(CM117_5*S17-CM317_5*C17);
  FM116_6 = s->m[16]*AlM116_6+FB117_6*C17+FB317_6*S17;
  FM216_6 = FB217_6+s->m[16]*AlM216_6;
  FM316_6 = s->m[16]*AlM315_6-FB117_6*S17+FB317_6*C17;
  CM166_117 = CM117_6*C17+CM317_6*S17-FB217_6*s->dpt[3][29];
  CM166_217 = CM217_6+s->dpt[3][29]*(FB117_6*C17+FB317_6*S17);
  CM166_317 = -(CM117_6*S17-CM317_6*C17);
  FM116_13 = s->m[16]*AlM116_13+FB117_13*C17+FB317_13*S17;
  FM216_13 = FB217_13+s->m[16]*AlM216_13;
  CM1613_117 = CM117_13*C17+CM317_13*S17-FB217_13*s->dpt[3][29];
  CM1613_217 = CM217_13+s->dpt[3][29]*(FB117_13*C17+FB317_13*S17);
  CM1613_317 = -(CM117_13*S17-CM317_13*C17);
  CM1614_117 = s->m[17]*OpM116_14*s->dpt[3][29]*s->dpt[3][29]+CM117_14*C17+CM317_14*S17;
  CM1614_217 = CM217_14+s->m[17]*OpM216_14*s->dpt[3][29]*s->dpt[3][29];
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
  CM1515_116 = C16*(s->m[17]*s->dpt[3][29]*s->dpt[3][29]*C16+CM117_15*C17+CM317_15*S17)+S16*S16*(s->In[5][17]+s->m[17]*
 s->dpt[3][29]*s->dpt[3][29]);
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
  CF113 = -(s->trq[1][13]-CF14_315*S14-CF15_116*C14-s->dpt[2][26]*(FF14_315*C14-FF15_116*S14));
  CF213 = -(s->trq[2][13]-CF14_215);
  CF313 = -(s->trq[3][13]-CF14_315*C14+CF15_116*S14+s->dpt[2][26]*(FF14_315*S14+FF15_116*C14));
  FM131_114 = FM141_315*S14+FM151_116*C14;
  FM131_314 = FM141_315*C14-FM151_116*S14;
  CM131_114 = CM141_315*S14+CM151_116*C14+s->dpt[2][26]*(FM141_315*C14-FM151_116*S14);
  CM131_314 = CM141_315*C14-CM151_116*S14-s->dpt[2][26]*(FM141_315*S14+FM151_116*C14);
  FM132_114 = FM142_315*S14+FM152_116*C14;
  FM132_314 = FM142_315*C14-FM152_116*S14;
  CM132_114 = CM142_315*S14+CM152_116*C14+s->dpt[2][26]*(FM142_315*C14-FM152_116*S14);
  CM132_314 = CM142_315*C14-CM152_116*S14-s->dpt[2][26]*(FM142_315*S14+FM152_116*C14);
  FM133_114 = FM143_315*S14+FM153_116*C14;
  FM133_314 = FM143_315*C14-FM153_116*S14;
  CM133_114 = CM143_315*S14+CM153_116*C14+s->dpt[2][26]*(FM143_315*C14-FM153_116*S14);
  CM133_314 = CM143_315*C14-CM153_116*S14-s->dpt[2][26]*(FM143_315*S14+FM153_116*C14);
  FM134_114 = FM144_315*S14+FM154_116*C14;
  FM134_314 = FM144_315*C14-FM154_116*S14;
  CM134_114 = CM144_315*S14+CM154_116*C14+s->dpt[2][26]*(FM144_315*C14-FM154_116*S14);
  CM134_314 = CM144_315*C14-CM154_116*S14-s->dpt[2][26]*(FM144_315*S14+FM154_116*C14);
  FM135_114 = FM145_315*S14+FM155_116*C14;
  FM135_314 = FM145_315*C14-FM155_116*S14;
  CM135_114 = CM145_315*S14+CM155_116*C14+s->dpt[2][26]*(FM145_315*C14-FM155_116*S14);
  CM135_314 = CM145_315*C14-CM155_116*S14-s->dpt[2][26]*(FM145_315*S14+FM155_116*C14);
  CM136_114 = CM146_315*S14+CM156_116*C14+s->dpt[2][26]*(FM146_315*C14-FM156_116*S14);
  CM1313_114 = CM1513_116*C14+s->dpt[2][26]*(C14*(C15*(s->m[16]*AlM315_13-FB117_13*S17+FB317_13*C17)+S15*(FM116_13*S16+
 FM216_13*C16))-S14*(FM116_13*C16-FM216_13*S16))+S14*(CM1513_216*S15+CM1613_317*C15);

// = = Block_0_2_0_2_0_9 = = 
 
// Backward Dynamics 

  FF122 = -(s->frc[1][22]-FF23_124-FF125*C25-FF325*S25);
  FF222 = -(s->frc[2][22]+s->frc[2][25]+s->frc[2][24]*C23+s->frc[2][27]*C26+FF23_324*S23+FF26_327*S26);
  FF322 = -(s->frc[3][22]+s->frc[2][24]*S23+FF125*S25-FF23_324*C23-FF325*C25);
  CF122 = -(s->trq[1][22]-CF23_124-CF125*C25-CF325*S25+s->dpt[2][40]*(s->frc[2][24]*S23-FF23_324*C23));
  CF222 = -(s->trq[2][22]-CF225+s->trq[2][24]*C23+CF23_324*S23-s->dpt[1][40]*(s->frc[2][24]*S23-FF23_324*C23));
  CF322 = -(s->trq[3][22]+s->trq[2][24]*S23+CF125*S25-CF23_324*C23-CF325*C25+FF23_124*s->dpt[2][40]+s->dpt[1][40]*(
 s->frc[2][24]*C23+FF23_324*S23));

// = = Block_0_2_0_2_0_12 = = 
 
// Backward Dynamics 

  FF128 = -(s->frc[1][28]-FF29_130-FF31_132-FF33_134-s->m[28]*AlF128);
  FF228 = -(s->frc[2][28]-FF33_234-s->frc[3][30]*S29-s->frc[3][32]*S31-s->m[28]*AlF228-FF29_230*C29-FF31_232*C31);
  FF328 = -(s->frc[3][28]-FA334+s->frc[3][30]*C29+s->frc[3][32]*C31-s->m[28]*AlF328-FF29_230*S29-FF31_232*S31);
  FM128_1 = FM331_134+s->m[28]*AlM16_1;
  FM228_1 = FM331_234+s->m[28]*AlM25_1;
  FM328_1 = FB334_1+s->m[28]*AlM36_1;
  FM128_2 = FM332_134+s->m[28]*AlM16_2;
  FM228_2 = FM332_234+s->m[28]*AlM25_2;
  FM328_2 = FB334_2+s->m[28]*AlM36_2;
  FM128_3 = FM333_134+s->m[28]*AlM16_3;
  FM228_3 = FM333_234+s->m[28]*S5;
  FM328_3 = FB334_3+s->m[28]*AlM36_3;
  FM128_4 = FM334_134+s->m[28]*AlM128_4;
  FM228_4 = FM334_234+s->m[28]*AlM228_4;
  FM328_4 = FB334_4+s->m[28]*AlM328_4;
  FM128_5 = FM335_134+s->m[28]*AlM128_5;
  FM228_5 = FM335_234+s->m[28]*AlM228_5;
  FM328_5 = FB334_5+s->m[28]*AlM328_5;
  FM228_28 = s->m[28]+s->m[34];

// = = Block_0_2_0_2_0_16 = = 
 
// Backward Dynamics 

  FA137 = -(s->frc[1][37]-s->m[37]*(C37*(AlF136+BeF336*s->dpt[3][53])-S37*(AlF336+BS936*s->dpt[3][53])));
  FA237 = -(s->frc[2][37]-s->m[37]*(AlF236-s->dpt[3][53]*(OpF16-OM236*OM336)));
  FA337 = -(s->frc[3][37]-s->m[37]*(C37*(AlF336+BS936*s->dpt[3][53])+S37*(AlF136+BeF336*s->dpt[3][53])));
  CF137 = -(s->trq[1][37]-s->In[1][37]*(C37*(OpF16-qd[37]*OM336)-S37*(OpF336+qd[37]*OM136))+OM237*OM337*(s->In[5][37]-
 s->In[9][37]));
  CF237 = -(s->trq[2][37]-s->In[5][37]*OpF236-OM137*OM337*(s->In[1][37]-s->In[9][37]));
  CF337 = -(s->trq[3][37]-s->In[9][37]*(C37*(OpF336+qd[37]*OM136)+S37*(OpF16-qd[37]*OM336))+OM137*OM237*(s->In[1][37]-
 s->In[5][37]));
  FB137_1 = s->m[37]*(AlM16_1*C37-AlM336_1*S37);
  FB237_1 = s->m[37]*AlM236_1;
  FB337_1 = s->m[37]*(AlM16_1*S37+AlM336_1*C37);
  FB137_2 = s->m[37]*(AlM16_2*C37-AlM336_2*S37);
  FB237_2 = s->m[37]*AlM236_2;
  FB337_2 = s->m[37]*(AlM16_2*S37+AlM336_2*C37);
  FB137_3 = s->m[37]*(AlM16_3*C37-AlM336_3*S37);
  FB237_3 = s->m[37]*AlM236_3;
  FB337_3 = s->m[37]*(AlM16_3*S37+AlM336_3*C37);
  FB137_4 = -s->m[37]*(AlM336_4*S37-C37*(AlM136_4+OpM236_4*s->dpt[3][53]));
  FB237_4 = s->m[37]*(AlM236_4-OpM16_4*s->dpt[3][53]);
  FB337_4 = s->m[37]*(AlM336_4*C37+S37*(AlM136_4+OpM236_4*s->dpt[3][53]));
  CM137_4 = s->In[1][37]*(OpM16_4*C37-OpM336_4*S37);
  CM237_4 = s->In[5][37]*OpM236_4;
  CM337_4 = s->In[9][37]*(OpM16_4*S37+OpM336_4*C37);
  FB137_5 = -s->m[37]*(AlM336_5*S37-C37*(AlM136_5+OpM236_5*s->dpt[3][53]));
  FB237_5 = s->m[37]*(AlM236_5-s->dpt[3][53]*C6);
  FB337_5 = s->m[37]*(AlM336_5*C37+S37*(AlM136_5+OpM236_5*s->dpt[3][53]));
  CM137_5 = -s->In[1][37]*(OpM336_5*S37-C37*C6);
  CM237_5 = s->In[5][37]*OpM236_5;
  CM337_5 = s->In[9][37]*(OpM336_5*C37+S37*C6);
  FB137_6 = -s->m[37]*(AlM336_6*S37-C37*(AlM136_6+s->dpt[3][53]*C35p36));
  FB237_6 = s->m[37]*AlM236_6;
  FB337_6 = s->m[37]*(AlM336_6*C37+S37*(AlM136_6+s->dpt[3][53]*C35p36));
  CM137_6 = s->In[1][37]*S35p36*S37;
  CM237_6 = s->In[5][37]*C35p36;
  CM337_6 = -s->In[9][37]*S35p36*C37;
  FB237_35 = s->m[37]*(AlM236_35-s->dpt[3][53]);
  FF136 = -(s->frc[1][36]-s->m[36]*AlF136-FA137*C37-FA337*S37);
  FF236 = -(s->frc[2][36]-FA237-s->m[36]*AlF236);
  FF336 = -(s->frc[3][36]-s->m[36]*AlF336+FA137*S37-FA337*C37);
  CF136 = -(s->trq[1][36]-CF137*C37-CF337*S37+FA237*s->dpt[3][53]);
  CF236 = -(s->trq[2][36]-CF237-s->dpt[3][53]*(FA137*C37+FA337*S37));
  CF336 = -(s->trq[3][36]+CF137*S37-CF337*C37);
  FM136_1 = s->m[36]*AlM16_1+FB137_1*C37+FB337_1*S37;
  FM236_1 = FB237_1+s->m[36]*AlM236_1;
  FM336_1 = s->m[36]*AlM336_1-FB137_1*S37+FB337_1*C37;
  CM361_137 = -FB237_1*s->dpt[3][53];
  CM361_237 = s->dpt[3][53]*(FB137_1*C37+FB337_1*S37);
  FM136_2 = s->m[36]*AlM16_2+FB137_2*C37+FB337_2*S37;
  FM236_2 = FB237_2+s->m[36]*AlM236_2;
  FM336_2 = s->m[36]*AlM336_2-FB137_2*S37+FB337_2*C37;
  CM362_137 = -FB237_2*s->dpt[3][53];
  CM362_237 = s->dpt[3][53]*(FB137_2*C37+FB337_2*S37);
  FM136_3 = s->m[36]*AlM16_3+FB137_3*C37+FB337_3*S37;
  FM236_3 = FB237_3+s->m[36]*AlM236_3;
  FM336_3 = s->m[36]*AlM336_3-FB137_3*S37+FB337_3*C37;
  CM363_137 = -FB237_3*s->dpt[3][53];
  CM363_237 = s->dpt[3][53]*(FB137_3*C37+FB337_3*S37);
  FM136_4 = s->m[36]*AlM136_4+FB137_4*C37+FB337_4*S37;
  FM236_4 = FB237_4+s->m[36]*AlM236_4;
  FM336_4 = s->m[36]*AlM336_4-FB137_4*S37+FB337_4*C37;
  CM364_137 = CM137_4*C37+CM337_4*S37-FB237_4*s->dpt[3][53];
  CM364_237 = CM237_4+s->dpt[3][53]*(FB137_4*C37+FB337_4*S37);
  CM364_337 = -(CM137_4*S37-CM337_4*C37);
  FM136_5 = s->m[36]*AlM136_5+FB137_5*C37+FB337_5*S37;
  FM236_5 = FB237_5+s->m[36]*AlM236_5;
  FM336_5 = s->m[36]*AlM336_5-FB137_5*S37+FB337_5*C37;
  CM365_137 = CM137_5*C37+CM337_5*S37-FB237_5*s->dpt[3][53];
  CM365_237 = CM237_5+s->dpt[3][53]*(FB137_5*C37+FB337_5*S37);
  CM365_337 = -(CM137_5*S37-CM337_5*C37);
  FM136_6 = s->m[36]*AlM136_6+FB137_6*C37+FB337_6*S37;
  FM236_6 = FB237_6+s->m[36]*AlM236_6;
  FM336_6 = s->m[36]*AlM336_6-FB137_6*S37+FB337_6*C37;
  CM366_137 = CM137_6*C37+CM337_6*S37-FB237_6*s->dpt[3][53];
  CM366_237 = CM237_6+s->dpt[3][53]*(FB137_6*C37+FB337_6*S37);
  CM366_337 = -(CM137_6*S37-CM337_6*C37);
  CM3635_137 = s->In[1][37]*C37*C37+s->In[9][37]*S37*S37-FB237_35*s->dpt[3][53];
  CM3636_137 = s->In[1][37]*C37*C37+s->In[9][37]*S37*S37+s->m[37]*s->dpt[3][53]*s->dpt[3][53];
  FF135 = -(s->frc[1][35]-FF136);
  FF235 = -(s->frc[2][35]-FF236*C36+FF336*S36);
  FF335 = -(s->frc[3][35]-FF236*S36-FF336*C36);
  CF135 = -(s->trq[1][35]-CF136-s->dpt[2][50]*(FF236*S36+FF336*C36));
  CF235 = -(s->trq[2][35]-CF236*C36+CF336*S36);
  CF335 = -(s->trq[3][35]-CF236*S36-CF336*C36+FF136*s->dpt[2][50]);
  FM351_236 = FM236_1*C36-FM336_1*S36;
  FM351_336 = FM236_1*S36+FM336_1*C36;
  CM351_136 = CM361_137+s->dpt[2][50]*(FM236_1*S36+FM336_1*C36);
  CM351_236 = CM361_237*C36;
  CM351_336 = CM361_237*S36-FM136_1*s->dpt[2][50];
  FM352_236 = FM236_2*C36-FM336_2*S36;
  FM352_336 = FM236_2*S36+FM336_2*C36;
  CM352_136 = CM362_137+s->dpt[2][50]*(FM236_2*S36+FM336_2*C36);
  CM352_236 = CM362_237*C36;
  CM352_336 = CM362_237*S36-FM136_2*s->dpt[2][50];
  FM353_236 = FM236_3*C36-FM336_3*S36;
  FM353_336 = FM236_3*S36+FM336_3*C36;
  CM353_136 = CM363_137+s->dpt[2][50]*(FM236_3*S36+FM336_3*C36);
  CM353_236 = CM363_237*C36;
  CM353_336 = CM363_237*S36-FM136_3*s->dpt[2][50];
  FM354_236 = FM236_4*C36-FM336_4*S36;
  FM354_336 = FM236_4*S36+FM336_4*C36;
  CM354_136 = CM364_137+s->dpt[2][50]*(FM236_4*S36+FM336_4*C36);
  CM354_236 = CM364_237*C36-CM364_337*S36;
  CM354_336 = CM364_237*S36+CM364_337*C36-FM136_4*s->dpt[2][50];
  FM355_236 = FM236_5*C36-FM336_5*S36;
  FM355_336 = FM236_5*S36+FM336_5*C36;
  CM355_136 = CM365_137+s->dpt[2][50]*(FM236_5*S36+FM336_5*C36);
  CM355_236 = CM365_237*C36-CM365_337*S36;
  CM355_336 = CM365_237*S36+CM365_337*C36-FM136_5*s->dpt[2][50];
  CM356_136 = CM366_137+s->dpt[2][50]*(FM236_6*S36+FM336_6*C36);
  CM3535_136 = CM3635_137+s->dpt[2][50]*(AlM336_35*C36*(s->m[36]+s->m[37])+S36*(FB237_35+s->m[36]*AlM236_35));

// = = Block_0_2_0_2_0_17 = = 
 
// Backward Dynamics 

  FA140 = -(s->frc[1][40]-s->m[40]*(C40*(AlF139+BeF339*s->dpt[3][57])-S40*(AlF339+BS939*s->dpt[3][57])));
  FA240 = -(s->frc[2][40]-s->m[40]*(AlF239-s->dpt[3][57]*(OpF16-OM239*OM339)));
  FA340 = -(s->frc[3][40]-s->m[40]*(C40*(AlF339+BS939*s->dpt[3][57])+S40*(AlF139+BeF339*s->dpt[3][57])));
  CF140 = -(s->trq[1][40]-s->In[1][40]*(C40*(OpF16-qd[40]*OM339)-S40*(OpF339+qd[40]*OM139))+OM240*OM340*(s->In[5][40]-
 s->In[9][40]));
  CF240 = -(s->trq[2][40]-s->In[5][40]*OpF239-OM140*OM340*(s->In[1][40]-s->In[9][40]));
  CF340 = -(s->trq[3][40]-s->In[9][40]*(C40*(OpF339+qd[40]*OM139)+S40*(OpF16-qd[40]*OM339))+OM140*OM240*(s->In[1][40]-
 s->In[5][40]));
  FB140_1 = s->m[40]*(AlM16_1*C40-AlM339_1*S40);
  FB240_1 = s->m[40]*AlM239_1;
  FB340_1 = s->m[40]*(AlM16_1*S40+AlM339_1*C40);
  FB140_2 = s->m[40]*(AlM16_2*C40-AlM339_2*S40);
  FB240_2 = s->m[40]*AlM239_2;
  FB340_2 = s->m[40]*(AlM16_2*S40+AlM339_2*C40);
  FB140_3 = s->m[40]*(AlM16_3*C40-AlM339_3*S40);
  FB240_3 = s->m[40]*AlM239_3;
  FB340_3 = s->m[40]*(AlM16_3*S40+AlM339_3*C40);
  FB140_4 = -s->m[40]*(AlM339_4*S40-C40*(AlM139_4+OpM239_4*s->dpt[3][57]));
  FB240_4 = s->m[40]*(AlM239_4-OpM16_4*s->dpt[3][57]);
  FB340_4 = s->m[40]*(AlM339_4*C40+S40*(AlM139_4+OpM239_4*s->dpt[3][57]));
  CM140_4 = s->In[1][40]*(OpM16_4*C40-OpM339_4*S40);
  CM240_4 = s->In[5][40]*OpM239_4;
  CM340_4 = s->In[9][40]*(OpM16_4*S40+OpM339_4*C40);
  FB140_5 = -s->m[40]*(AlM339_5*S40-C40*(AlM139_5+OpM239_5*s->dpt[3][57]));
  FB240_5 = s->m[40]*(AlM239_5-s->dpt[3][57]*C6);
  FB340_5 = s->m[40]*(AlM339_5*C40+S40*(AlM139_5+OpM239_5*s->dpt[3][57]));
  CM140_5 = -s->In[1][40]*(OpM339_5*S40-C40*C6);
  CM240_5 = s->In[5][40]*OpM239_5;
  CM340_5 = s->In[9][40]*(OpM339_5*C40+S40*C6);
  FB140_6 = -s->m[40]*(AlM339_6*S40-C40*(AlM139_6+s->dpt[3][57]*C38p39));
  FB240_6 = s->m[40]*AlM239_6;
  FB340_6 = s->m[40]*(AlM339_6*C40+S40*(AlM139_6+s->dpt[3][57]*C38p39));
  CM140_6 = s->In[1][40]*S38p39*S40;
  CM240_6 = s->In[5][40]*C38p39;
  CM340_6 = -s->In[9][40]*S38p39*C40;
  FB240_38 = s->m[40]*(AlM239_38-s->dpt[3][57]);
  FF139 = -(s->frc[1][39]-s->m[39]*AlF139-FA140*C40-FA340*S40);
  FF239 = -(s->frc[2][39]-FA240-s->m[39]*AlF239);
  FF339 = -(s->frc[3][39]-s->m[39]*AlF339+FA140*S40-FA340*C40);
  CF139 = -(s->trq[1][39]-CF140*C40-CF340*S40+FA240*s->dpt[3][57]);
  CF239 = -(s->trq[2][39]-CF240-s->dpt[3][57]*(FA140*C40+FA340*S40));
  CF339 = -(s->trq[3][39]+CF140*S40-CF340*C40);
  FM139_1 = s->m[39]*AlM16_1+FB140_1*C40+FB340_1*S40;
  FM239_1 = FB240_1+s->m[39]*AlM239_1;
  FM339_1 = s->m[39]*AlM339_1-FB140_1*S40+FB340_1*C40;
  CM391_140 = -FB240_1*s->dpt[3][57];
  CM391_240 = s->dpt[3][57]*(FB140_1*C40+FB340_1*S40);
  FM139_2 = s->m[39]*AlM16_2+FB140_2*C40+FB340_2*S40;
  FM239_2 = FB240_2+s->m[39]*AlM239_2;
  FM339_2 = s->m[39]*AlM339_2-FB140_2*S40+FB340_2*C40;
  CM392_140 = -FB240_2*s->dpt[3][57];
  CM392_240 = s->dpt[3][57]*(FB140_2*C40+FB340_2*S40);
  FM139_3 = s->m[39]*AlM16_3+FB140_3*C40+FB340_3*S40;
  FM239_3 = FB240_3+s->m[39]*AlM239_3;
  FM339_3 = s->m[39]*AlM339_3-FB140_3*S40+FB340_3*C40;
  CM393_140 = -FB240_3*s->dpt[3][57];
  CM393_240 = s->dpt[3][57]*(FB140_3*C40+FB340_3*S40);
  FM139_4 = s->m[39]*AlM139_4+FB140_4*C40+FB340_4*S40;
  FM239_4 = FB240_4+s->m[39]*AlM239_4;
  FM339_4 = s->m[39]*AlM339_4-FB140_4*S40+FB340_4*C40;
  CM394_140 = CM140_4*C40+CM340_4*S40-FB240_4*s->dpt[3][57];
  CM394_240 = CM240_4+s->dpt[3][57]*(FB140_4*C40+FB340_4*S40);
  CM394_340 = -(CM140_4*S40-CM340_4*C40);
  FM139_5 = s->m[39]*AlM139_5+FB140_5*C40+FB340_5*S40;
  FM239_5 = FB240_5+s->m[39]*AlM239_5;
  FM339_5 = s->m[39]*AlM339_5-FB140_5*S40+FB340_5*C40;
  CM395_140 = CM140_5*C40+CM340_5*S40-FB240_5*s->dpt[3][57];
  CM395_240 = CM240_5+s->dpt[3][57]*(FB140_5*C40+FB340_5*S40);
  CM395_340 = -(CM140_5*S40-CM340_5*C40);
  FM139_6 = s->m[39]*AlM139_6+FB140_6*C40+FB340_6*S40;
  FM239_6 = FB240_6+s->m[39]*AlM239_6;
  FM339_6 = s->m[39]*AlM339_6-FB140_6*S40+FB340_6*C40;
  CM396_140 = CM140_6*C40+CM340_6*S40-FB240_6*s->dpt[3][57];
  CM396_240 = CM240_6+s->dpt[3][57]*(FB140_6*C40+FB340_6*S40);
  CM396_340 = -(CM140_6*S40-CM340_6*C40);
  CM3938_140 = s->In[1][40]*C40*C40+s->In[9][40]*S40*S40-FB240_38*s->dpt[3][57];
  CM3939_140 = s->In[1][40]*C40*C40+s->In[9][40]*S40*S40+s->m[40]*s->dpt[3][57]*s->dpt[3][57];
  FF138 = -(s->frc[1][38]-FF139);
  FF238 = -(s->frc[2][38]-FF239*C39+FF339*S39);
  FF338 = -(s->frc[3][38]-FF239*S39-FF339*C39);
  CF138 = -(s->trq[1][38]-CF139-s->dpt[2][55]*(FF239*S39+FF339*C39));
  CF238 = -(s->trq[2][38]-CF239*C39+CF339*S39);
  CF338 = -(s->trq[3][38]-CF239*S39-CF339*C39+FF139*s->dpt[2][55]);
  FM381_239 = FM239_1*C39-FM339_1*S39;
  FM381_339 = FM239_1*S39+FM339_1*C39;
  CM381_139 = CM391_140+s->dpt[2][55]*(FM239_1*S39+FM339_1*C39);
  CM381_239 = CM391_240*C39;
  CM381_339 = CM391_240*S39-FM139_1*s->dpt[2][55];
  FM382_239 = FM239_2*C39-FM339_2*S39;
  FM382_339 = FM239_2*S39+FM339_2*C39;
  CM382_139 = CM392_140+s->dpt[2][55]*(FM239_2*S39+FM339_2*C39);
  CM382_239 = CM392_240*C39;
  CM382_339 = CM392_240*S39-FM139_2*s->dpt[2][55];
  FM383_239 = FM239_3*C39-FM339_3*S39;
  FM383_339 = FM239_3*S39+FM339_3*C39;
  CM383_139 = CM393_140+s->dpt[2][55]*(FM239_3*S39+FM339_3*C39);
  CM383_239 = CM393_240*C39;
  CM383_339 = CM393_240*S39-FM139_3*s->dpt[2][55];
  FM384_239 = FM239_4*C39-FM339_4*S39;
  FM384_339 = FM239_4*S39+FM339_4*C39;
  CM384_139 = CM394_140+s->dpt[2][55]*(FM239_4*S39+FM339_4*C39);
  CM384_239 = CM394_240*C39-CM394_340*S39;
  CM384_339 = CM394_240*S39+CM394_340*C39-FM139_4*s->dpt[2][55];
  FM385_239 = FM239_5*C39-FM339_5*S39;
  FM385_339 = FM239_5*S39+FM339_5*C39;
  CM385_139 = CM395_140+s->dpt[2][55]*(FM239_5*S39+FM339_5*C39);
  CM385_239 = CM395_240*C39-CM395_340*S39;
  CM385_339 = CM395_240*S39+CM395_340*C39-FM139_5*s->dpt[2][55];
  CM386_139 = CM396_140+s->dpt[2][55]*(FM239_6*S39+FM339_6*C39);
  CM3838_139 = CM3938_140+s->dpt[2][55]*(AlM339_38*C39*(s->m[39]+s->m[40])+S39*(FB240_38+s->m[39]*AlM239_38));

// = = Block_0_2_0_2_0_18 = = 
 
// Backward Dynamics 

  FF141 = -(s->frc[1][41]-FF45_146-FF142*C42-FF342*S42);
  FF241 = -(s->frc[2][41]+s->frc[2][42]+s->frc[2][44]*C43+s->frc[2][46]*C45+FF43_344*S43+FF45_346*S45);
  FF341 = -(s->frc[3][41]+s->frc[2][46]*S45+FF142*S42-FF342*C42-FF45_346*C45);
  CF141 = -(s->trq[1][41]-CF45_146-CF142*C42-CF342*S42+s->dpt[2][61]*(s->frc[2][46]*S45-FF45_346*C45));
  CF241 = -(s->trq[2][41]-CF242+s->trq[2][46]*C45+CF45_346*S45-s->dpt[1][61]*(s->frc[2][46]*S45-FF45_346*C45));
  CF341 = -(s->trq[3][41]+s->trq[2][46]*S45+CF142*S42-CF342*C42-CF45_346*C45+FF45_146*s->dpt[2][61]+s->dpt[1][61]*(
 s->frc[2][46]*C45+FF45_346*S45));

// = = Block_0_2_0_2_0_21 = = 
 
// Backward Dynamics 

  FA150 = -(s->frc[1][50]-s->m[50]*(AlF16+BS16*s->dpt[1][17]+BeF26*s->dpt[2][17]+BeF36*s->dpt[3][17]));
  FA250 = -(s->frc[2][50]-s->m[50]*(AlF249*C50+AlF349*S50));
  FA350 = -(s->frc[3][50]+s->m[50]*(AlF249*S50-AlF349*C50));
  CF150 = -(s->trq[1][50]-s->In[1][50]*OpF16+OM250*OM350*(s->In[5][50]-s->In[9][50]));
  CF250 = -(s->trq[2][50]-s->In[5][50]*(C50*(OpF249+qd[50]*OM349)+S50*(OpF349-qd[50]*OM249))-OM150*OM350*(s->In[1][50]-
 s->In[9][50]));
  CF350 = -(s->trq[3][50]-s->In[9][50]*(C50*(OpF349-qd[50]*OM249)-S50*(OpF249+qd[50]*OM349))+OM150*OM250*(s->In[1][50]-
 s->In[5][50]));
  FB150_1 = s->m[50]*AlM16_1;
  FB250_1 = s->m[50]*(AlM249_1*C50+AlM349_1*S50);
  FB350_1 = -s->m[50]*(AlM249_1*S50-AlM349_1*C50);
  FB150_2 = s->m[50]*AlM16_2;
  FB250_2 = s->m[50]*(AlM249_2*C50+AlM349_2*S50);
  FB350_2 = -s->m[50]*(AlM249_2*S50-AlM349_2*C50);
  FB150_3 = s->m[50]*AlM16_3;
  FB250_3 = s->m[50]*(AlM249_3*C50+AlM349_3*S50);
  FB350_3 = -s->m[50]*(AlM249_3*S50-AlM349_3*C50);
  FB150_4 = -s->m[50]*(OpM36_4*s->dpt[2][17]-s->dpt[3][17]*S5);
  FB250_4 = s->m[50]*(AlM249_4*C50+AlM349_4*S50);
  FB350_4 = -s->m[50]*(AlM249_4*S50-AlM349_4*C50);
  CM150_4 = s->In[1][50]*OpM16_4;
  CM250_4 = s->In[5][50]*(OpM249_4*C50+OpM349_4*S50);
  CM350_4 = -s->In[9][50]*(OpM249_4*S50-OpM349_4*C50);
  FB150_5 = -s->m[50]*s->dpt[2][17]*S6;
  FB250_5 = s->m[50]*(AlM249_5*C50+AlM349_5*S50);
  FB350_5 = -s->m[50]*(AlM249_5*S50-AlM349_5*C50);
  CM150_5 = s->In[1][50]*C6;
  CM250_5 = s->In[5][50]*S47p48p49p50*S6;
  CM350_5 = s->In[9][50]*C47p48p49p50*S6;
  CM250_6 = s->In[5][50]*C47p48p49p50;
  CM350_6 = -s->In[9][50]*S47p48p49p50;
  FF49_250 = FA250*C50-FA350*S50;
  FF49_350 = FA250*S50+FA350*C50;
  CF49_250 = CF250*C50-CF350*S50;
  CF49_350 = CF250*S50+CF350*C50;
  FM491_250 = FB250_1*C50-FB350_1*S50;
  FM491_350 = FB250_1*S50+FB350_1*C50;
  FM492_250 = FB250_2*C50-FB350_2*S50;
  FM492_350 = FB250_2*S50+FB350_2*C50;
  FM493_250 = FB250_3*C50-FB350_3*S50;
  FM493_350 = FB250_3*S50+FB350_3*C50;
  FM494_250 = FB250_4*C50-FB350_4*S50;
  FM494_350 = FB250_4*S50+FB350_4*C50;
  CM494_250 = CM250_4*C50-CM350_4*S50;
  CM494_350 = CM250_4*S50+CM350_4*C50;
  FM495_250 = FB250_5*C50-FB350_5*S50;
  FM495_350 = FB250_5*S50+FB350_5*C50;
  CM495_250 = CM250_5*C50-CM350_5*S50;
  CM495_350 = CM250_5*S50+CM350_5*C50;
  CM496_250 = CM250_6*C50-CM350_6*S50;
  CM496_350 = CM250_6*S50+CM350_6*C50;
  FF48_249 = FF49_250*C49-FF49_350*S49;
  FF48_349 = FF49_250*S49+FF49_350*C49;
  CF48_249 = CF49_250*C49-CF49_350*S49;
  CF48_349 = CF49_250*S49+CF49_350*C49;
  FM481_249 = FM491_250*C49-FM491_350*S49;
  FM481_349 = FM491_250*S49+FM491_350*C49;
  FM482_249 = FM492_250*C49-FM492_350*S49;
  FM482_349 = FM492_250*S49+FM492_350*C49;
  FM483_249 = FM493_250*C49-FM493_350*S49;
  FM483_349 = FM493_250*S49+FM493_350*C49;
  FM484_249 = FM494_250*C49-FM494_350*S49;
  FM484_349 = FM494_250*S49+FM494_350*C49;
  CM484_249 = CM494_250*C49-CM494_350*S49;
  CM484_349 = CM494_250*S49+CM494_350*C49;
  FM485_249 = FM495_250*C49-FM495_350*S49;
  FM485_349 = FM495_250*S49+FM495_350*C49;
  CM485_249 = CM495_250*C49-CM495_350*S49;
  CM485_349 = CM495_250*S49+CM495_350*C49;
  CM486_249 = CM496_250*C49-CM496_350*S49;
  CM486_349 = CM496_250*S49+CM496_350*C49;
  FF47_248 = FF48_249*C48-FF48_349*S48;
  FF47_348 = FF48_249*S48+FF48_349*C48;
  CF47_248 = CF48_249*C48-CF48_349*S48;
  CF47_348 = CF48_249*S48+CF48_349*C48;
  FM471_248 = FM481_249*C48-FM481_349*S48;
  FM471_348 = FM481_249*S48+FM481_349*C48;
  FM472_248 = FM482_249*C48-FM482_349*S48;
  FM472_348 = FM482_249*S48+FM482_349*C48;
  FM473_248 = FM483_249*C48-FM483_349*S48;
  FM473_348 = FM483_249*S48+FM483_349*C48;
  FM474_248 = FM484_249*C48-FM484_349*S48;
  FM474_348 = FM484_249*S48+FM484_349*C48;
  CM474_248 = CM484_249*C48-CM484_349*S48;
  CM474_348 = CM484_249*S48+CM484_349*C48;
  FM475_248 = FM485_249*C48-FM485_349*S48;
  FM475_348 = FM485_249*S48+FM485_349*C48;
  CM475_248 = CM485_249*C48-CM485_349*S48;
  CM475_348 = CM485_249*S48+CM485_349*C48;

// = = Block_0_2_0_2_0_22 = = 
 
// Backward Dynamics 

  FA154 = -(s->frc[1][54]-s->m[54]*(AlF154+BS154*s->l[1][54]+BeF354*s->l[3][54]));
  FA254 = -(s->frc[2][54]-s->m[54]*(AlF254+BeF454*s->l[1][54]+BeF654*s->l[3][54]));
  FA354 = -(s->frc[3][54]-s->m[54]*(AlF354+BS954*s->l[3][54]+BeF754*s->l[1][54]));
  FF154 = FA154+FA159+FA155*C55+FA156*C56+FA157*C57+FA158*C58+FA355*S55+FA356*S56+FA357*S57+FA358*S58;
  FF254 = FA254+FA255+FA256+FA257+FA258+FA259;
  FF354 = FA354+FA359-FA155*S55-FA156*S56-FA157*S57-FA158*S58+FA355*C55+FA356*C56+FA357*C57+FA358*C58;
  CF154 = -(s->trq[1][54]+s->trq[1][59]-s->In[1][54]*OpF153-s->In[1][59]*OpF153-CF155*C55-CF156*C56-CF157*C57-CF158*C58-
 CF355*S55-CF356*S56-CF357*S57-CF358*S58+Dz593*FA259+FA254*s->l[3][54]+FA255*s->dpt[3][65]+FA256*s->dpt[3][66]+FA257*
 s->dpt[3][67]+FA258*s->dpt[3][68]+OM253*OM353*(s->In[5][54]-s->In[9][54])+OM253*OM353*(s->In[5][59]-s->In[9][59])+
 s->dpt[2][65]*(FA155*S55-FA355*C55)+s->dpt[2][66]*(FA156*S56-FA356*C56)+s->dpt[2][67]*(FA157*S57-FA357*C57)+s->dpt[2][68]*(
 FA158*S58-FA358*C58));
  FB154_1 = s->m[54]*AlM153_1;
  FB254_1 = s->m[54]*AlM252_1;
  FB354_1 = s->m[54]*AlM353_1;
  FM154_1 = FB154_1+FB159_1+FB155_1*C55+FB156_1*C56+FB157_1*C57+FB158_1*C58+FB355_1*S55+FB356_1*S56+FB357_1*S57+FB358_1*
 S58;
  FM254_1 = FB254_1+FB255_1+FB256_1+FB257_1+FB258_1+FB259_1;
  FM354_1 = FB354_1+FB359_1-FB155_1*S55-FB156_1*S56-FB157_1*S57-FB158_1*S58+FB355_1*C55+FB356_1*C56+FB357_1*C57+FB358_1*
 C58;
  CM154_1 = -(Dz593*FB259_1+FB254_1*s->l[3][54]+FB255_1*s->dpt[3][65]+FB256_1*s->dpt[3][66]+FB257_1*s->dpt[3][67]+
 FB258_1*s->dpt[3][68]+s->dpt[2][65]*(FB155_1*S55-FB355_1*C55)+s->dpt[2][66]*(FB156_1*S56-FB356_1*C56)+s->dpt[2][67]*(FB157_1
 *S57-FB357_1*C57)+s->dpt[2][68]*(FB158_1*S58-FB358_1*C58));
  FB154_2 = s->m[54]*AlM153_2;
  FB254_2 = s->m[54]*AlM252_2;
  FB354_2 = s->m[54]*AlM353_2;
  FM154_2 = FB154_2+FB159_2+FB155_2*C55+FB156_2*C56+FB157_2*C57+FB158_2*C58+FB355_2*S55+FB356_2*S56+FB357_2*S57+FB358_2*
 S58;
  FM254_2 = FB254_2+FB255_2+FB256_2+FB257_2+FB258_2+FB259_2;
  FM354_2 = FB354_2+FB359_2-FB155_2*S55-FB156_2*S56-FB157_2*S57-FB158_2*S58+FB355_2*C55+FB356_2*C56+FB357_2*C57+FB358_2*
 C58;
  CM154_2 = -(Dz593*FB259_2+FB254_2*s->l[3][54]+FB255_2*s->dpt[3][65]+FB256_2*s->dpt[3][66]+FB257_2*s->dpt[3][67]+
 FB258_2*s->dpt[3][68]+s->dpt[2][65]*(FB155_2*S55-FB355_2*C55)+s->dpt[2][66]*(FB156_2*S56-FB356_2*C56)+s->dpt[2][67]*(FB157_2
 *S57-FB357_2*C57)+s->dpt[2][68]*(FB158_2*S58-FB358_2*C58));
  FB154_3 = s->m[54]*AlM153_3;
  FB254_3 = s->m[54]*AlM252_3;
  FB354_3 = s->m[54]*AlM353_3;
  FM154_3 = FB154_3+FB159_3+FB155_3*C55+FB156_3*C56+FB157_3*C57+FB158_3*C58+FB355_3*S55+FB356_3*S56+FB357_3*S57+FB358_3*
 S58;
  FM254_3 = FB254_3+FB255_3+FB256_3+FB257_3+FB258_3+FB259_3;
  FM354_3 = FB354_3+FB359_3-FB155_3*S55-FB156_3*S56-FB157_3*S57-FB158_3*S58+FB355_3*C55+FB356_3*C56+FB357_3*C57+FB358_3*
 C58;
  CM154_3 = -(Dz593*FB259_3+FB254_3*s->l[3][54]+FB255_3*s->dpt[3][65]+FB256_3*s->dpt[3][66]+FB257_3*s->dpt[3][67]+
 FB258_3*s->dpt[3][68]+s->dpt[2][65]*(FB155_3*S55-FB355_3*C55)+s->dpt[2][66]*(FB156_3*S56-FB356_3*C56)+s->dpt[2][67]*(FB157_3
 *S57-FB357_3*C57)+s->dpt[2][68]*(FB158_3*S58-FB358_3*C58));
  FB154_4 = s->m[54]*(AlM153_4+OpM252_4*s->l[3][54]);
  FB254_4 = s->m[54]*(AlM254_4-OpM153_4*s->l[3][54]+OpM353_4*s->l[1][54]);
  FB354_4 = s->m[54]*(AlM354_4-OpM252_4*s->l[1][54]);
  FM154_4 = FB154_4+FB159_4+FB155_4*C55+FB156_4*C56+FB157_4*C57+FB158_4*C58+FB355_4*S55+FB356_4*S56+FB357_4*S57+FB358_4*
 S58;
  FM254_4 = FB254_4+FB255_4+FB256_4+FB257_4+FB258_4+FB259_4;
  FM354_4 = FB354_4+FB359_4-FB155_4*S55-FB156_4*S56-FB157_4*S57-FB158_4*S58+FB355_4*C55+FB356_4*C56+FB357_4*C57+FB358_4*
 C58;
  CM154_4 = CM155_4*C55+CM156_4*C56+CM157_4*C57+CM158_4*C58+CM355_4*S55+CM356_4*S56+CM357_4*S57+CM358_4*S58-Dz593*
 FB259_4-FB254_4*s->l[3][54]-FB255_4*s->dpt[3][65]-FB256_4*s->dpt[3][66]-FB257_4*s->dpt[3][67]-FB258_4*s->dpt[3][68]+OpM153_4
 *(s->In[1][54]+s->In[1][59])-s->dpt[2][65]*(FB155_4*S55-FB355_4*C55)-s->dpt[2][66]*(FB156_4*S56-FB356_4*C56)-s->dpt[2][67]*(
 FB157_4*S57-FB357_4*C57)-s->dpt[2][68]*(FB158_4*S58-FB358_4*C58);
  FB154_5 = s->m[54]*(AlM153_5+OpM252_5*s->l[3][54]);
  FB254_5 = s->m[54]*(AlM254_5-OpM153_5*s->l[3][54]+OpM353_5*s->l[1][54]);
  FB354_5 = s->m[54]*(AlM354_5-OpM252_5*s->l[1][54]);
  FM154_5 = FB154_5+FB159_5+FB155_5*C55+FB156_5*C56+FB157_5*C57+FB158_5*C58+FB355_5*S55+FB356_5*S56+FB357_5*S57+FB358_5*
 S58;
  FM254_5 = FB254_5+FB255_5+FB256_5+FB257_5+FB258_5+FB259_5;
  FM354_5 = FB354_5+FB359_5-FB155_5*S55-FB156_5*S56-FB157_5*S57-FB158_5*S58+FB355_5*C55+FB356_5*C56+FB357_5*C57+FB358_5*
 C58;
  CM154_5 = CM155_5*C55+CM156_5*C56+CM157_5*C57+CM158_5*C58+CM355_5*S55+CM356_5*S56+CM357_5*S57+CM358_5*S58-Dz593*
 FB259_5-FB254_5*s->l[3][54]-FB255_5*s->dpt[3][65]-FB256_5*s->dpt[3][66]-FB257_5*s->dpt[3][67]-FB258_5*s->dpt[3][68]+OpM153_5
 *(s->In[1][54]+s->In[1][59])-s->dpt[2][65]*(FB155_5*S55-FB355_5*C55)-s->dpt[2][66]*(FB156_5*S56-FB356_5*C56)-s->dpt[2][67]*(
 FB157_5*S57-FB357_5*C57)-s->dpt[2][68]*(FB158_5*S58-FB358_5*C58);
  FB154_6 = s->m[54]*(AlM153_6+OpM252_6*s->l[3][54]);
  FB254_6 = s->m[54]*(AlM254_6-OpM153_6*s->l[3][54]+OpM353_6*s->l[1][54]);
  FB354_6 = s->m[54]*(AlM354_6-OpM252_6*s->l[1][54]);
  FM154_6 = FB154_6+FB159_6+FB155_6*C55+FB156_6*C56+FB157_6*C57+FB158_6*C58+FB355_6*S55+FB356_6*S56+FB357_6*S57+FB358_6*
 S58;
  FM254_6 = FB254_6+FB255_6+FB256_6+FB257_6+FB258_6+FB259_6;
  FM354_6 = FB354_6+FB359_6-FB155_6*S55-FB156_6*S56-FB157_6*S57-FB158_6*S58+FB355_6*C55+FB356_6*C56+FB357_6*C57+FB358_6*
 C58;
  CM154_6 = CM155_6*C55+CM156_6*C56+CM157_6*C57+CM158_6*C58+CM355_6*S55+CM356_6*S56+CM357_6*S57+CM358_6*S58-Dz593*
 FB259_6-FB254_6*s->l[3][54]-FB255_6*s->dpt[3][65]-FB256_6*s->dpt[3][66]-FB257_6*s->dpt[3][67]-FB258_6*s->dpt[3][68]+OpM153_6
 *(s->In[1][54]+s->In[1][59])-s->dpt[2][65]*(FB155_6*S55-FB355_6*C55)-s->dpt[2][66]*(FB156_6*S56-FB356_6*C56)-s->dpt[2][67]*(
 FB157_6*S57-FB357_6*C57)-s->dpt[2][68]*(FB158_6*S58-FB358_6*C58);
  FB154_51 = s->m[54]*s->l[3][54]*S52;
  FB254_51 = s->m[54]*(AlM254_51-OpM153_51*s->l[3][54]+OpM353_51*s->l[1][54]);
  FB354_51 = s->m[54]*(AlM354_51-s->l[1][54]*S52);
  FM154_51 = FB154_51+FB159_51+FB155_51*C55+FB156_51*C56+FB157_51*C57+FB158_51*C58+FB355_51*S55+FB356_51*S56+FB357_51*
 S57+FB358_51*S58;
  CM154_51 = CM155_51*C55+CM156_51*C56+CM157_51*C57+CM158_51*C58+CM355_51*S55+CM356_51*S56+CM357_51*S57+CM358_51*S58-
 Dz593*FB259_51-FB254_51*s->l[3][54]-FB255_51*s->dpt[3][65]-FB256_51*s->dpt[3][66]-FB257_51*s->dpt[3][67]-FB258_51*
 s->dpt[3][68]+OpM153_51*(s->In[1][54]+s->In[1][59])-s->dpt[2][65]*(FB155_51*S55-FB355_51*C55)-s->dpt[2][66]*(FB156_51*S56-
 FB356_51*C56)-s->dpt[2][67]*(FB157_51*S57-FB357_51*C57)-s->dpt[2][68]*(FB158_51*S58-FB358_51*C58);
  FB254_52 = s->m[54]*(AlM254_52+s->l[1][54]*S53-s->l[3][54]*C53);
  FM154_52 = -S53*(s->m[55]*s->dpt[2][65]+s->m[56]*s->dpt[2][66]+s->m[57]*s->dpt[2][67]+s->m[58]*s->dpt[2][68]);
  FB154_53 = s->m[54]*s->l[3][54];
  FB354_53 = -s->m[54]*(q[54]+s->l[1][54]);
  FM154_53 = FB154_53+FB159_53+FB155_53*C55+FB156_53*C56+FB157_53*C57+FB158_53*C58+FB355_53*S55+FB356_53*S56+FB357_53*
 S57+FB358_53*S58;
  FM154_54 = s->m[54]+s->m[55]+s->m[56]+s->m[57]+s->m[58]+s->m[59];
  CF53_254 = -(s->trq[2][54]+s->trq[2][59]-CF255-CF256-CF257-CF258+q[54]*FF354-s->In[5][54]*OpF252-s->In[5][59]*OpF252-
 Dz593*FA159-FA154*s->l[3][54]+FA354*s->l[1][54]+FA359*s->dpt[1][69]-OM153*OM353*(s->In[1][54]-s->In[9][54])-OM153*OM353*(
 s->In[1][59]-s->In[9][59])-s->dpt[1][65]*(FA155*S55-FA355*C55)-s->dpt[1][66]*(FA156*S56-FA356*C56)-s->dpt[1][67]*(FA157*S57-
 FA357*C57)-s->dpt[1][68]*(FA158*S58-FA358*C58)-s->dpt[3][65]*(FA155*C55+FA355*S55)-s->dpt[3][66]*(FA156*C56+FA356*S56)-
 s->dpt[3][67]*(FA157*C57+FA357*S57)-s->dpt[3][68]*(FA158*C58+FA358*S58));
  CF53_354 = -(s->trq[3][54]+s->trq[3][59]-q[54]*FF254-s->In[9][54]*OpF353-s->In[9][59]*OpF353+CF155*S55+CF156*S56+CF157
 *S57+CF158*S58-CF355*C55-CF356*C56-CF357*C57-CF358*C58-FA254*s->l[1][54]-FA255*s->dpt[1][65]-FA256*s->dpt[1][66]-FA257*
 s->dpt[1][67]-FA258*s->dpt[1][68]-FA259*s->dpt[1][69]+OM153*OM253*(s->In[1][54]-s->In[5][54])+OM153*OM253*(s->In[1][59]-
 s->In[5][59])+s->dpt[2][65]*(FA155*C55+FA355*S55)+s->dpt[2][66]*(FA156*C56+FA356*S56)+s->dpt[2][67]*(FA157*C57+FA357*S57)+
 s->dpt[2][68]*(FA158*C58+FA358*S58));
  CM531_254 = Dz593*FB159_1+FB154_1*s->l[3][54]-FB354_1*s->l[1][54]-FB359_1*s->dpt[1][69]+s->dpt[1][65]*(FB155_1*S55-
 FB355_1*C55)+s->dpt[1][66]*(FB156_1*S56-FB356_1*C56)+s->dpt[1][67]*(FB157_1*S57-FB357_1*C57)+s->dpt[1][68]*(FB158_1*S58-
 FB358_1*C58)+s->dpt[3][65]*(FB155_1*C55+FB355_1*S55)+s->dpt[3][66]*(FB156_1*C56+FB356_1*S56)+s->dpt[3][67]*(FB157_1*C57+
 FB357_1*S57)+s->dpt[3][68]*(FB158_1*C58+FB358_1*S58)-q[54]*FM354_1;
  CM531_354 = q[54]*FM254_1+FB254_1*s->l[1][54]+FB255_1*s->dpt[1][65]+FB256_1*s->dpt[1][66]+FB257_1*s->dpt[1][67]+
 FB258_1*s->dpt[1][68]+FB259_1*s->dpt[1][69]-s->dpt[2][65]*(FB155_1*C55+FB355_1*S55)-s->dpt[2][66]*(FB156_1*C56+FB356_1*S56)-
 s->dpt[2][67]*(FB157_1*C57+FB357_1*S57)-s->dpt[2][68]*(FB158_1*C58+FB358_1*S58);
  CM532_254 = Dz593*FB159_2+FB154_2*s->l[3][54]-FB354_2*s->l[1][54]-FB359_2*s->dpt[1][69]+s->dpt[1][65]*(FB155_2*S55-
 FB355_2*C55)+s->dpt[1][66]*(FB156_2*S56-FB356_2*C56)+s->dpt[1][67]*(FB157_2*S57-FB357_2*C57)+s->dpt[1][68]*(FB158_2*S58-
 FB358_2*C58)+s->dpt[3][65]*(FB155_2*C55+FB355_2*S55)+s->dpt[3][66]*(FB156_2*C56+FB356_2*S56)+s->dpt[3][67]*(FB157_2*C57+
 FB357_2*S57)+s->dpt[3][68]*(FB158_2*C58+FB358_2*S58)-q[54]*FM354_2;
  CM532_354 = q[54]*FM254_2+FB254_2*s->l[1][54]+FB255_2*s->dpt[1][65]+FB256_2*s->dpt[1][66]+FB257_2*s->dpt[1][67]+
 FB258_2*s->dpt[1][68]+FB259_2*s->dpt[1][69]-s->dpt[2][65]*(FB155_2*C55+FB355_2*S55)-s->dpt[2][66]*(FB156_2*C56+FB356_2*S56)-
 s->dpt[2][67]*(FB157_2*C57+FB357_2*S57)-s->dpt[2][68]*(FB158_2*C58+FB358_2*S58);
  CM533_254 = Dz593*FB159_3+FB154_3*s->l[3][54]-FB354_3*s->l[1][54]-FB359_3*s->dpt[1][69]+s->dpt[1][65]*(FB155_3*S55-
 FB355_3*C55)+s->dpt[1][66]*(FB156_3*S56-FB356_3*C56)+s->dpt[1][67]*(FB157_3*S57-FB357_3*C57)+s->dpt[1][68]*(FB158_3*S58-
 FB358_3*C58)+s->dpt[3][65]*(FB155_3*C55+FB355_3*S55)+s->dpt[3][66]*(FB156_3*C56+FB356_3*S56)+s->dpt[3][67]*(FB157_3*C57+
 FB357_3*S57)+s->dpt[3][68]*(FB158_3*C58+FB358_3*S58)-q[54]*FM354_3;
  CM533_354 = q[54]*FM254_3+FB254_3*s->l[1][54]+FB255_3*s->dpt[1][65]+FB256_3*s->dpt[1][66]+FB257_3*s->dpt[1][67]+
 FB258_3*s->dpt[1][68]+FB259_3*s->dpt[1][69]-s->dpt[2][65]*(FB155_3*C55+FB355_3*S55)-s->dpt[2][66]*(FB156_3*C56+FB356_3*S56)-
 s->dpt[2][67]*(FB157_3*C57+FB357_3*S57)-s->dpt[2][68]*(FB158_3*C58+FB358_3*S58);
  CM534_254 = CM255_4+CM256_4+CM257_4+CM258_4-q[54]*FM354_4+s->In[5][54]*OpM252_4+s->In[5][59]*OpM252_4+Dz593*FB159_4+
 FB154_4*s->l[3][54]-FB354_4*s->l[1][54]-FB359_4*s->dpt[1][69]+s->dpt[1][65]*(FB155_4*S55-FB355_4*C55)+s->dpt[1][66]*(FB156_4
 *S56-FB356_4*C56)+s->dpt[1][67]*(FB157_4*S57-FB357_4*C57)+s->dpt[1][68]*(FB158_4*S58-FB358_4*C58)+s->dpt[3][65]*(FB155_4*C55
 +FB355_4*S55)+s->dpt[3][66]*(FB156_4*C56+FB356_4*S56)+s->dpt[3][67]*(FB157_4*C57+FB357_4*S57)+s->dpt[3][68]*(FB158_4*C58+
 FB358_4*S58);
  CM534_354 = q[54]*FM254_4-CM155_4*S55-CM156_4*S56-CM157_4*S57-CM158_4*S58+CM355_4*C55+CM356_4*C56+CM357_4*C57+CM358_4*
 C58+FB254_4*s->l[1][54]+FB255_4*s->dpt[1][65]+FB256_4*s->dpt[1][66]+FB257_4*s->dpt[1][67]+FB258_4*s->dpt[1][68]+FB259_4*
 s->dpt[1][69]+OpM353_4*(s->In[9][54]+s->In[9][59])-s->dpt[2][65]*(FB155_4*C55+FB355_4*S55)-s->dpt[2][66]*(FB156_4*C56+
 FB356_4*S56)-s->dpt[2][67]*(FB157_4*C57+FB357_4*S57)-s->dpt[2][68]*(FB158_4*C58+FB358_4*S58);
  CM535_254 = CM255_5+CM256_5+CM257_5+CM258_5-q[54]*FM354_5+s->In[5][54]*OpM252_5+s->In[5][59]*OpM252_5+Dz593*FB159_5+
 FB154_5*s->l[3][54]-FB354_5*s->l[1][54]-FB359_5*s->dpt[1][69]+s->dpt[1][65]*(FB155_5*S55-FB355_5*C55)+s->dpt[1][66]*(FB156_5
 *S56-FB356_5*C56)+s->dpt[1][67]*(FB157_5*S57-FB357_5*C57)+s->dpt[1][68]*(FB158_5*S58-FB358_5*C58)+s->dpt[3][65]*(FB155_5*C55
 +FB355_5*S55)+s->dpt[3][66]*(FB156_5*C56+FB356_5*S56)+s->dpt[3][67]*(FB157_5*C57+FB357_5*S57)+s->dpt[3][68]*(FB158_5*C58+
 FB358_5*S58);
  CM535_354 = q[54]*FM254_5-CM155_5*S55-CM156_5*S56-CM157_5*S57-CM158_5*S58+CM355_5*C55+CM356_5*C56+CM357_5*C57+CM358_5*
 C58+FB254_5*s->l[1][54]+FB255_5*s->dpt[1][65]+FB256_5*s->dpt[1][66]+FB257_5*s->dpt[1][67]+FB258_5*s->dpt[1][68]+FB259_5*
 s->dpt[1][69]+OpM353_5*(s->In[9][54]+s->In[9][59])-s->dpt[2][65]*(FB155_5*C55+FB355_5*S55)-s->dpt[2][66]*(FB156_5*C56+
 FB356_5*S56)-s->dpt[2][67]*(FB157_5*C57+FB357_5*S57)-s->dpt[2][68]*(FB158_5*C58+FB358_5*S58);
  CM536_254 = CM255_6+CM256_6+CM257_6+CM258_6-q[54]*FM354_6+s->In[5][54]*OpM252_6+s->In[5][59]*OpM252_6+Dz593*FB159_6+
 FB154_6*s->l[3][54]-FB354_6*s->l[1][54]-FB359_6*s->dpt[1][69]+s->dpt[1][65]*(FB155_6*S55-FB355_6*C55)+s->dpt[1][66]*(FB156_6
 *S56-FB356_6*C56)+s->dpt[1][67]*(FB157_6*S57-FB357_6*C57)+s->dpt[1][68]*(FB158_6*S58-FB358_6*C58)+s->dpt[3][65]*(FB155_6*C55
 +FB355_6*S55)+s->dpt[3][66]*(FB156_6*C56+FB356_6*S56)+s->dpt[3][67]*(FB157_6*C57+FB357_6*S57)+s->dpt[3][68]*(FB158_6*C58+
 FB358_6*S58);
  CM536_354 = q[54]*FM254_6-CM155_6*S55-CM156_6*S56-CM157_6*S57-CM158_6*S58+CM355_6*C55+CM356_6*C56+CM357_6*C57+CM358_6*
 C58+FB254_6*s->l[1][54]+FB255_6*s->dpt[1][65]+FB256_6*s->dpt[1][66]+FB257_6*s->dpt[1][67]+FB258_6*s->dpt[1][68]+FB259_6*
 s->dpt[1][69]+OpM353_6*(s->In[9][54]+s->In[9][59])-s->dpt[2][65]*(FB155_6*C55+FB355_6*S55)-s->dpt[2][66]*(FB156_6*C56+
 FB356_6*S56)-s->dpt[2][67]*(FB157_6*C57+FB357_6*S57)-s->dpt[2][68]*(FB158_6*C58+FB358_6*S58);
  CM5351_254 = CM255_51+CM256_51+CM257_51+CM258_51-q[54]*(FB354_51+FB359_51-FB155_51*S55-FB156_51*S56-FB157_51*S57-
 FB158_51*S58+FB355_51*C55+FB356_51*C56+FB357_51*C57+FB358_51*C58)+s->In[5][54]*S52+s->In[5][59]*S52+Dz593*FB159_51+FB154_51*
 s->l[3][54]-FB354_51*s->l[1][54]-FB359_51*s->dpt[1][69]+s->dpt[1][65]*(FB155_51*S55-FB355_51*C55)+s->dpt[1][66]*(FB156_51*
 S56-FB356_51*C56)+s->dpt[1][67]*(FB157_51*S57-FB357_51*C57)+s->dpt[1][68]*(FB158_51*S58-FB358_51*C58)+s->dpt[3][65]*(
 FB155_51*C55+FB355_51*S55)+s->dpt[3][66]*(FB156_51*C56+FB356_51*S56)+s->dpt[3][67]*(FB157_51*C57+FB357_51*S57)+s->dpt[3][68]
 *(FB158_51*C58+FB358_51*S58);
  CM5351_354 = q[54]*(FB254_51+FB255_51+FB256_51+FB257_51+FB258_51+FB259_51)-CM155_51*S55-CM156_51*S56-CM157_51*S57-
 CM158_51*S58+CM355_51*C55+CM356_51*C56+CM357_51*C57+CM358_51*C58+FB254_51*s->l[1][54]+FB255_51*s->dpt[1][65]+FB256_51*
 s->dpt[1][66]+FB257_51*s->dpt[1][67]+FB258_51*s->dpt[1][68]+FB259_51*s->dpt[1][69]+OpM353_51*(s->In[9][54]+s->In[9][59])-
 s->dpt[2][65]*(FB155_51*C55+FB355_51*S55)-s->dpt[2][66]*(FB156_51*C56+FB356_51*S56)-s->dpt[2][67]*(FB157_51*C57+FB357_51*S57
 )-s->dpt[2][68]*(FB158_51*C58+FB358_51*S58);
  CM5352_254 = -(q[54]*C53*(s->m[55]*s->dpt[2][65]+s->m[56]*s->dpt[2][66]+s->m[57]*s->dpt[2][67]+s->m[58]*s->dpt[2][68])
 +s->m[55]*s->dpt[2][65]*(s->dpt[1][65]*C53+s->dpt[3][65]*S53)+s->m[56]*s->dpt[2][66]*(s->dpt[1][66]*C53+s->dpt[3][66]*S53)+
 s->m[57]*s->dpt[2][67]*(s->dpt[1][67]*C53+s->dpt[3][67]*S53)+s->m[58]*s->dpt[2][68]*(s->dpt[1][68]*C53+s->dpt[3][68]*S53));
  CM5353_254 = s->In[5][54]+s->In[5][55]+s->In[5][56]+s->In[5][57]+s->In[5][58]+s->In[5][59]-q[54]*(FB354_53+FB359_53-
 FB155_53*S55-FB156_53*S56-FB157_53*S57-FB158_53*S58+FB355_53*C55+FB356_53*C56+FB357_53*C57+FB358_53*C58)+Dz593*FB159_53+
 FB154_53*s->l[3][54]-FB354_53*s->l[1][54]-FB359_53*s->dpt[1][69]+s->dpt[1][65]*(FB155_53*S55-FB355_53*C55)+s->dpt[1][66]*(
 FB156_53*S56-FB356_53*C56)+s->dpt[1][67]*(FB157_53*S57-FB357_53*C57)+s->dpt[1][68]*(FB158_53*S58-FB358_53*C58)+s->dpt[3][65]
 *(FB155_53*C55+FB355_53*S55)+s->dpt[3][66]*(FB156_53*C56+FB356_53*S56)+s->dpt[3][67]*(FB157_53*C57+FB357_53*S57)+
 s->dpt[3][68]*(FB158_53*C58+FB358_53*S58);
  FF52_153 = FF154*C53+FF354*S53;
  FF52_353 = -(FF154*S53-FF354*C53);
  CF52_153 = CF154*C53+CF53_354*S53;
  CF52_353 = -(CF154*S53-CF53_354*C53);
  FM521_153 = FM154_1*C53+FM354_1*S53;
  FM521_353 = -(FM154_1*S53-FM354_1*C53);
  CM521_153 = CM154_1*C53+CM531_354*S53;
  CM521_353 = -(CM154_1*S53-CM531_354*C53);
  FM522_153 = FM154_2*C53+FM354_2*S53;
  FM522_353 = -(FM154_2*S53-FM354_2*C53);
  CM522_153 = CM154_2*C53+CM532_354*S53;
  CM522_353 = -(CM154_2*S53-CM532_354*C53);
  FM523_153 = FM154_3*C53+FM354_3*S53;
  FM523_353 = -(FM154_3*S53-FM354_3*C53);
  CM523_153 = CM154_3*C53+CM533_354*S53;
  CM523_353 = -(CM154_3*S53-CM533_354*C53);
  FM524_153 = FM154_4*C53+FM354_4*S53;
  FM524_353 = -(FM154_4*S53-FM354_4*C53);
  CM524_153 = CM154_4*C53+CM534_354*S53;
  CM524_353 = -(CM154_4*S53-CM534_354*C53);
  FM525_153 = FM154_5*C53+FM354_5*S53;
  FM525_353 = -(FM154_5*S53-FM354_5*C53);
  CM525_153 = CM154_5*C53+CM535_354*S53;
  CM525_353 = -(CM154_5*S53-CM535_354*C53);
  FM526_353 = -(FM154_6*S53-FM354_6*C53);
  CM526_153 = CM154_6*C53+CM536_354*S53;
  CM526_353 = -(CM154_6*S53-CM536_354*C53);
  CM5251_153 = CM154_51*C53+CM5351_354*S53;
  CM5252_153 = C53*(CM155_52*C55+CM156_52*C56+CM157_52*C57+CM158_52*C58+CM355_52*S55+CM356_52*S56+CM357_52*S57+CM358_52*
 S58-Dz593*FB259_52-FB254_52*s->l[3][54]-FB255_52*s->dpt[3][65]-FB256_52*s->dpt[3][66]-FB257_52*s->dpt[3][67]-FB258_52*
 s->dpt[3][68]-s->dpt[2][65]*(FB155_52*S55-FB355_52*C55)-s->dpt[2][66]*(FB156_52*S56-FB356_52*C56)-s->dpt[2][67]*(FB157_52*
 S57-FB357_52*C57)-s->dpt[2][68]*(FB158_52*S58-FB358_52*C58)+C53*(s->In[1][54]+s->In[1][59]))+S53*(q[54]*(FB254_52+FB255_52+
 FB256_52+FB257_52+FB258_52+FB259_52)-CM155_52*S55-CM156_52*S56-CM157_52*S57-CM158_52*S58+CM355_52*C55+CM356_52*C56+CM357_52*
 C57+CM358_52*C58+FB254_52*s->l[1][54]+FB255_52*s->dpt[1][65]+FB256_52*s->dpt[1][66]+FB257_52*s->dpt[1][67]+FB258_52*
 s->dpt[1][68]+FB259_52*s->dpt[1][69]-s->dpt[2][65]*(FB155_52*C55+FB355_52*S55)-s->dpt[2][66]*(FB156_52*C56+FB356_52*S56)-
 s->dpt[2][67]*(FB157_52*C57+FB357_52*S57)-s->dpt[2][68]*(FB158_52*C58+FB358_52*S58)+S53*(s->In[9][54]+s->In[9][59]));
  FF51_252 = FF254*C52-FF52_353*S52;
  FF51_352 = FF254*S52+FF52_353*C52;
  CF51_252 = -(CF52_353*S52-CF53_254*C52);
  CF51_352 = CF52_353*C52+CF53_254*S52;
  FM511_252 = FM254_1*C52-FM521_353*S52;
  FM511_352 = FM254_1*S52+FM521_353*C52;
  CM511_252 = -(CM521_353*S52-CM531_254*C52);
  CM511_352 = CM521_353*C52+CM531_254*S52;
  FM512_252 = FM254_2*C52-FM522_353*S52;
  FM512_352 = FM254_2*S52+FM522_353*C52;
  CM512_252 = -(CM522_353*S52-CM532_254*C52);
  CM512_352 = CM522_353*C52+CM532_254*S52;
  FM513_252 = FM254_3*C52-FM523_353*S52;
  FM513_352 = FM254_3*S52+FM523_353*C52;
  CM513_252 = -(CM523_353*S52-CM533_254*C52);
  CM513_352 = CM523_353*C52+CM533_254*S52;
  FM514_252 = FM254_4*C52-FM524_353*S52;
  CM514_252 = -(CM524_353*S52-CM534_254*C52);
  CM514_352 = CM524_353*C52+CM534_254*S52;
  FM515_252 = FM254_5*C52-FM525_353*S52;
  CM515_252 = -(CM525_353*S52-CM535_254*C52);
  CM515_352 = CM525_353*C52+CM535_254*S52;
  CM516_352 = CM526_353*C52+CM536_254*S52;
  CM5151_352 = CM5351_254*S52-C52*(CM154_51*S53-CM5351_354*C53);

// = = Block_0_2_0_3_0_1 = = 
 
// Backward Dynamics 

  FA16 = -(s->frc[1][6]-s->m[6]*(AlF16+BS16*s->l[1][6]+BeF36*s->l[3][6]));
  FA26 = -(s->frc[2][6]-s->m[6]*(AlF25+BeF46*s->l[1][6]+BeF66*s->l[3][6]));
  FA36 = -(s->frc[3][6]-s->m[6]*(AlF36+BS96*s->l[3][6]+BeF76*s->l[1][6]));
  FF16 = -(s->frc[1][18]+s->frc[1][19]+s->frc[1][20]+s->frc[1][21]-FA150-FA16-FA17-FF113-FF128-FF135-FF138-FF18-FF122*
 C22-FF141*C41-FF322*S22-FF341*S41+FF51_252*S51-FF52_153*C51);
  FF26 = FA26+FA27+FF222+FF228+FF241-s->frc[2][18]*C18-s->frc[2][19]*C19-s->frc[2][20]*C20-s->frc[2][21]*C21+
 s->frc[3][18]*S18+s->frc[3][19]*S19+s->frc[3][20]*S20+s->frc[3][21]*S21+FF213*C13+FF235*C35+FF238*C38+FF28*C8-FF313*S13-
 FF335*S35-FF338*S38-FF38*S8+FF47_248*C47-FF47_348*S47+FF51_252*C51+FF52_153*S51;
  FF36 = FA36+FA37+FF328+FF51_352-s->frc[2][18]*S18-s->frc[2][19]*S19-s->frc[2][20]*S20-s->frc[2][21]*S21-s->frc[3][18]*
 C18-s->frc[3][19]*C19-s->frc[3][20]*C20-s->frc[3][21]*C21-FF122*S22-FF141*S41+FF213*S13+FF235*S35+FF238*S38+FF28*S8+FF313*
 C13+FF322*C22+FF335*C35+FF338*C38+FF341*C41+FF38*C8+FF47_248*S47+FF47_348*C47;
  CF16 = -(s->trq[1][18]+s->trq[1][19]+s->trq[1][20]+s->trq[1][21]+s->trq[1][28]+s->trq[1][6]+s->trq[1][7]-CF113-CF135-
 CF138-CF150-CF18-CF29_130-CF31_132-q[28]*FF328-s->In[1][6]*OpF16-CF122*C22-CF134*C34-CF141*C41+CF234*S34-CF322*S22-CF341*S41
 +CF51_252*S51-CF52_153*C51-Dz332*FA334+FA26*s->l[3][6]+FF222*s->dpt[3][8]+FF228*s->dpt[3][9]+FF241*s->dpt[3][15]+OM26*OM36*(
 s->In[5][6]-s->In[9][6])-s->dpt[2][12]*(FF235*S35+FF335*C35)-s->dpt[2][13]*(FF238*S38+FF338*C38)-s->dpt[2][17]*(FF47_248*S47
 +FF47_348*C47)-s->dpt[2][1]*(FF28*S8+FF38*C8)-s->dpt[2][2]*(FF213*S13+FF313*C13)+s->dpt[2][3]*(s->frc[2][18]*S18+
 s->frc[3][18]*C18)+s->dpt[2][45]*(s->frc[3][30]*C29-FF29_230*S29)+s->dpt[2][46]*(s->frc[3][32]*C31-FF31_232*S31)+
 s->dpt[2][4]*(s->frc[2][19]*S19+s->frc[3][19]*C19)+s->dpt[2][5]*(s->frc[2][20]*S20+s->frc[3][20]*C20)+s->dpt[2][7]*(
 s->frc[2][21]*S21+s->frc[3][21]*C21)+s->dpt[3][12]*(FF235*C35-FF335*S35)+s->dpt[3][13]*(FF238*C38-FF338*S38)+s->dpt[3][17]*(
 FF47_248*C47-FF47_348*S47)+s->dpt[3][18]*(FF51_252*C51+FF52_153*S51)+s->dpt[3][1]*(FF28*C8-FF38*S8)+s->dpt[3][2]*(FF213*C13-
 FF313*S13));
  CF26 = -(s->trq[2][28]+s->trq[2][6]+s->trq[2][7]-CF222-CF241+q[7]*FA37-s->In[5][6]*OpF25+s->trq[2][18]*C18+
 s->trq[2][19]*C19+s->trq[2][20]*C20+s->trq[2][21]*C21-s->trq[3][18]*S18-s->trq[3][19]*S19-s->trq[3][20]*S20-s->trq[3][21]*
 S21-s->trq[3][30]*S29-s->trq[3][32]*S31-CF134*S34-CF213*C13-CF234*C34-CF235*C35-CF238*C38-CF28*C8-CF29_230*C29+CF313*S13-
 CF31_232*C31+CF335*S35+CF338*S38+CF38*S8-CF47_248*C47+CF47_348*S47-CF51_252*C51-CF52_153*S51-FA150*s->dpt[3][17]-FA16*
 s->l[3][6]+FA36*s->l[1][6]-FF113*s->dpt[3][2]-FF128*s->dpt[3][9]-FF135*s->dpt[3][12]-FF138*s->dpt[3][13]-FF18*s->dpt[3][1]+
 FF328*s->dpt[1][9]+FF51_352*s->dpt[1][18]-OM16*OM36*(s->In[1][6]-s->In[9][6])+s->dpt[1][12]*(FF235*S35+FF335*C35)+
 s->dpt[1][13]*(FF238*S38+FF338*C38)-s->dpt[1][15]*(FF141*S41-FF341*C41)+s->dpt[1][17]*(FF47_248*S47+FF47_348*C47)+
 s->dpt[1][1]*(FF28*S8+FF38*C8)+s->dpt[1][2]*(FF213*S13+FF313*C13)-s->dpt[1][5]*(s->frc[2][20]*S20+s->frc[3][20]*C20)-
 s->dpt[1][7]*(s->frc[2][21]*S21+s->frc[3][21]*C21)-s->dpt[1][8]*(FF122*S22-FF322*C22)-s->dpt[3][15]*(FF141*C41+FF341*S41)+
 s->dpt[3][18]*(FF51_252*S51-FF52_153*C51)-s->dpt[3][8]*(FF122*C22+FF322*S22));
  CF36 = -(s->trq[3][28]+s->trq[3][6]+s->trq[3][7]-CF334-CF51_352+q[28]*FF128-q[7]*FA27-s->In[9][6]*OpF36-s->frc[1][18]*
 s->dpt[2][3]-s->frc[1][19]*s->dpt[2][4]-s->frc[1][20]*s->dpt[2][5]-s->frc[1][21]*s->dpt[2][7]+s->trq[2][18]*S18+
 s->trq[2][19]*S19+s->trq[2][20]*S20+s->trq[2][21]*S21+s->trq[3][18]*C18+s->trq[3][19]*C19+s->trq[3][20]*C20+s->trq[3][21]*
 C21+s->trq[3][30]*C29+s->trq[3][32]*C31+CF122*S22+CF141*S41-CF213*S13-CF235*S35-CF238*S38-CF28*S8-CF29_230*S29-CF313*C13-
 CF31_232*S31-CF322*C22-CF335*C35-CF338*C38-CF341*C41-CF38*C8-CF47_248*S47-CF47_348*C47+Dz332*FF33_134+FA150*s->dpt[2][17]-
 FA26*s->l[1][6]+FF113*s->dpt[2][2]+FF135*s->dpt[2][12]+FF138*s->dpt[2][13]+FF18*s->dpt[2][1]-FF222*s->dpt[1][8]-FF228*
 s->dpt[1][9]-FF241*s->dpt[1][15]+FF29_130*s->dpt[2][45]+FF31_132*s->dpt[2][46]+OM16*OM26*(s->In[1][6]-s->In[5][6])-
 s->dpt[1][12]*(FF235*C35-FF335*S35)-s->dpt[1][13]*(FF238*C38-FF338*S38)-s->dpt[1][17]*(FF47_248*C47-FF47_348*S47)-
 s->dpt[1][18]*(FF51_252*C51+FF52_153*S51)-s->dpt[1][1]*(FF28*C8-FF38*S8)-s->dpt[1][2]*(FF213*C13-FF313*S13)+s->dpt[1][5]*(
 s->frc[2][20]*C20-s->frc[3][20]*S20)+s->dpt[1][7]*(s->frc[2][21]*C21-s->frc[3][21]*S21));
  FB16_1 = s->m[6]*AlM16_1;
  FB26_1 = s->m[6]*AlM25_1;
  FB36_1 = s->m[6]*AlM36_1;
  FM16_1 = FB150_1+FB16_1+FB17_1+FM128_1+FM131_114+FM136_1+FM139_1+FM81_19-FM511_252*S51+FM521_153*C51;
  FM26_1 = FB26_1+FB27_1+FM228_1-FM131_314*S13+FM141_215*C13+FM351_236*C35-FM351_336*S35+FM381_239*C38-FM381_339*S38+
 FM471_248*C47-FM471_348*S47+FM511_252*C51+FM521_153*S51-FM81_39*S8+FM91_210*C8;
  FM36_1 = FB36_1+FB37_1+FM328_1+FM511_352+FM131_314*C13+FM141_215*S13+FM351_236*S35+FM351_336*C35+FM381_239*S38+
 FM381_339*C38+FM471_248*S47+FM471_348*C47+FM81_39*C8+FM91_210*S8;
  CM16_1 = CM131_114+CM351_136+CM381_139+CM81_19+q[28]*FM328_1-CM511_252*S51+CM521_153*C51+Dz332*FB334_1-FB26_1*
 s->l[3][6]-FM228_1*s->dpt[3][9]+s->dpt[2][12]*(FM351_236*S35+FM351_336*C35)+s->dpt[2][13]*(FM381_239*S38+FM381_339*C38)+
 s->dpt[2][17]*(FM471_248*S47+FM471_348*C47)+s->dpt[2][1]*(FM81_39*C8+FM91_210*S8)+s->dpt[2][2]*(FM131_314*C13+FM141_215*S13)
 -s->dpt[3][12]*(FM351_236*C35-FM351_336*S35)-s->dpt[3][13]*(FM381_239*C38-FM381_339*S38)-s->dpt[3][17]*(FM471_248*C47-
 FM471_348*S47)-s->dpt[3][18]*(FM511_252*C51+FM521_153*S51)+s->dpt[3][1]*(FM81_39*S8-FM91_210*C8)+s->dpt[3][2]*(FM131_314*S13
 -FM141_215*C13);
  CM26_1 = CM511_252*C51+CM521_153*S51-FM511_352*s->dpt[1][18]-s->dpt[3][18]*(FM511_252*S51-FM521_153*C51)-q[7]*FB37_1-
 CM131_314*S13+CM141_215*C13+CM351_236*C35-CM351_336*S35+CM381_239*C38-CM381_339*S38-CM81_39*S8+CM91_210*C8+FB150_1*
 s->dpt[3][17]+FB16_1*s->l[3][6]-FB36_1*s->l[1][6]+FM128_1*s->dpt[3][9]+FM131_114*s->dpt[3][2]+FM136_1*s->dpt[3][12]+FM139_1*
 s->dpt[3][13]-FM328_1*s->dpt[1][9]+FM81_19*s->dpt[3][1]-s->dpt[1][12]*(FM351_236*S35+FM351_336*C35)-s->dpt[1][13]*(FM381_239
 *S38+FM381_339*C38)-s->dpt[1][17]*(FM471_248*S47+FM471_348*C47)-s->dpt[1][1]*(FM81_39*C8+FM91_210*S8)-s->dpt[1][2]*(
 FM131_314*C13+FM141_215*S13);
  CM36_1 = CM511_352-q[28]*FM128_1+q[7]*FB27_1+CM131_314*C13+CM141_215*S13+CM351_236*S35+CM351_336*C35+CM381_239*S38+
 CM381_339*C38+CM81_39*C8+CM91_210*S8-Dz332*FM331_134-FB150_1*s->dpt[2][17]+FB26_1*s->l[1][6]-FM131_114*s->dpt[2][2]-FM136_1*
 s->dpt[2][12]-FM139_1*s->dpt[2][13]+FM228_1*s->dpt[1][9]-FM81_19*s->dpt[2][1]+s->dpt[1][12]*(FM351_236*C35-FM351_336*S35)+
 s->dpt[1][13]*(FM381_239*C38-FM381_339*S38)+s->dpt[1][17]*(FM471_248*C47-FM471_348*S47)+s->dpt[1][18]*(FM511_252*C51+
 FM521_153*S51)-s->dpt[1][1]*(FM81_39*S8-FM91_210*C8)-s->dpt[1][2]*(FM131_314*S13-FM141_215*C13);
  FB16_2 = s->m[6]*AlM16_2;
  FB26_2 = s->m[6]*AlM25_2;
  FB36_2 = s->m[6]*AlM36_2;
  FM16_2 = FB150_2+FB16_2+FB17_2+FM128_2+FM132_114+FM136_2+FM139_2+FM82_19-FM512_252*S51+FM522_153*C51;
  FM26_2 = FB26_2+FB27_2+FM228_2-FM132_314*S13+FM142_215*C13+FM352_236*C35-FM352_336*S35+FM382_239*C38-FM382_339*S38+
 FM472_248*C47-FM472_348*S47+FM512_252*C51+FM522_153*S51-FM82_39*S8+FM92_210*C8;
  FM36_2 = FB36_2+FB37_2+FM328_2+FM512_352+FM132_314*C13+FM142_215*S13+FM352_236*S35+FM352_336*C35+FM382_239*S38+
 FM382_339*C38+FM472_248*S47+FM472_348*C47+FM82_39*C8+FM92_210*S8;
  CM16_2 = CM132_114+CM352_136+CM382_139+CM82_19+q[28]*FM328_2-CM512_252*S51+CM522_153*C51+Dz332*FB334_2-FB26_2*
 s->l[3][6]-FM228_2*s->dpt[3][9]+s->dpt[2][12]*(FM352_236*S35+FM352_336*C35)+s->dpt[2][13]*(FM382_239*S38+FM382_339*C38)+
 s->dpt[2][17]*(FM472_248*S47+FM472_348*C47)+s->dpt[2][1]*(FM82_39*C8+FM92_210*S8)+s->dpt[2][2]*(FM132_314*C13+FM142_215*S13)
 -s->dpt[3][12]*(FM352_236*C35-FM352_336*S35)-s->dpt[3][13]*(FM382_239*C38-FM382_339*S38)-s->dpt[3][17]*(FM472_248*C47-
 FM472_348*S47)-s->dpt[3][18]*(FM512_252*C51+FM522_153*S51)+s->dpt[3][1]*(FM82_39*S8-FM92_210*C8)+s->dpt[3][2]*(FM132_314*S13
 -FM142_215*C13);
  CM26_2 = CM512_252*C51+CM522_153*S51-FM512_352*s->dpt[1][18]-s->dpt[3][18]*(FM512_252*S51-FM522_153*C51)-q[7]*FB37_2-
 CM132_314*S13+CM142_215*C13+CM352_236*C35-CM352_336*S35+CM382_239*C38-CM382_339*S38-CM82_39*S8+CM92_210*C8+FB150_2*
 s->dpt[3][17]+FB16_2*s->l[3][6]-FB36_2*s->l[1][6]+FM128_2*s->dpt[3][9]+FM132_114*s->dpt[3][2]+FM136_2*s->dpt[3][12]+FM139_2*
 s->dpt[3][13]-FM328_2*s->dpt[1][9]+FM82_19*s->dpt[3][1]-s->dpt[1][12]*(FM352_236*S35+FM352_336*C35)-s->dpt[1][13]*(FM382_239
 *S38+FM382_339*C38)-s->dpt[1][17]*(FM472_248*S47+FM472_348*C47)-s->dpt[1][1]*(FM82_39*C8+FM92_210*S8)-s->dpt[1][2]*(
 FM132_314*C13+FM142_215*S13);
  CM36_2 = CM512_352-q[28]*FM128_2+q[7]*FB27_2+CM132_314*C13+CM142_215*S13+CM352_236*S35+CM352_336*C35+CM382_239*S38+
 CM382_339*C38+CM82_39*C8+CM92_210*S8-Dz332*FM332_134-FB150_2*s->dpt[2][17]+FB26_2*s->l[1][6]-FM132_114*s->dpt[2][2]-FM136_2*
 s->dpt[2][12]-FM139_2*s->dpt[2][13]+FM228_2*s->dpt[1][9]-FM82_19*s->dpt[2][1]+s->dpt[1][12]*(FM352_236*C35-FM352_336*S35)+
 s->dpt[1][13]*(FM382_239*C38-FM382_339*S38)+s->dpt[1][17]*(FM472_248*C47-FM472_348*S47)+s->dpt[1][18]*(FM512_252*C51+
 FM522_153*S51)-s->dpt[1][1]*(FM82_39*S8-FM92_210*C8)-s->dpt[1][2]*(FM132_314*S13-FM142_215*C13);
  FB16_3 = s->m[6]*AlM16_3;
  FB26_3 = s->m[6]*S5;
  FB36_3 = s->m[6]*AlM36_3;
  CM16_3 = CM133_114+CM353_136+CM383_139+CM83_19+q[28]*FM328_3-CM513_252*S51+CM523_153*C51+Dz332*FB334_3-FB26_3*
 s->l[3][6]-FM228_3*s->dpt[3][9]+s->dpt[2][12]*(FM353_236*S35+FM353_336*C35)+s->dpt[2][13]*(FM383_239*S38+FM383_339*C38)+
 s->dpt[2][17]*(FM473_248*S47+FM473_348*C47)+s->dpt[2][1]*(FM83_39*C8+FM93_210*S8)+s->dpt[2][2]*(FM133_314*C13+FM143_215*S13)
 -s->dpt[3][12]*(FM353_236*C35-FM353_336*S35)-s->dpt[3][13]*(FM383_239*C38-FM383_339*S38)-s->dpt[3][17]*(FM473_248*C47-
 FM473_348*S47)-s->dpt[3][18]*(FM513_252*C51+FM523_153*S51)+s->dpt[3][1]*(FM83_39*S8-FM93_210*C8)+s->dpt[3][2]*(FM133_314*S13
 -FM143_215*C13);
  CM26_3 = CM513_252*C51+CM523_153*S51-FM513_352*s->dpt[1][18]-s->dpt[3][18]*(FM513_252*S51-FM523_153*C51)-q[7]*FB37_3-
 CM133_314*S13+CM143_215*C13+CM353_236*C35-CM353_336*S35+CM383_239*C38-CM383_339*S38-CM83_39*S8+CM93_210*C8+FB150_3*
 s->dpt[3][17]+FB16_3*s->l[3][6]-FB36_3*s->l[1][6]+FM128_3*s->dpt[3][9]+FM133_114*s->dpt[3][2]+FM136_3*s->dpt[3][12]+FM139_3*
 s->dpt[3][13]-FM328_3*s->dpt[1][9]+FM83_19*s->dpt[3][1]-s->dpt[1][12]*(FM353_236*S35+FM353_336*C35)-s->dpt[1][13]*(FM383_239
 *S38+FM383_339*C38)-s->dpt[1][17]*(FM473_248*S47+FM473_348*C47)-s->dpt[1][1]*(FM83_39*C8+FM93_210*S8)-s->dpt[1][2]*(
 FM133_314*C13+FM143_215*S13);
  CM36_3 = CM513_352-q[28]*FM128_3+q[7]*FB27_3+CM133_314*C13+CM143_215*S13+CM353_236*S35+CM353_336*C35+CM383_239*S38+
 CM383_339*C38+CM83_39*C8+CM93_210*S8-Dz332*FM333_134-FB150_3*s->dpt[2][17]+FB26_3*s->l[1][6]-FM133_114*s->dpt[2][2]-FM136_3*
 s->dpt[2][12]-FM139_3*s->dpt[2][13]+FM228_3*s->dpt[1][9]-FM83_19*s->dpt[2][1]+s->dpt[1][12]*(FM353_236*C35-FM353_336*S35)+
 s->dpt[1][13]*(FM383_239*C38-FM383_339*S38)+s->dpt[1][17]*(FM473_248*C47-FM473_348*S47)+s->dpt[1][18]*(FM513_252*C51+
 FM523_153*S51)-s->dpt[1][1]*(FM83_39*S8-FM93_210*C8)-s->dpt[1][2]*(FM133_314*S13-FM143_215*C13);
  FB26_4 = -s->m[6]*(OpM16_4*s->l[3][6]-OpM36_4*s->l[1][6]);
  CM16_4 = CM134_114+CM150_4+CM354_136+CM384_139+CM84_19+q[28]*FM328_4+s->In[1][6]*OpM16_4+CM134_4*C34-CM234_4*S34-
 CM514_252*S51+CM524_153*C51+Dz332*FB334_4-FB26_4*s->l[3][6]-FM228_4*s->dpt[3][9]+s->dpt[2][12]*(FM354_236*S35+FM354_336*C35)
 +s->dpt[2][13]*(FM384_239*S38+FM384_339*C38)+s->dpt[2][17]*(FM474_248*S47+FM474_348*C47)+s->dpt[2][1]*(FM84_39*C8+FM94_210*
 S8)+s->dpt[2][2]*(FM134_314*C13+FM144_215*S13)-s->dpt[3][12]*(FM354_236*C35-FM354_336*S35)-s->dpt[3][13]*(FM384_239*C38-
 FM384_339*S38)-s->dpt[3][17]*(FM474_248*C47-FM474_348*S47)-s->dpt[3][18]*(FM514_252*C51+FM524_153*S51)+s->dpt[3][1]*(FM84_39
 *S8-FM94_210*C8)+s->dpt[3][2]*(FM134_314*S13-FM144_215*C13);
  CM26_4 = s->m[6]*S5*(s->l[1][6]*s->l[1][6]+s->l[3][6]*s->l[3][6])-CM134_314*S13+CM134_4*S34+CM144_215*C13+CM234_4*C34+
 CM354_236*C35-CM354_336*S35+CM384_239*C38-CM384_339*S38+CM474_248*C47-CM474_348*S47+CM514_252*C51+CM524_153*S51-CM84_39*S8+
 CM94_210*C8+FB150_4*s->dpt[3][17]+FM128_4*s->dpt[3][9]+FM134_114*s->dpt[3][2]+FM136_4*s->dpt[3][12]+FM139_4*s->dpt[3][13]-
 FM328_4*s->dpt[1][9]+FM84_19*s->dpt[3][1]-s->dpt[1][12]*(FM354_236*S35+FM354_336*C35)-s->dpt[1][13]*(FM384_239*S38+FM384_339
 *C38)-s->dpt[1][17]*(FM474_248*S47+FM474_348*C47)-s->dpt[1][18]*(FM254_4*S52+FM524_353*C52)-s->dpt[1][1]*(FM84_39*C8+
 FM94_210*S8)-s->dpt[1][2]*(FM134_314*C13+FM144_215*S13)-s->dpt[3][18]*(FM514_252*S51-FM524_153*C51)+S5*(s->In[5][6]+q[7]*
 q[7]*s->m[7]);
  CM36_4 = CM334_4+CM514_352-q[28]*FM128_4+q[7]*q[7]*s->m[7]*OpM36_4+s->In[9][6]*OpM36_4+CM134_314*C13+CM144_215*S13+
 CM354_236*S35+CM354_336*C35+CM384_239*S38+CM384_339*C38+CM474_248*S47+CM474_348*C47+CM84_39*C8+CM94_210*S8-Dz332*FM334_134-
 FB150_4*s->dpt[2][17]+FB26_4*s->l[1][6]-FM134_114*s->dpt[2][2]-FM136_4*s->dpt[2][12]-FM139_4*s->dpt[2][13]+FM228_4*
 s->dpt[1][9]-FM84_19*s->dpt[2][1]+s->dpt[1][12]*(FM354_236*C35-FM354_336*S35)+s->dpt[1][13]*(FM384_239*C38-FM384_339*S38)+
 s->dpt[1][17]*(FM474_248*C47-FM474_348*S47)+s->dpt[1][18]*(FM514_252*C51+FM524_153*S51)-s->dpt[1][1]*(FM84_39*S8-FM94_210*C8
 )-s->dpt[1][2]*(FM134_314*S13-FM144_215*C13);
  FB26_5 = s->m[6]*(s->l[1][6]*S6-s->l[3][6]*C6);
  CM26_5 = CM134_5*S34-CM135_314*S13+CM145_215*C13+CM234_5*C34+CM355_236*C35-CM355_336*S35+CM385_239*C38-CM385_339*S38+
 CM475_248*C47-CM475_348*S47+CM515_252*C51+CM525_153*S51-CM85_39*S8+CM95_210*C8+FB150_5*s->dpt[3][17]+FM128_5*s->dpt[3][9]+
 FM135_114*s->dpt[3][2]+FM136_5*s->dpt[3][12]+FM139_5*s->dpt[3][13]-FM328_5*s->dpt[1][9]+FM85_19*s->dpt[3][1]-s->dpt[1][12]*(
 FM355_236*S35+FM355_336*C35)-s->dpt[1][13]*(FM385_239*S38+FM385_339*C38)-s->dpt[1][17]*(FM475_248*S47+FM475_348*C47)-
 s->dpt[1][18]*(FM254_5*S52+FM525_353*C52)-s->dpt[1][1]*(FM85_39*C8+FM95_210*S8)-s->dpt[1][2]*(FM135_314*C13+FM145_215*S13)-
 s->dpt[3][18]*(FM515_252*S51-FM525_153*C51);
  CM26_6 = s->In[5][6]+q[7]*q[7]*s->m[7]+s->In[1][34]*S34*S34+s->In[5][34]*C34*C34+s->m[50]*(s->dpt[1][17]*s->dpt[1][17]
 +s->dpt[3][17]*s->dpt[3][17])+s->m[6]*s->l[1][6]*s->l[1][6]+s->m[6]*s->l[3][6]*s->l[3][6]+CM146_215*C13+CM526_153*S51+
 CM96_210*C8+FM136_6*s->dpt[3][12]+FM139_6*s->dpt[3][13]-s->dpt[1][12]*(C35*(FM236_6*S36+FM336_6*C36)+S35*(FM236_6*C36-
 FM336_6*S36))-s->dpt[1][13]*(C38*(FM239_6*S39+FM339_6*C39)+S38*(FM239_6*C39-FM339_6*S39))-s->dpt[1][18]*(FM254_6*S52+
 FM526_353*C52)+s->dpt[1][1]*(C8*(FM106_111*S9-FM96_310*C9)-S8*(FM106_211*C10-FM311_6*S10))-s->dpt[1][2]*(C13*(FM146_315*C14-
 FM156_116*S14)+S13*(FM156_216*C15-FM316_6*S15))+s->dpt[1][9]*s->dpt[1][9]*(s->m[28]+s->m[34])+s->dpt[3][18]*(C51*(FM154_6*
 C53+FM354_6*S53)-S51*(FM254_6*C52-FM526_353*S52))+s->dpt[3][1]*(FM106_111*C9+FM96_310*S9)+s->dpt[3][2]*(FM146_315*S14+
 FM156_116*C14)+s->dpt[3][9]*s->dpt[3][9]*(s->m[28]+s->m[34])-S13*(CM146_315*C14-CM156_116*S14-s->dpt[2][26]*(FM146_315*S14+
 FM156_116*C14))+C35*(CM366_237*C36-CM366_337*S36)-S35*(CM366_237*S36+CM366_337*C36-FM136_6*s->dpt[2][50])+C38*(CM396_240*C39
 -CM396_340*S39)-S38*(CM396_240*S39+CM396_340*C39-FM139_6*s->dpt[2][55])+C47*(CM486_249*C48-CM486_349*S48)-S47*(CM486_249*S48
 +CM486_349*C48)-C51*(CM526_353*S52-CM536_254*C52)+S8*(CM106_111*S9-CM96_310*C9+s->dpt[2][19]*(FM106_111*C9+FM96_310*S9));
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
  CM55_16 = C6*(CM135_114+CM150_5+CM355_136+CM385_139+CM85_19+q[28]*FM328_5+s->In[1][6]*C6+CM134_5*C34-CM234_5*S34-
 CM515_252*S51+CM525_153*C51+Dz332*FB334_5-FB26_5*s->l[3][6]-FM228_5*s->dpt[3][9]+s->dpt[2][12]*(FM355_236*S35+FM355_336*C35)
 +s->dpt[2][13]*(FM385_239*S38+FM385_339*C38)+s->dpt[2][17]*(FM475_248*S47+FM475_348*C47)+s->dpt[2][1]*(FM85_39*C8+FM95_210*
 S8)+s->dpt[2][2]*(FM135_314*C13+FM145_215*S13)-s->dpt[3][12]*(FM355_236*C35-FM355_336*S35)-s->dpt[3][13]*(FM385_239*C38-
 FM385_339*S38)-s->dpt[3][17]*(FM475_248*C47-FM475_348*S47)-s->dpt[3][18]*(FM515_252*C51+FM525_153*S51)+s->dpt[3][1]*(FM85_39
 *S8-FM95_210*C8)+s->dpt[3][2]*(FM135_314*S13-FM145_215*C13))+S6*(CM334_5+CM515_352-q[28]*FM128_5+q[7]*q[7]*s->m[7]*S6+
 s->In[9][6]*S6+CM135_314*C13+CM145_215*S13+CM355_236*S35+CM355_336*C35+CM385_239*S38+CM385_339*C38+CM475_248*S47+CM475_348*
 C47+CM85_39*C8+CM95_210*S8-Dz332*FM335_134-FB150_5*s->dpt[2][17]+FB26_5*s->l[1][6]-FM135_114*s->dpt[2][2]-FM136_5*
 s->dpt[2][12]-FM139_5*s->dpt[2][13]+FM228_5*s->dpt[1][9]-FM85_19*s->dpt[2][1]+s->dpt[1][12]*(FM355_236*C35-FM355_336*S35)+
 s->dpt[1][13]*(FM385_239*C38-FM385_339*S38)+s->dpt[1][17]*(FM475_248*C47-FM475_348*S47)+s->dpt[1][18]*(FM515_252*C51+
 FM525_153*S51)-s->dpt[1][1]*(FM85_39*S8-FM95_210*C8)-s->dpt[1][2]*(FM135_314*S13-FM145_215*C13));
  FF4_25 = FF26*C5-FF5_36*S5;
  FF4_35 = FF26*S5+FF5_36*C5;
  CF4_35 = CF26*S5-C5*(CF16*S6-CF36*C6);
  FM41_25 = FM26_1*C5-FM51_36*S5;
  FM41_35 = FM26_1*S5+FM51_36*C5;
  CM41_35 = CM26_1*S5-C5*(CM16_1*S6-CM36_1*C6);
  FM42_35 = FM26_2*S5+FM52_36*C5;
  CM42_35 = CM26_2*S5-C5*(CM16_2*S6-CM36_2*C6);
  FM43_35 = C5*(C6*(FB36_3+FB37_3+FM328_3+FM513_352+FM133_314*C13+FM143_215*S13+FM353_236*S35+FM353_336*C35+FM383_239*
 S38+FM383_339*C38+FM473_248*S47+FM473_348*C47+FM83_39*C8+FM93_210*S8)-S6*(FB150_3+FB16_3+FB17_3+FM128_3+FM133_114+FM136_3+
 FM139_3+FM83_19-FM513_252*S51+FM523_153*C51))+S5*(FB26_3+FB27_3+FM228_3-FM133_314*S13+FM143_215*C13+FM353_236*C35-FM353_336*
 S35+FM383_239*C38-FM383_339*S38+FM473_248*C47-FM473_348*S47+FM513_252*C51+FM523_153*S51-FM83_39*S8+FM93_210*C8);
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
  M[1][28] = FM228_1;
  M[1][33] = FM331_234;
  M[1][35] = CM351_136;
  M[1][36] = CM361_137;
  M[1][38] = CM381_139;
  M[1][39] = CM391_140;
  M[1][51] = CM511_352;
  M[1][52] = CM521_153;
  M[1][53] = CM531_254;
  M[1][54] = FM154_1;
  M[1][59] = FB359_1;
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
  M[2][28] = FM228_2;
  M[2][33] = FM332_234;
  M[2][35] = CM352_136;
  M[2][36] = CM362_137;
  M[2][38] = CM382_139;
  M[2][39] = CM392_140;
  M[2][51] = CM512_352;
  M[2][52] = CM522_153;
  M[2][53] = CM532_254;
  M[2][54] = FM154_2;
  M[2][59] = FB359_2;
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
  M[3][28] = FM228_3;
  M[3][33] = FM333_234;
  M[3][35] = CM353_136;
  M[3][36] = CM363_137;
  M[3][38] = CM383_139;
  M[3][39] = CM393_140;
  M[3][51] = CM513_352;
  M[3][52] = CM523_153;
  M[3][53] = CM533_254;
  M[3][54] = FM154_3;
  M[3][59] = FB359_3;
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
  M[4][28] = FM228_4;
  M[4][33] = FM334_234;
  M[4][34] = CM334_4;
  M[4][35] = CM354_136;
  M[4][36] = CM364_137;
  M[4][37] = CM237_4;
  M[4][38] = CM384_139;
  M[4][39] = CM394_140;
  M[4][40] = CM240_4;
  M[4][47] = CM150_4;
  M[4][48] = CM150_4;
  M[4][49] = CM150_4;
  M[4][50] = CM150_4;
  M[4][51] = CM514_352;
  M[4][52] = CM524_153;
  M[4][53] = CM534_254;
  M[4][54] = FM154_4;
  M[4][55] = CM255_4;
  M[4][56] = CM256_4;
  M[4][57] = CM257_4;
  M[4][58] = CM258_4;
  M[4][59] = FB359_4;
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
  M[5][28] = FM228_5;
  M[5][33] = FM335_234;
  M[5][34] = CM334_5;
  M[5][35] = CM355_136;
  M[5][36] = CM365_137;
  M[5][37] = CM237_5;
  M[5][38] = CM385_139;
  M[5][39] = CM395_140;
  M[5][40] = CM240_5;
  M[5][47] = CM150_5;
  M[5][48] = CM150_5;
  M[5][49] = CM150_5;
  M[5][50] = CM150_5;
  M[5][51] = CM515_352;
  M[5][52] = CM525_153;
  M[5][53] = CM535_254;
  M[5][54] = FM154_5;
  M[5][55] = CM255_5;
  M[5][56] = CM256_5;
  M[5][57] = CM257_5;
  M[5][58] = CM258_5;
  M[5][59] = FB359_5;
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
  M[6][35] = CM356_136;
  M[6][36] = CM366_137;
  M[6][37] = CM237_6;
  M[6][38] = CM386_139;
  M[6][39] = CM396_140;
  M[6][40] = CM240_6;
  M[6][51] = CM516_352;
  M[6][52] = CM526_153;
  M[6][53] = CM536_254;
  M[6][54] = FM154_6;
  M[6][55] = CM255_6;
  M[6][56] = CM256_6;
  M[6][57] = CM257_6;
  M[6][58] = CM258_6;
  M[6][59] = FB359_6;
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
  M[28][1] = FM228_1;
  M[28][2] = FM228_2;
  M[28][3] = FM228_3;
  M[28][4] = FM228_4;
  M[28][5] = FM228_5;
  M[28][28] = FM228_28;
  M[28][33] = s->m[34];
  M[33][1] = FM331_234;
  M[33][2] = FM332_234;
  M[33][3] = FM333_234;
  M[33][4] = FM334_234;
  M[33][5] = FM335_234;
  M[33][28] = s->m[34];
  M[33][33] = s->m[34];
  M[34][4] = CM334_4;
  M[34][5] = CM334_5;
  M[34][34] = s->In[9][34];
  M[35][1] = CM351_136;
  M[35][2] = CM352_136;
  M[35][3] = CM353_136;
  M[35][4] = CM354_136;
  M[35][5] = CM355_136;
  M[35][6] = CM356_136;
  M[35][35] = CM3535_136;
  M[35][36] = CM3635_137;
  M[36][1] = CM361_137;
  M[36][2] = CM362_137;
  M[36][3] = CM363_137;
  M[36][4] = CM364_137;
  M[36][5] = CM365_137;
  M[36][6] = CM366_137;
  M[36][35] = CM3635_137;
  M[36][36] = CM3636_137;
  M[37][4] = CM237_4;
  M[37][5] = CM237_5;
  M[37][6] = CM237_6;
  M[37][37] = s->In[5][37];
  M[38][1] = CM381_139;
  M[38][2] = CM382_139;
  M[38][3] = CM383_139;
  M[38][4] = CM384_139;
  M[38][5] = CM385_139;
  M[38][6] = CM386_139;
  M[38][38] = CM3838_139;
  M[38][39] = CM3938_140;
  M[39][1] = CM391_140;
  M[39][2] = CM392_140;
  M[39][3] = CM393_140;
  M[39][4] = CM394_140;
  M[39][5] = CM395_140;
  M[39][6] = CM396_140;
  M[39][38] = CM3938_140;
  M[39][39] = CM3939_140;
  M[40][4] = CM240_4;
  M[40][5] = CM240_5;
  M[40][6] = CM240_6;
  M[40][40] = s->In[5][40];
  M[47][4] = CM150_4;
  M[47][5] = CM150_5;
  M[47][47] = s->In[1][50];
  M[47][48] = s->In[1][50];
  M[47][49] = s->In[1][50];
  M[47][50] = s->In[1][50];
  M[48][4] = CM150_4;
  M[48][5] = CM150_5;
  M[48][47] = s->In[1][50];
  M[48][48] = s->In[1][50];
  M[48][49] = s->In[1][50];
  M[48][50] = s->In[1][50];
  M[49][4] = CM150_4;
  M[49][5] = CM150_5;
  M[49][47] = s->In[1][50];
  M[49][48] = s->In[1][50];
  M[49][49] = s->In[1][50];
  M[49][50] = s->In[1][50];
  M[50][4] = CM150_4;
  M[50][5] = CM150_5;
  M[50][47] = s->In[1][50];
  M[50][48] = s->In[1][50];
  M[50][49] = s->In[1][50];
  M[50][50] = s->In[1][50];
  M[51][1] = CM511_352;
  M[51][2] = CM512_352;
  M[51][3] = CM513_352;
  M[51][4] = CM514_352;
  M[51][5] = CM515_352;
  M[51][6] = CM516_352;
  M[51][51] = CM5151_352;
  M[51][52] = CM5251_153;
  M[51][53] = CM5351_254;
  M[51][54] = FM154_51;
  M[51][55] = CM255_51;
  M[51][56] = CM256_51;
  M[51][57] = CM257_51;
  M[51][58] = CM258_51;
  M[51][59] = FB359_51;
  M[52][1] = CM521_153;
  M[52][2] = CM522_153;
  M[52][3] = CM523_153;
  M[52][4] = CM524_153;
  M[52][5] = CM525_153;
  M[52][6] = CM526_153;
  M[52][51] = CM5251_153;
  M[52][52] = CM5252_153;
  M[52][53] = CM5352_254;
  M[52][54] = FM154_52;
  M[53][1] = CM531_254;
  M[53][2] = CM532_254;
  M[53][3] = CM533_254;
  M[53][4] = CM534_254;
  M[53][5] = CM535_254;
  M[53][6] = CM536_254;
  M[53][51] = CM5351_254;
  M[53][52] = CM5352_254;
  M[53][53] = CM5353_254;
  M[53][54] = FM154_53;
  M[53][55] = s->In[5][55];
  M[53][56] = s->In[5][56];
  M[53][57] = s->In[5][57];
  M[53][58] = s->In[5][58];
  M[53][59] = FB359_53;
  M[54][1] = FM154_1;
  M[54][2] = FM154_2;
  M[54][3] = FM154_3;
  M[54][4] = FM154_4;
  M[54][5] = FM154_5;
  M[54][6] = FM154_6;
  M[54][51] = FM154_51;
  M[54][52] = FM154_52;
  M[54][53] = FM154_53;
  M[54][54] = FM154_54;
  M[55][4] = CM255_4;
  M[55][5] = CM255_5;
  M[55][6] = CM255_6;
  M[55][51] = CM255_51;
  M[55][53] = s->In[5][55];
  M[55][55] = s->In[5][55];
  M[56][4] = CM256_4;
  M[56][5] = CM256_5;
  M[56][6] = CM256_6;
  M[56][51] = CM256_51;
  M[56][53] = s->In[5][56];
  M[56][56] = s->In[5][56];
  M[57][4] = CM257_4;
  M[57][5] = CM257_5;
  M[57][6] = CM257_6;
  M[57][51] = CM257_51;
  M[57][53] = s->In[5][57];
  M[57][57] = s->In[5][57];
  M[58][4] = CM258_4;
  M[58][5] = CM258_5;
  M[58][6] = CM258_6;
  M[58][51] = CM258_51;
  M[58][53] = s->In[5][58];
  M[58][58] = s->In[5][58];
  M[59][1] = FB359_1;
  M[59][2] = FB359_2;
  M[59][3] = FB359_3;
  M[59][4] = FB359_4;
  M[59][5] = FB359_5;
  M[59][6] = FB359_6;
  M[59][51] = FB359_51;
  M[59][53] = FB359_53;
  M[59][59] = s->m[59];
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
  c[33] = FF33_234;
  c[34] = CF334;
  c[35] = CF135;
  c[36] = CF136;
  c[37] = CF237;
  c[38] = CF138;
  c[39] = CF139;
  c[40] = CF240;
  c[41] = CF241;
  c[42] = CF242;
  c[43] = CF43_144;
  c[44] = -s->trq[2][44];
  c[45] = CF45_146;
  c[46] = -s->trq[2][46];
  c[47] = CF150;
  c[48] = CF150;
  c[49] = CF150;
  c[50] = CF150;
  c[51] = CF51_352;
  c[52] = CF52_153;
  c[53] = CF53_254;
  c[54] = FF154;
  c[55] = CF255;
  c[56] = CF256;
  c[57] = CF257;
  c[58] = CF258;
  c[59] = FA359;

// ====== END Task 0 ====== 


}
 

