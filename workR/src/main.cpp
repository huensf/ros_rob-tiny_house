/**
 *
 *   Universite catholique de Louvain
 *   CEREM : Centre for research in mechatronics
 *   http://www.robotran.be
 *   Contact : info@robotran.be
 *
 *
 *   MBsysC main script template for simple model:
 *   -----------------------------------------------
 *    This template loads the data file *.mbs and execute:
 *      - the coordinate partitioning module
 *      - the direct dynamic module (time integration of
 *        equations of motion).
 *    It may be adapted and completed by the user.
 *
 *    (c) Universite catholique de Louvain
 *
 * To turn this file as a C++ file, just change its extension to .cc (or .cpp).
 * If you plan to use some C++ files, it is usually better that the main is compiled as a C++ function.
 * Currently, most compilers do not require this, but it is a safer approach to port your code to other computers.
 */

#include <stdio.h>
#include "mbs_data.h"
#include "mbs_dirdyn.h"
#include "mbs_part.h"
#include "realtime.h"
#include "mbs_set.h"
#include "mbs_load_xml.h"
#include "cmake_config.h"
#include "mbs_buffer.h"
#include "mbs_equil.h"
#include "mbs_modal.h"
#include "user_all_id.h"
#include "user_model.h"

int main(int argc, char const* argv[])
{

	MbsData* mbs_data;

	MbsPart* mbs_part;
	MbsDirdyn* mbs_dirdyn;
	MbsEquil* mbs_equil;
	MbsModal* mbs_modal;


	printf("Starting Car MBS project!\n");


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                     LOADING                               *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	printf("Loading the Car data file !\n");
	mbs_data = mbs_load(PROJECT_SOURCE_DIR"/../dataR/Car.mbs", BUILD_PATH);
	printf("*.mbs file loaded!\n");
	
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
   /*                      PARAMETERS                           *
   /* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	// Hauteur centre de masse
	mbs_data->q[T3_chargement_id] = 1; //Geometrie de base: 1.5  -  Centre de masse abaisse: 1.0  

   // Position du centre de masse
   //mbs_data->dpt[1][Remorque_AP_chargement_id] = -4.8;       // centre de masse avancé       -> -3.5
   //printf("Le centre de masse du chargement a ete avance \n");

   //mbs_data->dpt[1][Remorque_AP_chargement_id] = -4.9;       // centre de masse reculé       -> -5.0
   //printf("Le centre de masse du chargement a ete recule \n");
	
   // Longueur du timon
	//double avance = 1; // si avance > 0 : longueur du timon réduite (tout a été avancé)
	//						// si avance < 0 : longueur du timon augmentee (tout a ete recule)
 //  mbs_data->dpt[1][Remorque_AP_chargement_id] = -4.38 + avance;  
 //  mbs_data->l[1][Remorque_id] = -4.581 + avance;
 //  mbs_data->dpt[1][Remorque_AP_av_g_id] = -4.559 + avance;
 //  mbs_data->dpt[1][Remorque_AP_av_d_id] = -4.559 + avance;
 //  mbs_data->dpt[1][Remorque_AP_ar_d_id] = -5.359 + avance;
 //  mbs_data->dpt[1][Remorque_AP_ar_g_id] = -5.359 + avance;
 //  printf("La longueur du timon a ete changee de %f m \n",avance);

   // Rapport des masses (remorque/voiture)
   //mbs_data->m[Chassis_1_id] = 1606;         // rapport de 2
   //printf("La masse du châssis de la voiture a ete modifiee : m = %f kg \n", mbs_data->m[Chassis_1_id]);
   //printf("Le nouveau rapport de masse (remorque/voiture) est de 1.5 \n");

   //mbs_data->m[Chargement_id] = 432.5;        // rapport de 1
   //printf("La masse du chargement a ete modifiee : m = %f kg \n", mbs_data->m[Chargement_id]);
   //printf("Le nouveau rapport de masse (remorque/voiture) est de 1 \n");


   /* Vitesse chassis en fonction du scenario */ 
   //freinage en courbe : 27.77778 [m/s] 
   //route sinueuse : 25 [m/s] 
   //evitement d'obstacle : 18 [m/s]
   double V_scenar = 18;


   // Test pour retirer l'influence de la remorque
   /*mbs_data->m[Remorque_id] = 0;
   mbs_data->m[Chargement_id] = 0;
   mbs_data->m[Roue_av_d_id] = 0;
   mbs_data->m[Roue_av_g_id] = 0;
   mbs_data->m[Roue_ar_d_id] = 0;
   mbs_data->m[Roue_ar_g_id] = 0;*/

   // Test pour raidir les suspensions
   /*mbs_data->user_model->FrontSuspension.K = 60000;
   mbs_data->user_model->RearSuspension.K = 60000;
   mbs_data->user_model->FrontSuspension.D = 12000;
   mbs_data->user_model->FrontSuspension.K = 12000;*/


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*              COORDINATE PARTITIONING                      *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	mbs_part = mbs_new_part(mbs_data);
	mbs_part->options->rowperm = 1;
	mbs_part->options->verbose = 1;
	mbs_run_part(mbs_part, mbs_data);

	mbs_delete_part(mbs_part);



	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                STATIC EQUILIBRIUM at given height         *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	mbs_data->process = 1;
	mbs_equil = mbs_new_equil(mbs_data);
	// equil options (see documentations for additional options)
	mbs_equil->options->senstol = 1e-2; //1e-2
	mbs_equil->options->devjac = 1e-2; //1e-2
	mbs_equil->options->verbose = 1;
	mbs_equil->options->resfilename = "static_1";

	mbs_data->q[R2_chassis_id] = 0.0;
	mbs_data->q[T3_chassis_id] = 0.2;
	mbs_data->q[R3_rem_id] = 0.0;

	// --- Variable exchange, quch->xch
	mbs_equil->options->nquch = 2; // nquch = nxch number of exchanged variables
	mbs_equil_exchange(mbs_equil->options); // allocates the memory
	mbs_equil->options->quch[1] = T3_chassis_id; // which free coordinate has to be replaced
	mbs_equil->options->xch_ptr[1] = &(mbs_data->user_model->FrontSuspension.L0); // which variable is the new one
	mbs_equil->options->quch[2] = R2_chassis_id;
	mbs_equil->options->xch_ptr[2] = &(mbs_data->user_model->RearSuspension.L0);

	// equilibrium procedure
	mbs_run_equil(mbs_equil, mbs_data);
	mbs_print_equil(mbs_equil);


	///* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	///*                STATIC EQUILIBRIUM set wheel orientation   *
	///* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	mbs_data->process = 1;
	mbs_equil = mbs_new_equil(mbs_data);
	// equil options (see documentations for additional options)
	mbs_equil->options->senstol = 1e-2; //1e-2
	mbs_equil->options->devjac = 1e-2; //1e-2
	mbs_equil->options->verbose = 1;
	mbs_equil->options->equitol = 1e-7;
	mbs_equil->options->resfilename = "static_2";
	// --- Variable addition, xe (straight wheels)
	mbs_equil->options->nxe = 2;
	mbs_equil_addition(mbs_equil->options);

	mbs_equil->options->xe_ptr[1] = &(mbs_data->dpt[2][44]);
	mbs_equil->options->xe_ptr[2] = &(mbs_data->dpt[2][45]);
	mbs_run_equil(mbs_equil, mbs_data);
	mbs_print_equil(mbs_equil);
	mbs_delete_equil(mbs_equil, mbs_data);


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                STRAIGHT LINE  EQUILIBRIUM                 *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	//double V = 10.0;
	//printf("-----------------Straight line equil-----------------\n");

	//mbs_data->process = 2;
	//mbs_equil = mbs_new_equil(mbs_data);
	//// equil options (see documentations for additional options)
	//mbs_equil->options->senstol = 1e-06;
	////mbs_equil->options->devjac = 1e-05;
	//mbs_equil->options->verbose = 1;
	//mbs_equil->options->mode = 2;
	//mbs_equil->options->resfilename = "sl";
	//// set a desired speed
	//mbs_data->qd[T1_chassis_id] = V; //[m/s]
	//                                     // calculation of the true height of the center of the wheel = R_wheel (0.3) - penetration
	//                                     // see in ExtForces : printf("%f\n", PxF[3]); height of the sensor
	//mbs_data->qd[R2_wheel_ft_rt_id] = V / 0.279422;
	//mbs_data->qd[R2_wheel_ft_lt_id] = V / 0.279422;
	//mbs_data->qd[R2_wheel_rr_lt_id] = V / 0.280650;
	//mbs_data->qd[R2_wheel_rr_rt_id] = V / 0.280650;
	//// --- Variable exchange, quch->xch
	//mbs_equil->options->nquch = 5; // nquch = nxch number of exchanged variables
	//mbs_equil_exchange(mbs_equil->options); // allocates the memory
	//mbs_equil->options->quch[1] = T1_chassis_id; // which free coordinate has to be replaced
	//mbs_equil->options->xch_ptr[1] = &(mbs_data->user_model->EquilQuantities.Qpropulsion); // which variable is the new one
	//mbs_equil->options->quch[2] = R2_wheel_ft_rt_id;
	//mbs_equil->options->xch_ptr[2] = &(mbs_data->qd[R2_wheel_ft_rt_id]);
	//mbs_equil->options->quch[3] = R2_wheel_ft_lt_id;
	//mbs_equil->options->xch_ptr[3] = &(mbs_data->qd[R2_wheel_ft_lt_id]);
	//mbs_equil->options->quch[4] = R2_wheel_rr_lt_id;
	//mbs_equil->options->xch_ptr[4] = &(mbs_data->qd[R2_wheel_rr_lt_id]);
	//mbs_equil->options->quch[5] = R2_wheel_rr_rt_id;
	//mbs_equil->options->xch_ptr[5] = &(mbs_data->qd[R2_wheel_rr_rt_id]);
	//// equilibrium procedure
	//mbs_run_equil(mbs_equil, mbs_data);
	//mbs_print_equil(mbs_equil);

	///* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	///*                MODAL ANALYSIS around the straight line    *
	///* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	//mbs_modal = mbs_new_modal(mbs_data);
	//// modal options (see documentations for additional options)
	//mbs_modal->options->save_mat = 1;
	//mbs_modal->options->save_eval = 1;
	//mbs_modal->options->save_evec = 1;
	//mbs_modal->options->save_result = 1;
	//mbs_modal->options->save_anim = 1;
	////mbs_modal->options->mode_ampl = 0.2;
	//mbs_modal->options->verbose = 1;
	//mbs_run_modal(mbs_modal, mbs_data);
	//mbs_delete_modal(mbs_modal, mbs_data);



	///* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	///*                SS CORNERING  EQUILIBRIUM                  *
	///* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	//printf("-----------------SS cornering equil-----------------\n");

	//// conditions initiales pour la vitesse de rotation des roues (approximation rsg)
	//mbs_data->qd[T1_chassis_id] = V;
	//mbs_data->qd[R2_wheel_ft_lt_id] = V / 0.279572;
	//mbs_data->qd[R2_wheel_ft_rt_id] = V / 0.279572;
	//mbs_data->qd[R2_wheel_rr_lt_id] = V / 0.280501;
	//mbs_data->qd[R2_wheel_rr_rt_id] = V / 0.280501;

	//V = 10;
	//double R = 100; 
	//int i;
	//for (i = 100; i > 15; i--) // looping on the radius from R=100 m allows to reach R=20 m for V=10.
	//{
	//    R = i;
	//    mbs_data->q[T2_chassis_id] = -R;   // Does not intervene : -R
	//    mbs_data->qd[T1_chassis_id] = V;
	//    mbs_data->qd[R3_chassis_id] = +V / R;
	//    mbs_data->qdd[T2_chassis_id] = +V * V / R;

	//    mbs_equil = mbs_new_equil(mbs_data);
	//    // options
	//    mbs_equil->options->verbose = 0;
	//    mbs_equil->options->mode = 3;
	//    mbs_data->process = 2;
	//    mbs_equil->options->senstol = 1e-3;
	//    mbs_equil->options->devjac = 1e-6;
	//    mbs_equil->options->equitol = 1e-3;
	//    mbs_equil->options->itermax = 50;
	//    mbs_equil->options->resfilename = "equil_cornering";
	//    // --- Variable exchange, quch->xch
	//    mbs_equil->options->nquch = 6;
	//    mbs_equil_exchange(mbs_equil->options);
	//    mbs_equil->options->quch[1] = T1_chassis_id;
	//    mbs_equil->options->xch_ptr[1] = &(mbs_data->user_model->EquilQuantities.Qpropulsion);
	//    mbs_equil->options->quch[2] = R2_wheel_ft_lt_id;
	//    mbs_equil->options->xch_ptr[2] = &(mbs_data->qd[R2_wheel_ft_lt_id]);
	//    mbs_equil->options->quch[3] = R2_wheel_ft_rt_id;
	//    mbs_equil->options->xch_ptr[3] = &(mbs_data->qd[R2_wheel_ft_rt_id]);
	//    mbs_equil->options->quch[4] = R2_wheel_rr_lt_id;
	//    mbs_equil->options->xch_ptr[4] = &(mbs_data->qd[R2_wheel_rr_lt_id]);
	//    mbs_equil->options->quch[5] = R2_wheel_rr_rt_id;
	//    mbs_equil->options->xch_ptr[5] = &(mbs_data->qd[R2_wheel_rr_rt_id]);
	//    mbs_equil->options->quch[6] = T2_chassis_id;
	//    mbs_equil->options->xch_ptr[6] = &(mbs_data->user_model->EquilQuantities.Qrack);
	//    mbs_run_equil(mbs_equil, mbs_data);
	//    mbs_delete_equil(mbs_equil, mbs_data);
	//}

	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   DIRECT DYNAMICS                        *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

	mbs_dirdyn = mbs_new_dirdyn(mbs_data);

	mbs_data->q[R2_chassis_id] = 0.0;
	mbs_data->q[R3_rem_id] = 0;
	mbs_data->q[T3_chassis_id] = 0.2;
	mbs_data->process = 2;
	mbs_data->qd[T1_chassis_id] = V_scenar;
	mbs_data->qd[R2_wheel_ft_lt_id] = V_scenar / 0.279572;
	mbs_data->qd[R2_wheel_ft_rt_id] = V_scenar / 0.279572;
	mbs_data->qd[R2_wheel_rr_lt_id] = V_scenar / 0.280501;
	mbs_data->qd[R2_wheel_rr_rt_id] = V_scenar / 0.280501;

	// dirdyn options (see documentations for additional options)
	mbs_dirdyn->options->dt0 = 0.82 * 1e-3;
	mbs_dirdyn->options->tf = 12; //20;
	mbs_dirdyn->options->save2file = 1;
	//mbs_dirdyn->options->realtime = 1;

	mbs_run_dirdyn(mbs_dirdyn, mbs_data);

	
	mbs_delete_dirdyn(mbs_dirdyn, mbs_data);


	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
	/*                   CLOSING OPERATIONS                      *
	/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */
	mbs_delete_data(mbs_data);

	return 0;
}

