//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h" 
#include "user_all_id.h"
#include "mbs_data.h"
#include "user_model.h"
#include "set_output.h"
#include "useful_functions.h"

#define G 9.81

double* user_JointForces(MbsData *mbs_data, double tsim)
{
/*-- Begin of user code --*/

    if (tsim == 0.0) // equilibrium process and modal analysis
    {
        mbs_data->Qq[R2_wheel_rr_lt_id] = mbs_data->user_model->EquilQuantities.Qpropulsion;
        mbs_data->Qq[R2_wheel_rr_rt_id] = mbs_data->user_model->EquilQuantities.Qpropulsion;
        mbs_data->Qq[T2_rack_id] = mbs_data->user_model->EquilQuantities.Qrack;
    }

    // anti-roll bar
    mbs_data->Qq[R2_def_bar_ft_id] = -mbs_data->user_model->FrontSuspension.C_bar*mbs_data->q[R2_def_bar_ft_id];
    mbs_data->Qq[R2_def_bar_rr_id] = -mbs_data->user_model->RearSuspension.C_bar*mbs_data->q[R2_def_bar_rr_id];

/*-- End of user code --*/

	/* DEBUT FREINAGE */

	double masseVoit = mbs_data->m[Chassis_1_id] + 4 * mbs_data->m[Fusee_ar_d_1_id] + 4 * mbs_data->m[Roue_av_d_1_id] + mbs_data->m[Bar_direction_1_id] + mbs_data->m[Weight_id];
	double mufrottement = 0.5; //-> Frottement critique
	double r = 0.3;
	double r_rem = 0.2625;
	double amplitude = 0.7;
	double mtot = masseVoit;
	// Mettre couple a 0 si on ne veut pas de freinage
	double couplefrein_voit = -amplitude * mtot / 4 * (G)*mufrottement * r;
	double force_max = 0.25 * 3500 * G; //= abs(couplefrein * 4 / (r));
	double couplefreinmax_rem = -force_max / 4 * r_rem;

	/* Freinage de la remorque -> A laisser dans tous les cas */
	if (tsim > 0)
	{
		if (mbs_data->Qc[T1_frein_id] < 0)
		{
			mbs_data->Qq[R2_av_d_id] = abs(mbs_data->Qc[T1_frein_id]) / force_max * couplefreinmax_rem;
			mbs_data->Qq[R2_av_g_id] = abs(mbs_data->Qc[T1_frein_id]) / force_max * couplefreinmax_rem;
			mbs_data->Qq[R2_ar_d_id] = abs(mbs_data->Qc[T1_frein_id]) / force_max * couplefreinmax_rem;
			mbs_data->Qq[R2_ar_g_id] = abs(mbs_data->Qc[T1_frein_id]) / force_max * couplefreinmax_rem;
		}
		else
		{
			mbs_data->Qq[R2_av_d_id] = 0.0;
			mbs_data->Qq[R2_av_g_id] = 0.0;
			mbs_data->Qq[R2_ar_d_id] = 0.0;
			mbs_data->Qq[R2_ar_g_id] = 0.0;
		}
	}

	/*  Freinage en courbe de 27.78 m/s a 5 m/s*/
	/*printf("force de %f at time %f\n", force_max, tsim);
	printf("couple de %f at time %f\n", couplefreinmax_rem, tsim);
	printf("Force sur la remorque de %f at time %f\n", (abs(mbs_data->Qc[T1_frein_id]) / force_max * couplefrein *4)/0.26, tsim);
	printf("force %f at time %f\n", mbs_data->Qc[T1_frein_id], tsim);*/

	//double vitesse_voit = sqrt((mbs_data->qd[T1_chassis_id] * mbs_data->qd[T1_chassis_id]) + (mbs_data->qd[T2_chassis_id] * mbs_data->qd[T2_chassis_id]));
	//if (vitesse_voit <= 5) //une fois 5m/s atteint, il arrete de freiner
	//{
	//	mbs_data->Qq[R2_wheel_ft_rt_id] = 0.0;
	//	mbs_data->Qq[R2_wheel_ft_lt_id] = 0.0;
	//	mbs_data->Qq[R2_wheel_rr_rt_id] = 0.0;
	//	mbs_data->Qq[R2_wheel_rr_lt_id] = 0.0;
	//}
	//else if (tsim > 2 && vitesse_voit > 5)
	//{
	//	mbs_data->Qq[R2_wheel_ft_rt_id] = couplefrein_voit;
	//	mbs_data->Qq[R2_wheel_ft_lt_id] = couplefrein_voit;
	//	mbs_data->Qq[R2_wheel_rr_rt_id] = couplefrein_voit;
	//	mbs_data->Qq[R2_wheel_rr_lt_id] = couplefrein_voit;
	//}


	/* FIN FREINAGE */

	/* DEBUT VIRAGE */

	//Virage progressif
	//if (tsim > 1 && tsim <= 3.5)
	//	mbs_data->q[T2_rack_id] = 0.0075 * (tsim - 1);
	//else if (tsim > 3.5 && tsim <= 6)
	//	mbs_data->q[T2_rack_id] = 0.0075 * (3.5 - 1) - 0.0075 * (tsim - 3.5);
	//else mbs_data->q[T2_rack_id] = 0.0;

	/*   Depassement en ligne droite (double coup de volant sans freinage construit pour 18 m/s)   */
	double t_1 = 1; //debut braquage(1)
	double t_2 = 1.4; //fin braquage(1) - debut contrebraquage(1)
	double t_3 = 2.6; //fin contrebraquage(1) - debut retour au calme(1)
	double t_4 = 4.0;
	double t_5 = 4.8; //fin retour au calme(1) - debut calme(1)
	double t_6 = 5.8; //fin calme(1) - debut braquage(2)
	double t_7 = 6; //fin braquage(2) - debut contrebraquage(2)
	double angle = 0.02;
	if (tsim > t_1 && tsim <= t_2)
		mbs_data->q[T2_rack_id] = -7 * angle / 8 * (tsim - t_1); //fin : -0.007

	else if (tsim > t_2 && tsim <= t_3)
		mbs_data->q[T2_rack_id] = -7 * angle / 8 * (t_2 - t_1) + 11 * angle / 24 * (tsim - t_2); //fin : 0.004

	else if (tsim > t_3 && tsim <= t_4)
		mbs_data->q[T2_rack_id] = -7 * angle / 8 * (t_2 - t_1) + 11 * angle / 24 * (t_3 - t_2); // contrebraquage cst(0.004)

	else if (tsim > t_4 && tsim <= t_5)
		mbs_data->q[T2_rack_id] = -7 * angle / 8 * (t_2 - t_1) + 11 * angle / 24 * (t_3 - t_2) - 21 * angle / 40 * (tsim - t_4);//fin : -0.0044
	
	else if (tsim > t_5 && tsim <= t_6)
		mbs_data->q[T2_rack_id] = -7*angle/8 * (t_2 - t_1) + 11* angle /24 * (t_3 - t_2) - 21*angle/40 * (t_5 - t_4) + 3*angle/25 * (tsim - t_5); //fin: -0.002
	
	else if (tsim > t_6 && tsim <= t_7)
		mbs_data->q[T2_rack_id] = -7 * angle / 8 * (t_2 - t_1) + 11 * angle / 24 * (t_3 - t_2) - 21 * angle / 40 * (t_5 - t_4) + 3*angle/25 * (t_6 - t_5) + angle * (tsim - t_6); //fin : 0.002
		
	else mbs_data->q[T2_rack_id] = 0.0;

	/*   Conduite sur route sinusoidale (faible amplitude) ; 0.2 Hz et 25 m/s */
	//double V = 25; 
	//double t_0 = 0; //debut route sinusoidale
	//double ampli_sinus = 0.002; //0.0025;
	//if (tsim > t_0)
	//	mbs_data->q[T2_rack_id] = -ampli_sinus * sin((tsim+1.25)*M_PI/2.5);
	//else mbs_data->q[T2_rack_id] = 0.0;

	//double vitesse_voit = sqrt((mbs_data->qd[T1_chassis_id] * mbs_data->qd[T1_chassis_id]) + (mbs_data->qd[T2_chassis_id] * mbs_data->qd[T2_chassis_id]));
	//if (tsim > 0)
	//{
	//	if (vitesse_voit < V)
	//	{
	//		mbs_data->Qq[R2_wheel_ft_rt_id] = 200;
	//		mbs_data->Qq[R2_wheel_ft_lt_id] = 200;
	//		mbs_data->Qq[R2_wheel_rr_rt_id] = 200;
	//		mbs_data->Qq[R2_wheel_rr_lt_id] = 200;
	//	}
	//	else
	//	{
	//		mbs_data->Qq[R2_wheel_ft_rt_id] = 0.0;
	//		mbs_data->Qq[R2_wheel_ft_lt_id] = 0.0;
	//		mbs_data->Qq[R2_wheel_rr_rt_id] = 0.0;
	//		mbs_data->Qq[R2_wheel_rr_lt_id] = 0.0;
	//	}
	//}
	//printf("The Velocity is %f at time %f\n", mbs_data->qd[T1_chassis_id],tsim);

	/*   Mise en courbe pour Freinage d'urgence  */
	//double t_1 = 1;
	//double t_2 = 3;
	//double t_3 = 5;
	//double ampl = 0.0011;//0.0012;
	//if (tsim > t_1 && tsim <= t_2)
	//	mbs_data->q[T2_rack_id] = ampl * (tsim - t_1);
	//else if (tsim > t_2 && tsim <= t_3)
	//	mbs_data->q[T2_rack_id] = ampl * (t_2 - t_1) - ampl * (tsim - t_2);
	//else mbs_data->q[T2_rack_id] = 0.0;

	//printf("Deplacement de %f at time %f\n", mbs_data->q[R3_direction_id], tsim);
	//printf("Deplacement de %f at time %f\n", mbs_data->q[T2_rack_id], tsim);

//}
	/* FIN VIRAGE */

	return mbs_data->Qq;
}
