/* ---------------------------
 * Robotran - MBsysC
 * 
 * Template file for direct dynamics module
 * 
 * This files enable the user to call custom at
 * specific places in the time simulation. It is a template
 * file that can be edited by the user.
 * 
 * (c) Universite catholique de Louvain
 *     
 */

#include "math.h"
#include "set_output.h"
#include "user_all_id.h"
#include "mbs_sensor.h"
#include "mbs_data.h"
#include "mbs_dirdyn_struct.h"


/*! \brief user own initialization functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_init(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{

}

/*! \brief user own loop functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_loop(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{
    //int id = Sensor_chassis_id; //Sensor_remorque_id;

    //// retrieve the pointer to the sensor structure defined in mbs_aux
    //MbsSensor* PtrSensor = mbs_dd->mbs_aux->psens;

    //// compute the sensor (position, velocity...)
    //mbs_comp_S_sensor(PtrSensor, mbs_data, id);

    //// save the data
    //set_output(PtrSensor->V[1], "vitesse_x");
    //set_output(PtrSensor->V[2], "vitesse_y");
}

/*! \brief user own finishing functions
 *
 * \param[in,out] mbs_data data structure of the model
 * \param[in,out] mbs_dd general structure of the direct dynamic module (for advance users)
 *
 * For beginners, it is advised to only use the MbsData structure.
 * The field MbsDirdyn is provided for more advance users.
 */
void user_dirdyn_finish(MbsData *mbs_data, MbsDirdyn *mbs_dd)
{

}
