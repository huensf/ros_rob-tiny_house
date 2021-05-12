//---------------------------
// UCL-CEREM-MBS
//
// @version MBsysLab_s 1.7.a
//
// Creation : 2006
// Last update : 01/10/2008
//---------------------------

#include "math.h"

#include "mbs_data.h"
#include "user_all_id.h"


void user_DrivenJoints(MbsData* mbs_data, double tsim)
{
    mbs_data->q[T3_chargement_id] = 1;
    mbs_data->qd[T3_chargement_id] = 0;
    mbs_data->qdd[T3_chargement_id] = 0;

    mbs_data->q[T1_frein_id] = 0;
    mbs_data->qd[T1_frein_id] = 0;
    mbs_data->qdd[T1_frein_id] = 0;
}

 
