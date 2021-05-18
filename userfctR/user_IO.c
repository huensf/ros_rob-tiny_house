/* --------------------------------------------------------
 * This code was generated automatically by MBsysC modules.
 * MBsysC modules are distributed as part of the ROBOTRAN 
 * software. They provides functionalities for dealing with
 * symbolic equations generated by ROBOTRAN. 
 *
 * More info on www.robotran.be 
 *
 * Universite catholique de Louvain, Belgium 
 *
 * Last update : Tue May 18 15:39:38 2021
 * --------------------------------------------------------
 *
 */
#include <stdlib.h>
#include "user_IO.h"

// ============================================================ //


UserIO* mbs_new_user_IO(UserIoInfo* ioInfo) 
{
    UserIO *uio;
    uio = (UserIO*) malloc(sizeof(UserIO));

    return uio;
}



void mbs_delete_user_IO(UserIO *uio)
{

    free(uio);
}

void mbs_get_user_IO_size(int *n_in, int *n_out, int *n_user_IO) 
{
    *n_in  = 0; 
    *n_out = 0; 
    *n_user_IO = 0; 
}

// ============================================================ //
 
