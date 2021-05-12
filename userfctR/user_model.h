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
 * Last update : Sat Jan  9 22:36:47 2021
 * --------------------------------------------------------
 *
 */
#ifndef USERMODEL_h
#define USERMODEL_h



#include "mbs_user_interface.h"
// ============================================================ //


struct UserModel 
{
    struct FrontSuspension{
        double K;
        double D;
        double L0;
        double C_bar;
    } FrontSuspension;
 
    struct RearSuspension{
        double K;
        double D;
        double L0;
        double C_bar;
    } RearSuspension;
 
    struct FrontTire{
        double R;
        double K;
    } FrontTire;
 
    struct RearTire{
        double R;
        double K;
    } RearTire;
 
    struct EquilQuantities{
        double Qpropulsion;
        double Qrack;
    } EquilQuantities;
 
    struct SteeringAssembly{
        double delta_left;
        double delta_right;
    } SteeringAssembly;
 
    struct Rotations{
        double R11G;
        double R11D;
        double R12G;
        double R12D;
        double R11C;
        double R12C;
    } Rotations;
 
    struct FrontTire_rem{
        double R;
        double K;
    } FrontTire_rem;
 
    struct RearTire_rem{
        double R;
        double K;
    } RearTire_rem;
 
};

// ============================================================ //
 
# endif