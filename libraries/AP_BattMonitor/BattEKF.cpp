
// Copyright (C) 2004        Vincent Zalzal, Sylvain Marleau
// Copyright (C) 2001, 2004  Richard Gourdeau
// Copyright (C) 2004        GRPR and DGE's Automation sector
//                           École Polytechnique de Montréal
//
// Code adapted from algorithms presented in :
//      Bierman, G. J. "Factorization Methods for Discrete Sequential
//      Estimation", Academic Press, 1977.
//
// This library is free software; you can redistribute it and/or
// modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation; either
// version 2.1 of the License, or (at your option) any later version.
//
// This library is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public
// License along with this library; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA

// Refer http://kalman.sourceforge.net/ for help on EKF library functions

#include "BattEKF.h"
#include "AP_Math.h"

extern AP_HAL::HAL& hal;

BattEKF::BattEKF() 
{
    setDim(3, 3, 3, 2, 2);
}

void BattEKF::makeBaseA()            //state transition matrix, F as per wikipedia
{
    A(1,1) = 1.0;
    A(1,2) = 0.0;
    A(1,3) = 0.0;

    A(2,1) = 0.0;
    A(2,2) = 1.0;
    A(2,3) = 0.0;

    A(3,1) = 0.0;
    A(3,2) = 0.0;
    A(3,3) = 1.0;

}

void BattEKF::makeBaseW()            //Process noise matrix, L as per wikipedia
{
    W(1,1) = 1.0;
    W(1,2) = 0.0;
    W(1,3) = 0.0;

    W(2,1) = 0.0;
    W(2,2) = 1.0;
    W(2,3) = 0.0;

    W(3,1) = 0.0;
    W(3,2) = 0.0;
    W(3,3) = 1.0;
}


void BattEKF::makeH()                //Observation matrix, H as per wikipedia
{
    H(1,1) = 1.0;
    H(1,2) = -x(3);
    H(1,3) = -x(2);

    H(2,1) = 0.0;
    H(2,2) = 1.0;
    H(2,3) = 0.0;
}

void BattEKF::makeBaseV()            //Measurement noise matrix, M as per wikipedia
{
    V(1,1) = 1.0;
    V(1,2) = 0.0;

    V(2,1) = 0.0;
    V(2,2) = 1.0;
}

void BattEKF::makeQ()                //Process noise covariance, Q as per wikipedia
{
    float V_PNOISE = 0.05;
    float I_PNOISE = 1400;
    float R_PNOISE  = 0.05/60.0;

    float dt = _prev_time - hal.scheduler->micros()/1000;
    _prev_time = hal.scheduler->micros()/1000;

    Q(1,1) = sq(dt*V_PNOISE);
    Q(1,2) = 0.0;
    Q(1,3) = 0.0;

    Q(2,1) = 0.0;
    Q(2,2) = sq(dt*I_PNOISE);
    Q(2,3) = 0.0;

    Q(3,1) = 0.0;
    Q(3,2) = 0.0;
    Q(3,3) = sq(dt*R_PNOISE);
}


void BattEKF::makeBaseR()        //measurement noise covariance, R as per wikipedia
{
    R(1,1) = sq(0.1);
    R(2,2) = sq(0.1);
}

void BattEKF::makeProcess()        //state estimate, f as per wikipedia
{
    if (x(2) < 0){
        x(2) = 0;
    }
    if (x(3) < 0){
        x(3) = 0;
    }
}

void BattEKF::makeMeasure()        // h as per wikipedia
{
    z(1)=x(1) - x(2)*x(3);
    z(2)=x(2);
}

