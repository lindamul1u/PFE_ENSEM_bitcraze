/*
The MIT License (MIT)

Copyright (c) 2018 Wolfgang Hoenig and James Alan Preiss

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

/*
This controller is based on the following publication:

Daniel Mellinger, Vijay Kumar:
Minimum snap trajectory generation and control for quadrotors.
IEEE International Conference on Robotics and Automation (ICRA), 2011.

We added the following:
 * Integral terms (compensates for: battery voltage drop over time, unbalanced center of mass due to asymmmetries, and uneven wear on propellers and motors)
 * D-term for angular velocity
 * Support to use this controller as an attitude-only controller for manual flight
*/

#include <math.h>

#include "param.h"
#include "log.h"
#include "math3d.h"

#include "controller_ensem.h"


static double kp=53.2010;
static double kdp=74.8919;
static double kd2p=56.3921;
static double kd3p=20.0179;


static double kpsi=17.0082;
static double kdpsi=17.9804;


void controllerENSEMInit(void)
{

}

bool controllerENSEMTest(void)
{
  return true;
}

void controllerENSEM(commande_t *commande, reff_t *reff,const X_t *X,const uint32_t tick)


{// V=[p^(4)_reff psi(2)_reff]-K[X_tilde-reff]

commande->c1=(reff->dx10)-kp*(X->x1 -reff->x1)-kdp*(X->x4-reff->x4)-kd2p*(X->x7-reff->x7)-kd3p*(X->x10-reff->x10);
commande->c2=(reff->dx11)-kp*(X->x2-reff->x2)-kdp*(X->x5-reff->x5)-kd2p*(X->x8-reff->x8)-kd3p*(X->x11-reff->x11);
commande->c3=(reff->dx12)-kp*(X->x3 -reff->x3)-kdp*(X->x6-reff->x6)-kd2p*(X->x9-reff->x9)-kd3p*(X->x12-reff->x12);
commande->c4=(reff->dx14)-kpsi*(X->x13 -reff->x13)-kdpsi*(X->x14-reff->x14);



}



