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




static double kp=49.2151;
static double kdp=69.3918;
static double kd2p=52.3827;
static double kd3p=18.7078;


static double kpsi=15.8445;
static double kdpsi=16.8148;


void controllerENSEMInit(void)
{

}

bool controllerENSEMTest(void)
{
	return true;
}

void controllerENSEM(commande_t *commande, X_t *reff, X_t *X,const uint32_t tick)


{// V=[p^(4)_reff psi(2)_reff]-K[X_tilde-reff]

		double x1= (double) X->x1;
		double x2= (double) X->x2;
		double x3= (double) X->x3;

		double x4= (double) X->x4;
		double x5= (double) X->x5;
		double x6= (double) X->x6;

		double x7= (double) X->x7;
		double x8= (double) X->x8;
		double x9= (double) X->x9;

		double x10= (double) X->x10;
		double x11= (double) X->x11;
		double x12= (double) X->x12;

		double x13= (double) X->x13;
		double x14= (double) X->x14;

		double reff1= (double) reff->x1;
		double reff2= (double) reff->x2;
		double reff3= (double) reff->x3;

		double reff4= (double) reff->x4;
		double reff5= (double) reff->x5;
		double reff6= (double) reff->x6;

		double reff7= (double) reff->x7;
		double reff8= (double) reff->x8;
		double reff9= (double) reff->x9;

		double reff10= (double) reff->x10;
		double reff11= (double) reff->x11;
		double reff12= (double) reff->x12;

		double reff13= (double) reff->x13;
		double reff14= (double) reff->x14;

		double reffd4x= (double) reff->d4x;
		double reffd4y= (double) reff->d4y;
		double reffd4z= (double) reff->d4z;
		double reffd2psi= (double) reff->d2psi;


		double c1=(double) ((reffd4x)-kp*( x1 -reff1)-kdp*( x4-reff4)-kd2p*( x7-reff7)-kd3p*( x10-reff10));
		double c2=(double) ((reffd4y)-kp*( x2-reff2)-kdp*( x5-reff5)-kd2p*( x8-reff8)-kd3p*( x11-reff11));
		double c3=(double)((reffd4z)-kp*( x3 -reff3)-kdp*( x6-reff6)-kd2p*( x9-reff9)-kd3p*( x12-reff12));
		double c4=(double) ((reffd2psi)-kpsi*( x13 -reff13)-kdpsi*( x14-reff14));
		commande->c1= (float) c1;
		commande->c2= (float) c2;
		commande->c3= (float) c3;
		commande->c4= (float) c4;





}



