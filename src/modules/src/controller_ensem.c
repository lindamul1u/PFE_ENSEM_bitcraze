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
/*
   kp= 10.0000   ;
  kdp=  25.0864 ;
  kd2p= 26.4663 ;
  kd3p= 12.3666;


  kpsi=10.0000 ;
  kdpsi=  10.9545;*/

static double kp=  16.0 ;
static double kdp=  32.0  ;
static double kd2p=  24.0  ;
static double kd3p= 8.0  ;


static double kpsi= 4.00;
static double kdpsi=4.0 ;



void controllerENSEMInit(void)
{

}

bool controllerENSEMTest(void)
{
	return true;
}

void controllerENSEM(commande_t *commande, X_t *reff, X_t *X,const uint32_t tick)


{// V=[p^(4)_reff psi(2)_reff]-K[X_tilde-reff]

		double x1=   X->x1;
		double x2=   X->x2;
		double x3=   X->x3;

		double x4=   X->x4;
		double x5=   X->x5;
		double x6=   X->x6;

		double x7=   X->x7;
		double x8=   X->x8;
		double x9=   X->x9;

		double x10=   X->x10;
		double x11=   X->x11;
		double x12=   X->x12;

		double x13=   X->x13;
		double x14=   X->x14;

		double reff1=   reff->x1;
		double reff2=   reff->x2;
		double reff3=   reff->x3;

		double reff4=   reff->x4;
		double reff5=   reff->x5;
		double reff6=   reff->x6;

		double reff7=   reff->x7;
		double reff8=   reff->x8;
		double reff9=   reff->x9;

		double reff10=   reff->x10;
		double reff11=   reff->x11;
		double reff12=   reff->x12;

		double reff13=   reff->x13;
		double reff14=   reff->x14;

		double reffd4x=   reff->d4x;
		double reffd4y=   reff->d4y;
		double reffd4z=   reff->d4z;
		double reffd2psi=   reff->d2psi;


		double c1=  ((reffd4x)-kp*( x1 -reff1)-kdp*( x4-reff4)-kd2p*( x7-reff7)-kd3p*( x10-reff10));
		double c2=  ((reffd4y)-kp*( x2-reff2)-kdp*( x5-reff5)-kd2p*( x8-reff8)-kd3p*( x11-reff11));
		double c3= ((reffd4z)-kp*( x3 -reff3)-kdp*( x6-reff6)-kd2p*( x9-reff9)-kd3p*( x12-reff12));
		double c4=  ((reffd2psi)-kpsi*( x13 -reff13)-kdpsi*( x14-reff14));
		commande->c1=   c1;
		commande->c2=   c2;
		commande->c3=   c3;
		commande->c4=   c4;





}



