/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2016 Bitcraze AB
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, in version 3.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 *
 * power_distribution_stock.c - Crazyflie stock power distribution code
 */
#define DEBUG_MODULE "PWR_DIST"

#include "power_distribution.h"

#include <string.h>
#include "log.h"
#include "param.h"
#include "num.h"
#include "platform.h"
#include "motors.h"
#include "debug.h"
#include <math.h>
static bool motorSetEnable = false;
static struct {
	uint32_t m1;
	uint32_t m2;
	uint32_t m3;
	uint32_t m4;
} motorPower;

static struct {
	uint16_t m1;
	uint16_t m2;
	uint16_t m3;
	uint16_t m4;
} motorPowerSet;

void powerDistributionInit(void)
{
	motorsInit(platformConfigGetMotorMapping());
}

bool powerDistributionTest(void)
{
	bool pass = true;

	pass &= motorsTest();

	return pass;
}

#define limitThrust(VAL) limitUint16(VAL)
int sign(double a);
void powerStop()
{
	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);
}
int sign(double a){
	if(a<0){
		return -1;
	}
	else{
		return 1;
	}
}
void powerDistributionENSEM(commande_t *powerENSEM, commande_t *commande, X_t *X){

	//V, wx,wy,wz,phi thet psi f df dphi dtheta ->[M d2f]->[M f]-> w1 w2 w3 w4 -> pwm

		double   c1= (double) commande->c1;
		double   c2= (double) commande->c2;
		double   c3= (double) commande->c3;
		double   c4= (double) commande->c4;

		double  w1=(double) X->w1;
		double    w2= (double) X->w2;
		double   w3= (double) X->w3;
		double   phi= (double) X->phi;
		double   theta= (double) X->theta;
		double   psi= (double) X->psi;
		//double   dphi= (double) X->dphi;
		//double   dtheta= (double) X->dtheta;
		double f= (double)X->f;
		double df= (double)X->df;
		//double d2f=(double) X->d2f;
		double  dt= X->dt;

		double cpsi=cos(psi);
		double spsi=sin(psi);
		double stheta=sin(theta);
		double ctheta=cos(theta);
		double cphi=cos(phi);
		double sphi=sin(phi);
		double pres=0.01;
		if (fabs(ctheta)<pres){
		    ctheta=pres*sign(ctheta);
		}
		if (fabs(cphi)<pres){
		    cphi=pres*sign(cphi);
		}
		//double ttheta=stheta/ctheta;
		if (f<1){
		    f=1.0;
		    df=0.0;
		}

		double j11=1.0e-04 *0.1395;
		double j22= 1.0e-04 *0.1436;
		double   j33=1.0e-04 *0.2173;



		 double v1=(j11*(cphi*spsi - cpsi*sphi*stheta)*(c1 - (cphi*spsi - cpsi*sphi*stheta)*(2*df*w1 - f*w2*w3) + f*(w1*w1 + w2*w2)*(sphi*spsi + cphi*cpsi*stheta) - cpsi*ctheta*(2*df*w2 + f*w1*w3)))/f - w2*w3*(j22 - j33) - (j11*(cphi*cpsi + sphi*spsi*stheta)*(c2 + (cphi*cpsi + sphi*spsi*stheta)*(2*df*w1 - f*w2*w3) - f*(w1*w1 + w2*w2)*(cpsi*sphi - cphi*spsi*stheta) - ctheta*spsi*(2*df*w2 + f*w1*w3)))/f - (ctheta*j11*sphi*(c3 + stheta*(2*df*w2 + f*w1*w3) + ctheta*sphi*(2*df*w1 - f*w2*w3) + cphi*ctheta*f*(w1*w1 + w2*w2)))/f;
		 double v2=w1*w3*(j11 - j33) - (j22*stheta*(c3 + stheta*(2*df*w2 + f*w1*w3) + ctheta*sphi*(2*df*w1 - f*w2*w3) + cphi*ctheta*f*(w1*w1 + w2*w2)))/f + (cpsi*ctheta*j22*(c1 - (cphi*spsi - cpsi*sphi*stheta)*(2*df*w1 - f*w2*w3) + f*(w1*w1 + w2*w2)*(sphi*spsi + cphi*cpsi*stheta) - cpsi*ctheta*(2*df*w2 + f*w1*w3)))/f + (ctheta*j22*spsi*(c2 + (cphi*cpsi + sphi*spsi*stheta)*(2*df*w1 - f*w2*w3) - f*(w1*w1 + w2*w2)*(cpsi*sphi - cphi*spsi*stheta) - ctheta*spsi*(2*df*w2 + f*w1*w3)))/f;
		 double v3=(ctheta*j33*(c4 - w2*((cphi*(w1 + (cphi*stheta*w3)/ctheta + (sphi*stheta*w2)/ctheta))/ctheta + (sphi*stheta*(cphi*w2 - sphi*w3))/ctheta*ctheta) + w3*((sphi*(w1 + (cphi*stheta*w3)/ctheta + (sphi*stheta*w2)/ctheta))/ctheta - (cphi*stheta*(cphi*w2 - sphi*w3))/ctheta*ctheta)))/cphi - w1*w2*(j11 - j22) + (j33*sphi*stheta*(c3 + stheta*(2*df*w2 + f*w1*w3) + ctheta*sphi*(2*df*w1 - f*w2*w3) + cphi*ctheta*f*(w1*w1 + w2*w2)))/(cphi*f) - (cpsi*ctheta*j33*sphi*(c1 - (cphi*spsi - cpsi*sphi*stheta)*(2*df*w1 - f*w2*w3) + f*(w1*w1 + w2*w2)*(sphi*spsi + cphi*cpsi*stheta) - cpsi*ctheta*(2*df*w2 + f*w1*w3)))/(cphi*f) - (ctheta*j33*sphi*spsi*(c2 + (cphi*cpsi + sphi*spsi*stheta)*(2*df*w1 - f*w2*w3) - f*(w1*w1 + w2*w2)*(cpsi*sphi - cphi*spsi*stheta) - ctheta*spsi*(2*df*w2 + f*w1*w3)))/(cphi*f);
		 double ddf=(sphi*spsi + cphi*cpsi*stheta)*(c1 - (cphi*spsi - cpsi*sphi*stheta)*(2*df*w1 - f*w2*w3) + f*(w1*w1 + w2*w2)*(sphi*spsi + cphi*cpsi*stheta) - cpsi*ctheta*(2*df*w2 + f*w1*w3)) - (cpsi*sphi - cphi*spsi*stheta)*(c2 + (cphi*cpsi + sphi*spsi*stheta)*(2*df*w1 - f*w2*w3) - f*(w1*w1 + w2*w2)*(cpsi*sphi - cphi*spsi*stheta) - ctheta*spsi*(2*df*w2 + f*w1*w3)) + cphi*ctheta*(c3 + stheta*(2*df*w2 + f*w1*w3) + ctheta*sphi*(2*df*w1 - f*w2*w3) + cphi*ctheta*f*(w1*w1 + w2*w2));



		  //double v1= (j11*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(c1 - (2*df*w1 - f*w2*w3)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + f*(w1*w1 + w2*w2)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(psi)*cos(theta)*(2*df*w2 + f*w1*w3)))/f - w2*w3*(j22 - j33) - (j11*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(c2 + (2*df*w1 - f*w2*w3)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - f*(w1*w1 + w2*w2)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - cos(theta)*sin(psi)*(2*df*w2 + f*w1*w3)))/f - (j11*cos(theta)*sin(phi)*(c3 + sin(theta)*(2*df*w2 + f*w1*w3) + cos(theta)*sin(phi)*(2*df*w1 - f*w2*w3) + f*cos(phi)*cos(theta)*(w1*w1 + w2*w2)))/f;
			//	  double v2=    w1*w3*(j11 - j33) - (j22*sin(theta)*(c3 + sin(theta)*(2*df*w2 + f*w1*w3) + cos(theta)*sin(phi)*(2*df*w1 - f*w2*w3) + f*cos(phi)*cos(theta)*(w1*w1 + w2*w2)))/f + (j22*cos(psi)*cos(theta)*(c1 - (2*df*w1 - f*w2*w3)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + f*(w1*w1 + w2*w2)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(psi)*cos(theta)*(2*df*w2 + f*w1*w3)))/f + (j22*cos(theta)*sin(psi)*(c2 + (2*df*w1 - f*w2*w3)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - f*(w1*w1 + w2*w2)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - cos(theta)*sin(psi)*(2*df*w2 + f*w1*w3)))/f;
				//  double v3= (j33*cos(theta)*(c4 - w2*((cos(phi)*(w1 + (w3*cos(phi)*sin(theta))/cos(theta) + (w2*sin(phi)*sin(theta))/cos(theta)))/cos(theta) + (sin(phi)*sin(theta)*(w2*cos(phi) - w3*sin(phi)))/cos(theta)*cos(theta)) + w3*((sin(phi)*(w1 + (w3*cos(phi)*sin(theta))/cos(theta) + (w2*sin(phi)*sin(theta))/cos(theta)))/cos(theta) - (cos(phi)*sin(theta)*(w2*cos(phi) - w3*sin(phi)))/cos(theta)*cos(theta))))/cos(phi) - w1*w2*(j11 - j22) + (j33*sin(phi)*sin(theta)*(c3 + sin(theta)*(2*df*w2 + f*w1*w3) + cos(theta)*sin(phi)*(2*df*w1 - f*w2*w3) + f*cos(phi)*cos(theta)*(w1*w1 + w2*w2)))/(f*cos(phi)) - (j33*cos(psi)*cos(theta)*sin(phi)*(c1 - (2*df*w1 - f*w2*w3)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + f*(w1*w1 + w2*w2)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(psi)*cos(theta)*(2*df*w2 + f*w1*w3)))/(f*cos(phi)) - (j33*cos(theta)*sin(phi)*sin(psi)*(c2 + (2*df*w1 - f*w2*w3)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - f*(w1*w1 + w2*w2)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - cos(theta)*sin(psi)*(2*df*w2 + f*w1*w3)))/(f*cos(phi));


//		double ddf =(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(c1 - (2*df*w1 - f*w2*w3)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + f*(w1*w1 + w2*w2)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(psi)*cos(theta)*(2*df*w2 + f*w1*w3)) - (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*(c2 + (2*df*w1 - f*w2*w3)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - f*(w1*w1 + w2*w2)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - cos(theta)*sin(psi)*(2*df*w2 + f*w1*w3)) + cos(phi)*cos(theta)*(c3 + sin(theta)*(2*df*w2 + f*w1*w3) + cos(theta)*sin(phi)*(2*df*w1 - f*w2*w3) + f*cos(phi)*cos(theta)*(w1*w1 + w2*w2));



// calcule de M et f

		f=f+dt*df;
		df= df+dt*ddf;

		//f=X->f;
		//df=X->df;





		powerENSEM->c1=v1;
		powerENSEM->c2=v2;
		powerENSEM->c3=v3;
		powerENSEM->c4=f;

		X->f=  f;
		X->df=  df;
		X->d2f=  ddf;
// Calcul des wi

/*

		double V1= -861194.48*v1 -861194.48*v2 -969248.09*v3+ 24358.258*f;
		double V2= -861194.48*v1+  861194.48*v2+  969248.09*v3+ 24358.258*f;
		double V3=  861194.48*v1+  861194.48*v2 -969248.09*v3+ 24358.258*f;
		double V4=  861194.48*v1 -861194.48*v2+  969248.09*v3+ 24358.258*f;*/


		double V1=1e5*pow(0.0026915769*f - 2.798871*v1 - 2.798871*v2 - 3.1494476*v3,0.5);
		double V2=1e5*pow(0.0026915769*f - 2.798871*v1 + 2.798871*v2 + 3.1494476*v3,0.5);
		double V3=1e5*pow(0.0026915769*f + 2.798871*v1 + 2.798871*v2 - 3.1494476*v3,0.5);
		double V4=1e5*pow(0.0026915769*f + 2.798871*v1 - 2.798871*v2 + 3.1494476*v3,0.5);

			/*double V1= -27986938017.9591064807307910272*v1+ -27986938017.9591064807307910272*v2 -31498443976.8675449997919459645*v3+ 26914064.9737192075631391775929*f;
			double V2=  -27986938017.9591064807307910272*v1+  27986938017.9591064807307910272*v2+  31498443976.8675449997919459645*v3+ 26914064.9737192075631391775929*f;
			double V3=   27986938017.9591064807307910272*v1+ 27986938017.9591064807307910272*v2 -31498443976.8675449997919459645*v3+ 26914064.9737192075631391775929*f;
			double V4=   27986938017.9591064807307910272*v1 -27986938017.9591064807307910272*v2+  31498443976.8675449997919459645*v3+ 26914064.9737192075631391775929*f;
			V1=sqrt(V1);
			V2=sqrt(V2);
			V3=sqrt(V3);
			V4=sqrt(V4);*/

			//Vi vitesse moteur en tr/min



		double  M1=((V1)-4070.3)/0.2685;
		double  M2=((V2)-4070.3)/0.2685;
		double  M3=((V3)-4070.3)/0.2685;
		double  M4=((V4)-4070.3)/0.2685;








			motorPower.m1= (uint16_t) M1;
			motorPower.m2=(uint16_t) M2;
			motorPower.m3=(uint16_t) M3;
			motorPower.m4=(uint16_t) M4;
			motorsSetRatio(MOTOR_M1, motorPower.m1);
			motorsSetRatio(MOTOR_M2, motorPower.m2);
			motorsSetRatio(MOTOR_M3, motorPower.m3);
			motorsSetRatio(MOTOR_M4, motorPower.m4);


		M1=V1;
		M2=V2;
		M3=V3;
		M4=V4;
		powerENSEM->w1=M1;
		powerENSEM->w2=M2;
		powerENSEM->w3=M3;
		powerENSEM->w4=M4;
		double t=usecTimestamp() / 1e6;
		powerENSEM->currenttime=t;
		powerENSEM->start=1;




}

void powerDistribution(const control_t *control,commande_t *powerLog)
{


#ifdef QUAD_FORMATION_X
	int16_t r = control->roll / 2.0f;
	int16_t p = control->pitch / 2.0f;
	motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
	motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
	motorPower.m3 =  limitThrust(control->thrust + r - p + control->yaw);
	motorPower.m4 =  limitThrust(control->thrust + r + p - control->yaw);
	powerLog->w1bc=( (motorPower.m1))*0.2685+4070.3;
	powerLog->w2bc=( (motorPower.m2))*0.2685+4070.3;
	powerLog->w3bc=( (motorPower.m3))*0.2685+4070.3;
	powerLog->w4bc=( (motorPower.m4))*0.2685+4070.3;

#else // QUAD_FORMATION_NORMAL
	motorPower.m1 = limitThrust(control->thrust + control->pitch +
			control->yaw);

	motorPower.m2 = limitThrust(control->thrust - control->roll -
			control->yaw);

	motorPower.m3 =  limitThrust(control->thrust - control->pitch +
			control->yaw);

	motorPower.m4 =  limitThrust(control->thrust + control->roll -
			control->yaw);
	powerLog->w1bc=( (motorPower.m1))*0.2685+4070.3;
	powerLog->w2bc=( (motorPower.m2))*0.2685+4070.3;
	powerLog->w3bc=( (motorPower.m3))*0.2685+4070.3;
	powerLog->w4bc=( (motorPower.m4))*0.2685+4070.3;
#endif

	if (motorSetEnable)
	{
		motorsSetRatio(MOTOR_M1, motorPowerSet.m1);
		motorsSetRatio(MOTOR_M2, motorPowerSet.m2);
		motorsSetRatio(MOTOR_M3, motorPowerSet.m3);
		motorsSetRatio(MOTOR_M4, motorPowerSet.m4);
	}
	else
	{
		motorsSetRatio(MOTOR_M1, motorPower.m1);
		motorsSetRatio(MOTOR_M2, motorPower.m2);
		motorsSetRatio(MOTOR_M3, motorPower.m3);
		motorsSetRatio(MOTOR_M4, motorPower.m4);
	}
}


PARAM_GROUP_START(motorPowerSet)
PARAM_ADD(PARAM_UINT8, enable, &motorSetEnable)
PARAM_ADD(PARAM_UINT16, m1, &motorPowerSet.m1)
PARAM_ADD(PARAM_UINT16, m2, &motorPowerSet.m2)
PARAM_ADD(PARAM_UINT16, m3, &motorPowerSet.m3)
PARAM_ADD(PARAM_UINT16, m4, &motorPowerSet.m4)
PARAM_GROUP_STOP(ring)

LOG_GROUP_START(motor)
LOG_ADD(LOG_INT32, m4, &motorPower.m4)
LOG_ADD(LOG_INT32, m1, &motorPower.m1)
LOG_ADD(LOG_INT32, m2, &motorPower.m2)
LOG_ADD(LOG_INT32, m3, &motorPower.m3)
LOG_GROUP_STOP(motor)
