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

void powerStop()
{
	motorsSetRatio(MOTOR_M1, 0);
	motorsSetRatio(MOTOR_M2, 0);
	motorsSetRatio(MOTOR_M3, 0);
	motorsSetRatio(MOTOR_M4, 0);
}
void powerDistributionENSEM(commande_t *powerENSEM, commande_t *commande, X_t *X){

	//[d4p,d2psi] -> w1 w2 w3 w4 -> pwm

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
		double   dphi= (double) X->dphi;
		double   dtheta= (double) X->dtheta;
		double f=(double) X->f;
		double df=(double) X->df;
		double d2f=(double) X->d2f;
		double  dt= (double) X->dt;


		f=f+dt*df+0.5*dt*dt*d2f;
		df= df+dt*d2f;



		double v1= 0.00002132*w2*w3 - (0.0000279*df*w1 - 0.00001395*c1*cos(phi)*cos(psi) + 0.00001395*c2*cos(phi)*sin(psi) + 0.00001395*c3*cos(theta)*sin(phi) + 0.00001395*c2*cos(psi)*sin(phi)*sin(theta) + 0.00001395*c1*sin(phi)*sin(psi)*sin(theta))/f;
		double v2=- (0.00002872*df*w2 + 0.00001436*c3*sin(theta) - 0.00001436*c2*cos(psi)*cos(theta) - 0.00001436*c1*cos(theta)*sin(psi))/f - 0.00002214*w1*w3;
		double v3=(0.00002173*c4*cos(theta) - 0.00002173*dphi*w2*cos(phi) + 0.00002173*dphi*w3*sin(phi) + 4.1e-7*w1*w2*cos(phi) + 0.00002173*w1*w3*sin(phi) - 0.00002173*dtheta*w2*sin(phi)*tan(theta) - 0.00002173*dtheta*w3*cos(phi)*tan(theta))/(2.0*cos(phi/2)*cos(phi/2) - 1.0) + (0.00004346*df*w2*sin(phi) + 0.00002173*c3*sin(phi)*sin(theta) - 0.00002173*c2*cos(psi)*cos(theta)*sin(phi) - 0.00002173*c1*cos(theta)*sin(phi)*sin(psi))/(f*(2.0*cos(phi/2)*cos(phi/2) - 1.0));
		double v4=f*w1*w1 + f*w2*w2 + c3*cos(phi)*cos(theta) + c1*cos(psi)*sin(phi) - 1.0*c2*sin(phi)*sin(psi) + c2*cos(phi)*cos(psi)*sin(theta) + c1*cos(phi)*sin(psi)*sin(theta);
		//double v4= (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(c1 - (cos(phi)*sin(psi) - 1.0*cos(psi)*sin(phi)*sin(theta))*(2.0*df*w1 - 1.0*f*w2*w3) + f*(w1*w1 + w2*w2)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(psi)*cos(theta)*(2.0*df*w2 + f*w1*w3)) - 1.0*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*(c2 + (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(2.0*df*w1 - 1.0*f*w2*w3) - 1.0*f*(w1*w1 + w2*w2)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - cos(theta)*sin(psi)*(2.0*df*w2 + f*w1*w3)) + cos(phi)*cos(theta)*(c3 + sin(theta)*(2.0*df*w2 + f*w1*w3) + cos(theta)*sin(phi)*(2.0*df*w1 - 1.0*f*w2*w3) + f*cos(phi)*cos(theta)*(w1*w1 + w2*w2));


		//f=X->f;
		//df=X->df;


		X->f=(float) f;
		X->df=(float) df;
		X->d2f=(float) v4;

		/*
	double  V1=2.1e+7*f - 2.8e+10*v1 - 2.8e+10*v2 - 3.1e+10*v3
		double  V2=2.1e+7*f - 2.8e+10*v1 + 2.8e+10*v2 + 3.1e+10*v3;
		double  V3=2.1e+7*f + 2.8e+10*v1 + 2.8e+10*v2 - 3.1e+10*v3;
		double  V4=2.1e+7*f + 2.8e+10*v1 - 2.8e+10*v2 + 3.1e+10*v3;*/


		double  V1=	 2.1374e+7*f - 2.7989e+10*v1 - 2.7989e+10*v2 - 3.1494e+10*v3;
		double  V2=	 2.1374e+7*f - 2.7989e+10*v1 + 2.7989e+10*v2 + 3.1494e+10*v3;
		double  V3= 2.1374e+7*f + 2.7989e+10*v1 + 2.7989e+10*v2 - 3.1494e+10*v3;
		double V4= 2.1374e+7*f + 2.7989e+10*v1 - 2.7989e+10*v2 + 3.1494e+10*v3;
		double  M1=(sqrt(V1)-4070.3)/0.2685;
		double  M2=(sqrt(V2)-4070.3)/0.2685;
		double  M3=(sqrt(V3)-4070.3)/0.2685;
		double  M4=(sqrt(V4)-4070.3)/0.2685;
		powerENSEM->c1=(float) M1;
		powerENSEM->c2= (float) M2;
		powerENSEM->c3= (float) M3;
		powerENSEM->c4= (float) M4;



	//consolePrintf("f %f \n",(double) X->f);







	/*double  V1=21374288.0*f - 2.798871e+10*v1 - 2.798871e+10*v2 - 3.1494476e+10*v3;
	double  V2= 21374288.0*f - 2.798871e+10*v1 + 2.798871e+10*v2 + 3.1494476e+10*v3;
	double  V3=21374288.0*f + 2.798871e+10*v1 + 2.798871e+10*v2 - 3.1494476e+10*v3;
	double  V4=21374288.0*f + 2.798871e+10*v1 - 2.798871e+10*v2 + 3.1494476e+10*v3;

	 */


	/*
 		motorPower.m1= (uint32_t) M1;
		motorPower.m2=(uint32_t) M2;
		motorPower.m3=(uint32_t) M3;
		motorPower.m4=(uint32_t) M4;

		motorsSetRatio(MOTOR_M1, motorPower.m1);
		motorsSetRatio(MOTOR_M2, motorPower.m2);
		motorsSetRatio(MOTOR_M3, motorPower.m3);
		motorsSetRatio(MOTOR_M4, motorPower.m4);*/
}

void powerDistribution(const control_t *control)
{
#ifdef QUAD_FORMATION_X
	int16_t r = control->roll / 2.0f;
	int16_t p = control->pitch / 2.0f;
	motorPower.m1 = limitThrust(control->thrust - r + p + control->yaw);
	motorPower.m2 = limitThrust(control->thrust - r - p - control->yaw);
	motorPower.m3 =  limitThrust(control->thrust + r - p + control->yaw);
	motorPower.m4 =  limitThrust(control->thrust + r + p - control->yaw);
#else // QUAD_FORMATION_NORMAL
	motorPower.m1 = limitThrust(control->thrust + control->pitch +
			control->yaw);
	motorPower.m2 = limitThrust(control->thrust - control->roll -
			control->yaw);
	motorPower.m3 =  limitThrust(control->thrust - control->pitch +
			control->yaw);
	motorPower.m4 =  limitThrust(control->thrust + control->roll -
			control->yaw);
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
