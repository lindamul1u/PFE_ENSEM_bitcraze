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
void powerDistributionENSEM(const commande_t *commande, X_t *X){

	//[d4p,d2psi] -> w1 w2 w3 w4 -> pwm
	double w1=X->w1;
	double w2=X->w2;
	double w3=X->w3;
	double phi=X->phi;
	double theta=X->theta;
	double psi=X->psi;
	double dphi=X->dphi;
	double dtheta=X->dtheta;
	double f=X->f;
	double df=X->df;
	double c1=commande->c1;
	double c2=commande->c2;
	double c3=commande->c3;
	double c4=commande->c4;

	double v1=  0.00000737*w2*w3 - (0.00001395*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(c2 + (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(2.0*df*w1 - 1.0*f*w2*w3) - 1.0*f*(w1*w1 + w2*w2)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - cos(theta)*sin(psi)*(2.0*df*w2 + f*w1*w3)))/f + (0.00001395*(cos(phi)*sin(psi) - 1.0*cos(psi)*sin(phi)*sin(theta))*(c1 - (cos(phi)*sin(psi) - 1.0*cos(psi)*sin(phi)*sin(theta))*(2.0*df*w1 - 1.0*f*w2*w3) + f*(w1*w1 + w2*w2)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(psi)*cos(theta)*(2.0*df*w2 + f*w1*w3)))/f - (0.00001395*cos(theta)*sin(phi)*(c3 + sin(theta)*(2.0*df*w2 + f*w1*w3) + cos(theta)*sin(phi)*(2.0*df*w1 - 1.0*f*w2*w3) + f*cos(phi)*cos(theta)*(w1*w1 + w2*w2)))/f;
	double v2=(0.00001436*cos(psi)*cos(theta)*(c1 - (cos(phi)*sin(psi) - 1.0*cos(psi)*sin(phi)*sin(theta))*(2.0*df*w1 - 1.0*f*w2*w3) + f*(w1*w1 + w2*w2)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(psi)*cos(theta)*(2.0*df*w2 + f*w1*w3)))/f - (0.00001436*sin(theta)*(c3 + sin(theta)*(2.0*df*w2 + f*w1*w3) + cos(theta)*sin(phi)*(2.0*df*w1 - 1.0*f*w2*w3) + f*cos(phi)*cos(theta)*(w1*w1 + w2*w2)))/f - 0.00000778*w1*w3 + (0.00001436*cos(theta)*sin(psi)*(c2 + (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(2.0*df*w1 - 1.0*f*w2*w3) - 1.0*f*(w1*w1 + w2*w2)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - cos(theta)*sin(psi)*(2.0*df*w2 + f*w1*w3)))/f;
	double v3=(0.00000001*(2173.0*c4*f*cos(theta)*cos(theta) + 2173.0*c3*cos(theta)*sin(phi)*sin(theta) - 2173.0*c1*cos(psi)*cos(theta)*cos(theta)*sin(phi) - 2173.0*c2*cos(theta)*cos(theta)*sin(phi)*sin(psi) + 4346.0*df*w2*cos(theta)*sin(phi) - 2173.0*dphi*f*w2*cos(phi)*cos(theta) + 2173.0*dphi*f*w3*cos(theta)*sin(phi) - 2173.0*dtheta*f*w3*cos(phi)*sin(theta) + 41.0*f*w1*w2*cos(phi)*cos(theta) - 2173.0*dtheta*f*w2*sin(phi)*sin(theta) + 2173.0*f*w1*w3*cos(theta)*sin(phi)))/(f*cos(phi)*cos(theta));
	double v4= (sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(c1 - (cos(phi)*sin(psi) - 1.0*cos(psi)*sin(phi)*sin(theta))*(2.0*df*w1 - 1.0*f*w2*w3) + f*(w1*w1 + w2*w2)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(psi)*cos(theta)*(2.0*df*w2 + f*w1*w3)) - 1.0*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*(c2 + (cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(2.0*df*w1 - 1.0*f*w2*w3) - 1.0*f*(w1*w1 + w2*w2)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - cos(theta)*sin(psi)*(2.0*df*w2 + f*w1*w3)) + cos(phi)*cos(theta)*(c3 + sin(theta)*(2.0*df*w2 + f*w1*w3) + cos(theta)*sin(phi)*(2.0*df*w1 - 1.0*f*w2*w3) + f*cos(phi)*cos(theta)*(w1*w1 + w2*w2));

	double dt=X->dt;

	X->df=df+dt*v4;
	X->f=f+dt*df;
	f=X->f;
	double V1=21374288.0*f - 2.798871e+10*v1 - 2.798871e+10*v2 - 3.1494476e+10*v3;
	double V2= 21374288.0*f - 2.798871e+10*v1 + 2.798871e+10*v2 + 3.1494476e+10*v3;
	double V3=21374288.0*f + 2.798871e+10*v1 + 2.798871e+10*v2 - 3.1494476e+10*v3;
	double V4=21374288.0*f + 2.798871e+10*v1 - 2.798871e+10*v2 + 3.1494476e+10*v3;
	double M1=(9.5493*sqrt(V1)-4070.3)/0.2685;
	double M2=(9.5493*sqrt(V2)-4070.3)/0.2685;
	double M3=(9.5493*sqrt(V3)-4070.3)/0.2685;
	double M4=(9.5493*sqrt(V4)-4070.3)/0.2685;
 		motorPower.m1= (uint32_t) M1;
		motorPower.m2=(uint32_t) M2;
		motorPower.m3=(uint32_t) M3;
		motorPower.m4=(uint32_t) M4;

		motorsSetRatio(MOTOR_M1, motorPower.m1);
		motorsSetRatio(MOTOR_M2, motorPower.m2);
		motorsSetRatio(MOTOR_M3, motorPower.m3);
		motorsSetRatio(MOTOR_M4, motorPower.m4);
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
