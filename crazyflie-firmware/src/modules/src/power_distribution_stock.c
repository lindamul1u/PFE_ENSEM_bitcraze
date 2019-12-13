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
#define PI 3.14159265358979323846
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
void powerDistributionENSEM( commande_t *commande, X_t *X){

	//[d4p,d2psi] -> w1 w2 w3 w4 -> pwm
	float w1=X->w1;
	float w2=X->w2;
	float w3=X->w3;
	float phi=X->phi;
	float theta=X->theta;
	float psi=X->psi;
	float dphi=X->dphi;
	float dtheta=X->dtheta;
	float dpsi=X->dpsi;
	float f=X->f;
	float df=X->df;
	float c1=commande->c1;
	float c2=commande->c2;
	float c3=commande->c3;
	float c4=commande->c4;



	float v1=(4350480122343661*w2*w3)/590295810358705651712 + (1029328319312993*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta))*(c1 - (2*df*w1 - f*w2*w3)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + f*(w1^2 + w2^2)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(psi)*cos(theta)*(2*df*w2 + f*w1*w3)))/(73786976294838206464*f) - (1029328319312993*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(c2 + (2*df*w1 - f*w2*w3)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - f*(w1^2 + w2^2)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - cos(theta)*sin(psi)*(2*df*w2 + f*w1*w3)))/(73786976294838206464*f) - (1029328319312993*cos(theta)*sin(phi)*(c3 + sin(theta)*(2*df*w2 + f*w1*w3) + cos(theta)*sin(phi)*(2*df*w1 - f*w2*w3) + f*cos(phi)*cos(theta)*(w1^2 + w2^2)))/(73786976294838206464*f);
	float v2=(8476647836751013*cos(psi)*cos(theta)*(c1 - (2*df*w1 - f*w2*w3)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + f*(w1^2 + w2^2)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(psi)*cos(theta)*(2*df*w2 + f*w1*w3)))/(590295810358705651712*f) - (8476647836751013*sin(theta)*(c3 + sin(theta)*(2*df*w2 + f*w1*w3) + cos(theta)*sin(phi)*(2*df*w1 - f*w2*w3) + f*cos(phi)*cos(theta)*(w1^2 + w2^2)))/(590295810358705651712*f) - (2296250702295365*w1*w3)/295147905179352825856 + (8476647836751013*cos(theta)*sin(psi)*(c2 + (2*df*w1 - f*w2*w3)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - f*(w1^2 + w2^2)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - cos(theta)*sin(psi)*(2*df*w2 + f*w1*w3)))/(590295810358705651712*f);
	float v3=(242021282247069*w1*w2)/590295810358705651712 + (6413563979547337*cos(theta)*(c4 - w2*((dphi*cos(phi))/cos(theta) + (dtheta*sin(phi)*sin(theta))/cos(theta)^2) + w3*((dphi*sin(phi))/cos(theta) - (dtheta*cos(phi)*sin(theta))/cos(theta)^2)))/(295147905179352825856*cos(phi)) + (6413563979547337*tan(phi)*sin(theta)*(c3 + sin(theta)*(2*df*w2 + f*w1*w3) + cos(theta)*sin(phi)*(2*df*w1 - f*w2*w3) + f*cos(phi)*cos(theta)*(w1^2 + w2^2)))/(295147905179352825856*f) - (6413563979547337*cos(psi)*cos(theta)*tan(phi)*(c1 - (2*df*w1 - f*w2*w3)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + f*(w1^2 + w2^2)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(psi)*cos(theta)*(2*df*w2 + f*w1*w3)))/(295147905179352825856*f) - (6413563979547337*cos(theta)*sin(psi)*tan(phi)*(c2 + (2*df*w1 - f*w2*w3)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - f*(w1^2 + w2^2)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - cos(theta)*sin(psi)*(2*df*w2 + f*w1*w3)))/(295147905179352825856*f);
	float v4=(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(c1 - (2*df*w1 - f*w2*w3)*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + f*(w1^2 + w2^2)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - cos(psi)*cos(theta)*(2*df*w2 + f*w1*w3)) - (cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*(c2 + (2*df*w1 - f*w2*w3)*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - f*(w1^2 + w2^2)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) - cos(theta)*sin(psi)*(2*df*w2 + f*w1*w3)) + cos(phi)*cos(theta)*(c3 + sin(theta)*(2*df*w2 + f*w1*w3) + cos(theta)*sin(phi)*(2*df*w1 - f*w2*w3) + f*cos(phi)*cos(theta)*(w1^2 + w2^2));
	float dt=X->dt;

	X->df=df+dt*v4;
	X->f=f+dt*df;
	f=X->f;
	float V1=(8160249282398746929266688*f)/381778773834299875 - (96191379038360232867659776*v3)/3054230190674399 - (60446290980731458735308800*2^(1/2)*v1)/3054230190674399 - (60446290980731458735308800*2^(1/2)*v2)/3054230190674399;
	float V2=(8160249282398746929266688*f)/381778773834299875 + (96191379038360232867659776*v3)/3054230190674399 - (60446290980731458735308800*2^(1/2)*v1)/3054230190674399 + (60446290980731458735308800*2^(1/2)*v2)/3054230190674399;
	float V3=(8160249282398746929266688*f)/381778773834299875 - (96191379038360232867659776*v3)/3054230190674399 + (60446290980731458735308800*2^(1/2)*v1)/3054230190674399 + (60446290980731458735308800*2^(1/2)*v2)/3054230190674399;
	float V4=(8160249282398746929266688*f)/381778773834299875 + (96191379038360232867659776*v3)/3054230190674399 + (60446290980731458735308800*2^(1/2)*v1)/3054230190674399 - (60446290980731458735308800*2^(1/2)*v2)/3054230190674399;
	float M1=(60.0f/(2*PI)*sqrt(V1)-4070.3)/0.2685;
	float M2=(60.0f/(2*PI)*sqrt(V2)-4070.3)/0.2685;
	float M3=(60.0f/(2*PI)*sqrt(V3)-4070.3)/0.2685;
	float M4=(60.0f/(2*PI)*sqrt(V4)-4070.3)/0.2685;
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
