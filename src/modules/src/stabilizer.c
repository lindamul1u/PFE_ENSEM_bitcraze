/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie Firmware
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
 *
 */
#define DEBUG_MODULE "STAB"

#include <math.h>

#include "FreeRTOS.h"
#include "task.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "debug.h"
#include "motors.h"
#include "pm.h"

#include "stabilizer.h"

#include "sensors.h"
#include "commander.h"
#include "crtp_localization_service.h"
#include "sitaw.h"
#include "controller.h"
#include "power_distribution.h"
#include "generateur_traj.h"

#include "estimator.h"
#include "usddeck.h"
#include "quatcompress.h"
#include "statsCnt.h"
#include "physicalConstants.h"

static bool isInit;
static bool emergencyStop = false;
static int emergencyStopTimeout = EMERGENCY_STOP_TIMEOUT_DISABLED;

#define PROPTEST_NBR_OF_VARIANCE_VALUES   100
static bool startPropTest = false;

uint32_t inToOutLatency;

// State variables for the stabilizer
static setpoint_t setpoint;
static sensorData_t sensorData;
static state_t state;
static control_t control;
static X_t reff;// structure reff
static X_t ST;// structure Modèle et mesure (vérité terrain)

static X_t error_state;//
static X_log_t Xlog;//Logging de X
static X_log_t Errlog;//
static X_log_t Refflog;//
static X_log_t STlog;//
static Logcommande_t powerLog;// Logging de powerENSEM
static Logcommande_t commandeLog;//
//static X_log_t Reflog;

static X_t X;// Structure correspondant à l'estimation d'etat


static commande_t commande;//commande V=[d4p;d2psi]-K(X-Xref)
static commande_t powerENSEM;// [M,f] et [wi_moteurs]
//pour voir la définition des structures aller voir dans stabilize_type.h
static StateEstimatorType estimatorType;
static ControllerType controllerType;
typedef enum { configureAcc, measureNoiseFloor, measureProp, testBattery, restartBatTest, evaluateResult, testDone } TestState;
#ifdef RUN_PROP_TEST_AT_STARTUP
static TestState testState = configureAcc;
#else
static TestState testState = testDone;
#endif
// Externalized ... => log (voir fin du fichier) Logging des structures à lire depuis python
void ExternalizedErr();
void ExternalizedReff();
void ExternalizedMf();
void ExternalizedCommande();
void ExternalizedST();
void ExternalizedST(){

	STlog.x1=(float) ST.x1;
	STlog.x2=(float) ST.x2;
	STlog.x3=(float) ST.x3;
	STlog.x4=(float) ST.x4;
	STlog.x5=(float) ST.x5;
	STlog.x6=(float) ST.x6;
	STlog.x7=(float) ST.x7;
	STlog.x8=(float) ST.x8;
	STlog.x9=(float) ST.x9;
	STlog.x10=(float) ST.x10;
	STlog.x11=(float) ST.x11;
	STlog.x12=(float) ST.x12;
	STlog.x13=(float) ST.x13;
	STlog.x14=(float) ST.x14;
	STlog.d4x=(float) ST.d4x;
	STlog.d4y=(float) ST.d4y;
	STlog.d4z=(float) ST.d4z;
	STlog.dpsi=(float) ST.x13;
	STlog.d2psi=(float) ST.x14;

	STlog.phi=(float) ST.phi;
	STlog.theta=(float) ST.theta;
	STlog.psi=(float) ST.psi;
	STlog.f=(float) ST.f;
	STlog.currenttime=(float) ST.currenttime;
	STlog.start=ST.start;

}
void ExternalizedErr(){
	Errlog.x1=(float) error_state.x1;
	Errlog.x2=(float) error_state.x2;
	Errlog.x3=(float) error_state.x3;
	Errlog.x4=(float) error_state.x4;
	Errlog.x5=(float) error_state.x5;
	Errlog.x6=(float) error_state.x6;
	Errlog.x7=(float) error_state.x7;
	Errlog.x8=(float) error_state.x8;
	Errlog.x9=(float) error_state.x9;
	Errlog.x10=(float) error_state.x10;
	Errlog.x11=(float) error_state.x11;
	Errlog.x12=(float) error_state.x12;
	Errlog.x13=(float) error_state.x13;
	Errlog.x14=(float) error_state.x14;

	Errlog.phi=(float) error_state.phi;
	Errlog.theta=(float) error_state.theta;
	Errlog.psi=(float) error_state.psi;
	Errlog.f=(float) error_state.f;




}
void ExternalizedReff(){

	Refflog.x1=(float) reff.x1;
	Refflog.x2=(float) reff.x2;
	Refflog.x3=(float) reff.x3;
	Refflog.x4=(float) reff.x4;
	Refflog.x5=(float) reff.x5;
	Refflog.x6=(float) reff.x6;
	Refflog.x7=(float) reff.x7;
	Refflog.x8=(float) reff.x8;
	Refflog.x9=(float) reff.x9;
	Refflog.x10=(float) reff.x10;
	Refflog.x11=(float) reff.x11;
	Refflog.x12=(float) reff.x12;
	Refflog.x13=(float) reff.x13;
	Refflog.x14=(float) reff.x14;

	Refflog.d4x=(float) reff.d4x;
	Refflog.d4y=(float) reff.d4y;
	Refflog.d4z=(float) reff.d4z;

	Refflog.dpsi=(float) reff.dpsi;
	Refflog.d2psi=(float) reff.d2psi;

	Refflog.phi=(float) reff.phi;
	Refflog.theta=(float) reff.theta;
	Refflog.psi=(float) reff.psi;




}
void ExternalizedMf(){
	powerLog.c1=(float)powerENSEM.c1;
	powerLog.c2=(float)powerENSEM.c2;
	powerLog.c3=(float)powerENSEM.c3;
	powerLog.c4=(float)powerENSEM.c4;
	powerLog.w1=(float) powerENSEM.w1;
	powerLog.w2=(float) powerENSEM.w2;
	powerLog.w3=(float) powerENSEM.w3;
	powerLog.w4=(float) powerENSEM.w4;

	powerLog.w1bc=(float) powerENSEM.w1bc;
	powerLog.w2bc=(float) powerENSEM.w2bc;
	powerLog.w3bc=(float) powerENSEM.w3bc;
	powerLog.w4bc=(float) powerENSEM.w4bc;
	powerLog.currenttime=(float) powerENSEM.currenttime;
	powerLog.start=powerENSEM.start;

}
void ExternalizedCommande(){
	commandeLog.c1=(float) commande.c1;
	commandeLog.c2=(float) commande.c2;
	commandeLog.c3=(float) commande.c3;
	commandeLog.c4=(float) commande.c4;
	commandeLog.currenttime=(float) commande.currenttime;
	commandeLog.start=commande.start;

}
void ExternalizedState(){
	Xlog.x1=(float) X.x1;
	Xlog.x2=(float) X.x2;
	Xlog.x3=(float) X.x3;
	Xlog.x4=(float) X.x4;
	Xlog.x5=(float) X.x5;
	Xlog.x6=(float) X.x6;
	Xlog.x7=(float) X.x7;
	Xlog.x8=(float) X.x8;
	Xlog.x9=(float) X.x9;
	Xlog.x10=(float) X.x10;
	Xlog.x11=(float) X.x11;
	Xlog.x12=(float) X.x12;
	Xlog.x13=(float) X.x13;
	Xlog.x14=(float) X.x14;
	Xlog.d4x=(float) X.d4x;
	Xlog.d4y=(float) X.d4y;
	Xlog.d4z=(float) X.d4z;
	Xlog.dpsi=(float) X.x13;
	Xlog.d2psi=(float) X.x14;

	Xlog.phi=((float) X.phi);
	Xlog.theta=((float) X.theta);
	Xlog.psi=((float) X.psi);
	Xlog.dphi=(float) X.dphi;
	Xlog.dtheta=(float) X.dtheta;
	Xlog.dpsi=(float) X.dpsi;
	Xlog.w1=((float) X.w1);
	Xlog.w2=((float) X.w2);
	Xlog.w3=((float) X.w3);
	Xlog.f=(float) X.f;
	Xlog.currenttime=(float) X.currenttime;
	Xlog.start=X.start;




}
static STATS_CNT_RATE_DEFINE(stabilizerRate, 100);// Boucle stabilizer task tourne à X hz (definir X  à 100, 250 ou 500 hZ)
static struct {
	// position - mm
	int16_t x;
	int16_t y;
	int16_t z;
	// velocity - mm / sec
	int16_t vx;
	int16_t vy;
	int16_t vz;
	// acceleration - mm / sec^2
	int16_t ax;
	int16_t ay;
	int16_t az;
	// compressed quaternion, see quatcompress.h
	int32_t quat;
	// angular velocity - milliradians / sec
	int16_t rateRoll;
	int16_t ratePitch;
	int16_t rateYaw;
} stateCompressed;

static struct {
	// position - mm
	int16_t x;
	int16_t y;
	int16_t z;
	// velocity - mm / sec
	int16_t vx;
	int16_t vy;
	int16_t vz;
	// acceleration - mm / sec^2
	int16_t ax;
	int16_t ay;
	int16_t az;
} setpointCompressed;

static float accVarX[NBR_OF_MOTORS];
static float accVarY[NBR_OF_MOTORS];
static float accVarZ[NBR_OF_MOTORS];
// Bit field indicating if the motors passed the motor test.
// Bit 0 - 1 = M1 passed
// Bit 1 - 1 = M2 passed
// Bit 2 - 1 = M3 passed
// Bit 3 - 1 = M4 passed
static uint8_t motorPass = 0;
static uint16_t motorTestCount = 0;


static void stabilizerTask(void* param);
static void testProps(sensorData_t *sensors);

static void calcSensorToOutputLatency(const sensorData_t *sensorData)
{
	uint64_t outTimestamp = usecTimestamp();
	inToOutLatency = outTimestamp - sensorData->interruptTimestamp;
}

static void compressState()
{
	stateCompressed.x = state.position.x * 1000.0f;
	stateCompressed.y = state.position.y * 1000.0f;
	stateCompressed.z = state.position.z * 1000.0f;

	stateCompressed.vx = state.velocity.x * 1000.0f;
	stateCompressed.vy = state.velocity.y * 1000.0f;
	stateCompressed.vz = state.velocity.z * 1000.0f;

	stateCompressed.ax = state.acc.x * 9.81f * 1000.0f;
	stateCompressed.ay = state.acc.y * 9.81f * 1000.0f;
	stateCompressed.az = (state.acc.z + 1) * 9.81f * 1000.0f;

	float const q[4] = {
			state.attitudeQuaternion.x,
			state.attitudeQuaternion.y,
			state.attitudeQuaternion.z,
			state.attitudeQuaternion.w};
	stateCompressed.quat = quatcompress(q);

	float const deg2millirad = ((float)M_PI * 1000.0f) / 180.0f;
	stateCompressed.rateRoll = sensorData.gyro.x * deg2millirad;
	stateCompressed.ratePitch = -sensorData.gyro.y * deg2millirad;
	stateCompressed.rateYaw = sensorData.gyro.z * deg2millirad;
}

static void compressSetpoint()
{
	setpointCompressed.x = setpoint.position.x * 1000.0f;
	setpointCompressed.y = setpoint.position.y * 1000.0f;
	setpointCompressed.z = setpoint.position.z * 1000.0f;

	setpointCompressed.vx = setpoint.velocity.x * 1000.0f;
	setpointCompressed.vy = setpoint.velocity.y * 1000.0f;
	setpointCompressed.vz = setpoint.velocity.z * 1000.0f;

	setpointCompressed.ax = setpoint.acceleration.x * 1000.0f;
	setpointCompressed.ay = setpoint.acceleration.y * 1000.0f;
	setpointCompressed.az = setpoint.acceleration.z * 1000.0f;
}

void stabilizerInit(StateEstimatorType estimator)
{
	if(isInit)
		return;

	sensorsInit();
	stateEstimatorInit(kalmanEstimator);//initialisation avec le kalman Bitcraze
	controllerInit(ControllerTypePID);//initialisation avec le PID
	powerDistributionInit();
	sitAwInit();
	estimatorType = getStateEstimator();
	controllerType = getControllerType();

	xTaskCreate(stabilizerTask, STABILIZER_TASK_NAME,
			STABILIZER_TASK_STACKSIZE, NULL, STABILIZER_TASK_PRI, NULL);// Task = "thread "

	isInit = true;
}

bool stabilizerTest(void)
{
	bool pass = true;

	pass &= sensorsTest();
	pass &= stateEstimatorTest();
	pass &= controllerTest();
	pass &= powerDistributionTest();

	return pass;
}

static void checkEmergencyStopTimeout()
{
	if (emergencyStopTimeout >= 0) {
		emergencyStopTimeout -= 1;

		if (emergencyStopTimeout == 0) {
			emergencyStop = true;
		}
	}
}

/* The stabilizer loop runs at 1kHz (stock) or 500Hz (kalman). It is the
 * responsibility of the different functions to run slower by skipping call
 * (ie. returning without modifying the output structure).
 */

void dynamique(commande_t *power,X_t *X,double dt);// Hardware in the loop : Permet de tester la commande plate sur notre modele
void dynamique(commande_t *power,X_t *X,double dt){
	double x1=   X->x1;
	double x2=   X->x2;
	double x3=   X->x3;

	double x4=   X->x4;
	double x5=   X->x5;
	double x6=   X->x6;

	double v1=power->c1;
	double v2=power->c2;
	double v3=power->c3;
	double f=power->c4;
	double phi=X->phi;
	double theta=X->theta;
	double psi=X->psi;
	double w1=X->w1;
	double w2=X->w2;
	double w3=X->w3;


	double j11=1.0e-04 *0.1395;
	double j22= 1.0e-04 *0.1436;
	double  j33=1.0e-04 *0.2173;

	double dw1=(v1 + j22*w2*w3 - j33*w2*w3)/j11;
	double dw2=(v2 - j11*w1*w3 + j33*w1*w3)/j22;
	double dw3=(v3 + j11*w1*w2 - j22*w1*w2)/j33;


	w1 =w1+dt*dw1;
	w2=w2+dt*dw2;
	w3=w3+dt*dw3;





	double dphi=w1 + w3*cos(phi)*tan(theta) + w2*sin(phi)*tan(theta);
	double dtheta= w2*cos(phi) - w3*sin(phi);
	double dpsi=(w3*cos(phi))/cos(theta) + (w2*sin(phi))/cos(theta);


	phi =phi + dt*dphi;


	theta =theta + dt*dtheta;


	psi =psi + dt*dpsi;

	double x7= f*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta));
	double x8=-f*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta));
	double x9=f*cos(phi)*cos(theta) - 9.81;
	double accx= f*cos(psi)*cos(theta)*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - sin(theta)*(f*cos(phi)*cos(theta) - 9.81) - f*cos(theta)*sin(psi)*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta));
	double accy=cos(theta)*sin(phi)*(f*cos(phi)*cos(theta) - 9.81) - f*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) - f*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta))*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta));
	double accz=f*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta))*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) + f*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta))*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + cos(phi)*cos(theta)*(f*cos(phi)*cos(theta) - 9.81);
	/*double x7=f*(sin(phi + dt*(w1 + w3*cos(phi)*tan(theta) + w2*sin(phi)*tan(theta)))*sin(psi + dt*((w3*cos(phi))/cos(theta) + (w2*sin(phi))/cos(theta))) + cos(phi + dt*(w1 + w3*cos(phi)*tan(theta) + w2*sin(phi)*tan(theta)))*cos(psi + dt*((w3*cos(phi))/cos(theta) + (w2*sin(phi))/cos(theta)))*sin(theta + dt*(w2*cos(phi) - w3*sin(phi))));
	 double x8= -f*(sin(phi + dt*(w1 + w3*cos(phi)*tan(theta) + w2*sin(phi)*tan(theta)))*cos(psi + dt*((w3*cos(phi))/cos(theta) + (w2*sin(phi))/cos(theta))) - cos(phi + dt*(w1 + w3*cos(phi)*tan(theta) + w2*sin(phi)*tan(theta)))*sin(psi + dt*((w3*cos(phi))/cos(theta) + (w2*sin(phi))/cos(theta)))*sin(theta + dt*(w2*cos(phi) - w3*sin(phi))));
	 double x9=f*cos(phi + dt*(w1 + w3*cos(phi)*tan(theta) + w2*sin(phi)*tan(theta)))*cos(theta + dt*(w2*cos(phi) - w3*sin(phi))) - 9.81;


	     double accx=x7*cos(psi)*cos(theta) - x9*sin(theta) + x8*cos(theta)*sin(psi);
		double accy=x8*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - x7*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + x9*cos(theta)*sin(phi);
		double accz=x7*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - x8*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + x9*cos(phi)*cos(theta);
	 */


	x6=x6+dt*x9;
	x5=x5+dt*x8;
	x4=x4+dt*x7;
	x3=x3+dt*x6+0.5*dt*dt*x9;
	x2=x2+dt*x5+0.5*dt*dt*x8;
	x1=x1+dt*x4+0.5*dt*dt*x7;


	(X->x1)=   x1;
	(X->x2)=   x2;
	(X->x3)=   x3;

	(X->x4)=   x4;
	(X->x5)=   x5;
	(X->x6)=   x6;

	(X->x7)=   x7;
	(X->x8)=   x8;
	(X->x9)=   x9;

	(X->phi)=   phi;
	(X->theta)=   theta;
	(X->psi)=   psi;
	//	(X->x13)=   psi;

	(X->w1)=   w1;
	(X->w2)=   w2;
	(X->w3)=   w3;
	X->accbx=accx;
	X->accby=accy;
	X->accbz=accz;
	X->start=1;




}
double t;// temps


static void stabilizerTask(void* param)
{
	uint32_t tick;
	uint32_t lastWakeTime;
	vTaskSetApplicationTaskTag(0, (void*)TASK_STABILIZER_ID_NBR);

	//Wait for the system to be fully started to start stabilization loop
	systemWaitStart();

	DEBUG_PRINT("Wait for sensor calibration...\n");

	// Wait for sensors to be calibrated
	lastWakeTime = xTaskGetTickCount ();
	while(!sensorsAreCalibrated()) {
		vTaskDelayUntil(&lastWakeTime, F2T(RATE_MAIN_LOOP));
	}
	// Initialize tick to something else then 0
	tick = 1;
	t=usecTimestamp() / 1e6;
// init attribut
	X.f=9.81;
	X.dt=0.0;
	X.df=0.0;
	ST.f=9.81;
	ST.psi=0;
	X.x1=0.0;
	X.x2=0.0;


	X.currenttime=t;

	reff.dt=0.0;

	Xlog.f=9.81;
	reff.currenttime=t;
	DEBUG_PRINT("Ready to fly.\n");
	// trajset(&reff,&X,&Piecetraj);

	while(1) {
		// The sensor should unlock at 1kHz
		sensorsWaitDataReady();


		if (startPropTest != false) {
			// TODO: What happens with estimator when we run tests after startup?
			testState = configureAcc;
			startPropTest = false;
		}

		if (testState != testDone) {
			sensorsAcquire(&sensorData, tick);
			testProps(&sensorData);
		} else {

			if(controllerType==ControllerTypeENSEM && estimatorType==kalmanENSEMEstimator){// Si on utilise le controleur/Estimateur de l'ensem alors ..
				stateEstimator(kalmanENSEMEstimator,&reff,&X,&error_state,&ST,&commande,&state, &sensorData, &control, tick);
				//trajset(&reff,&state0,&X,toffset);
				commanderGetSetpoint(&setpoint, &state);
				reff.x1=(double)setpoint.position.x;
				reff.x2=(double)setpoint.position.y;
				reff.x3=(double)setpoint.position.z;
				reff.x4= (double)setpoint.velocity.x;
				reff.x5= (double)setpoint.velocity.y;
				reff.x6= (double)setpoint.velocity.z;
				reff.x7= (double)setpoint.acceleration.x;
				reff.x8=  (double)setpoint.acceleration.y;
				reff.x9=  (double)setpoint.acceleration.z;
				reff.x10= (double)setpoint.jerk.x;
				reff.x11= (double)setpoint.jerk.y;
				reff.x12= (double)setpoint.jerk.z;
				reff.x13= (double)setpoint.psi;
				reff.x14= (double)setpoint.dpsi;
				reff.d4x=(double) setpoint.d4p.x;
				reff.d4y=(double) setpoint.d4p.y;
				reff.d4z=(double) setpoint.d4p.z;
				reff.d2psi=(double) setpoint.d2psi;
				reff.dpsi=(double) setpoint.dpsi;
				reff.psi=(double) setpoint.psi;
				//sitAwUpdateSetpoint(&setpoint, &sensorData, &state);


				if(X.isInit){// X is Init est mis à jours en modifiant le parametre Stabiliser.isInit depuis python (relier à la variable isInit de X)
					controller(ControllerTypeENSEM,&control, &setpoint,&commande,&reff,&X, &sensorData, &state, tick);
					controller(ControllerTypePID,&control, &setpoint,&commande,&reff,&X, &sensorData, &state, tick);
					ST.x1=state.position.x;
					ST.x2=state.position.y;
					ST.x3=state.position.z;
					ST.x4=state.velocity.x;
					ST.x5=state.velocity.y;
					ST.x6=state.velocity.z;
					ST.x7=(state.acc.x);
					ST.x8=(state.acc.y);
					ST.x9=(state.acc.z);
/*ControllerTypeMellinger
					ST.x1=state.position.x;
					ST.x2=state.position.y;
					ST.x3=state.position.z;
					ST.x4=state.velocity.x;
					ST.x5=state.velocity.y;
					ST.x6=state.velocity.z;
					ST.x7=(state.acc.x)*9.81f;
					ST.x8=(state.acc.y)*9.81f;
					ST.x9=(state.acc.z)*9.81f;*/
/*
					X.w1=(sensorData.gyro.x* 0.0175f);
						X.w2=(sensorData.gyro.y* 0.0175f);
						X.w3=(sensorData.gyro.z* 0.0175f);*/
				}
				else{



					ST.x1=state.position.x;
					ST.x2=state.position.y;
					ST.x3=state.position.z;
					ST.x4=state.velocity.x;
					ST.x5=state.velocity.y;
					ST.x6=state.velocity.z;
					ST.x7=(state.acc.x)*9.81f;
					ST.x8=(state.acc.y)*9.81f;
					ST.x9=(state.acc.z)*9.81f;


					commande.c1=0;
					commande.c2=0;
					commande.c3=0;
					commande.c4=0;
					commande.start=0;
					X.x4=0.0;
					X.x5=0.0;
					X.x6=0.0;
					X.x7=0.0;
					X.x8=0.0;
					X.x9=0.0;
					X.x10=0.0;
					X.x11=0.0;
					X.x12=0.0;
					X.x13=0.0;
					X.x14=0.0;
					X.phi=0.0;
					X.theta=0.0;
					X.psi=0.0;
					X.dphi=0.0;
					X.dtheta=0.0;
					X.dpsi=0.0;
					X.w1=0.0;
					X.w2=0.0;
					X.w3=0.0;
					/*
					X.w1=(sensorData.gyro.x* 0.0175f);
						X.w2=(sensorData.gyro.y* 0.0175f);
						X.w3=(sensorData.gyro.z* 0.0175f);
					*/

					commande.start=0;
					powerENSEM.start=0;
					powerENSEM.c3=0.0;
					powerENSEM.c2=0.0;
					powerENSEM.c1=0.0;
					powerENSEM.c4=9.81;



					reff.currenttime=0.0;
					reff.Morceau_traj=0;
					reff.isInit=0;

					reff.x1=(double)setpoint.position.x;
					reff.x2=(double)setpoint.position.y;
					reff.x3=(double)setpoint.position.z;
					reff.x4= (double)setpoint.velocity.x;
					reff.x5= (double)setpoint.velocity.y;
					reff.x6= (double)setpoint.velocity.z;
					reff.x7= (double)setpoint.acceleration.x;
					reff.x8=  (double)setpoint.acceleration.y;
					reff.x9=  (double)setpoint.acceleration.z;
					reff.x10= (double)setpoint.jerk.x;
					reff.x11= (double)setpoint.jerk.y;
					reff.x12= (double)setpoint.jerk.z;
					reff.x13= (double)setpoint.psi;
					reff.x14= (double)setpoint.dpsi;
					reff.dpsi=(double) setpoint.dpsi;
					reff.psi=(double) setpoint.psi;


					X.f=9.81;
					X.df=0.0;
					ST.f=9.81;
					ST.df=0.0;

					controller(ControllerTypePID,&control, &setpoint,&commande,&reff,&X, &sensorData, &state, tick);
				}
				//consolePrintf("c1 %f c2 %f c3 %f c4 %f\n",(double) commande.c1,(double) commande.c2,(double) commande.c3,(double) commande.c4);
			}
			else{
				stateEstimator(kalmanEstimator,&reff,&X,&error_state,&ST,&commande,&state, &sensorData, &control, tick);
				compressState();
				commanderGetSetpoint(&setpoint, &state);

				// inititialisation : decolage avec le pid dX/dt=0 et V=0 df=0
				commande.c1=0;
				commande.c2=0;
				commande.c3=0;
				commande.c4=0;
				X.x1=0.0;
				X.x2=0.0;
				X.x3=0.0;
				X.x4=0.0;
				X.x5=0.0;
				X.x6=0.0;
				X.x7=0.0;
				X.x8=0.0;
				X.x9=0.0;
				X.x10=0.0;
				X.x11=0.0;
				X.x12=0.0;
				X.x13=0.0;
				X.x14=0.0;
				X.phi=0.0;
				X.theta=0.0;
				X.psi=0.0;
				X.dphi=0.0;
				X.dtheta=0.0;
				X.dpsi=0.0;
				X.w1=0.0;
				X.w2=0.0;
				X.w3=0.0;
				X.start=0;
				commande.start=0;
				powerENSEM.start=0;
				powerENSEM.c3=0.0;
				powerENSEM.c2=0.0;
				powerENSEM.c1=0.0;
				powerENSEM.c4=9.81;
				X.df=0.0;
				ST.df=0.0;
				X.f=9.81;
				ST.f=9.81;

				reff.currenttime=0.0;
				reff.Morceau_traj=0;
				reff.isInit=0;

				reff.x1=(double)setpoint.position.x;
				reff.x2=(double)setpoint.position.y;
				reff.x3=(double)setpoint.position.z;
				reff.x4= (double)setpoint.velocity.x;
				reff.x5= (double)setpoint.velocity.y;
				reff.x6= (double)setpoint.velocity.z;
				reff.x7= (double)setpoint.acceleration.x;
				reff.x8=  (double)setpoint.acceleration.y;
				reff.x9=  (double)setpoint.acceleration.z;
				reff.x10= (double)setpoint.jerk.x;
				reff.x11= (double)setpoint.jerk.y;
				reff.x12= (double)setpoint.jerk.z;
				reff.x13= (double)setpoint.psi;
				reff.x14= (double)setpoint.dpsi;
				reff.dpsi=(double) setpoint.dpsi;
				reff.psi=(double) setpoint.psi;
				compressSetpoint();
				sitAwUpdateSetpoint(&setpoint, &sensorData, &state);
				controller(ControllerTypePID,&control, &setpoint,&commande,&reff,&X, &sensorData, &state, tick);
			}
			//consolePrintf("f %f  \n",(double) X.f);
			checkEmergencyStopTimeout();
			if (emergencyStop) {
				powerStop();
			} else {

				if(controllerType==ControllerTypeENSEM && estimatorType==kalmanENSEMEstimator){
					//powerDistribution(&control,&powerLog);
					if(X.isInit){
						//powerDistribution(&control,&powerENSEM);

						powerDistributionENSEM(&powerENSEM,&commande,&X);

						//dynamique(&powerENSEM,&ST,X.dt);
					}
					else{
						powerDistribution(&control,&powerENSEM);
					}

				}
				else{
					powerDistribution(&control,&powerENSEM);
				}
				ExternalizedST();
				ExternalizedMf();
				ExternalizedCommande();
				ExternalizedState();
				ExternalizedErr();
				ExternalizedReff();
			}

			// Log data to uSD card if configured
			if (   usddeckLoggingEnabled()
					&& usddeckLoggingMode() == usddeckLoggingMode_SynchronousStabilizer
					&& RATE_DO_EXECUTE(usddeckFrequency(), tick)) {
				usddeckTriggerLogging();
			}
		}
		t=(usecTimestamp() / 1e6);
		X.dt=t-X.currenttime;
		X.currenttime=t;
		reff.dt=t-reff.currenttime;
		reff.currenttime=t;
		ST.currenttime=t;
		powerENSEM.currenttime=t;
		commande.currenttime=t;
		calcSensorToOutputLatency(&sensorData);
		tick++;
		STATS_CNT_RATE_EVENT(&stabilizerRate);



	}
}

void stabilizerSetEmergencyStop()
{
	emergencyStop = true;
}

void stabilizerResetEmergencyStop()
{
	emergencyStop = false;
}

void stabilizerSetEmergencyStopTimeout(int timeout)
{
	emergencyStop = false;
	emergencyStopTimeout = timeout;
}

static float variance(float *buffer, uint32_t length)
{
	uint32_t i;
	float sum = 0;
	float sumSq = 0;

	for (i = 0; i < length; i++)
	{
		sum += buffer[i];
		sumSq += buffer[i] * buffer[i];
	}

	return sumSq - (sum * sum) / length;
}

/** Evaluate the values from the propeller test
 * @param low The low limit of the self test
 * @param high The high limit of the self test
 * @param value The value to compare with.
 * @param string A pointer to a string describing the value.
 * @return True if self test within low - high limit, false otherwise
 */
static bool evaluateTest(float low, float high, float value, uint8_t motor)
{
	if (value < low || value > high)
	{
		DEBUG_PRINT("Propeller test on M%d [FAIL]. low: %0.2f, high: %0.2f, measured: %0.2f\n",
				motor + 1, (double)low, (double)high, (double)value);
		return false;
	}

	motorPass |= (1 << motor);

	return true;
}


static void testProps(sensorData_t *sensors)
{
	static uint32_t i = 0;
	static float accX[PROPTEST_NBR_OF_VARIANCE_VALUES];
	static float accY[PROPTEST_NBR_OF_VARIANCE_VALUES];
	static float accZ[PROPTEST_NBR_OF_VARIANCE_VALUES];
	static float accVarXnf;
	static float accVarYnf;
	static float accVarZnf;
	static int motorToTest = 0;
	static uint8_t nrFailedTests = 0;
	static float idleVoltage;
	static float minSingleLoadedVoltage[NBR_OF_MOTORS];
	static float minLoadedVoltage;

	if (testState == configureAcc)
	{
		motorPass = 0;
		sensorsSetAccMode(ACC_MODE_PROPTEST);
		testState = measureNoiseFloor;
		minLoadedVoltage = idleVoltage = pmGetBatteryVoltage();
		minSingleLoadedVoltage[MOTOR_M1] = minLoadedVoltage;
		minSingleLoadedVoltage[MOTOR_M2] = minLoadedVoltage;
		minSingleLoadedVoltage[MOTOR_M3] = minLoadedVoltage;
		minSingleLoadedVoltage[MOTOR_M4] = minLoadedVoltage;
	}
	if (testState == measureNoiseFloor)
	{
		accX[i] = sensors->acc.x;
		accY[i] = sensors->acc.y;
		accZ[i] = sensors->acc.z;

		if (++i >= PROPTEST_NBR_OF_VARIANCE_VALUES)
		{
			i = 0;
			accVarXnf = variance(accX, PROPTEST_NBR_OF_VARIANCE_VALUES);
			accVarYnf = variance(accY, PROPTEST_NBR_OF_VARIANCE_VALUES);
			accVarZnf = variance(accZ, PROPTEST_NBR_OF_VARIANCE_VALUES);
			DEBUG_PRINT("Acc noise floor variance X+Y:%f, (Z:%f)\n",
					(double)accVarXnf + (double)accVarYnf, (double)accVarZnf);
			testState = measureProp;
		}

	}
	else if (testState == measureProp)
	{
		if (i < PROPTEST_NBR_OF_VARIANCE_VALUES)
		{
			accX[i] = sensors->acc.x;
			accY[i] = sensors->acc.y;
			accZ[i] = sensors->acc.z;
			if (pmGetBatteryVoltage() < minSingleLoadedVoltage[motorToTest])
			{
				minSingleLoadedVoltage[motorToTest] = pmGetBatteryVoltage();
			}
		}
		i++;

		if (i == 1)
		{
			motorsSetRatio(motorToTest, 0xFFFF);
		}
		else if (i == 50)
		{
			motorsSetRatio(motorToTest, 0);
		}
		else if (i == PROPTEST_NBR_OF_VARIANCE_VALUES)
		{
			accVarX[motorToTest] = variance(accX, PROPTEST_NBR_OF_VARIANCE_VALUES);
			accVarY[motorToTest] = variance(accY, PROPTEST_NBR_OF_VARIANCE_VALUES);
			accVarZ[motorToTest] = variance(accZ, PROPTEST_NBR_OF_VARIANCE_VALUES);
			DEBUG_PRINT("Motor M%d variance X+Y:%f (Z:%f)\n",
					motorToTest+1, (double)accVarX[motorToTest] + (double)accVarY[motorToTest],
					(double)accVarZ[motorToTest]);
		}
		else if (i >= 1000)
		{
			i = 0;
			motorToTest++;
			if (motorToTest >= NBR_OF_MOTORS)
			{
				i = 0;
				motorToTest = 0;
				testState = evaluateResult;
				sensorsSetAccMode(ACC_MODE_FLIGHT);
			}
		}
	}
	else if (testState == testBattery)
	{
		if (i == 0)
		{
			minLoadedVoltage = idleVoltage = pmGetBatteryVoltage();
		}
		if (i == 1)
		{
			motorsSetRatio(MOTOR_M1, 0xFFFF);
			motorsSetRatio(MOTOR_M2, 0xFFFF);
			motorsSetRatio(MOTOR_M3, 0xFFFF);
			motorsSetRatio(MOTOR_M4, 0xFFFF);
		}
		else if (i < 50)
		{
			if (pmGetBatteryVoltage() < minLoadedVoltage)
				minLoadedVoltage = pmGetBatteryVoltage();
		}
		else if (i == 50)
		{
			motorsSetRatio(MOTOR_M1, 0);
			motorsSetRatio(MOTOR_M2, 0);
			motorsSetRatio(MOTOR_M3, 0);
			motorsSetRatio(MOTOR_M4, 0);
			//      DEBUG_PRINT("IdleV: %f, minV: %f, M1V: %f, M2V: %f, M3V: %f, M4V: %f\n", (double)idleVoltage,
			//                  (double)minLoadedVoltage,
			//                  (double)minSingleLoadedVoltage[MOTOR_M1],
			//                  (double)minSingleLoadedVoltage[MOTOR_M2],
			//                  (double)minSingleLoadedVoltage[MOTOR_M3],
			//                  (double)minSingleLoadedVoltage[MOTOR_M4]);
			DEBUG_PRINT("%f %f %f %f %f %f\n", (double)idleVoltage,
					(double)(idleVoltage - minLoadedVoltage),
					(double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M1]),
					(double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M2]),
					(double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M3]),
					(double)(idleVoltage - minSingleLoadedVoltage[MOTOR_M4]));
			testState = restartBatTest;
			i = 0;
		}
		i++;
	}
	else if (testState == restartBatTest)
	{
		if (i++ > 2000)
		{
			testState = configureAcc;
			i = 0;
		}
	}
	else if (testState == evaluateResult)
	{
		for (int m = 0; m < NBR_OF_MOTORS; m++)
		{
			if (!evaluateTest(0, PROPELLER_BALANCE_TEST_THRESHOLD,  accVarX[m] + accVarY[m], m))
			{
				nrFailedTests++;
				for (int j = 0; j < 3; j++)
				{
					motorsBeep(m, true, testsound[m], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
					vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
					motorsBeep(m, false, 0, 0);
					vTaskDelay(M2T(100));
				}
			}
		}
#ifdef PLAY_STARTUP_MELODY_ON_MOTORS
		if (nrFailedTests == 0)
		{
			for (int m = 0; m < NBR_OF_MOTORS; m++)
			{
				motorsBeep(m, true, testsound[m], (uint16_t)(MOTORS_TIM_BEEP_CLK_FREQ / A4)/ 20);
				vTaskDelay(M2T(MOTORS_TEST_ON_TIME_MS));
				motorsBeep(m, false, 0, 0);
				vTaskDelay(M2T(MOTORS_TEST_DELAY_TIME_MS));
			}
		}
#endif
		motorTestCount++;
		testState = testDone;
	}
}
PARAM_GROUP_START(health)
PARAM_ADD(PARAM_UINT8, startPropTest, &startPropTest)
PARAM_GROUP_STOP(health)


PARAM_GROUP_START(stabilizer)
PARAM_ADD(PARAM_UINT8, estimator, &estimatorType)
PARAM_ADD(PARAM_UINT8, controller, &controllerType)
PARAM_ADD(PARAM_UINT8, stop, &emergencyStop)
PARAM_ADD(PARAM_UINT8, isInit, &X.isInit)
PARAM_GROUP_STOP(stabilizer)

LOG_GROUP_START(health)
LOG_ADD(LOG_FLOAT, motorVarXM1, &accVarX[0])
LOG_ADD(LOG_FLOAT, motorVarYM1, &accVarY[0])
LOG_ADD(LOG_FLOAT, motorVarXM2, &accVarX[1])
LOG_ADD(LOG_FLOAT, motorVarYM2, &accVarY[1])
LOG_ADD(LOG_FLOAT, motorVarXM3, &accVarX[2])
LOG_ADD(LOG_FLOAT, motorVarYM3, &accVarY[2])
LOG_ADD(LOG_FLOAT, motorVarXM4, &accVarX[3])
LOG_ADD(LOG_FLOAT, motorVarYM4, &accVarY[3])
LOG_ADD(LOG_UINT8, motorPass, &motorPass)
LOG_ADD(LOG_UINT16, motorTestCount, &motorTestCount)
LOG_GROUP_STOP(health)

LOG_GROUP_START(ctrltarget)
LOG_ADD(LOG_FLOAT, x, &setpoint.position.x)
LOG_ADD(LOG_FLOAT, y, &setpoint.position.y)
LOG_ADD(LOG_FLOAT, z, &setpoint.position.z)

LOG_ADD(LOG_FLOAT, vx, &setpoint.velocity.x)
LOG_ADD(LOG_FLOAT, vy, &setpoint.velocity.y)
LOG_ADD(LOG_FLOAT, vz, &setpoint.velocity.z)

LOG_ADD(LOG_FLOAT, ax, &setpoint.acceleration.x)
LOG_ADD(LOG_FLOAT, ay, &setpoint.acceleration.y)
LOG_ADD(LOG_FLOAT, az, &setpoint.acceleration.z)

LOG_ADD(LOG_FLOAT, roll, &setpoint.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &setpoint.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &setpoint.attitudeRate.yaw)
LOG_GROUP_STOP(ctrltarget)

LOG_GROUP_START(ctrltargetZ)
LOG_ADD(LOG_INT16, x, &setpointCompressed.x)   // position - mm
LOG_ADD(LOG_INT16, y, &setpointCompressed.y)
LOG_ADD(LOG_INT16, z, &setpointCompressed.z)

LOG_ADD(LOG_INT16, vx, &setpointCompressed.vx) // velocity - mm / sec
LOG_ADD(LOG_INT16, vy, &setpointCompressed.vy)
LOG_ADD(LOG_INT16, vz, &setpointCompressed.vz)

LOG_ADD(LOG_INT16, ax, &setpointCompressed.ax) // acceleration - mm / sec^2
LOG_ADD(LOG_INT16, ay, &setpointCompressed.ay)
LOG_ADD(LOG_INT16, az, &setpointCompressed.az)
LOG_GROUP_STOP(ctrltargetZ)

LOG_GROUP_START(stabilizer)
LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)
LOG_ADD(LOG_FLOAT, thrust, &control.thrust)

STATS_CNT_RATE_LOG_ADD(rtStab, &stabilizerRate)
LOG_ADD(LOG_UINT32, intToOut, &inToOutLatency)
LOG_GROUP_STOP(stabilizer)

LOG_GROUP_START(acc)
LOG_ADD(LOG_FLOAT, x, &sensorData.acc.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.acc.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.acc.z)
LOG_GROUP_STOP(acc)


#ifdef LOG_SEC_IMU
LOG_GROUP_START(accSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.accSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.accSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.accSec.z)
LOG_GROUP_STOP(accSec)
#endif

LOG_GROUP_START(baro)
LOG_ADD(LOG_FLOAT, asl, &sensorData.baro.asl)
LOG_ADD(LOG_FLOAT, temp, &sensorData.baro.temperature)
LOG_ADD(LOG_FLOAT, pressure, &sensorData.baro.pressure)
LOG_GROUP_STOP(baro)

LOG_GROUP_START(gyro)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyro.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyro.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyro.z)
LOG_GROUP_STOP(gyro)





#ifdef LOG_SEC_IMU
LOG_GROUP_START(gyroSec)
LOG_ADD(LOG_FLOAT, x, &sensorData.gyroSec.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.gyroSec.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.gyroSec.z)
LOG_GROUP_STOP(gyroSec)
#endif

LOG_GROUP_START(mag)
LOG_ADD(LOG_FLOAT, x, &sensorData.mag.x)
LOG_ADD(LOG_FLOAT, y, &sensorData.mag.y)
LOG_ADD(LOG_FLOAT, z, &sensorData.mag.z)
LOG_GROUP_STOP(mag)

LOG_GROUP_START(controller)
LOG_ADD(LOG_INT16, ctr_yaw, &control.yaw)
LOG_GROUP_STOP(controller)


LOG_GROUP_START(Commandelog)

LOG_ADD(LOG_FLOAT, c1, &commandeLog.c1)
LOG_ADD(LOG_FLOAT, c2, &commandeLog.c2)
LOG_ADD(LOG_FLOAT, c3, &commandeLog.c3)
STATS_CNT_RATE_LOG_ADD(rtStab, &stabilizerRate)

LOG_ADD(LOG_FLOAT, c4, &commandeLog.c4)
LOG_ADD(LOG_FLOAT, tc, &commandeLog.currenttime)
LOG_ADD(LOG_INT16, startc, &commandeLog.start)




LOG_GROUP_STOP(Commandelog)
LOG_GROUP_START(Powerlog)

LOG_ADD(LOG_FLOAT, m1, &powerLog.c1)
LOG_ADD(LOG_FLOAT, m2, &powerLog.c2)
LOG_ADD(LOG_FLOAT, m3, &powerLog.c3)

LOG_ADD(LOG_FLOAT, f, &powerLog.c4)
STATS_CNT_RATE_LOG_ADD(rtStab, &stabilizerRate)

LOG_ADD(LOG_FLOAT, w1, &powerLog.w1)
LOG_ADD(LOG_FLOAT, w2, &powerLog.w2)
LOG_ADD(LOG_FLOAT, w3, &powerLog.w3)

LOG_ADD(LOG_FLOAT, w4, &powerLog.w4)
LOG_ADD(LOG_FLOAT, w1bcbc, &powerLog.w1bc)
LOG_ADD(LOG_FLOAT, w2bc, &powerLog.w2bc)
LOG_ADD(LOG_FLOAT, w3bc, &powerLog.w3bc)

LOG_ADD(LOG_FLOAT, w4bc, &powerLog.w4bc)
LOG_ADD(LOG_FLOAT, tp, &powerLog.currenttime)
LOG_ADD(LOG_INT16, startp, &powerLog.start)

LOG_GROUP_STOP(Powerlog)

LOG_GROUP_START(Refflog)
STATS_CNT_RATE_LOG_ADD(rtStab, &stabilizerRate)

LOG_ADD(LOG_FLOAT, x1, &Refflog.x1)
LOG_ADD(LOG_FLOAT, x2, &Refflog.x2)
LOG_ADD(LOG_FLOAT, x3, &Refflog.x3)

LOG_ADD(LOG_FLOAT, x7, &Refflog.x7)
LOG_ADD(LOG_FLOAT, x8, &Refflog.x8)
LOG_ADD(LOG_FLOAT, x9, &Refflog.x9)

LOG_ADD(LOG_FLOAT, x4, &Refflog.x4)
LOG_ADD(LOG_FLOAT, x5, &Refflog.x5)
LOG_ADD(LOG_FLOAT, x6, &Refflog.x6)

LOG_ADD(LOG_FLOAT, x10, &Refflog.x10)
LOG_ADD(LOG_FLOAT, x11, &Refflog.x11)
LOG_ADD(LOG_FLOAT, x12, &Refflog.x12)

LOG_ADD(LOG_FLOAT, d4x, &Refflog.d4x)
LOG_ADD(LOG_FLOAT, d4y, &Refflog.d4y)
LOG_ADD(LOG_FLOAT, d4z, &Refflog.d4z)

LOG_ADD(LOG_FLOAT, dpsi, &Refflog.dpsi)
LOG_ADD(LOG_FLOAT, d2psi, &Refflog.d2psi)



LOG_ADD(LOG_FLOAT, phi, &Refflog.phi)
LOG_ADD(LOG_FLOAT, theta, &Refflog.theta)

LOG_ADD(LOG_FLOAT, psi, &Refflog.psi)


LOG_GROUP_STOP(Refflog)


LOG_GROUP_START(STlog)

STATS_CNT_RATE_LOG_ADD(rtStab, &stabilizerRate)
LOG_ADD(LOG_FLOAT, x1, &STlog.x1)
LOG_ADD(LOG_FLOAT, x2, &STlog.x2)
LOG_ADD(LOG_FLOAT, x3, &STlog.x3)

LOG_ADD(LOG_FLOAT, x7, &STlog.x7)
LOG_ADD(LOG_FLOAT, x8, &STlog.x8)
LOG_ADD(LOG_FLOAT, x9, &STlog.x9)

LOG_ADD(LOG_FLOAT, x4, &STlog.x4)
LOG_ADD(LOG_FLOAT, x5, &STlog.x5)
LOG_ADD(LOG_FLOAT, x6, &STlog.x6)

LOG_ADD(LOG_FLOAT, x10, &STlog.x10)
LOG_ADD(LOG_FLOAT, x11, &STlog.x11)
LOG_ADD(LOG_FLOAT, x12, &STlog.x12)

LOG_ADD(LOG_FLOAT, d4x, &STlog.d4x)
LOG_ADD(LOG_FLOAT, d4y, &STlog.d4y)
LOG_ADD(LOG_FLOAT, d4z, &STlog.d4z)

LOG_ADD(LOG_FLOAT, dpsi, &STlog.dpsi)
LOG_ADD(LOG_FLOAT, d2psi, &STlog.d2psi)



LOG_ADD(LOG_FLOAT, phi, &STlog.phi)
LOG_ADD(LOG_FLOAT, theta, &STlog.theta)

LOG_ADD(LOG_FLOAT, psi, &STlog.psi)
LOG_ADD(LOG_FLOAT,f,&STlog.f)
LOG_ADD(LOG_FLOAT,t,&STlog.currenttime)
LOG_ADD(LOG_INT16,starte,&STlog.start)



LOG_GROUP_STOP(STlog)


LOG_GROUP_START(Xlog)
STATS_CNT_RATE_LOG_ADD(rtStab, &stabilizerRate)
LOG_ADD(LOG_FLOAT, x1, &Xlog.x1)
LOG_ADD(LOG_FLOAT, x2, &Xlog.x2)
LOG_ADD(LOG_FLOAT, x3, &Xlog.x3)

LOG_ADD(LOG_FLOAT, x7, &Xlog.x7)
LOG_ADD(LOG_FLOAT, x8, &Xlog.x8)
LOG_ADD(LOG_FLOAT, x9, &Xlog.x9)

LOG_ADD(LOG_FLOAT, x4, &Xlog.x4)
LOG_ADD(LOG_FLOAT, x5, &Xlog.x5)
LOG_ADD(LOG_FLOAT, x6, &Xlog.x6)

LOG_ADD(LOG_FLOAT, x10, &Xlog.x10)
LOG_ADD(LOG_FLOAT, x11, &Xlog.x11)
LOG_ADD(LOG_FLOAT, x12, &Xlog.x12)

LOG_ADD(LOG_FLOAT, d4x, &Xlog.d4x)
LOG_ADD(LOG_FLOAT, d4y, &Xlog.d4y)
LOG_ADD(LOG_FLOAT, d4z, &Xlog.d4z)

LOG_ADD(LOG_FLOAT, dpsi, &Xlog.dpsi)
LOG_ADD(LOG_FLOAT, d2psi, &Xlog.d2psi)



LOG_ADD(LOG_FLOAT, phi, &Xlog.phi)
LOG_ADD(LOG_FLOAT, theta, &Xlog.theta)

LOG_ADD(LOG_FLOAT, psi, &Xlog.psi)

LOG_ADD(LOG_FLOAT, dphi, &Xlog.dphi)
LOG_ADD(LOG_FLOAT, dtheta, &Xlog.dtheta)

LOG_ADD(LOG_FLOAT, dpsi, &Xlog.dpsi)

LOG_ADD(LOG_FLOAT, w1, &Xlog.w1)
LOG_ADD(LOG_FLOAT, w2, &Xlog.w2)

LOG_ADD(LOG_FLOAT, w3, &Xlog.w3)
LOG_ADD(LOG_FLOAT,f,&Xlog.f)
LOG_ADD(LOG_FLOAT,t,&Xlog.currenttime)
LOG_ADD(LOG_INT16,starte,&Xlog.start)



LOG_GROUP_STOP(Xlog)


LOG_GROUP_START(stateEstimate)
LOG_ADD(LOG_FLOAT, x, &state.position.x)
LOG_ADD(LOG_FLOAT, y, &state.position.y)
LOG_ADD(LOG_FLOAT, z, &state.position.z)

LOG_ADD(LOG_FLOAT, vx, &state.velocity.x)
LOG_ADD(LOG_FLOAT, vy, &state.velocity.y)
LOG_ADD(LOG_FLOAT, vz, &state.velocity.z)

LOG_ADD(LOG_FLOAT, ax, &state.acc.x)
LOG_ADD(LOG_FLOAT, ay, &state.acc.y)
LOG_ADD(LOG_FLOAT, az, &state.acc.z)

LOG_ADD(LOG_FLOAT, roll, &state.attitude.roll)
LOG_ADD(LOG_FLOAT, pitch, &state.attitude.pitch)
LOG_ADD(LOG_FLOAT, yaw, &state.attitude.yaw)

LOG_ADD(LOG_FLOAT, qx, &state.attitudeQuaternion.x)
LOG_ADD(LOG_FLOAT, qy, &state.attitudeQuaternion.y)
LOG_ADD(LOG_FLOAT, qz, &state.attitudeQuaternion.z)
LOG_ADD(LOG_FLOAT, qw, &state.attitudeQuaternion.w)
LOG_GROUP_STOP(stateEstimate)


