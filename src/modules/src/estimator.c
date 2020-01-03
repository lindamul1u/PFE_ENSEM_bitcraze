#define DEBUG_MODULE "ESTIMATOR"

#include "cfassert.h"
#include "estimator.h"
#include "estimator_complementary.h"
#include "estimator_kalman.h"
#include "physicalConstants.h"
#include "debug.h"

#define DEFAULT_ESTIMATOR complementaryEstimator
static StateEstimatorType currentEstimator = anyEstimator;
static double pi =3.14159265358979323846;
static void initEstimator(const StateEstimatorType estimator);
static void deinitEstimator(const StateEstimatorType estimator);

typedef struct {
	void (*init)(void);
	void (*deinit)(void);
	bool (*test)(void);
	void (*update)(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick);
	const char* name;
	bool (*estimatorEnqueueTDOA)(const tdoaMeasurement_t *uwb);
	bool (*estimatorEnqueuePosition)(const positionMeasurement_t *pos);
	bool (*estimatorEnqueuePose)(const poseMeasurement_t *pose);
	bool (*estimatorEnqueueDistance)(const distanceMeasurement_t *dist);
	bool (*estimatorEnqueueTOF)(const tofMeasurement_t *tof);
	bool (*estimatorEnqueueAbsoluteHeight)(const heightMeasurement_t *height);
	bool (*estimatorEnqueueFlow)(const flowMeasurement_t *flow);
	bool (*estimatorEnqueueYawError)(const yawErrorMeasurement_t *error);
	bool (*estimatorEnqueueSweepAngles)(const sweepAngleMeasurement_t *angles);
} EstimatorFcns;

#define NOT_IMPLEMENTED ((void*)0)

static EstimatorFcns estimatorFunctions[] = {
		{
				.init = NOT_IMPLEMENTED,
				.deinit = NOT_IMPLEMENTED,
				.test = NOT_IMPLEMENTED,
				.update = NOT_IMPLEMENTED,
				.name = "None",
				.estimatorEnqueueTDOA = NOT_IMPLEMENTED,
				.estimatorEnqueuePosition = NOT_IMPLEMENTED,
				.estimatorEnqueuePose = NOT_IMPLEMENTED,
				.estimatorEnqueueDistance = NOT_IMPLEMENTED,
				.estimatorEnqueueTOF = NOT_IMPLEMENTED,
				.estimatorEnqueueAbsoluteHeight = NOT_IMPLEMENTED,
				.estimatorEnqueueFlow = NOT_IMPLEMENTED,
				.estimatorEnqueueYawError = NOT_IMPLEMENTED,
				.estimatorEnqueueSweepAngles = NOT_IMPLEMENTED,
		}, // Any estimator
		{
				.init = estimatorComplementaryInit,
				.deinit = NOT_IMPLEMENTED,
				.test = estimatorComplementaryTest,
				.update = estimatorComplementary,
				.name = "Complementary",
				.estimatorEnqueueTDOA = NOT_IMPLEMENTED,
				.estimatorEnqueuePosition = NOT_IMPLEMENTED,
				.estimatorEnqueuePose = NOT_IMPLEMENTED,
				.estimatorEnqueueDistance = NOT_IMPLEMENTED,
				.estimatorEnqueueTOF = estimatorComplementaryEnqueueTOF,
				.estimatorEnqueueAbsoluteHeight = NOT_IMPLEMENTED,
				.estimatorEnqueueFlow = NOT_IMPLEMENTED,
				.estimatorEnqueueYawError = NOT_IMPLEMENTED,
				.estimatorEnqueueSweepAngles = NOT_IMPLEMENTED,
		},
		{
				.init = estimatorKalmanInit,
				.deinit = NOT_IMPLEMENTED,
				.test = estimatorKalmanTest,
				.update = estimatorKalman,
				.name = "Kalman",
				.estimatorEnqueueTDOA = estimatorKalmanEnqueueTDOA,
				.estimatorEnqueuePosition = estimatorKalmanEnqueuePosition,
				.estimatorEnqueuePose = estimatorKalmanEnqueuePose,
				.estimatorEnqueueDistance = estimatorKalmanEnqueueDistance,
				.estimatorEnqueueTOF = estimatorKalmanEnqueueTOF,
				.estimatorEnqueueAbsoluteHeight = estimatorKalmanEnqueueAbsoluteHeight,
				.estimatorEnqueueFlow = estimatorKalmanEnqueueFlow,
				.estimatorEnqueueYawError = estimatorKalmanEnqueueYawError,
				.estimatorEnqueueSweepAngles = estimatorKalmanEnqueueSweepAngles,
		},
		{
				.init = estimatorKalmanInit,
				.deinit = NOT_IMPLEMENTED,
				.test = estimatorKalmanTest,
				.update = estimatorKalman,
				.name = "Kalman",
				.estimatorEnqueueTDOA = estimatorKalmanEnqueueTDOA,
				.estimatorEnqueuePosition = estimatorKalmanEnqueuePosition,
				.estimatorEnqueuePose = estimatorKalmanEnqueuePose,
				.estimatorEnqueueDistance = estimatorKalmanEnqueueDistance,
				.estimatorEnqueueTOF = estimatorKalmanEnqueueTOF,
				.estimatorEnqueueAbsoluteHeight = estimatorKalmanEnqueueAbsoluteHeight,
				.estimatorEnqueueFlow = estimatorKalmanEnqueueFlow,
				.estimatorEnqueueYawError = estimatorKalmanEnqueueYawError,
				.estimatorEnqueueSweepAngles = estimatorKalmanEnqueueSweepAngles,
		},

};

void stateEstimatorInit(StateEstimatorType estimator) {
	stateEstimatorSwitchTo(estimator);
}

void stateEstimatorSwitchTo(StateEstimatorType estimator) {
	if (estimator < 0 || estimator >= StateEstimatorTypeCount) {
		return;
	}

	StateEstimatorType newEstimator = estimator;

	if (anyEstimator == newEstimator) {
		newEstimator = DEFAULT_ESTIMATOR;
	}

	StateEstimatorType forcedEstimator = ESTIMATOR_NAME;
	if (forcedEstimator != anyEstimator) {
		DEBUG_PRINT("Estimator type forced\n");
		newEstimator = forcedEstimator;
	}

	initEstimator(newEstimator);
	StateEstimatorType previousEstimator = currentEstimator;
	currentEstimator = newEstimator;
	deinitEstimator(previousEstimator);

	DEBUG_PRINT("Using %s (%d) estimator\n", stateEstimatorGetName(), currentEstimator);
}

StateEstimatorType getStateEstimator(void) {
	return currentEstimator;
}

static void initEstimator(const StateEstimatorType estimator) {
	if (estimatorFunctions[estimator].init) {
		estimatorFunctions[estimator].init();
	}
}

static void deinitEstimator(const StateEstimatorType estimator) {
	if (estimatorFunctions[estimator].deinit) {
		estimatorFunctions[estimator].deinit();
	}
}

bool stateEstimatorTest(void) {
	return estimatorFunctions[currentEstimator].test();
}

void stateEstimator(X_t *reff,X_t *X, commande_t *commande,state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {
	if(currentEstimator==kalmanENSEMEstimator){
		TestingEstimator(state,reff);

		//estimatorFunctions[currentEstimator].update(state, sensors, control, tick);
		ENSEMkalmanupdate(X,commande,state);
	}
	else{
		estimatorFunctions[currentEstimator].update(state, sensors, control, tick);
	}
}

void TestingEstimator(state_t * state,X_t * reff){
	state->position.x=reff->x1;
	state->position.y=reff->x2;
	state->position.z=reff->x3;
		float x7=reff->x7;
		float x8=reff->x8;
		float x9=reff->x9;
	state->acc.x=(float) (x7);
	state->acc.y=(float) (x8);
	state->acc.z=(float) (x9);
	state->attitude.yaw=reff->x13;






}

void ENSEMkalmanupdate(X_t *X, commande_t *commande,state_t *state){

	// Xtilde(n)

	double x1= (double) (X->x1);
	double x2= (double) (X->x2);
	double x3= (double) (X->x3);

	double x4= (double) (X->x4);
	double x5= (double) (X->x5);
	double x6= (double) (X->x6);

	double x7= (double) (X->x7);
	double x8= (double) (X->x8);
	double x9= (double) (X->x9);


	double x10= (double) (X->x10);
	double x11= (double) (X->x11);
	double x12= (double) (X->x12);

	double x13= (double) (X->x13);
	double x14= (double) (X->x14);
	//V(n+1)
	double c1= (double) (commande->c1);
	double c2= (double) (commande->c2);
	double c3= (double) (commande->c3);
	double c4= (double) (commande->c4);

	//Y(n+1)
	double y1= (double) (state->position.x);
	double y2= (double) (state->position.y);
	double y3= (double) (state->position.z);
	double y4= (double) ((state->acc.x));
	double y5= (double) ((state->acc.y));
	double y6= (double) ((state->acc.z));
	double y7= (double) (state->attitude.yaw);
	y7=y7*pi/180.0d;

	//Xtilde(n+1)=(Ad-Ld*Cd)X_tilde(n)+Bd*V(n+1)+Ld*Y(n+1)
/*

	double x_tilde1=  0.94474121*x1 + 0.002*x4 + 0.000002*x7  + 0.055258788*y1;
	double x_tilde2=  0.94474121*x2 + 0.002*x5 + 0.000002*x8 +   0.055258788*y2;
	double x_tilde3=  0.94474121*x3 + 0.002*x6 + 0.000002*x9 +  0.055258788*y3;
	double x_tilde4=  0.053275965*x1 + x4 - 0.000028583049*x7 + 0.000002*x10 + 0.053275965*y1 + 0.002028583*y4;
	double x_tilde5= 0.053275965*x2 + x5 - 0.000028583049*x8 + 0.000002*x11 + 0.053275965*y2 + 0.002028583*y5;
	double x_tilde6= 0.053275965*x3 + x6 - 0.000028583049*x9 + 0.000002*x12 + 0.053275965*y3 + 0.002028583*y6;
	double x_tilde7=0.000002*c1 + 0.9447425*x7 + 0.002*x10 + 0.055257501*y4;
	double x_tilde8=0.000002*c2 + 0.9447425*x8 + 0.002*x11 + 0.055257501*y5;
	double x_tilde9=0.000002*c3 + 0.9447425*x9 + 0.002*x12 + 0.055257501*y6;
	double x_tilde10=0.002*c1 - 0.053240457*x7 + x10 + 0.053240457*y4;
	double x_tilde11= 0.002*c2 - 0.053240457*x8 + x11 + 0.053240457*y5;
	double x_tilde12= 0.002*c3 - 0.053240457*x9 + x12 + 0.053240457*y6;
	double x_tilde13=0.000002*c4 + 0.94474249*x13 + 0.002*x14 + 0.055257512*y7;
	double x_tilde14=    0.002*c4 - 0.053240463*x13 + x14 + 0.053240463*y7;*/
	//DEBUG_PRINT("x %f \n",(double) x_tilde1);
	//DEBUG_PRINT("x (%f)  \n", x_tilde1 );

	double x_tilde1=4.17e-10*c1 + 0.937*x1 + 0.01*x4 + 5.0e-5*x7 + 1.67e-7*x10 + 0.0627*y1;
	double x_tilde2=4.17e-10*c2 + 0.937*x2 + 0.01*x5 + 5.0e-5*x8 + 1.67e-7*x11 + 0.0627*y2;
	double x_tilde3=4.17e-10*c3 + 0.937*x3 + 0.01*x6 + 5.0e-5*x9 + 1.67e-7*x12 + 0.0627*y3;
	double x_tilde4=1.67e-7*c1 - 0.0539*x1 + x4 - 1.71e-4*x7 + 5.0e-5*x10 + 0.0539*y1 + 0.0102*y4;
	double x_tilde5=1.67e-7*c2 - 0.0539*x2 + x5 - 1.71e-4*x8 + 5.0e-5*x11 + 0.0539*y2 + 0.0102*y5;
	double x_tilde6=1.67e-7*c3 - 0.0539*x3 + x6 - 1.71e-4*x9 + 5.0e-5*x12 + 0.0539*y3 + 0.0102*y6;
	double x_tilde7= 5.0e-5*c1 + 0.937*x7 + 0.01*x10 + 0.0625*y4;
	double x_tilde8= 5.0e-5*c2 + 0.937*x8 + 0.01*x11 + 0.0625*y5;
	double x_tilde9= 5.0e-5*c3 + 0.937*x9 + 0.01*x12 + 0.0625*y6;
	double x_tilde10= 0.01*c1 - 0.053*x7 + x10 + 0.053*y4;
	double x_tilde11= 0.01*c2 - 0.053*x8 + x11 + 0.053*y5;
	double x_tilde12= 0.01*c3 - 0.053*x9 + x12 + 0.053*y6;
	double x_tilde13=5.0e-5*c4 + 0.937*x13 + 0.01*x14 + 0.0625*y7;
	double x_tilde14= 0.01*c4 - 0.053*x13 + x14 + 0.053*y7;
	(X->x1)= (float) x_tilde1;
	(X->x2)= (float) x_tilde2;
	(X->x3)= (float) x_tilde3;

	(X->x4)= (float) x_tilde4;
	(X->x5)= (float) x_tilde5;
	(X->x6)= (float) x_tilde6;

	(X->x7)= (float) x_tilde7;
	(X->x8)= (float) x_tilde8;
	(X->x9)= (float) x_tilde9;

	(X->x10)= (float) x_tilde10;
	(X->x11)= (float) x_tilde11;
	(X->x12)= (float) x_tilde12;

	(X->x13)= (float) x_tilde13;
	(X->x14)= (float) x_tilde14;
	(X->psi)= (float) (X->x13);
	(X->dpsi)= (float) (X->x14);

	x1= (double) (X->x1);
	x2= (double) (X->x2);
	x3= (double) (X->x3);

	x4= (double) (X->x4);
	x5= (double) (X->x5);
	x6= (double) (X->x6);

	x7= (double) (X->x7);
	x8= (double) (X->x8);
	x9= (double) (X->x9);


	x10= (double) (X->x10);
	x11= (double) (X->x11);
	x12= (double) (X->x12);

	x13= (double) (X->x13);
	x14= (double) (X->x14);
	double psi= (double) x13;
	double dpsi=(double) x14;


	//float f=sqrt(x7*x7+x8*x8+(x9+9.81)*(x9+9.81));


	double phi=-1.0*atan2((7.07*fabs(100.0*x9 + 981.0)*(x8*cos(psi) - x7*sin(psi))),((100.0*x9 + 981.0)*sqrt(981.0*x9 + 50.0*x7*x7*cos(psi)*cos(psi) + 50.0*x8*x8*sin(psi)*sin(psi) + 50.0*x9*x9 + 100.0*x7*x8*cos(psi)*sin(psi) + 4810.0)));
	double theta=atan2((100.0*(x7*cos(psi) + x8*sin(psi))),(100.0*x9 + 981.0));

	double dphi=(100*dpsi*(x7*cos(psi) + x8*sin(psi)))/((100.0*x9 + 981.0)*sqrt((1.96e+5*x9 + 1.0e+4*x7*x7*cos(psi)*cos(psi) + 1.0e+4*x8*x8*sin(psi)*sin(psi) + 1.0e+4*x9*x9 + 2.0e+4*x7*x8*cos(psi)*sin(psi) + 9.62e+5)/(1.0e+4*x9*x9 + 1.96e+5*x9 + 9.62e+5))) - (25.0*(100.0*x9 + 981.0)*(x7*cos(psi + 1.57) + x8*sin(psi + 1.57))*(981.0*x7*x10 + 981.0*x8*x11 - 100.0*x7*x7*x12 - 100.0*x8*x8*x12 + 100.0*x7*x9*x10 + 100.0*x8*x9*x11))/(sqrt(cos(psi + 1.57)*cos(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + sin(psi + 1.57)*sin(psi + 1.57)*sin(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 1.0e+4*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57))*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57)))*(50.0*x7*x7 + 50.0*x8*x8 + 50.0*x9*x9 + 981.0*x9 + 4810.0)*(50.0*x7*x7 + 50.0*x8*x8 + 50.0*x9*x9 + 981.0*x9 + 4810.0)) - (0.5*(1.0e+4*cos(psi + 1.57)*x8*x8 - 1.0e+4*x7*sin(psi + 1.57)*x8 + 1.0e+4*cos(psi + 1.57)*x9*x9 + 1.96e+5*cos(psi + 1.57)*x9 + 9.62e+5*cos(psi + 1.57))*(9620.0*x10 - 981.0*x7*x12 + 1960.0*x9*x10 + 100.0*x8*x8*x10 + 100.0*x9*x9*x10 - 100.0*x7*x8*x11 - 100.0*x7*x9*x12))/(sqrt(cos(psi + 1.57)*cos(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + sin(psi + 1.57)*sin(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 1.0e+4*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57))*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57)))*(50.0*x7*x7 + 50.0*x8*x8 + 50.0*x9*x9 + 981.0*x9 + 4810.0)*(100.0*x7*x7 + 100.0*x8*x8 + 100.0*x9*x9 + 1960.0*x9 + 9620.0)) - (0.5*(1.0e+4*sin(psi + 1.57)*x7*x7 - 1.0e+4*x8*cos(psi + 1.57)*x7 + 1.0e+4*sin(psi + 1.57)*x9*x9 + 1.96e+5*sin(psi + 1.57)*x9 + 9.62e+5*sin(psi + 1.57))*(9620.0*x11 - 981.0*x8*x12 + 1960.0*x9*x11 + 100.0*x7*x7*x11 + 100.0*x9*x9*x11 - 100.0*x7*x8*x10 - 100.0*x8*x9*x12))/(sqrt(cos(psi + 1.57)*cos(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + sin(psi + 1.57)*sin(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 1.0e+4*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57))*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57)))*(50.0*x7*x7 + 50.0*x8*x8 + 50.0*x9*x9 + 981.0*x9 + 4810.0)*(100.0*x7*x7 + 100.0*x8*x8 + 100.0*x9*x9 + 1960.0*x9 + 9620.0));
	double dtheta=-sqrt(1/(981.0*x9 + 25.0*x7*x7*cos(2.0*psi) - 25.0*x8*x8*cos(2.0*psi) + 25.0*x7*x7 + 25.0*x8*x8 + 50.0*x9*x9 + 50.0*x7*x8*sin(2.0*psi) + 4810.0))*((50.0*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57))*(981.0*x7*x10 + 981.0*x8*x11 - 100.0*x7*x7*x12 - 100.0*x8*x8*x12 + 100.0*x7*x9*x10 + 100.0*x8*x9*x11))/(sqrt(cos(psi + 1.57)*cos(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + sin(psi + 1.57)*sin(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 1.0e+4*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57))*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57)))*sqrt(x7*x7 + x8*x8 + x9*x9 + 19.6*x9 + 96.2)*(50.0*x7*x7 + 50.0*x8*x8 + 50.0*x9*x9 + 981.0*x9 + 4810.0)) + (100.0*cos(psi + 1.57)*(x9 + 9.81)*(9620.0*x11 - 981.0*x8*x12 + 1960.0*x9*x11 + 100.0*x7*x7*x11 + 100.0*x9*x9*x11 - 100.0*x7*x8*x10 - 100.0*x8*x9*x12))/(sqrt(cos(psi + 1.57)*cos(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + sin(psi + 1.57)*sin(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 1.0e+4*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57))*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57)))*sqrt(x7*x7 + x8*x8 + x9*x9 + 19.6*x9 + 96.2)*(100.0*x7*x7 + 100.0*x8*x8 + 100.0*x9*x9 + 1960.0*x9 + 9620.0)) - (100.0*sin(psi + 1.57)*(x9 + 9.81)*(9620.0*x10 - 981.0*x7*x12 + 1960.0*x9*x10 + 100.0*x8*x8*x10 + 100.0*x9*x9*x10 - 100.0*x7*x8*x11 - 100.0*x7*x9*x12))/(sqrt(cos(psi + 1.57)*cos(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + sin(psi + 1.57)*sin(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 1.0e+4*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57))*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57)))*sqrt(x7*x7 + x8*x8 + x9*x9 + 19.6*x9 + 96.2)*(100.0*x7*x7 + 100.0*x8*x8 + 100.0*x9*x9 + 1960.0*x9 + 9620.0)) - (1.0*dpsi*(100.0*x9 + 981.0)*(100.0*x9 + 981.0)*(x8*cos(psi) - x7*sin(psi)))/((x9 + 9.81)*sqrt(1/(981.0*x9 + 25.0*x7*x7*cos(2.0*psi) - 25.0*x8*x8*cos(2.0*psi) + 25.0*x7*x7 + 25.0*x8*x8 + 50.0*x9*x9 + 50.0*x7*x8*sin(2.0*psi) + 4810.0))*sqrt(50.0*x7*x7 + 50.0*x8*x8 + 50.0*x9*x9 + 981.0*x9 + 4810.0)*(1.96e+5*x9 + 1.0e+4*x7*x7*cos(psi)*cos(psi) + 1.0e+4*x8*x8*sin(psi)*sin(psi) + 1.0e+4*x9*x9 + 2.0e+4*x7*x8*cos(psi)*sin(psi) + 9.62e+5)))*sqrt(50.0*x7*x7 + 50.0*x8*x8 + 50.0*x9*x9 + 981.0*x9 + 4810.0);
	double w3=dpsi/(sqrt((981.0*x9 + 25.0*x7*x7*cos(2.0*psi) - 25.0*x8*x8*cos(2.0*psi) + 25.0*x7*x7 + 25.0*x8*x8 + 50.0*x9*x9 + 50.0*x7*x8*sin(2.0*psi) + 4810.0)/(50.0*x9*x9 + 981.0*x9 + 4810.0))*sqrt(1/(981.0*x9 + 25.0*x7*x7*cos(2.0*psi) - 25.0*x8*x8*cos(2.0*psi) + 25.0*x7*x7 + 25.0*x8*x8 + 50.0*x9*x9 + 50.0*x7*x8*sin(2.0*psi) + 4810.0))*sqrt(50.0*x7*x7 + 50.0*x8*x8 + 50.0*x9*x9 + 981.0*x9 + 4810.0)) - (fabs(x9 + 9.81)*(x8*cos(psi) - x7*sin(psi))*((50.0*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57))*(981.0*x7*x10 + 981.0*x8*x11 - 100.0*x7*x7*x12 - 100.0*x8*x8*x12 + 100.0*x7*x9*x10 + 100.0*x8*x9*x11))/(sqrt(cos(psi + 1.57)*cos(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + sin(psi + 1.57)*sin(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 1.0e+4*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57))*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57)))*sqrt(x7*x7 + x8*x8 + x9*x9 + 19.6*x9 + 96.2)*(50.0*x7*x7 + 50.0*x8*x8 + 50.0*x9*x9 + 981.0*x9 + 4810.0)) + (100.0*cos(psi + 1.57)*(x9 + 9.81)*(9620.0*x11 - 981.0*x8*x12 + 1960.0*x9*x11 + 100.0*x7*x7*x11 + 100.0*x9*x9*x11 - 100.0*x7*x8*x10 - 100.0*x8*x9*x12))/(sqrt(cos(psi + 1.57)*cos(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + sin(psi + 1.57)*sin(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 1.0e+4*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57))*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57)))*sqrt(x7*x7 + x8*x8 + x9*x9 + 19.6*x9 + 96.2)*(100.0*x7*x7 + 100.0*x8*x8 + 100.0*x9*x9 + 1960.0*x9 + 9620.0)) - (100.0*sin(psi + 1.57)*(x9 + 9.81)*(9620.0*x10 - 981.0*x7*x12 + 1960.0*x9*x10 + 100.0*x8*x8*x10 + 100.0*x9*x9*x10 - 100.0*x7*x8*x11 - 100.0*x7*x9*x12))/(sqrt(cos(psi + 1.57)*cos(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + sin(psi + 1.57)*sin(psi+1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 1.0e+4*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57))*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57)))*sqrt(x7*x7 + x8*x8 + x9*x9 + 19.6*x9 + 96.2)*(100.0*x7*x7 + 100.0*x8*x8 + 100.0*x9*x9 + 1960.0*x9 + 9620.0)) - (1.0*dpsi*(100.0*x9 + 981.0)*(100.0*x9 + 981.0)*(x8*cos(psi) - x7*sin(psi)))/((x9 + 9.81)*sqrt(1/(981.0*x9 + 25.0*x7*x7*cos(2.0*psi) - 25.0*x8*x8*cos(2.0*psi) + 25.0*x7*x7 + 25.0*x8*x8 + 50.0*x9*x9 + 50.0*x7*x8*sin(2.0*psi) + 4810.0))*sqrt(50.0*x7*x7 + 50.0*x8*x8 + 50.0*x9*x9 + 981.0*x9 + 4810.0)*(1.96e+5*x9 + 1.0e+4*x7*x7*cos(psi)*cos(psi) + 1.0e+4*x8*x8*sin(psi)*sin(psi) + 1.0e+4*x9*x9 + 2.0e+4*x7*x8*cos(psi)*sin(psi) + 9.62e+5))))/(sqrt((x7*cos(psi) + x8*sin(psi))*(x7*cos(psi) + x8*sin(psi)) + (x9 + 9.81)*(x9 + 9.81))*(x9 + 9.81));
	double w2=-sqrt(50.0*(x7*x7 + x8*x8 + x9*x9 + 19.6*x9 + 96.2)*(981.0*x11*cos(psi + 1.57) - 981.0*x10*sin(psi + 1.57) - 100.0*x8*x12*cos(psi + 1.57) + 100.0*x9*x11*cos(psi + 1.57) + 100.0*x7*x12*sin(psi + 1.57) - 100.0*x9*x10*sin(psi + 1.57)))/(sqrt(cos(psi + 1.57)*cos(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + sin(psi + 1.57)*sin(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 1.0e+4*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57))*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57)))*(50.0*x7*x7 + 50.0*x8*x8 + 50.0*x9*x9 + 981.0*x9 + 4810.0));
	double w1=-(0.01*(9.62e+5*x10*cos(psi + 1.57) + 9.62e+5*x11*sin(psi + 1.57) + 1.0e+4*x8*x8*x10*cos(psi + 1.57) + 1.0e+4*x9*x9*x10*cos(psi + 1.57) + 1.0e+4*x7*x7*x11*sin(psi + 1.57) + 1.0e+4*x9*x9*x11*sin(psi + 1.57) - 9.81e+4*x7*x12*cos(psi + 1.57) + 1.96e+5*x9*x10*cos(psi + 1.57) - 9.81e+4*x8*x12*sin(psi + 1.57) + 1.96e+5*x9*x11*sin(psi + 1.57) - 1.0e+4*x7*x8*x11*cos(psi + 1.57) - 1.0e+4*x7*x9*x12*cos(psi + 1.57) - 1.0e+4*x7*x8*x10*sin(psi + 1.57) - 1.0e+4*x8*x9*x12*sin(psi + 1.57)))/(sqrt(cos(psi + 1.57)*cos(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + sin(psi + 1.57)*sin(psi + 1.57)*(100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 1.0e+4*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57))*(x8*cos(psi + 1.57) - 1.0*x7*sin(psi + 1.57)))*(x7*x7 + x8*x8 + x9*x9 + 19.6*x9 + 96.2));
	if(isnan(w2)){
		w2=0.0;

	}
	if(isnan(w1)){
		w1=0.0;

	}
	if(isnan(w3)){
		w3=0.0;
	}
	X->theta= (float )theta;



	X->phi= (float )phi;

	X->dphi=(float) dphi;
	X->dtheta=(float) dtheta;

	X->w1=(float) w1;
	X->w2=(float) w2;
	X->w3=(float) w3;
}

const char* stateEstimatorGetName() {
	return estimatorFunctions[currentEstimator].name;
}


bool estimatorEnqueueTDOA(const tdoaMeasurement_t *uwb) {
	if (estimatorFunctions[currentEstimator].estimatorEnqueueTDOA) {
		return estimatorFunctions[currentEstimator].estimatorEnqueueTDOA(uwb);
	}

	return false;
}

bool estimatorEnqueueYawError(const yawErrorMeasurement_t* error) {
	if (estimatorFunctions[currentEstimator].estimatorEnqueueYawError) {
		return estimatorFunctions[currentEstimator].estimatorEnqueueYawError(error);
	}

	return false;
}

bool estimatorEnqueuePosition(const positionMeasurement_t *pos) {
	if (estimatorFunctions[currentEstimator].estimatorEnqueuePosition) {
		return estimatorFunctions[currentEstimator].estimatorEnqueuePosition(pos);
	}

	return false;
}

bool estimatorEnqueuePose(const poseMeasurement_t *pose) {
	if (estimatorFunctions[currentEstimator].estimatorEnqueuePose) {
		return estimatorFunctions[currentEstimator].estimatorEnqueuePose(pose);
	}

	return false;
}

bool estimatorEnqueueDistance(const distanceMeasurement_t *dist) {
	if (estimatorFunctions[currentEstimator].estimatorEnqueueDistance) {
		return estimatorFunctions[currentEstimator].estimatorEnqueueDistance(dist);
	}

	return false;
}

bool estimatorEnqueueTOF(const tofMeasurement_t *tof) {
	if (estimatorFunctions[currentEstimator].estimatorEnqueueTOF) {
		return estimatorFunctions[currentEstimator].estimatorEnqueueTOF(tof);
	}

	return false;
}

bool estimatorEnqueueAbsoluteHeight(const heightMeasurement_t *height) {
	if (estimatorFunctions[currentEstimator].estimatorEnqueueAbsoluteHeight) {
		return estimatorFunctions[currentEstimator].estimatorEnqueueAbsoluteHeight(height);
	}

	return false;
}

bool estimatorEnqueueFlow(const flowMeasurement_t *flow) {
	if (estimatorFunctions[currentEstimator].estimatorEnqueueFlow) {
		return estimatorFunctions[currentEstimator].estimatorEnqueueFlow(flow);
	}

	return false;
}

bool estimatorEnqueueSweepAngles(const sweepAngleMeasurement_t *angles) {
	if (estimatorFunctions[currentEstimator].estimatorEnqueueSweepAngles) {
		return estimatorFunctions[currentEstimator].estimatorEnqueueSweepAngles(angles);
	}

	return false;
}
