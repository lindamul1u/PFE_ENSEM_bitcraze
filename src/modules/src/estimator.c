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

void stateEstimator(X_t *reff,X_t *X,X_t *err, commande_t *commande,state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {
	if(currentEstimator==kalmanENSEMEstimator){

		ENSEMkalmanupdate(X,reff,err,commande,state);
		//TestingEstimator(state,X,reff);


		estimatorFunctions[currentEstimator].update(state, sensors, control, tick);




	}
	else{
		estimatorFunctions[currentEstimator].update(state, sensors, control, tick);
	}
}

void TestingEstimator(state_t *state, X_t *X , X_t * reff){
	state->position.x=reff->x1;
	state->position.y=reff->x2;
	state->position.z=reff->x3;
		double x7=reff->x7;
		double x8=reff->x8;
		double  x9=reff->x9;
	state->acc.x=  (float) (x7);
	state->acc.y=  (float) (x8);
	state->acc.z=  (float) (x9);
	state->attitude.yaw=(float) reff->x13;
	estimatorPSI( state,  X , reff);
}
void estimatorPSI(state_t *state, X_t *X , X_t * reff){
	int alpha=4;
	double w3=X->w3;
	double w2=X->w2;
	double phi=X->phi;
	double theta=X->theta;
	double dpsi=(w3*cos(phi) + w2*sin(phi))/cos(theta);
	double psi=(double) state->attitude.yaw;
	double dt=X->dt;
	double phiref=reff->phi;
	double thetaref=reff->theta;
	double dthetadpsi=X->dthetadpsi;
	double dphidpsi=X->dphidpsi;
	double psinext=psi+dt*dpsi+alpha*((phiref-phi)*dphidpsi+(thetaref-theta)*dthetadpsi);
	state->attitude.yaw=(float) psinext;
}

void ENSEMkalmanupdate(X_t *X,X_t *reff,X_t *err ,commande_t *commande,state_t *state){

	// Xtilde(n)

	double r1=   (reff->x1);
	double r2=   (reff->x2);
	double r3=   (reff->x3);

	double r4=   (reff->x4);
	double r5=   (reff->x5);
	double r6=   (reff->x6);

	double r7=   (reff->x7);
	double r8=   (reff->x8);
	double r9=   (reff->x9);


	double r10=   (reff->x10);
	double r11=   (reff->x11);
	double r12=   (reff->x12);

	double r13=   (reff->x13);
	double r14=   (reff->x14);

	double rpsi=   r13;
	double rdpsi=  r14;




	double rphi=   (reff->phi);
	double rtheta=  (reff->theta);

	double rdphi=  (reff->dphi);
	double rdtheta=  (reff->dtheta);
	double rw3=   (reff->w3);
	double rw2=   (reff->w2);
	double rw1=   (reff->w1);





	double x1=   (X->x1);
	double x2=   (X->x2);
	double x3=   (X->x3);

	double x4=   (X->x4);
	double x5=   (X->x5);
	double x6=   (X->x6);

	double x7=   (X->x7);
	double x8=   (X->x8);
	double x9=   (X->x9);


	double x10=   (X->x10);
	double x11=   (X->x11);
	double x12=   (X->x12);

	double x13=   (X->x13);
	double x14=   (X->x14);
	//V(n+1)
	double c1=   (commande->c1);
	double c2=   (commande->c2);
	double c3=   (commande->c3);
	double c4=   (commande->c4);

	//Y(n+1)
	double y1=   (state->position.x);
	double y2=   (state->position.y);
	double y3=   (state->position.z);
	double y4=   ((state->acc.x));
	double y5=   ((state->acc.y));
	double y6=   ((state->acc.z));
	double y7=   (state->attitude.yaw);
	y7=y7*pi/180.0d;
	double dt=  X->dt;

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
	//DEBUG_PRINT("x %f \n",  x_tilde1);
	//DEBUG_PRINT("x (%f)  \n", x_tilde1 );
/*
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
	double x_tilde14= 0.01*c4 - 0.053*x13 + x14 + 0.053*y7;*/
/*
			double x_tilde1=6.66667e-13*c1 + 0.580113*x1 + 0.002*x4 + 0.000002*x7 + 1.33333e-9*x10 + 0.419887*y1;
			double x_tilde2=6.66667e-13*c2 + 0.580113*x2 + 0.002*x5 + 0.000002*x8 + 1.33333e-9*x11 + 0.419887*y2;
			double x_tilde3=6.66667e-13*c3 + 0.580113*x3 + 0.002*x6 + 0.000002*x9 + 1.33333e-9*x12 + 0.419887*y3;
			double x_tilde4=1.33333e-9*c1 - 0.417477*x1 + x4 - 0.000211009*x7 + 0.000002*x10 + 0.417477*y1 + 0.00221101*y4;
			double x_tilde5=1.33333e-9*c2 - 0.417477*x2 + x5 - 0.000211009*x8 + 0.000002*x11 + 0.417477*y2 + 0.00221101*y5;
			double x_tilde6=1.33333e-9*c3 - 0.417477*x3 + x6 - 0.000211009*x9 + 0.000002*x12 + 0.417477*y3 + 0.00221101*y6;
			double x_tilde7=0.000002*c1 + 0.580113*x7 + 0.002*x10 + 0.419887*y4;
			double x_tilde8=0.000002*c2 + 0.580113*x8 + 0.002*x11 + 0.419887*y5;
			double x_tilde9= 0.000002*c3 + 0.580113*x9 + 0.002*x12 + 0.419887*y6;
			double x_tilde10=0.002*c1 - 0.000207302*x1 - 0.417474*x7 + x10 + 0.000207302*y1 + 0.417474*y4;
			double x_tilde11=0.002*c2 - 0.000207302*x2 - 0.417474*x8 + x11 + 0.000207302*y2 + 0.417474*y5;
			double x_tilde12=0.002*c3 - 0.000207302*x3 - 0.417474*x9 + x12 + 0.000207302*y3 + 0.417474*y6;
			double x_tilde13=0.000002*c4 + 0.580113*x13 + 0.002*x14 + 0.419887*y7;
			double x_tilde14= 0.002*c4 - 0.417474*x13 + x14 + 0.417474*y7;*/
	/*double x_tilde1=6.66667e-9*c1 + 0.562487*x1 + 0.02*x4 - 0.000367526*x7 + 0.00000133333*x10 + 0.437513*y1 + 0.000567526*y4;
	double x_tilde2=6.66667e-9*c2 + 0.562487*x2 + 0.02*x5 - 0.000367526*x8 + 0.00000133333*x11 + 0.437513*y2 + 0.000567526*y5;
	double x_tilde3=6.66667e-9*c3 + 0.562487*x3 + 0.02*x6 - 0.000367526*x9 + 0.00000133333*x12 + 0.437513*y3 + 0.000567526*y6;
	double x_tilde4= 0.00000133333*c1 - 0.414095*x1 + x4 - 0.00229003*x7 + 0.0002*x10 + 0.414095*y1 + 0.02229*y4;
	double x_tilde5= 0.00000133333*c2 - 0.414095*x2 + x5 - 0.00229003*x8 + 0.0002*x11 + 0.414095*y2 + 0.02229*y5;
	double x_tilde6= 0.00000133333*c3 - 0.414095*x3 + x6 - 0.00229003*x9 + 0.0002*x12 + 0.414095*y3 + 0.02229*y6;
	double x_tilde7= 0.0002*c1 - 0.000247317*x1 + 0.562501*x7 + 0.02*x10 + 0.000247317*y1 + 0.437499*y4;
	double x_tilde8= 0.0002*c2 - 0.000247317*x2 + 0.562501*x8 + 0.02*x11 + 0.000247317*y2 + 0.437499*y5;
	double x_tilde9= 0.0002*c3 - 0.000247317*x3 + 0.562501*x9 + 0.02*x12 + 0.000247317*y3 + 0.437499*y6;
	double x_tilde10=0.02*c1 - 0.00193218*x1 - 0.413798*x7 + x10 + 0.00193218*y1 + 0.413798*y4;
	double x_tilde11=0.02*c2 - 0.00193218*x2 - 0.413798*x8 + x11 + 0.00193218*y2 + 0.413798*y5;
	double x_tilde12=0.02*c3 - 0.00193218*x3 - 0.413798*x9 + x12 + 0.00193218*y3 + 0.413798*y6;
	double x_tilde13= 0.0002*c4 + 0.5625*x13 + 0.02*x14 + 0.4375*y7;
	double x_tilde14= 0.02*c4 - 0.413803*x13 + x14 + 0.413803*y7;

*/

		/*	double dxtilde1= 0.0435032*x3 - 0.00405789*x2 - 61.8416*x1 + x4 - 0.206403*x7 + 0.173603*x8 - 0.016467*x9 + 0.532014*x13 + 61.8416*y1 + 0.00405789*y2 - 0.0435032*y3 + 0.206403*y4 - 0.173603*y5 + 0.016467*y6 - 0.532014*y7;
			double dxtilde2=0.0122909*x1 - 61.7936*x2 - 0.0489433*x3 + x5 + 0.0237322*x7 + 0.0163783*x8 + 0.00215891*x9 + 0.115644*x13 - 0.0122909*y1 + 61.7936*y2 + 0.0489433*y3 - 0.0237322*y4 - 0.0163783*y5 - 0.00215891*y6 - 0.115644*y7;
			double dxtilde3=0.0270605*x1 - 0.058981*x2 - 61.6121*x3 + x6 + 0.122986*x7 - 0.0406272*x8 + 0.00331056*x9 + 0.500143*x13 - 0.0270605*y1 + 0.058981*y2 + 61.6121*y3 - 0.122986*y4 + 0.0406272*y5 - 0.00331056*y6 - 0.500143*y7;
			double dxtilde4=1.33357*x3 - 0.137145*x2 - 956.04*x1 - 6.38672*x7 + 5.39164*x8 - 0.507462*x9 + 16.4218*x13 + 956.04*y1 + 0.137145*y2 - 1.33357*y3 + 7.38672*y4 - 5.39164*y5 + 0.507462*y6 - 16.4218*y7;
			double dxtilde5=0.367616*x1 - 954.528*x2 - 1.50121*x3 + 0.747105*x7 + 0.510056*x8 + 0.0663203*x9 + 3.57511*x13 - 0.367616*y1 + 954.528*y2 + 1.50121*y3 - 0.747105*y4 + 0.489944*y5 - 0.0663203*y6 - 3.57511*y7;
			double dxtilde6=0.820469*x1 - 1.81457*x2 - 948.977*x3 + 3.80267*x7 - 1.25914*x8 + 0.101848*x9 + 15.3869*x13 - 0.820469*y1 + 1.81457*y2 + 948.977*y3 - 3.80267*y4 + 1.25914*y5 + 0.898152*y6 - 15.3869*y7;
			double dxtilde7=0.0465568*x2 - 0.21532*x1 + 0.108366*x3 - 61.8701*x7 + 0.118502*x8 - 0.0126382*x9 + x10 + 0.0876866*x13 + 0.21532*y1 - 0.0465568*y2 - 0.108366*y3 + 61.8701*y4 - 0.118502*y5 + 0.0126382*y6 - 0.0876866*y7;
			double dxtilde8=0.101388*x1 + 0.00296385*x2 - 0.0293477*x3 + 0.0754031*x7 - 61.683*x8 + 0.0035864*x9 + x11 - 0.0685971*x13 - 0.101388*y1 - 0.00296385*y2 + 0.0293477*y3 - 0.0754031*y4 + 61.683*y5 - 0.0035864*y6 + 0.0685971*y7;
			double dxtilde9=0.00218873*x2 - 0.0167204*x1 + 0.00354586*x3 - 0.0122028*x7 + 0.00597792*x8 - 61.3006*x9 + x12 + 0.0129756*x13 + 0.0167204*y1 - 0.00218873*y2 - 0.00354586*y3 + 0.0122028*y4 - 0.00597792*y5 + 61.3006*y6 - 0.0129756*y7;
			double dxtilde10=c1 - 6.66035*x1 + 1.45392*x2 + 3.35416*x3 - 956.908*x7 + 3.69891*x8 - 0.389923*x9 + 2.72689*x13 + 6.66035*y1 - 1.45392*y2 - 3.35416*y3 + 956.908*y4 - 3.69891*y5 + 0.389923*y6 - 2.72689*y7;
			double dxtilde11= c2 + 3.14561*x1 + 0.0942428*x2 - 0.911937*x3 + 2.3563*x7 - 950.942*x8 + 0.111468*x9 - 2.12921*x13 - 3.14561*y1 - 0.0942428*y2 + 0.911937*y3 - 2.3563*y4 + 950.942*y5 - 0.111468*y6 + 2.12921*y7;
			double dxtilde12=c3 - 0.515245*x1 + 0.0672762*x2 + 0.109268*x3 - 0.376514*x7 + 0.185863*x8 - 939.378*x9 + 0.400871*x13 + 0.515245*y1 - 0.0672762*y2 - 0.109268*y3 + 0.376514*y4 - 0.185863*y5 + 939.378*y6 - 0.400871*y7;
			double dxtilde13=0.0737045*x1 + 0.0193169*x2 + 0.0542162*x3 + 0.015494*x7 - 0.0198816*x8 + 0.00187697*x9 - 60.3991*x13 + x14 - 0.0737045*y1 - 0.0193169*y2 - 0.0542162*y3 - 0.015494*y4 + 0.0198816*y5 - 0.00187697*y6 + 60.3991*y7;
			double dxtilde14=c4 + 2.22269*x1 + 0.582521*x2 + 1.63508*x3 + 0.467216*x7 - 0.599501*x8 + 0.056603*x9 - 912.008*x13 - 2.22269*y1 - 0.582521*y2 - 1.63508*y3 - 0.467216*y4 + 0.599501*y5 - 0.056603*y6 + 912.008*y7;
					*/
					double dxtilde1= x4 - 32.6079*x1 - 0.0153338*x7 + 32.6079*y1 + 0.0153338*y4;
					double dxtilde2=  x5 - 32.6079*x2 - 0.0153338*x8 + 32.6079*y2 + 0.0153338*y5;
					double dxtilde3= x6 - 32.6079*x3 - 0.0153338*x9 + 32.6079*y3 + 0.0153338*y6;
					double dxtilde4=  0.00000371766*x7 - 31.6386*x1 + 31.6386*y1 + 0.999996*y4;
					double dxtilde5= 0.00000371766*x8 - 31.6386*x2 + 31.6386*y2 + 0.999996*y5;
					double dxtilde6=   0.00000371766*x9 - 31.6386*x3 + 31.6386*y3 + 0.999996*y6;
					double dxtilde7=x10 - 32.6074*x7 - 0.0153338*x1 + 0.0153338*y1 + 32.6074*y4;
					double dxtilde8= x11 - 32.6074*x8 - 0.0153338*x2 + 0.0153338*y2 + 32.6074*y5;
					double dxtilde9= x12 - 32.6074*x9 - 0.0153338*x3 + 0.0153338*y3 + 32.6074*y6;
					double dxtilde10=c1 - 0.00000371581*x1 - 31.6228*x7 + 0.00000371581*y1 + 31.6228*y4;
					double dxtilde11=c2 - 0.00000371581*x2 - 31.6228*x8 + 0.00000371581*y2 + 31.6228*y5;
					double dxtilde12= c3 - 0.00000371581*x3 - 31.6228*x9 + 0.00000371581*y3 + 31.6228*y6;
					double dxtilde13= x14 - 32.6074*x13 + 32.6074*y7;
					double dxtilde14=  c4 - 31.6228*x13 + 31.6228*y7;


/*

    		double dxtilde1= x4 - 0.0473672*x2 - 0.0445077*x3 - 21.4227*x1 + 0.0233852*x7 + 0.0270001*x8 + 0.0850054*x9 - 0.575746*x13 + 21.4227*y1 + 0.0473672*y2 + 0.0445077*y3 - 0.0233852*y4 - 0.0270001*y5 - 0.0850054*y6 + 0.575746*y7;
    		double dxtilde2=0.0170416*x3 - 21.7753*x2 - 0.0546611*x1 + x5 + 0.208606*x7 + 0.0668915*x8 + 0.0128038*x9 + 0.266192*x13 + 0.0546611*y1 + 21.7753*y2 - 0.0170416*y3 - 0.208606*y4 - 0.0668915*y5 - 0.0128038*y6 - 0.266192*y7;
			double dxtilde3= 0.0236429*x2 - 0.0382284*x1 - 21.597*x3 + x6 + 0.0653013*x7 - 0.172569*x8 - 0.0266136*x9 + 0.175064*x13 + 0.0382284*y1 - 0.0236429*y2 + 21.597*y3 - 0.0653013*y4 + 0.172569*y5 + 0.0266136*y6 - 0.175064*y7;
			double dxtilde4=  0.253015*x7 - 0.513875*x2 - 0.492597*x3 - 114.644*x1 + 0.295803*x8 + 0.87813*x9 - 6.28034*x13 + 114.644*y1 + 0.513875*y2 + 0.492597*y3 + 0.746985*y4 - 0.295803*y5 - 0.87813*y6 + 6.28034*y7;
			double dxtilde5=  0.187076*x3 - 118.459*x2 - 0.591901*x1 + 2.27546*x7 + 0.731181*x8 + 0.149054*x9 + 2.90116*x13 + 0.591901*y1 + 118.459*y2 - 0.187076*y3 - 2.27546*y4 + 0.268819*y5 - 0.149054*y6 - 2.90116*y7;
			double dxtilde6=   0.258575*x2 - 0.423072*x1 - 116.559*x3 + 0.704178*x7 - 1.86753*x8 - 0.275106*x9 + 1.90814*x13 + 0.423072*y1 - 0.258575*y2 + 116.559*y3 - 0.704178*y4 + 1.86753*y5 + 1.27511*y6 - 1.90814*y7;
			double dxtilde7= 0.0182063*x1 + 0.213163*x2 + 0.0715032*x3 - 21.7949*x7 + 0.149367*x8 - 0.0108718*x9 + x10 + 0.25508*x13 - 0.0182063*y1 - 0.213163*y2 - 0.0715032*y3 + 21.7949*y4 - 0.149367*y5 + 0.0108718*y6 - 0.25508*y7;
			double dxtilde8=0.0223687*x1 + 0.0395567*x2 - 0.0898914*x3 + 0.0860963*x7 - 21.7075*x8 - 0.0126756*x9 + x11 + 0.0277628*x13 - 0.0223687*y1 - 0.0395567*y2 + 0.0898914*y3 - 0.0860963*y4 + 21.7075*y5 + 0.0126756*y6 - 0.0277628*y7;
			double dxtilde9=  0.094517*x1 + 0.0115873*x2 - 0.026597*x3 - 0.0140477*x7 - 0.0273837*x8 - 21.3814*x9 + x12 + 0.607727*x13 - 0.094517*y1 - 0.0115873*y2 + 0.026597*y3 + 0.0140477*y4 + 0.0273837*y5 + 21.3814*y6 - 0.607727*y7;
			double dxtilde10=  c1 + 0.194763*x1 + 2.32796*x2 + 0.773662*x3 - 118.671*x7 + 1.63738*x8 - 0.107169*x9 + 2.78669*x13 - 0.194763*y1 - 2.32796*y2 - 0.773662*y3 + 118.671*y4 - 1.63738*y5 + 0.107169*y6 - 2.78669*y7;
			double dxtilde11=c2 + 0.245109*x1 + 0.434142*x2 - 0.95838*x3 + 0.941615*x7 - 117.552*x8 - 0.140944*x9 + 0.301324*x13 - 0.245109*y1 - 0.434142*y2 + 0.95838*y3 - 0.941615*y4 + 117.552*y5 + 0.140944*y6 - 0.301324*y7;
			double dxtilde12=   c3 + 0.982874*x1 + 0.136341*x2 - 0.274414*x3 - 0.145237*x7 - 0.30381*x8 - 114.199*x9 + 6.6306*x13 - 0.982874*y1 - 0.136341*y2 + 0.274414*y3 + 0.145237*y4 + 0.30381*y5 + 114.199*y6 - 6.6306*y7;
			double dxtilde13=0.0448636*x2 - 0.0996652*x1 + 0.0325293*x3 + 0.0452125*x7 + 0.0120152*x8 + 0.106875*x9 - 20.8212*x13 + x14 + 0.0996652*y1 - 0.0448636*y2 - 0.0325293*y3 - 0.0452125*y4 - 0.0120152*y5 - 0.106875*y6 + 20.8212*y7;
			double dxtilde14= c4 - 1.01289*x1 + 0.455993*x2 + 0.330627*x3 + 0.45941*x7 + 0.122018*x8 + 1.08629*x9 - 108.311*x13 + 1.01289*y1 - 0.455993*y2 - 0.330627*y3 - 0.45941*y4 - 0.122018*y5 - 1.08629*y6 + 108.311*y7;
*/
					/*

	double dxtilde1= 0.0435032*x3 - 0.00405789*x2 - 61.8416*x1 + x4 - 0.206403*x7 + 0.173603*x8 - 0.016467*x9 + 0.532014*x13 + 61.8416*y1 + 0.00405789*y2 - 0.0435032*y3 + 0.206403*y4 - 0.173603*y5 + 0.016467*y6 - 0.532014*y7;
	double dxtilde2= 0.0122909*x1 - 61.7936*x2 - 0.0489433*x3 + x5 + 0.0237322*x7 + 0.0163783*x8 + 0.00215891*x9 + 0.115644*x13 - 0.0122909*y1 + 61.7936*y2 + 0.0489433*y3 - 0.0237322*y4 - 0.0163783*y5 - 0.00215891*y6 - 0.115644*y7;
	double dxtilde3= 0.0270605*x1 - 0.058981*x2 - 61.6121*x3 + x6 + 0.122986*x7 - 0.0406272*x8 + 0.00331056*x9 + 0.500143*x13 - 0.0270605*y1 + 0.058981*y2 + 61.6121*y3 - 0.122986*y4 + 0.0406272*y5 - 0.00331056*y6 - 0.500143*y7;
	double dxtilde4= 1.33357*x3 - 0.137145*x2 - 956.04*x1 - 6.38672*x7 + 5.39164*x8 - 0.507462*x9 + 16.4218*x13 + 956.04*y1 + 0.137145*y2 - 1.33357*y3 + 7.38672*y4 - 5.39164*y5 + 0.507462*y6 - 16.4218*y7;
	double dxtilde5= 0.367616*x1 - 954.528*x2 - 1.50121*x3 + 0.747105*x7 + 0.510056*x8 + 0.0663203*x9 + 3.57511*x13 - 0.367616*y1 + 954.528*y2 + 1.50121*y3 - 0.747105*y4 + 0.489944*y5 - 0.0663203*y6 - 3.57511*y7;
	double dxtilde6= 0.820469*x1 - 1.81457*x2 - 948.977*x3 + 3.80267*x7 - 1.25914*x8 + 0.101848*x9 + 15.3869*x13 - 0.820469*y1 + 1.81457*y2 + 948.977*y3 - 3.80267*y4 + 1.25914*y5 + 0.898152*y6 - 15.3869*y7;
	double dxtilde7=0.0465568*x2 - 0.21532*x1 + 0.108366*x3 - 61.8701*x7 + 0.118502*x8 - 0.0126382*x9 + x10 + 0.0876866*x13 + 0.21532*y1 - 0.0465568*y2 - 0.108366*y3 + 61.8701*y4 - 0.118502*y5 + 0.0126382*y6 - 0.0876866*y7;
	double dxtilde8=0.101388*x1 + 0.00296385*x2 - 0.0293477*x3 + 0.0754031*x7 - 61.683*x8 + 0.0035864*x9 + x11 - 0.0685971*x13 - 0.101388*y1 - 0.00296385*y2 + 0.0293477*y3 - 0.0754031*y4 + 61.683*y5 - 0.0035864*y6 + 0.0685971*y7;
	double dxtilde9= 0.00218873*x2 - 0.0167204*x1 + 0.00354586*x3 - 0.0122028*x7 + 0.00597792*x8 - 61.3006*x9 + x12 + 0.0129756*x13 + 0.0167204*y1 - 0.00218873*y2 - 0.00354586*y3 + 0.0122028*y4 - 0.00597792*y5 + 61.3006*y6 - 0.0129756*y7;
	double dxtilde10= c1 - 6.66035*x1 + 1.45392*x2 + 3.35416*x3 - 956.908*x7 + 3.69891*x8 - 0.389923*x9 + 2.72689*x13 + 6.66035*y1 - 1.45392*y2 - 3.35416*y3 + 956.908*y4 - 3.69891*y5 + 0.389923*y6 - 2.72689*y7;
	double dxtilde11=  c2 + 3.14561*x1 + 0.0942428*x2 - 0.911937*x3 + 2.3563*x7 - 950.942*x8 + 0.111468*x9 - 2.12921*x13 - 3.14561*y1 - 0.0942428*y2 + 0.911937*y3 - 2.3563*y4 + 950.942*y5 - 0.111468*y6 + 2.12921*y7;
	double dxtilde12= c3 - 0.515245*x1 + 0.0672762*x2 + 0.109268*x3 - 0.376514*x7 + 0.185863*x8 - 939.378*x9 + 0.400871*x13 + 0.515245*y1 - 0.0672762*y2 - 0.109268*y3 + 0.376514*y4 - 0.185863*y5 + 939.378*y6 - 0.400871*y7;
	double dxtilde13= 0.0737045*x1 + 0.0193169*x2 + 0.0542162*x3 + 0.015494*x7 - 0.0198816*x8 + 0.00187697*x9 - 60.3991*x13 + x14 - 0.0737045*y1 - 0.0193169*y2 - 0.0542162*y3 - 0.015494*y4 + 0.0198816*y5 - 0.00187697*y6 + 60.3991*y7;
	double dxtilde14= c4 + 2.22269*x1 + 0.582521*x2 + 1.63508*x3 + 0.467216*x7 - 0.599501*x8 + 0.056603*x9 - 912.008*x13 - 2.22269*y1 - 0.582521*y2 - 1.63508*y3 - 0.467216*y4 + 0.599501*y5 - 0.056603*y6 + 912.008*y7;*/
/*
	double dxtilde1=x4 - 100.995*x1 - 0.00495074*x7 + 100.995*y1 + 0.00495074*y4;
	double dxtilde2=x5 - 100.995*x2 - 0.00495074*x8 + 100.995*y2 + 0.00495074*y5;
	double dxtilde3=x6 - 100.995*x3 - 0.00495074*x9 + 100.995*y3 + 0.00495074*y6;
	double dxtilde4= 1.22549e-7*x7 - 100.005*x1 + 100.005*y1 + 1.0*y4;
	double dxtilde5=1.22549e-7*x8 - 100.005*x2 + 100.005*y2 + 1.0*y5;
	double dxtilde6=1.22549e-7*x9 - 100.005*x3 + 100.005*y3 + 1.0*y6;
	double dxtilde7=x10 - 100.995*x7 - 0.00495074*x1 + 0.00495074*y1 + 100.995*y4;
	double dxtilde8=x11 - 100.995*x8 - 0.00495074*x2 + 0.00495074*y2 + 100.995*y5;
	double dxtilde9=x12 - 100.995*x9 - 0.00495074*x3 + 0.00495074*y3 + 100.995*y6;
	double dxtilde10=c1 - 1.22543e-7*x1 - 100.0*x7 + 1.22543e-7*y1 + 100.0*y4;
	double dxtilde11=c2 - 1.22543e-7*x2 - 100.0*x8 + 1.22543e-7*y2 + 100.0*y5;
	double dxtilde12=c3 - 1.22543e-7*x3 - 100.0*x9 + 1.22543e-7*y3 + 100.0*y6;
	double dxtilde13=x14 - 100.995*x13 + 100.995*y7;
	double dxtilde14=c4 - 100.0*x13 + 100.0*y7;
*/
			double x_tilde1=x1+dt*dxtilde1;
			double x_tilde2=x2+dt*dxtilde2;
			double x_tilde3=x3+dt*dxtilde3;
			double x_tilde4=x4+dt*dxtilde4;
			double x_tilde5=x5+dt*dxtilde5;
			double x_tilde6=x6+dt*dxtilde6;
			double x_tilde7=x7+dt*dxtilde7;
			double x_tilde8=x8+dt*dxtilde8;
			double x_tilde9=x9+dt*dxtilde9;
			double x_tilde10=x10+dt*dxtilde10;
			double x_tilde11=x11+dt*dxtilde11;
			double x_tilde12=x12+dt*dxtilde12;
			double x_tilde13=x13+dt*dxtilde13;
			double x_tilde14=x14+dt*dxtilde14;
	(X->x1)=   x_tilde1;
	(X->x2)=   x_tilde2;
	(X->x3)=   x_tilde3;

	(X->x4)=   x_tilde4;
	(X->x5)=   x_tilde5;
	(X->x6)=   x_tilde6;

	(X->x7)=   x_tilde7;
	(X->x8)=   x_tilde8;
	(X->x9)=   x_tilde9;

	(X->x10)=   x_tilde10;
	(X->x11)=   x_tilde11;
	(X->x12)=   x_tilde12;

	(X->x13)=   x_tilde13;
	(X->x14)=   x_tilde14;
	(X->psi)=   (X->x13);
	(X->dpsi)=   (X->x14);

	x1=  (X->x1);
	x2=  (X->x2);
	x3=  (X->x3);

	x4=   (X->x4);
	x5=   (X->x5);
	x6=   (X->x6);

	x7=   (X->x7);
	x8=   (X->x8);
	x9=   (X->x9);


	x10=   (X->x10);
	x11=   (X->x11);
	x12=   (X->x12);

	x13=   (X->x13);
	x14=   (X->x14);
	double psi=   x13;
	double dpsi=  x14;

	double theta=atan2((100.0*(x7*cos(psi) + x8*sin(psi))),(100.0*x9 + 981.0));
	double dthetadpsi =((x8*cos(psi) - x7*sin(psi))*(10000.0*x9 + 98100.0))/(196200.0*x9 + 10000.0*x7*x7*cos(psi)*cos(psi) + 10000.0*x8*x8*sin(psi)*sin(psi) + 10000.0*x9*x9 + 20000.0*x7*x8*cos(psi)*sin(psi) + 962361.0);

	double phi=-1.0*atan2((7.07*fabs(100.0*x9 + 981.0)*(x8*cos(psi) - x7*sin(psi))),((100.0*x9 + 981.0)*sqrt(981.0*x9 + 50.0*x7*x7*cos(psi)*cos(psi) + 50.0*x8*x8*sin(psi)*sin(psi) + 50.0*x9*x9 + 100.0*x7*x8*cos(psi)*sin(psi) + 4810.0)));
	double dphidpsi =(0.01*sqrt((x7*cos(psi) + x8*sin(psi))*(x7*cos(psi) + x8*sin(psi)) + (x9 + 9.81)*(x9 + 9.81))*(x7*cos(psi) + x8*sin(psi))*(100.0*x9 + 100.0*x8*cos(psi) - 100.0*x7*sin(psi) + 981.0))/(fabs(x9 + 9.81)*(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361));

	double dtheta=-sqrt(1/(196200.0*x9 + 5000.0*x7*x7*cos(2.0*psi) - 5000.0*x8*x8*cos(2.0*psi) + 5000.0*x7*x7 + 5000.0*x8*x8 + 10000.0*x9*x9 + 10000.0*x7*x8*sin(2.0*psi) + 962361.0))*((0.01*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708))*(1.49012e-8*x12 + 98100.0*x7*x10 + 98100.0*x8*x11 - 10000.0*x7*x7*x12 - 10000.0*x8*x8*x12 + 10000.0*x7*x9*x10 + 10000.0*x8*x9*x11))/(sqrt((100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 10000.0*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708))*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708)))*pow(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361,3/2)) + (1.0*cos(psi + 1.5708)*(x9 + 9.81)*(9623.61*x11 - 981.0*x8*x12 + 1962.0*x9*x11 + 100.0*x7*x7*x11 + 100.0*x9*x9*x11 - 100.0*x7*x8*x10 - 100.0*x8*x9*x12))/(sqrt((100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 10000.0*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708))*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708)))*pow(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361,3/2)) - (1.0*sin(psi + 1.5708)*(x9 + 9.81)*(9623.61*x10 - 981.0*x7*x12 + 1962.0*x9*x10 + 100.0*x8*x8*x10 + 100.0*x9*x9*x10 - 100.0*x7*x8*x11 - 100.0*x7*x9*x12))/(sqrt((100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 10000.0*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708))*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708)))*pow(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361,3/2)) - (1.0*dpsi*(100.0*x9 + 981.0)*(100.0*x9 + 981.0)*(x8*cos(psi) - x7*sin(psi)))/(sqrt(1/(196200.0*x9 + 5000.0*x7*x7*cos(2.0*psi) - 5000.0*x8*x8*cos(2.0*psi) + 5000.0*x7*x7 + 5000.0*x8*x8 + 10000.0*x9*x9 + 10000.0*x7*x8*sin(2.0*psi) + 962361.0))*(x9 + 9.81)*sqrt(10000.0*x7*x7 + 10000.0*x8*x8 + 10000.0*x9*x9 + 196200.0*x9 + 962361.0)*(196200.0*x9 + 5000.0*x7*x7*cos(2*psi) - 5000.0*x8*x8*cos(2*psi) + 5000.0*x7*x7 + 5000.0*x8*x8 + 10000.0*x9*x9 + 10000.0*x7*x8*sin(2*psi) + 962361.0)))*sqrt(10000.0*x7*x7 + 10000.0*x8*x8 + 10000.0*x9*x9 + 196200.0*x9 + 962361.0);

	double dphi=(100*dpsi*fabs(100.0*x9 + 981.0)*(x7*cos(psi) + x8*sin(psi)))/((100.0*x9 + 981.0)*sqrt(196200.0*x9 + 5000.0*x7*x7*cos(2*psi) - 5000.0*x8*x8*cos(2*psi) + 5000.0*x7*x7 + 5000.0*x8*x8 + 10000.0*x9*x9 + 10000.0*x7*x8*sin(2*psi) + 962361.0)) - (0.0001*(10000.0*sin(psi + 1.5708)*x7*x7 - 10000.0*x8*cos(psi + 1.5708)*x7 + 10000.0*sin(psi + 1.5708)*x9*x9 + 196200.0*sin(psi + 1.5708)*x9 + 962361.0*sin(psi + 1.5708))*(9623.61*x11 - 981.0*x8*x12 + 1962.0*x9*x11 + 100.0*x7*x7*x11 + 100.0*x9*x9*x11 - 100.0*x7*x8*x10 - 100.0*x8*x9*x12))/(sqrt((100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 10000.0*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708))*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708)))*(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361)*(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361)) - (0.0001*(100.0*x9 + 981.0)*(x7*cos(psi + 1.5708) + x8*sin(psi + 1.5708))*(1.49012e-8*x12 + 98100.0*x7*x10 + 98100.0*x8*x11 - 10000.0*x7*x7*x12 - 10000.0*x8*x8*x12 + 10000.0*x7*x9*x10 + 10000.0*x8*x9*x11))/(sqrt((100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 10000.0*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708))*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708)))*(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361)*(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361)) - (0.0001*(10000.0*cos(psi + 1.5708)*x8*x8 - 10000.0*x7*sin(psi + 1.5708)*x8 + 10000.0*cos(psi + 1.5708)*x9*x9 + 196200.0*cos(psi + 1.5708)*x9 + 962361.0*cos(psi + 1.5708))*(9623.61*x10 - 981.0*x7*x12 + 1962.0*x9*x10 + 100.0*x8*x8*x10 + 100.0*x9*x9*x10 - 100.0*x7*x8*x11 - 100.0*x7*x9*x12))/(sqrt((100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 10000.0*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708))*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708)))*(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361)*(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361));




	double w1=-(0.01*(962361.0*x10*cos(psi + 1.5708) + 962361.0*x11*sin(psi + 1.5708) + 10000.0*x8*x8*x10*cos(psi + 1.5708) + 10000.0*x9*x9*x10*cos(psi + 1.5708) + 10000.0*x7*x7*x11*sin(psi + 1.5708) + 10000.0*x9*x9*x11*sin(psi + 1.5708) - 98100.0*x7*x12*cos(psi + 1.5708) + 196200.0*x9*x10*cos(psi + 1.5708) - 98100.0*x8*x12*sin(psi + 1.5708) + 196200.0*x9*x11*sin(psi + 1.5708) - 10000.0*x7*x8*x11*cos(psi + 1.5708) - 10000.0*x7*x9*x12*cos(psi + 1.5708) - 10000.0*x7*x8*x10*sin(psi + 1.5708) - 10000.0*x8*x9*x12*sin(psi + 1.5708)))/(sqrt((100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 10000.0*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708))*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708)))*(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361));
	double w2=-(1.0*(981.0*x11*cos(psi + 1.5708) - 981.0*x10*sin(psi + 1.5708) - 100.0*x8*x12*cos(psi + 1.5708) + 100.0*x9*x11*cos(psi + 1.5708) + 100.0*x7*x12*sin(psi + 1.5708) - 100.0*x9*x10*sin(psi + 1.5708)))/(sqrt((100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 10000.0*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708))*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708)))*sqrt(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361));
	double w3=(dpsi*fabs(100.0*x9 + 981.0))/(sqrt(1/(196200.0*x9 + 5000.0*x7*x7*cos(2.0*psi) - 5000.0*x8*x8*cos(2.0*psi) + 5000.0*x7*x7 + 5000.0*x8*x8 + 10000.0*x9*x9 + 10000.0*x7*x8*sin(2.0*psi) + 962361.0))*sqrt(10000.0*x7*x7 + 10000.0*x8*x8 + 10000.0*x9*x9 + 196200.0*x9 + 962361.0)*sqrt(196200.0*x9 + 5000.0*x7*x7*cos(2.0*psi) - 5000.0*x8*x8*cos(2.0*psi) + 5000.0*x7*x7 + 5000.0*x8*x8 + 10000.0*x9*x9 + 10000.0*x7*x8*sin(2.0*psi) + 962361.0)) - (fabs(x9 + 9.81)*(x8*cos(psi) - x7*sin(psi))*((0.01*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708))*(1.49012e-8*x12 + 98100.0*x7*x10 + 98100.0*x8*x11 - 10000.0*x7*x7*x12 - 10000.0*x8*x8*x12 + 10000.0*x7*x9*x10 + 10000.0*x8*x9*x11))/(sqrt((100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 10000.0*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708))*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708)))*pow(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361,3/2)) + (1.0*cos(psi + 1.5708)*(x9 + 9.81)*(9623.61*x11 - 981.0*x8*x12 + 1962.0*x9*x11 + 100.0*x7*x7*x11 + 100.0*x9*x9*x11 - 100.0*x7*x8*x10 - 100.0*x8*x9*x12))/(sqrt((100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 10000.0*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708))*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708)))*pow(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361,3/2)) - (1.0*sin(psi + 1.5708)*(x9 + 9.81)*(9623.61*x10 - 981.0*x7*x12 + 1962.0*x9*x10 + 100.0*x8*x8*x10 + 100.0*x9*x9*x10 - 100.0*x7*x8*x11 - 100.0*x7*x9*x12))/(sqrt((100.0*x9 + 981.0)*(100.0*x9 + 981.0) + 10000.0*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708))*(x8*cos(psi + 1.5708) - 1.0*x7*sin(psi + 1.5708)))*pow(x7*x7 + x8*x8 + x9*x9 + 19.62*x9 + 96.2361,3/2)) - (1.0*dpsi*(100.0*x9 + 981.0)*(100.0*x9 + 981.0)*(x8*cos(psi) - x7*sin(psi)))/(sqrt(1/(196200.0*x9 + 5000.0*x7*x7*cos(2.0*psi) - 5000.0*x8*x8*cos(2.0*psi) + 5000.0*x7*x7 + 5000.0*x8*x8 + 10000.0*x9*x9 + 10000.0*x7*x8*sin(2.0*psi) + 962361.0))*(x9 + 9.81)*sqrt(10000.0*x7*x7 + 10000.0*x8*x8 + 10000.0*x9*x9 + 196200.0*x9 + 962361.0)*(196200.0*x9 + 5000.0*x7*x7*cos(2*psi) - 5000.0*x8*x8*cos(2*psi) + 5000.0*x7*x7 + 5000.0*x8*x8 + 10000.0*x9*x9 + 10000.0*x7*x8*sin(2*psi) + 962361.0))))/(sqrt((x7*cos(psi) + x8*sin(psi))*(x7*cos(psi) + x8*sin(psi)) + (x9 + 9.81)*(x9 + 9.81)*(x9 + 9.81))*(x9 + 9.81));


 	if(isnan(w2)){
		w2=0.0;

	}
	if(isnan(w1)){
		w1=0.0;

	}
	if(isnan(w3)){
		w3=0.0;
	}
	X->theta=  theta;



	X->phi=  phi;

	X->dphi=  dphi;
	X->dtheta=  dtheta;
	X->dphidpsi=dphidpsi;
	X->dthetadpsi=dthetadpsi;
	X->w1=  w1;
	X->w2=  w2;
	X->w3=  w3;
	double errx1=  fabs(x1-r1);
	double errx2=  fabs(x2-r2);
	double errx3=  fabs(x3-r3);
	double errx4=  fabs(x4-r4);
	double errx5=  fabs(x5-r5);
	double errx6=  fabs(x6-r6);
	double errx7=  fabs(x7-r7);
	double errx8=  fabs(x8-r8);
	double errx9=  fabs(x9-r9);
	double errx10=  fabs(x10-r10);
	double errx11=  fabs(x11-r11);
	double errx12=  fabs(x12-r12);
	double errx13=  fabs(x13-r13);
	double errx14=  fabs(x14-r14);
	double errphi=  fabs(phi-rphi);
	double errtheta=  fabs(theta-rtheta);
	double errpsi=  fabs(psi-rpsi);
	double errdphi=  fabs(dphi-rdphi);
	double errdtheta=  fabs(dtheta-rdtheta);
	double errdpsi=  fabs(dpsi-rdpsi);
	double errw1=  fabs(w1-rw1);
	double errw2=  fabs(w2-rw2);
	double errw3=  fabs(w3-rw3);
	(err->x1)=   errx1;
		(err->x2)=   errx2;
		(err->x3)=   errx3;

		(err->x4)=   errx4;
		(err->x5)=   errx5;
		(err->x6)=   errx6;

		(err->x7)=   errx7;
		(err->x8)=   errx8;
		(err->x9)=   errx9;

		(err->x10)=   errx10;
		(err->x11)=   errx11;
		(err->x12)=   errx12;

		(err->x13)=   errx13;
		(err->x14)=   errx14;
		(err->psi)=   errpsi;
		(err->dpsi)=   errdpsi;
		(err->theta)=  errtheta;
		(err->dtheta)=  errdtheta;
		(err->phi)=  errphi;
		(err->dphi)=  errdphi;
		(err->w1)=  errw1;
		(err->w2)=  errw2;
		(err->w3)=  errw3;




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
