#define DEBUG_MODULE "ESTIMATOR"

#include "cfassert.h"
#include "estimator.h"
#include "estimator_complementary.h"
#include "estimator_kalman.h"
#include "physicalConstants.h"
#include "debug.h"
#define pi 3.1415926535897932384d

#define DEFAULT_ESTIMATOR complementaryEstimator
static StateEstimatorType currentEstimator = anyEstimator;
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
				.estimatorEnqueuePosition = NOT_IMPLEMENTED,
				.estimatorEnqueuePose = NOT_IMPLEMENTED,
				.estimatorEnqueueDistance = NOT_IMPLEMENTED,
				.estimatorEnqueueTOF = NOT_IMPLEMENTED,
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
				.name = "KalmanENSEM",
				.estimatorEnqueueTDOA = estimatorKalmanEnqueueTDOA,
				.estimatorEnqueuePosition = NOT_IMPLEMENTED,
				.estimatorEnqueuePose = NOT_IMPLEMENTED,
				.estimatorEnqueueDistance = NOT_IMPLEMENTED,
				.estimatorEnqueueTOF = NOT_IMPLEMENTED,
				.estimatorEnqueueAbsoluteHeight = NOT_IMPLEMENTED,
				.estimatorEnqueueFlow = NOT_IMPLEMENTED,
				.estimatorEnqueueYawError = estimatorKalmanEnqueueYawError,
				.estimatorEnqueueSweepAngles = NOT_IMPLEMENTED,
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
void statetoST(X_t *ST,state_t *state);
void statetoST(X_t *ST,state_t *state){
			ST->x1=state->position.x;
			ST->x2=state->position.y;
			ST->x3=state->position.z;
			ST->x7=(state->acc.x)*9.81f;
			ST->x8=(state->acc.y)*9.81f;
			ST->x9=(state->acc.z)*9.81f;
			ST->x13=(state->attitude.yaw)*0.0175f;
}
void STtostate(X_t *ST,state_t *state,sensorData_t *sensor);
void STtostate(X_t *ST,state_t *state,sensorData_t *sensor){
			state->position.x=(float)ST->x1;
			state->position.y=(float)ST->x2;
			state->position.z=(float)ST->x3;


			state->acc.x=(float)(ST->x7/9.81d);
			state->acc.y=(float)(ST->x8/9.81d);
			state->acc.z=(float)(ST->x9/9.81d);

			state->attitude.roll=(float)(ST->phi*57.2958d);
			state->attitude.pitch=(float)(ST->theta*57.2958d);
			state->attitude.yaw=(float)(ST->psi*57.2958d);
			sensor->acc.x=(float)(ST->accbx/9.81);
			sensor->acc.y=(float)(ST->accby/9.81);
			sensor->acc.z=(float)(ST->accbz/9.81);
			sensor->gyro.x=(float)(ST->w1*57.2958d);
			sensor->gyro.y=(float)(ST->w1*57.2958d);
			sensor->gyro.z=(float)(ST->w1*57.2958d);




}
static int n;
void stateEstimator(StateEstimatorType estimator,X_t *reff,X_t *X,X_t *err,X_t *ST, commande_t *commande,state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick) {
	if(estimator==kalmanENSEMEstimator){

		estimatorFunctions[estimator].update(state, sensors, control, tick);



		ENSEMkalmanupdate(X,reff,err, ST,commande);
		statetoST(ST,state);

		X->start=1;

	}
	else{
		estimatorFunctions[estimator].update(state, sensors, control, tick);

	}
	n++;
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
//	estimatorPSI( state,  X , reff);
}
void estimatorPSI(X_t *S, X_t *X , X_t * reff){
	int alpha=20;
		double w3=S->w3;
		double w2=S->w2;
		double phi=X->phi;
		double theta=X->theta;
		double dpsi=(w3*cos(phi) + w2*sin(phi))/cos(theta);
		double psi=(double) S->x13;
		double dt=X->dt;
		double phiref=reff->phi;
		double thetaref=reff->theta;
		double dthetadpsi=X->dthetadpsi;
		double dphidpsi=X->dphidpsi;
		double psinext=psi+dt*dpsi-alpha*((phiref-phi)*dphidpsi+(thetaref-theta)*dthetadpsi);
		S->x13=(double) psinext;
}

void ENSEMkalmanupdate(X_t *X,X_t *reff,X_t *err,X_t *S ,commande_t *commande){

	//estimatorPSI( S,  X , reff);



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
	double y1=   (S->x1);
	double y2=   (S->x2);
	double y3=   (S->x3);
	double y4=   ((S->x7));
	double y5=   ((S->x8));
	double y6=   ((S->x9));
	double y7=   (S->x13);
 	double dt=  X->dt;

	//Xtilde(n+1)=(Ad-Ld*Cd)X_tilde(n)+Bd*V(n)+Ld*Y(n)

 			double dxtilde1= x4 - 0.0473672083*x2 - 0.0445077045*x3 - 21.422689*x1 + 0.0233852062*x7 + 0.0270001322*x8 + 0.0850053708*x9 - 0.575746188*x13 + 21.422689*y1 + 0.0473672083*y2 + 0.0445077045*y3 - 0.0233852062*y4 - 0.0270001322*y5 - 0.0850053708*y6 + 0.575746188*y7;
 			double dxtilde2=0.0170415703*x3 - 21.7753145*x2 - 0.0546611241*x1 + x5 + 0.208606412*x7 + 0.0668915113*x8 + 0.0128038338*x9 + 0.266191992*x13 + 0.0546611241*y1 + 21.7753145*y2 - 0.0170415703*y3 - 0.208606412*y4 - 0.0668915113*y5 - 0.0128038338*y6 - 0.266191992*y7;
			double dxtilde3=0.0236428762*x2 - 0.0382284247*x1 - 21.5970268*x3 + x6 + 0.0653013459*x7 - 0.172568978*x8 - 0.0266135984*x9 + 0.175064282*x13 + 0.0382284247*y1 - 0.0236428762*y2 + 21.5970268*y3 - 0.0653013459*y4 + 0.172568978*y5 + 0.0266135984*y6 - 0.175064282*y7;
			double dxtilde4=   0.253015303*x7 - 0.513875171*x2 - 0.492596575*x3 - 114.644112*x1 + 0.295802667*x8 + 0.878130414*x9 - 6.28034101*x13 + 114.644112*y1 + 0.513875171*y2 + 0.492596575*y3 + 0.746984697*y4 - 0.295802667*y5 - 0.878130414*y6 + 6.28034101*y7;
			double dxtilde5=      0.18707638*x3 - 118.458724*x2 - 0.591900661*x1 + 2.27546479*x7 + 0.731180994*x8 + 0.149053921*x9 + 2.90116345*x13 + 0.591900661*y1 + 118.458724*y2 - 0.18707638*y3 - 2.27546479*y4 + 0.268819006*y5 - 0.149053921*y6 - 2.90116345*y7;
			double dxtilde6=           0.258574736*x2 - 0.423071629*x1 - 116.559144*x3 + 0.70417811*x7 - 1.8675316*x8 - 0.275105727*x9 + 1.90813745*x13 + 0.423071629*y1 - 0.258574736*y2 + 116.559144*y3 - 0.70417811*y4 + 1.8675316*y5 + 1.27510573*y6 - 1.90813745*y7;
			double dxtilde7= 0.0182062573*x1 + 0.213163258*x2 + 0.0715031653*x3 - 21.7949162*x7 + 0.149366728*x8 - 0.0108717823*x9 + x10 + 0.255080336*x13 - 0.0182062573*y1 - 0.213163258*y2 - 0.0715031653*y3 + 21.7949162*y4 - 0.149366728*y5 + 0.0108717823*y6 - 0.255080336*y7;
			double dxtilde8=0.022368685*x1 + 0.0395567158*x2 - 0.0898913579*x3 + 0.0860962862*x7 - 21.7075409*x8 - 0.0126755897*x9 + x11 + 0.0277627699*x13 - 0.022368685*y1 - 0.0395567158*y2 + 0.0898913579*y3 - 0.0860962862*y4 + 21.7075409*y5 + 0.0126755897*y6 - 0.0277627699*y7;
			double dxtilde9=   0.094516989*x1 + 0.0115872738*x2 - 0.0265970383*x3 - 0.0140476961*x7 - 0.0273837499*x8 - 21.3813585*x9 + x12 + 0.607727183*x13 - 0.094516989*y1 - 0.0115872738*y2 + 0.0265970383*y3 + 0.0140476961*y4 + 0.0273837499*y5 + 21.3813585*y6 - 0.607727183*y7;
			double dxtilde10=                c1 + 0.194763036*x1 + 2.32795534*x2 + 0.773662403*x3 - 118.671083*x7 + 1.63737829*x8 - 0.107168962*x9 + 2.78669367*x13 - 0.194763036*y1 - 2.32795534*y2 - 0.773662403*y3 + 118.671083*y4 - 1.63737829*y5 + 0.107168962*y6 - 2.78669367*y7;
			double dxtilde11=         c2 + 0.245108604*x1 + 0.434142086*x2 - 0.958380001*x3 + 0.941614566*x7 - 117.552076*x8 - 0.140944222*x9 + 0.301323611*x13 - 0.245108604*y1 - 0.434142086*y2 + 0.958380001*y3 - 0.941614566*y4 + 117.552076*y5 + 0.140944222*y6 - 0.301323611*y7;
			double dxtilde12=           c3 + 0.982873545*x1 + 0.136341161*x2 - 0.274413534*x3 - 0.145237131*x7 - 0.303809552*x8 - 114.198594*x9 + 6.63060427*x13 - 0.982873545*y1 - 0.136341161*y2 + 0.274413534*y3 + 0.145237131*y4 + 0.303809552*y5 + 114.198594*y6 - 6.63060427*y7;
			double dxtilde13= 0.0448636044*x2 - 0.0996651893*x1 + 0.0325292727*x3 + 0.0452124957*x7 + 0.0120151515*x8 + 0.106875127*x9 - 20.8211541*x13 + x14 + 0.0996651893*y1 - 0.0448636044*y2 - 0.0325292727*y3 - 0.0452124957*y4 - 0.0120151515*y5 - 0.106875127*y6 + 20.8211541*y7;
			double dxtilde14=          c4 - 1.01289292*x1 + 0.455992722*x2 + 0.330627033*x3 + 0.45940987*x7 + 0.122018009*x8 + 1.0862855*x9 - 108.311445*x13 + 1.01289292*y1 - 0.455992722*y2 - 0.330627033*y3 - 0.45940987*y4 - 0.122018009*y5 - 1.0862855*y6 + 108.311445*y7;

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
	// estimation phi theta dtheta dphi wx wy wz
	double phi =-1.0*atan((((x8*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/((x9 + 9.81)*sqrt((((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/(x9 + 9.81)*(x9 + 9.81) + 1.0)));

	double dphi =(((x7*cos(psi + 1.5707963)*(x9 + 9.81))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))) + (x8*sin(psi + 1.5707963)*(x9 + 9.81))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*(x12 - (1.0*(x9 + 9.81)*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*(x10 - (1.0*x7*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((cos(psi + 1.5707963)*(x9 + 9.81)*(x9 + 9.81))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))) + (x8*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) + (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - ((x11 - (1.0*x8*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((1.0*sin(psi + 1.5707963)*(x9 + 9.81)*(x9 + 9.81))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))) - (x7*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (dpsi*((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/((x9 + 9.81)*sqrt((((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/(x9 + 9.81)*(x9 + 9.81) + 1.0));


	double theta =atan((((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/(x9 + 9.81));


	double dtheta =sqrt((((x8*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/((x9 + 9.81)*(x9 + 9.81)*((((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/(x9 + 9.81)*(x9 + 9.81) + 1.0)) + 1.0)*((((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*(x12 - (1.0*(x9 + 9.81)*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))) - (1.0*cos(psi + 1.5707963)*(x9 + 9.81)*(x11 - (1.0*x8*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))) + (sin(psi + 1.5707963)*(x9 + 9.81)*(x10 - (1.0*x7*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))) + (dpsi*((x8*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/((x9 + 9.81)*((((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/(x9 + 9.81)*(x9 + 9.81) + 1.0)*sqrt((((x8*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/((x9 + 9.81)*(x9 + 9.81)*((((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/(x9 + 9.81)*(x9 + 9.81) + 1.0)) + 1.0)));


    	double w1=dphi*cos(psi)*cos(theta) - dpsi*sin(theta) + dtheta*cos(theta)*sin(psi);
		double w2=dtheta*(cos(phi)*cos(psi) + sin(phi)*sin(psi)*sin(theta)) - dphi*(cos(phi)*sin(psi) - cos(psi)*sin(phi)*sin(theta)) + dpsi*cos(theta)*sin(phi);
		double w3=dphi*(sin(phi)*sin(psi) + cos(phi)*cos(psi)*sin(theta)) - dtheta*(cos(psi)*sin(phi) - cos(phi)*sin(psi)*sin(theta)) + dpsi*cos(phi)*cos(theta);


/*
double w1=(((x7*cos(psi + 1.5707963)*(x9 + 9.81))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))) + (x8*sin(psi + 1.5707963)*(x9 + 9.81))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*(x12 - (1.0*(x9 + 9.81)*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*(x10 - (1.0*x7*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((cos(psi + 1.5707963)*(x9 + 9.81)*(x9 + 9.81))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))) + (x8*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - ((x11 - (1.0*x8*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((1.0*sin(psi + 1.5707963)*(x9 + 9.81)*(x9 + 9.81))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))) - (x7*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8));
double w2=(((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*(x12 - (1.0*(x9 + 9.81)*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))) - (1.0*cos(psi + 1.5707963)*(x9 + 9.81)*(x11 - (1.0*x8*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))) + (sin(psi + 1.5707963)*(x9 + 9.81)*(x10 - (1.0*x7*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)));
double w3= dpsi/(sqrt((((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/(x9 + 9.81)*(x9 + 9.81) + 1.0)*sqrt((((x8*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/((x9 + 9.81)*(x9 + 9.81)*((((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/(x9 + 9.81)*(x9 + 9.81) + 1.0)) + 1.0)) + (((x8*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))*((((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*(x12 - (1.0*(x9 + 9.81)*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))) - (1.0*cos(psi + 1.5707963)*(x9 + 9.81)*(x11 - (1.0*x8*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))) + (sin(psi + 1.5707963)*(x9 + 9.81)*(x10 - (1.0*x7*((x12*(x9 + 9.81))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x7*x10)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*x11)/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))/(sqrt( (cos(psi + 1.5707963)*(x9 + 9.81))*(cos(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  (sin(psi + 1.5707963)*(x9 + 9.81))*(sin(psi + 1.5707963)*(x9 + 9.81))/( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) +  ((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi + 1.5707963))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8))) + (dpsi*((x8*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/((x9 + 9.81)*((((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/(x9 + 9.81)*(x9 + 9.81) + 1.0)*sqrt((((x8*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x8*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) - (1.0*x7*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/((x9 + 9.81)*(x9 + 9.81)*((((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/(x9 + 9.81)*(x9 + 9.81) + 1.0)) + 1.0))))/((x9 + 9.81)*sqrt((((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*((x7*cos(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)) + (x8*sin(psi))/sqrt( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))*( (x9 + 9.81)*(x9 + 9.81) +  (x7*x7) +  (x8*x8)))/(x9 + 9.81)*(x9 + 9.81) + 1.0));
*/
/*

// generateur de code matlab
	  double phi_tmp_tmp;
	  double b_phi_tmp_tmp;
	  double a_tmp_tmp_tmp;
	  double a_tmp_tmp;
	  double a_tmp;
	  double b_a_tmp;
	  double phi_tmp_tmp_tmp;
	  double c_phi_tmp_tmp;
	  double phi_tmp;
	  double b_phi_tmp;
	  double c_a_tmp;
	  double dphidpsi_tmp;
	  double b_dphidpsi_tmp;
	  double b_a_tmp_tmp;
	  double d_a_tmp;
	  double c_a_tmp_tmp;
	  double e_a_tmp;
	  double d_a_tmp_tmp;
	  double f_a_tmp;
	  double dphi_tmp;
	  double b_dphi_tmp;
	  double c_dphi_tmp;
	  double d_dphi_tmp;
	  double e_dphi_tmp;
	  double f_dphi_tmp;
	  double g_dphi_tmp;
	  double dtheta_tmp;
	  phi_tmp_tmp = cos(psi);
	  b_phi_tmp_tmp = sin(psi);
	  a_tmp_tmp_tmp = (x9 + 9.81) * (x9 + 9.81);
	  a_tmp_tmp = (a_tmp_tmp_tmp + x7 * x7) + x8 * x8;
	  a_tmp = sqrt(a_tmp_tmp);
	  b_a_tmp = x7 * phi_tmp_tmp / a_tmp + x8 * b_phi_tmp_tmp / a_tmp;
	  phi_tmp_tmp_tmp = b_a_tmp * b_a_tmp;
	  c_phi_tmp_tmp = phi_tmp_tmp_tmp * a_tmp_tmp / a_tmp_tmp_tmp + 1.0;
	  phi_tmp = sqrt(c_phi_tmp_tmp);
	  b_phi_tmp = phi_tmp * (x9 + 9.81);
	  double phi = -atan((x8 * phi_tmp_tmp / a_tmp - x7 * b_phi_tmp_tmp / a_tmp) * a_tmp /
	               b_phi_tmp);
	  c_a_tmp = x8 * cos(psi) / sqrt(((x9 + 9.81) * (x9 + 9.81) + x7 * x7) + x8 * x8)
	    - x7 * sin(psi) / sqrt(((x9 + 9.81) * (x9 + 9.81) + x7 * x7) + x8 * x8);
	  dphidpsi_tmp = c_a_tmp * c_a_tmp;
	  b_dphidpsi_tmp = c_phi_tmp_tmp * a_tmp_tmp_tmp;
	  double dphidpsi = (phi_tmp * b_a_tmp * c_a_tmp + b_phi_tmp * b_a_tmp / a_tmp) /
	    (dphidpsi_tmp + b_dphidpsi_tmp / a_tmp_tmp);
	  double theta = atan(b_a_tmp * a_tmp / (x9 + 9.81));
	  double dthetadpsi = (x9 + 9.81) * c_a_tmp / ((phi_tmp_tmp_tmp + a_tmp_tmp_tmp /
	    a_tmp_tmp) * a_tmp);
	  b_phi_tmp_tmp = cos(psi + 1.57);
	  phi_tmp_tmp_tmp = sin(psi + 1.57);
	  b_a_tmp_tmp = x8 * b_phi_tmp_tmp / a_tmp - x7 * phi_tmp_tmp_tmp / a_tmp;
	  d_a_tmp = fabs(b_a_tmp_tmp);
	  c_a_tmp_tmp = b_phi_tmp_tmp * (x9 + 9.81);
	  e_a_tmp = fabs(c_a_tmp_tmp);
	  d_a_tmp_tmp = phi_tmp_tmp_tmp * (x9 + 9.81);
	  f_a_tmp = fabs(d_a_tmp_tmp);
	  dphi_tmp = (x7 * x10 / a_tmp + x8 * x11 / a_tmp) + x12 * (x9 + 9.81) / a_tmp;
	  b_dphi_tmp = x12 - (x9 + 9.81) * dphi_tmp / a_tmp;
	  c_dphi_tmp = x11 - x8 * dphi_tmp / a_tmp;
	  dphi_tmp = x10 - x7 * dphi_tmp / a_tmp;
	  d_dphi_tmp = phi_tmp_tmp_tmp * a_tmp_tmp_tmp;
	  e_dphi_tmp = x8 * b_a_tmp_tmp;
	  f_dphi_tmp = b_phi_tmp_tmp * a_tmp_tmp_tmp;
	  phi_tmp_tmp = sqrt((d_a_tmp * d_a_tmp + e_a_tmp * e_a_tmp / a_tmp_tmp) +
	                     f_a_tmp * f_a_tmp / a_tmp_tmp);
	  g_dphi_tmp = phi_tmp_tmp * a_tmp_tmp;
	  phi_tmp_tmp *= a_tmp;
	  double dphi = (((x7 * b_phi_tmp_tmp * (x9 + 9.81) / g_dphi_tmp + x8 *
	             phi_tmp_tmp_tmp * (x9 + 9.81) / g_dphi_tmp) * b_dphi_tmp / a_tmp +
	            (x7 * b_a_tmp_tmp / phi_tmp_tmp - d_dphi_tmp / g_dphi_tmp) *
	            c_dphi_tmp / a_tmp) - dphi_tmp * (e_dphi_tmp / phi_tmp_tmp +
	            f_dphi_tmp / g_dphi_tmp) / a_tmp) + dpsi * b_a_tmp * a_tmp /
	    b_phi_tmp;
	  b_a_tmp_tmp *= b_dphi_tmp;
	  dtheta_tmp = dpsi * c_a_tmp * a_tmp;
	  phi_tmp_tmp_tmp = sqrt(dphidpsi_tmp * a_tmp_tmp / b_dphidpsi_tmp + 1.0);
	  double dtheta = phi_tmp_tmp_tmp * (((b_a_tmp_tmp / phi_tmp_tmp - c_a_tmp_tmp *
	    c_dphi_tmp / g_dphi_tmp) + d_a_tmp_tmp * dphi_tmp / g_dphi_tmp) + dtheta_tmp
	    / (c_phi_tmp_tmp * (x9 + 9.81) * phi_tmp_tmp_tmp));
	  double w1 = ((x7 * cos(psi + 1.57) * (x9 + 9.81) / (sqrt((e_a_tmp * e_a_tmp /
	             a_tmp_tmp + d_a_tmp * d_a_tmp) + f_a_tmp * f_a_tmp / a_tmp_tmp) *
	           a_tmp_tmp) + x8 * sin(psi + 1.57) * (x9 + 9.81) / (sqrt((e_a_tmp *
	             e_a_tmp / a_tmp_tmp + d_a_tmp * d_a_tmp) + f_a_tmp * f_a_tmp /
	            a_tmp_tmp) * a_tmp_tmp)) * b_dphi_tmp / a_tmp + (x7 * (x8 * cos(psi
	            + 1.57) / sqrt(((x9 + 9.81) * (x9 + 9.81) + x7 * x7) + x8 * x8) - x7
	           * sin(psi + 1.57) / sqrt(((x9 + 9.81) * (x9 + 9.81) + x7 * x7) + x8 *
	            x8)) / (sqrt((e_a_tmp * e_a_tmp / a_tmp_tmp + d_a_tmp * d_a_tmp) +
	            f_a_tmp * f_a_tmp / a_tmp_tmp) * a_tmp) - d_dphi_tmp / (sqrt
	           ((e_a_tmp * e_a_tmp / a_tmp_tmp + d_a_tmp * d_a_tmp) + f_a_tmp *
	            f_a_tmp / a_tmp_tmp) * a_tmp_tmp)) * c_dphi_tmp / a_tmp) - dphi_tmp *
	    (e_dphi_tmp / (sqrt((e_a_tmp * e_a_tmp / a_tmp_tmp + d_a_tmp * d_a_tmp) +
	                        f_a_tmp * f_a_tmp / a_tmp_tmp) * a_tmp) + f_dphi_tmp /
	     (sqrt((e_a_tmp * e_a_tmp / a_tmp_tmp + d_a_tmp * d_a_tmp) + f_a_tmp *
	           f_a_tmp / a_tmp_tmp) * a_tmp_tmp)) / a_tmp;
	  phi_tmp_tmp = cos(psi + 1.57) * (x9 + 9.81) * (x11 - x8 * ((x7 * x10 / sqrt
	    (((x9 + 9.81) * (x9 + 9.81) + x7 * x7) + x8 * x8) + x8 * x11 / sqrt(((x9 +
	    9.81) * (x9 + 9.81) + x7 * x7) + x8 * x8)) + x12 * (x9 + 9.81) / sqrt(((x9 +
	    9.81) * (x9 + 9.81) + x7 * x7) + x8 * x8)) / sqrt(((x9 + 9.81) * (x9 + 9.81)
	    + x7 * x7) + x8 * x8));
	  b_phi_tmp_tmp = sin(psi + 1.57) * (x9 + 9.81) * (x10 - x7 * ((x7 * x10 / sqrt
	    (((x9 + 9.81) * (x9 + 9.81) + x7 * x7) + x8 * x8) + x8 * x11 / sqrt(((x9 +
	    9.81) * (x9 + 9.81) + x7 * x7) + x8 * x8)) + x12 * (x9 + 9.81) / sqrt(((x9 +
	    9.81) * (x9 + 9.81) + x7 * x7) + x8 * x8)) / sqrt(((x9 + 9.81) * (x9 + 9.81)
	    + x7 * x7) + x8 * x8));
	  double w2 = (b_a_tmp_tmp / (sqrt((e_a_tmp * e_a_tmp / a_tmp_tmp + d_a_tmp * d_a_tmp)
	           + f_a_tmp * f_a_tmp / a_tmp_tmp) * a_tmp) - phi_tmp_tmp / (sqrt
	          ((e_a_tmp * e_a_tmp / a_tmp_tmp + d_a_tmp * d_a_tmp) + f_a_tmp *
	           f_a_tmp / a_tmp_tmp) * a_tmp_tmp)) + b_phi_tmp_tmp / (sqrt((e_a_tmp *
	    e_a_tmp / a_tmp_tmp + d_a_tmp * d_a_tmp) + f_a_tmp * f_a_tmp / a_tmp_tmp) *
	    a_tmp_tmp);
	  double w3 = dpsi / (phi_tmp * phi_tmp_tmp_tmp) + (x8 * cos(psi) / sqrt(((x9 + 9.81) *
	    (x9 + 9.81) + x7 * x7) + x8 * x8) - x7 * sin(psi) / sqrt(((x9 + 9.81) * (x9
	    + 9.81) + x7 * x7) + x8 * x8)) * sqrt(((x9 + 9.81) * (x9 + 9.81) + x7 * x7)
	    + x8 * x8) * (((b_a_tmp_tmp / (sqrt((e_a_tmp * e_a_tmp / a_tmp_tmp + d_a_tmp
	    * d_a_tmp) + f_a_tmp * f_a_tmp / a_tmp_tmp) * a_tmp) - phi_tmp_tmp / (sqrt
	    ((e_a_tmp * e_a_tmp / a_tmp_tmp + d_a_tmp * d_a_tmp) + f_a_tmp * f_a_tmp /
	     a_tmp_tmp) * a_tmp_tmp)) + b_phi_tmp_tmp / (sqrt((e_a_tmp * e_a_tmp /
	    a_tmp_tmp + d_a_tmp * d_a_tmp) + f_a_tmp * f_a_tmp / a_tmp_tmp) * a_tmp_tmp))
	                  + dtheta_tmp / ((b_a_tmp * b_a_tmp * a_tmp_tmp / a_tmp_tmp_tmp
	    + 1.0) * (x9 + 9.81) * sqrt(c_a_tmp * c_a_tmp * a_tmp_tmp / ((b_a_tmp *
	    b_a_tmp * a_tmp_tmp / a_tmp_tmp_tmp + 1.0) * a_tmp_tmp_tmp) + 1.0))) /
	    b_phi_tmp;

	  */
	X->theta=  theta;



	X->phi=  phi;


	X->dphi=  dphi;
	X->dtheta=  dtheta;

	X->w1=  w1;
	X->w2=  w2;
	X->w3=  w3;

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
	double r13=(reff->psi);





	double rphi=   (reff->phi);
	double rtheta=  (reff->theta);

  	double rw3=   (reff->w3);
	double rw2=   (reff->w2);
	double rw1=   (reff->w1);


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
 	double errphi=  fabs(phi-rphi);
	double errtheta=  fabs(theta-rtheta);
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
 		(err->psi)=   errx13;
 		(err->theta)=  errtheta;
 		(err->phi)=  errphi;
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
