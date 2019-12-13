/**
 * Authored by Michael Hamer (http://www.mikehamer.info), June 2016
 * Thank you to Mark Mueller (www.mwm.im) for advice during implementation,
 * and for derivation of the original filter in the below-cited paper.
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
 * ============================================================================
 *
 * The Kalman filter implemented in this file is based on the papers:
 *
 * "Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation"
 * http://ieeexplore.ieee.org/xpl/articleDetails.jsp?arnumber=7139421
 *
 * and
 *
 * "Covariance Correction Step for Kalman Filtering with an Attitude"
 * http://arc.aiaa.org/doi/abs/10.2514/1.G000848
 *
 * Academic citation would be appreciated.
 *
 * BIBTEX ENTRIES:
      @INPROCEEDINGS{MuellerHamerUWB2015,
      author  = {Mueller, Mark W and Hamer, Michael and D’Andrea, Raffaello},
      title   = {Fusing ultra-wideband range measurements with accelerometers and rate gyroscopes for quadrocopter state estimation},
      booktitle = {2015 IEEE International Conference on Robotics and Automation (ICRA)},
      year    = {2015},
      month   = {May},
      pages   = {1730-1736},
      doi     = {10.1109/ICRA.2015.7139421},
      ISSN    = {1050-4729}}

      @ARTICLE{MuellerCovariance2016,
      author={Mueller, Mark W and Hehn, Markus and D’Andrea, Raffaello},
      title={Covariance Correction Step for Kalman Filtering with an Attitude},
      journal={Journal of Guidance, Control, and Dynamics},
      pages={1--7},
      year={2016},
      publisher={American Institute of Aeronautics and Astronautics}}
 *
 * ============================================================================
 *
 * MAJOR CHANGELOG:
 * 2016.06.28, Mike Hamer: Initial version
 * 2019.04.12, Kristoffer Richardsson: Refactored, separated kalman implementation from OS related functionality
 *
 */
#include <math.h>

#include "kalman_core.h"
#include "estimator_kalman.h"
#include "kalman_supervisor.h"

#include "stm32f4xx.h"

#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include "sensors.h"

#include "system.h"
#include "log.h"
#include "param.h"
#include "physicalConstants.h"

#include "statsCnt.h"

#define DEBUG_MODULE "ESTKALMAN"
#include "debug.h"
#define pi 3.14159265358979323846

// #define KALMAN_USE_BARO_UPDATE


/**
 * Additionally, the filter supports the incorporation of additional sensors into the state estimate
 *
 * This is done via the external functions:
 * - bool estimatorKalmanEnqueueUWBPacket(uwbPacket_t *uwb)
 * - bool estimatorKalmanEnqueuePosition(positionMeasurement_t *pos)
 * - bool estimatorKalmanEnqueueDistance(distanceMeasurement_t *dist)
 *
 * As well as by the following internal functions and datatypes
 */

// Distance-to-point measurements
static xQueueHandle distDataQueue;
#define DIST_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasDistanceMeasurement(distanceMeasurement_t *dist) {
	return (pdTRUE == xQueueReceive(distDataQueue, dist, 0));
}

// Direct measurements of Crazyflie position
static xQueueHandle posDataQueue;
#define POS_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasPositionMeasurement(positionMeasurement_t *pos) {
	return (pdTRUE == xQueueReceive(posDataQueue, pos, 0));
}

// Direct measurements of Crazyflie pose
static xQueueHandle poseDataQueue;
#define POSE_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasPoseMeasurement(poseMeasurement_t *pose) {
	return (pdTRUE == xQueueReceive(poseDataQueue, pose, 0));
}

// Measurements of a UWB Tx/Rx
static xQueueHandle tdoaDataQueue;
#define UWB_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasTDOAPacket(tdoaMeasurement_t *uwb) {
	return (pdTRUE == xQueueReceive(tdoaDataQueue, uwb, 0));
}


// Measurements of flow (dnx, dny)
static xQueueHandle flowDataQueue;
#define FLOW_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasFlowPacket(flowMeasurement_t *flow) {
	return (pdTRUE == xQueueReceive(flowDataQueue, flow, 0));
}

// Measurements of TOF from laser sensor
static xQueueHandle tofDataQueue;
#define TOF_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasTOFPacket(tofMeasurement_t *tof) {
	return (pdTRUE == xQueueReceive(tofDataQueue, tof, 0));
}

// Absolute height measurement along the room Z
static xQueueHandle heightDataQueue;
#define HEIGHT_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasHeightPacket(heightMeasurement_t *height) {
	return (pdTRUE == xQueueReceive(heightDataQueue, height, 0));
}

static xQueueHandle yawErrorDataQueue;
#define YAW_ERROR_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasYawErrorPacket(yawErrorMeasurement_t *error)
{
	return (pdTRUE == xQueueReceive(yawErrorDataQueue, error, 0));
}

static xQueueHandle sweepAnglesDataQueue;
#define SWEEP_ANGLES_QUEUE_LENGTH (10)

static inline bool stateEstimatorHasSweepAnglesPacket(sweepAngleMeasurement_t *angles)
{
	return (pdTRUE == xQueueReceive(sweepAnglesDataQueue, angles, 0));
}

// Semaphore to signal that we got data from the stabilzer loop to process
static SemaphoreHandle_t runTaskSemaphore;

// Mutex to protect data that is shared between the task and
// functions called by the stabilizer loop
static SemaphoreHandle_t dataMutex;


/**
 * Constants used in the estimator
 */

#define CRAZYFLIE_WEIGHT_grams (27.0f)

//thrust is thrust mapped for 65536 <==> 60 GRAMS!
#define CONTROL_TO_ACC (GRAVITY_MAGNITUDE*60.0f/CRAZYFLIE_WEIGHT_grams/65536.0f)


/**
 * Tuning parameters
 */
#define PREDICT_RATE RATE_100_HZ // this is slower than the IMU update rate of 500Hz
#define BARO_RATE RATE_25_HZ

// the point at which the dynamics change from stationary to flying
#define IN_FLIGHT_THRUST_THRESHOLD (GRAVITY_MAGNITUDE*0.1f)
#define IN_FLIGHT_TIME_THRESHOLD (500)

// The bounds on the covariance, these shouldn't be hit, but sometimes are... why?
#define MAX_COVARIANCE (100)
#define MIN_COVARIANCE (1e-6f)



/**
 * Quadrocopter State
 *
 * The internally-estimated state is:
 * - X, Y, Z: the quad's position in the global frame
 * - PX, PY, PZ: the quad's velocity in its body frame
 * - D0, D1, D2: attitude error
 *
 * For more information, refer to the paper
 */

static kalmanCoreData_t coreData;

/**
 * Internal variables. Note that static declaration results in default initialization (to 0)
 */

static bool isInit = false;

static Axis3f accAccumulator;
static float thrustAccumulator;
static Axis3f gyroAccumulator;
static float baroAslAccumulator;
static uint32_t accAccumulatorCount;
static uint32_t thrustAccumulatorCount;
static uint32_t gyroAccumulatorCount;
static uint32_t baroAccumulatorCount;
static bool quadIsFlying = false;
static uint32_t lastFlightCmd;
static uint32_t takeoffTime;

// Data used to enable the task and stabilizer loop to run with minimal locking
static state_t taskEstimatorState; // The estimator state produced by the task, copied to the stabilzer when needed.
static Axis3f gyroSnapshot; // A snpashot of the latest gyro data, used by the task
static Axis3f accSnapshot; // A snpashot of the latest acc data, used by the task

// Statistics
#define ONE_SECOND 1000
static STATS_CNT_RATE_DEFINE(updateCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(predictionCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(baroUpdateCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(finalizeCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(measurementAppendedCounter, ONE_SECOND);
static STATS_CNT_RATE_DEFINE(measurementNotAppendedCounter, ONE_SECOND);


static const bool useBaroUpdate = false;


/**
 * Supporting and utility functions
 */

static inline void mat_trans(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst)); }
static inline void mat_inv(const arm_matrix_instance_f32 * pSrc, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst)); }
static inline void mat_mult(const arm_matrix_instance_f32 * pSrcA, const arm_matrix_instance_f32 * pSrcB, arm_matrix_instance_f32 * pDst)
{ configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst)); }
static inline float arm_sqrt(float32_t in)
{ float pOut = 0; arm_status result = arm_sqrt_f32(in, &pOut); configASSERT(ARM_MATH_SUCCESS == result); return pOut; }

static void kalmanTask(void* parameters);
static bool predictStateForward(uint32_t osTick, float dt);
static bool updateQueuedMeasurments(const Axis3f *gyro, const uint32_t tick);


// --------------------------------------------------

// Called one time during system startup
void estimatorKalmanTaskInit() {

	tdoaDataQueue = xQueueCreate(UWB_QUEUE_LENGTH, sizeof(tdoaMeasurement_t));

	vSemaphoreCreateBinary(runTaskSemaphore);

	dataMutex = xSemaphoreCreateMutex();

	xTaskCreate(kalmanTask, KALMAN_TASK_NAME, 3 * configMINIMAL_STACK_SIZE, NULL, KALMAN_TASK_PRI, NULL);

	isInit = true;
}

bool estimatorKalmanTaskTest() {
	return isInit;
}

static void kalmanTask(void* parameters) {
	systemWaitStart();

	uint32_t lastPrediction = xTaskGetTickCount();
	uint32_t nextPrediction = xTaskGetTickCount();
	uint32_t lastPNUpdate = xTaskGetTickCount();
	uint32_t nextBaroUpdate = xTaskGetTickCount();

	while (true) {
		xSemaphoreTake(runTaskSemaphore, portMAX_DELAY);

		// If the client triggers an estimator reset via parameter update
		if (coreData.resetEstimation) {
			estimatorKalmanInit();
			coreData.resetEstimation = false;
		}

		// Tracks whether an update to the state has been made, and the state therefore requires finalization
		bool doneUpdate = false;

		uint32_t osTick = xTaskGetTickCount(); // would be nice if this had a precision higher than 1ms...

#ifdef KALMAN_DECOUPLE_XY
		kalmanCoreDecoupleXY(&coreData);
#endif

		// Run the system dynamics to predict the state forward.
		if (osTick >= nextPrediction) { // update at the PREDICT_RATE
			float dt = T2S(osTick - lastPrediction);
			if (predictStateForward(osTick, dt)) {
				coreData.dt=dt;
				lastPrediction = osTick;
				doneUpdate = true;
				STATS_CNT_RATE_EVENT(&predictionCounter);
			}

			nextPrediction = osTick + S2T(1.0f / PREDICT_RATE);
		}

		/**
		 * Add process noise every loop, rather than every prediction
		 */
		{
			float dt = T2S(osTick - lastPNUpdate);
			if (dt > 0.0f) {
				kalmanCoreAddProcessNoise(&coreData, dt);
				lastPNUpdate = osTick;
			}
		}

		/**
		 * Update the state estimate with the barometer measurements
		 */
		// Accumulate the barometer measurements
		if (useBaroUpdate) {
			if (osTick > nextBaroUpdate // update at BARO_RATE
					&& baroAccumulatorCount > 0)
			{
				xSemaphoreTake(dataMutex, portMAX_DELAY);
				float baroAslAverage = baroAslAccumulator / baroAccumulatorCount;
				baroAslAccumulator = 0;
				baroAccumulatorCount = 0;
				xSemaphoreGive(dataMutex);

				kalmanCoreUpdateWithBaro(&coreData, baroAslAverage, quadIsFlying);

				nextBaroUpdate = osTick + S2T(1.0f / BARO_RATE);
				doneUpdate = true;

				STATS_CNT_RATE_EVENT(&baroUpdateCounter);
			}
		}

		{
			Axis3f gyro;
			xSemaphoreTake(dataMutex, portMAX_DELAY);
			memcpy(&gyro, &gyroSnapshot, sizeof(gyro));
			xSemaphoreGive(dataMutex);
			doneUpdate = doneUpdate || updateQueuedMeasurments(&gyro, osTick);
		}

		/**
		 * If an update has been made, the state is finalized:
		 * - the attitude error is moved into the body attitude quaternion,
		 * - the body attitude is converted into a rotation matrix for the next prediction, and
		 * - correctness of the covariance matrix is ensured
		 */

		if (doneUpdate)
		{
			kalmanCoreFinalize(&coreData, osTick);
			STATS_CNT_RATE_EVENT(&finalizeCounter);
			if (! kalmanSupervisorIsStateWithinBounds(&coreData)) {
				coreData.resetEstimation = true;
				DEBUG_PRINT("State out of bounds, resetting\n");
			}
		}

		/**
		 * Finally, the internal state is externalized.
		 * This is done every round, since the external state includes some sensor data
		 */
		xSemaphoreTake(dataMutex, portMAX_DELAY);
		kalmanCoreExternalizeState(&coreData, &taskEstimatorState, &accSnapshot, osTick);
		xSemaphoreGive(dataMutex);

		STATS_CNT_RATE_EVENT(&updateCounter);
	}
}

void estimatorKalman(state_t *state, sensorData_t *sensors, control_t *control, const uint32_t tick)
{
	// This function is called from the stabilizer loop. It is important that this call returns
	// as quickly as possible. The dataMutex must only be locked short periods by the task.
	xSemaphoreTake(dataMutex, portMAX_DELAY);

	// Average the last IMU measurements. We do this because the prediction loop is
	// slower than the IMU loop, but the IMU information is required externally at
	// a higher rate (for body rate control).
	if (sensorsReadAcc(&sensors->acc)) {
		accAccumulator.x += sensors->acc.x;
		accAccumulator.y += sensors->acc.y;
		accAccumulator.z += sensors->acc.z;
		accAccumulatorCount++;
	}

	if (sensorsReadGyro(&sensors->gyro)) {
		gyroAccumulator.x += sensors->gyro.x;
		gyroAccumulator.y += sensors->gyro.y;
		gyroAccumulator.z += sensors->gyro.z;
		gyroAccumulatorCount++;
	}

	// Average the thrust command from the last time steps, generated externally by the controller
	thrustAccumulator += control->thrust;
	thrustAccumulatorCount++;

	// Average barometer data
	if (useBaroUpdate) {
		if (sensorsReadBaro(&sensors->baro)) {
			baroAslAccumulator += sensors->baro.asl;
			baroAccumulatorCount++;
		}
	}

	// Make a copy of sensor data to be used by the task
	memcpy(&gyroSnapshot, &sensors->gyro, sizeof(gyroSnapshot));
	memcpy(&accSnapshot, &sensors->acc, sizeof(accSnapshot));

	// Copy the latest state, calculated by the task
	memcpy(state, &taskEstimatorState, sizeof(state_t));
	xSemaphoreGive(dataMutex);

	xSemaphoreGive(runTaskSemaphore);
}


void estimatorKalmanENSEM(state_t *state,X_t *X, sensorData_t *sensors, commande_t *commande, const uint32_t tick)
{
	// This function is called from the stabilizer loop. It is important that this call returns
	// as quickly as possible. The dataMutex must only be locked short periods by the task.
	xSemaphoreTake(dataMutex, portMAX_DELAY);

	// Average the last IMU measurements. We do this because the prediction loop is
	// slower than the IMU loop, but the IMU information is required externally at
	// a higher rate (for body rate control).
	if (sensorsReadAcc(&sensors->acc)) {
		accAccumulator.x += sensors->acc.x;
		accAccumulator.y += sensors->acc.y;
		accAccumulator.z += sensors->acc.z;
		accAccumulatorCount++;
	}

	if (sensorsReadGyro(&sensors->gyro)) {
		gyroAccumulator.x += sensors->gyro.x;
		gyroAccumulator.y += sensors->gyro.y;
		gyroAccumulator.z += sensors->gyro.z;
		gyroAccumulatorCount++;
	}




	// Make a copy of sensor data to be used by the task
	memcpy(&gyroSnapshot, &sensors->gyro, sizeof(gyroSnapshot));
	memcpy(&accSnapshot, &sensors->acc, sizeof(accSnapshot));

	// Copy the latest state, calculated by the task
	memcpy(state, &taskEstimatorState, sizeof(state_t));
	// Xtilde(n)
	X->dt=state->dt;

	float x1=(X->x1);
	float x2=(X->x2);
	float x3=(X->x3);

	float x4=(X->x4);
	float x5=(X->x5);
	float x6=(X->x6);

	float x7=(X->x7);
	float x8=(X->x8);
	float x9=(X->x9);


	float x10=(X->x10);
	float x11=(X->x11);
	float x12=(X->x12);

	float x13=(X->x13);
	float x14=(X->x14);
	//V(n+1)
	float c1=(commande->c1);
	float c2=(commande->c2);
	float c3=(commande->c3);
	float c4=(commande->c4);

	//Y(n+1)
	float y1=(state->position.x);
	float y2=(state->position.y);
	float y3=(state->position.z);
	float y4=(state->acc.x);
	float y5=(state->acc.y);
	float y6=(state->acc.z);
	float y7=(state->attitude.yaw);
	y7=y7*PI/180;

	//Xtilde(n+1)=(Ad-Ld*Cd)X_tilde(n)+Bd*V(n+1)+Ld*Y(n+1)
	float x_tilde1=(1650586719047173*c1)/2475880078570760549798248448 + (8509472336763341*x1)/9007199254740992 + (434285472110257*x2)/5070602400912917605986812821504 + (128217160858545*x3)/10141204801825835211973625643008 + x4/500 - (5204186148374251*x7)/147573952589676412928 - (8150004463588079*x8)/324518553658426726783156020576256 - (3462033884534349*x9)/20282409603651670423947251286016 + (6447604371278021*x10)/4835703278458516698824704 + (8614535209131591*x13)/162259276829213363391578010288128 + (1990907671910605*y1)/36028797018963968 - (434285472110257*y2)/5070602400912917605986812821504 - (128217160858545*y3)/10141204801825835211973625643008 + (1374833513388401*y4)/36893488147419103232 + (8150004463588079*y5)/324518553658426726783156020576256 + (3462033884534349*y6)/20282409603651670423947251286016 - (8614535209131591*y7)/162259276829213363391578010288128;
	float x_tilde2=(1650586719047173*c2)/2475880078570760549798248448 + (2431018203653399*x1)/40564819207303340847894502572032 + (4254736168381665*x2)/4503599627370496 - (2383896189763537*x3)/1267650600228229401496703205376 + x5/500 + (5532113893109173*x7)/10141204801825835211973625643008 - (1301046537101863*x8)/36893488147419103232 - (1186191873737937*x9)/316912650057057350374175801344 + (6447604371278021*x11)/4835703278458516698824704 - (1248099424504189*x13)/5070602400912917605986812821504 - (2431018203653399*y1)/40564819207303340847894502572032 + (7963630687642595*y2)/144115188075855872 + (2383896189763537*y3)/1267650600228229401496703205376 - (5532113893109173*y4)/10141204801825835211973625643008 + (5499334053586805*y5)/147573952589676412928 + (1186191873737937*y6)/316912650057057350374175801344 + (1248099424504189*y7)/5070602400912917605986812821504;
	float x_tilde3=(1650586719047173*c3)/2475880078570760549798248448 - (6462429602744535*x1)/10384593717069655257060992658440192 - (4812460142247641*x2)/2535301200456458802993406410752 + (8509472336763353*x3)/9007199254740992 + x6/500 - (8529798949465035*x7)/20282409603651670423947251286016 - (5055378628564317*x8)/2535301200456458802993406410752 - (162630817137699*x9)/4611686018427387904 + (6447604371278021*x12)/4835703278458516698824704 + (4524149914017877*x13)/5070602400912917605986812821504 + (6462429602744535*y1)/10384593717069655257060992658440192 + (4812460142247641*y2)/2535301200456458802993406410752 + (1990907671910555*y3)/36028797018963968 + (8529798949465035*y4)/20282409603651670423947251286016 + (5055378628564317*y5)/2535301200456458802993406410752 + (5499334053585721*y6)/147573952589676412928 - (4524149914017877*y7)/5070602400912917605986812821504;
	float x_tilde4=(6447604371278021*c1)/4835703278458516698824704 - (3838937864563719*x1)/72057594037927936 + (6453468104883251*x2)/2535301200456458802993406410752 - (5103004577771195*x3)/5192296858534827628530496329220096 + x4 - (4119251494795*x7)/144115188075855872 - (1648476162610775*x8)/2535301200456458802993406410752 - (1428470663752133*x9)/316912650057057350374175801344 + (4722366482869645*x10)/2361183241434822606848 + (7160657078187969*x13)/5070602400912917605986812821504 + (3838937864563719*y1)/72057594037927936 - (6453468104883251*y2)/2535301200456458802993406410752 + (5103004577771195*y3)/5192296858534827628530496329220096 + (1169398510586027*y4)/576460752303423488 + (1648476162610775*y5)/2535301200456458802993406410752 + (1428470663752133*y6)/316912650057057350374175801344 - (7160657078187969*y7)/5070602400912917605986812821504;
	float x_tilde5=(6447604371278021*c2)/4835703278458516698824704 + (1829799135812031*x1)/1267650600228229401496703205376 - (3838937864562305*x2)/72057594037927936 - (101800042382003*x3)/19807040628566084398385987584 + x5 - (6467916565626025*x7)/10141204801825835211973625643008 - (65908023910781*x8)/2305843009213693952 - (7561097745634471*x9)/1267650600228229401496703205376 + (4722366482869645*x11)/2361183241434822606848 - (3083336365796785*x13)/316912650057057350374175801344 - (1829799135812031*y1)/1267650600228229401496703205376 + (3838937864562305*y2)/72057594037927936 + (101800042382003*y3)/19807040628566084398385987584 + (6467916565626025*y4)/10141204801825835211973625643008 + (4677594042338169*y5)/2305843009213693952 + (7561097745634471*y6)/1267650600228229401496703205376 + (3083336365796785*y7)/316912650057057350374175801344;
	float x_tilde6=(6447604371278021*c3)/4835703278458516698824704 + (3487215537945483*x1)/40564819207303340847894502572032 - (6862253710276683*x2)/1267650600228229401496703205376 - (3838937864564071*x3)/72057594037927936 + x6 - (7171302989186517*x7)/81129638414606681695789005144064 - (8387330305563933*x8)/633825300114114700748351602688 - (16477005979989*x9)/576460752303423488 + (4722366482869645*x12)/2361183241434822606848 - (2443319463388235*x13)/633825300114114700748351602688 - (3487215537945483*y1)/40564819207303340847894502572032 + (6862253710276683*y2)/1267650600228229401496703205376 + (3838937864564071*y3)/72057594037927936 + (7171302989186517*y4)/81129638414606681695789005144064 + (8387330305563933*y5)/633825300114114700748351602688 + (292349627646709*y6)/144115188075855872 + (2443319463388235*y7)/633825300114114700748351602688;
	float x_tilde7=(4722366482869645*c1)/2361183241434822606848 - (4924214215908721*x1)/147573952589676412928 + (2832707400678951*x2)/5070602400912917605986812821504 - (8273691095220397*x3)/20282409603651670423947251286016 + (4254741967356617*x7)/4503599627370496 - (4652664044234409*x8)/40564819207303340847894502572032 + (5273728876385753*x9)/1267650600228229401496703205376 + x10/500 - (1010320465786493*x13)/1267650600228229401496703205376 + (4924214215908721*y1)/147573952589676412928 - (2832707400678951*y2)/5070602400912917605986812821504 + (8273691095220397*y3)/20282409603651670423947251286016 + (7963445120444127*y4)/144115188075855872 + (4652664044234409*y5)/40564819207303340847894502572032 - (5273728876385753*y6)/1267650600228229401496703205376 + (1010320465786493*y7)/1267650600228229401496703205376;
	float x_tilde8=(4722366482869645*c2)/2361183241434822606848 - (4386779871887833*x1)/162259276829213363391578010288128 - (4924214215952477*x2)/147573952589676412928 - (4775265156125719*x3)/2535301200456458802993406410752 - (5036428155296089*x7)/40564819207303340847894502572032 + (8509483934713203*x8)/9007199254740992 - (2890002318274399*x9)/1267650600228229401496703205376 + x11/500 + (400898180437171*x13)/633825300114114700748351602688 + (4386779871887833*y1)/162259276829213363391578010288128 + (4924214215952477*y2)/147573952589676412928 + (4775265156125719*y3)/2535301200456458802993406410752 + (5036428155296089*y4)/40564819207303340847894502572032 + (7963445120444623*y5)/144115188075855872 + (2890002318274399*y6)/1267650600228229401496703205376 - (400898180437171*y7)/633825300114114700748351602688;
	float x_tilde9=(4722366482869645*c3)/2361183241434822606848 - (4930641196106199*x1)/40564819207303340847894502572032 - (146797386130473*x2)/39614081257132168796771975168 - (4924214215943069*x3)/147573952589676412928 + (5248306583967421*x7)/1267650600228229401496703205376 - (2882442201222095*x8)/1267650600228229401496703205376 + (66480343239947*x9)/70368744177664 + x12/500 - (1745580614379499*x13)/633825300114114700748351602688 + (4930641196106199*y1)/40564819207303340847894502572032 + (146797386130473*y2)/39614081257132168796771975168 + (4924214215943069*y3)/147573952589676412928 - (5248306583967421*y4)/1267650600228229401496703205376 + (2882442201222095*y5)/1267650600228229401496703205376 + (3981722560222207*y6)/72057594037927936 + (1745580614379499*y7)/633825300114114700748351602688;
	float x_tilde10=c1/500 - (7313540037622495*x1)/295147905179352825856 + (4889320531538109*x2)/2535301200456458802993406410752 - (138523193178619*x3)/316912650057057350374175801344 - (3836379215375067*x7)/72057594037927936 + (4515652717810793*x8)/5070602400912917605986812821504 + (5269295292070735*x9)/1267650600228229401496703205376 + x10 + (4667258545635077*x13)/5070602400912917605986812821504 + (7313540037622495*y1)/295147905179352825856 - (4889320531538109*y2)/2535301200456458802993406410752 + (138523193178619*y3)/316912650057057350374175801344 + (3836379215375067*y4)/72057594037927936 - (4515652717810793*y5)/5070602400912917605986812821504 - (5269295292070735*y6)/1267650600228229401496703205376 - (4667258545635077*y7)/5070602400912917605986812821504;
	float x_tilde11= c2/500 - (2543015514268719*x1)/5070602400912917605986812821504 - (7313540039712093*x2)/295147905179352825856 + (7077438181441335*x3)/2535301200456458802993406410752 - (2445472278431711*x7)/5070602400912917605986812821504 - (1918189607687975*x8)/36028797018963968 + (2242150229551613*x9)/1267650600228229401496703205376 + x11 + (1648596557181699*x13)/633825300114114700748351602688 + (2543015514268719*y1)/5070602400912917605986812821504 + (7313540039712093*y2)/295147905179352825856 - (7077438181441335*y3)/2535301200456458802993406410752 + (2445472278431711*y4)/5070602400912917605986812821504 + (1918189607687975*y5)/36028797018963968 - (2242150229551613*y6)/1267650600228229401496703205376 - (1648596557181699*y7)/633825300114114700748351602688;
	float x_tilde12=c3/500 + (235778336882067*x1)/1267650600228229401496703205376 - (11380195459903*x2)/2475880078570760549798248448 - (3571064472065*x3)/144115188075855872 - (7743546158645517*x7)/2535301200456458802993406410752 + (167258565022555*x8)/39614081257132168796771975168 - (7672758430748715*x9)/144115188075855872 + x12 + (1438111292793017*x13)/316912650057057350374175801344 - (235778336882067*y1)/1267650600228229401496703205376 + (11380195459903*y2)/2475880078570760549798248448 + (3571064472065*y3)/144115188075855872 + (7743546158645517*y4)/2535301200456458802993406410752 - (167258565022555*y5)/39614081257132168796771975168 + (7672758430748715*y6)/144115188075855872 - (1438111292793017*y7)/316912650057057350374175801344;
	float x_tilde13=(4722366482869645*c4)/2361183241434822606848 + (585556179541431*x1)/10141204801825835211973625643008 - (4034969232072761*x2)/20282409603651670423947251286016 + (4551946813241353*x3)/5070602400912917605986812821504 - (4222708384697523*x7)/5070602400912917605986812821504 + (7241313967716515*x8)/10141204801825835211973625643008 - (6951187768151161*x9)/2535301200456458802993406410752 + (4254741917011557*x13)/4503599627370496 + x14/500 - (585556179541431*y1)/10141204801825835211973625643008 + (4034969232072761*y2)/20282409603651670423947251286016 - (4551946813241353*y3)/5070602400912917605986812821504 + (4222708384697523*y4)/5070602400912917605986812821504 - (7241313967716515*y5)/10141204801825835211973625643008 + (6951187768151161*y6)/2535301200456458802993406410752 + (3981723365743025*y7)/72057594037927936;
	float x_tilde14=c4/500 - (4713840440822021*x1)/10141204801825835211973625643008 - (1401580579134241*x2)/633825300114114700748351602688 - (7294227486092599*x3)/1267650600228229401496703205376 - (5386170889304005*x7)/5070602400912917605986812821504 + (2101879118736829*x8)/158456325028528675187087900672 + (447478013441805*x9)/79228162514264337593543950336 - (3836379673566835*x13)/72057594037927936 + x14 + (4713840440822021*y1)/10141204801825835211973625643008 + (1401580579134241*y2)/633825300114114700748351602688 + (7294227486092599*y3)/1267650600228229401496703205376 + (5386170889304005*y4)/5070602400912917605986812821504 - (2101879118736829*y5)/158456325028528675187087900672 - (447478013441805*y6)/79228162514264337593543950336 + (3836379673566835*y7)/72057594037927936;
 	(X->x1)=x_tilde1;
	(X->x2)=x_tilde2;
	(X->x3)=x_tilde3;

	(X->x4)=x_tilde4;
	(X->x5)=x_tilde5;
	(X->x6)=x_tilde6;

	(X->x7)=x_tilde7;
	(X->x8)=x_tilde8;
	(X->x9)=x_tilde9;

	(X->x10)=x_tilde10;
	(X->x11)=x_tilde11;
	(X->x12)=x_tilde12;

	(X->x13)=x_tilde13;
	(X->x14)=x_tilde14;
	(X->psi)=(X->x13);
	(X->dpsi)=(X->x14);
	// (phi,theta,dphi,dtheta,w1,w2,w3)_tilde(n+1)
	float cpsi=cos(x_tilde13);
	float spsi=sin(x_tilde13);

	float f=sqrt(pow(x_tilde7,2)+pow(x_tilde8,2)+pow(x_tilde9,2));

 	float num=(x_tilde9+9.81)/f;
	float den=x_tilde7*cpsi+x_tilde8*spsi;

	X->theta=atan2(num,denum);

	float stheta=sin(theta);
	float ctheta=cos(theta);


	num=x_tilde9/ctheta;
	denum=x_tilde7*spsi-x_tilde8*cpsi;

	X->phi=atan2(num,denum);
	X->dphi =x_tilde14*sin(theta) + (((7447868781240427*x_tilde8^2)/(9007199254740992*f^2) - 7447868781240427/9007199254740992)*((sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100)^2)/(f^2*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2)) - (x_tilde7*((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f))/(f*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2))))/f - (7447868781240427*x_tilde8*(x_tilde9 + 981/100)*((x_tilde7*cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))/(f^2*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2)) + (x_tilde8*sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))/(f^2*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2))))/(9007199254740992*f^3) + (7447868781240427*x_tilde7*x_tilde8*((cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100)^2)/(f^2*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2)) + (x_tilde8*((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f))/(f*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2))))/(9007199254740992*f^3);
	X->dtheta =-(x_tilde14*cos(theta)*sin(phi) - (cos(x_tilde13 + pi/2)*((7447868781240427*x_tilde8^2)/(9007199254740992*f^2) - 7447868781240427/9007199254740992)*(x_tilde9 + 981/100))/(f^2*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2)) + (7447868781240427*x_tilde8*((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)*(x_tilde9 + 981/100))/(9007199254740992*f^3*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2)) + (7447868781240427*x_tilde7*x_tilde8*sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))/(9007199254740992*f^4*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2)))/cos(phi);
	X->w1=(((7447868781240427*x_tilde8^2)/(9007199254740992*f^2) - 7447868781240427/9007199254740992)*((sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100)^2)/(f^2*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2)) - (x_tilde7*((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f))/(f*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2))))/f - (7447868781240427*x_tilde8*(x_tilde9 + 981/100)*((x_tilde7*cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))/(f^2*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2)) + (x_tilde8*sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))/(f^2*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2))))/(9007199254740992*f^3) + (7447868781240427*x_tilde7*x_tilde8*((cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100)^2)/(f^2*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2)) + (x_tilde8*((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f))/(f*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2))))/(9007199254740992*f^3);
 	X->w2=(cos(x_tilde13 + pi/2)*((7447868781240427*x_tilde8^2)/(9007199254740992*f^2) - 7447868781240427/9007199254740992)*(x_tilde9 + 981/100))/(f^2*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2)) - (7447868781240427*x_tilde8*((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)*(x_tilde9 + 981/100))/(9007199254740992*f^3*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2)) - (7447868781240427*x_tilde7*x_tilde8*sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))/(9007199254740992*f^4*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2));
	X->w3=x_tilde14*cos(phi)*cos(theta) + (sin(phi)*(x_tilde14*cos(theta)*sin(phi) - (cos(x_tilde13 + pi/2)*((7447868781240427*x_tilde8^2)/(9007199254740992*f^2) - 7447868781240427/9007199254740992)*(x_tilde9 + 981/100))/(f^2*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2)) + (7447868781240427*x_tilde8*((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)*(x_tilde9 + 981/100))/(9007199254740992*f^3*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2)) + (7447868781240427*x_tilde7*x_tilde8*sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))/(9007199254740992*f^4*(abs((x_tilde8*cos(x_tilde13 + pi/2))/f - (x_tilde7*sin(x_tilde13 + pi/2))/f)^2 + abs(cos(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2 + abs(sin(x_tilde13 + pi/2)*(x_tilde9 + 981/100))^2/abs(f)^2)^(1/2))))/cos(phi);
 	xSemaphoreGive(dataMutex);
	xSemaphoreGive(runTaskSemaphore);
}

static bool predictStateForward(uint32_t osTick, float dt) {
	/*  if (gyroAccumulatorCount == 0
      || accAccumulatorCount == 0
      || thrustAccumulatorCount == 0)
  {
    return false;
  }*/
	if (gyroAccumulatorCount == 0
			|| accAccumulatorCount == 0
	)
	{
		return false;
	}
	xSemaphoreTake(dataMutex, portMAX_DELAY);

	// gyro is in deg/sec but the estimator requires rad/sec
	Axis3f gyroAverage;
	gyroAverage.x = gyroAccumulator.x * DEG_TO_RAD / gyroAccumulatorCount;
	gyroAverage.y = gyroAccumulator.y * DEG_TO_RAD / gyroAccumulatorCount;
	gyroAverage.z = gyroAccumulator.z * DEG_TO_RAD / gyroAccumulatorCount;

	// accelerometer is in Gs but the estimator requires ms^-2
	Axis3f accAverage;
	accAverage.x = accAccumulator.x * GRAVITY_MAGNITUDE / accAccumulatorCount;
	accAverage.y = accAccumulator.y * GRAVITY_MAGNITUDE / accAccumulatorCount;
	accAverage.z = accAccumulator.z * GRAVITY_MAGNITUDE / accAccumulatorCount;

	// thrust is in grams, we need ms^-2
	//float thrustAverage = thrustAccumulator * CONTROL_TO_ACC / thrustAccumulatorCount;
	float thrustAverage=9999;
	accAccumulator = (Axis3f){.axis={0}};
	accAccumulatorCount = 0;
	gyroAccumulator = (Axis3f){.axis={0}};
	gyroAccumulatorCount = 0;
	/* thrustAccumulator = 0;
  thrustAccumulatorCount = 0;
	 */
	xSemaphoreGive(dataMutex);

	// TODO: Find a better check for whether the quad is flying
	// Assume that the flight begins when the thrust is large enough and for now we never stop "flying".
	if (thrustAverage > IN_FLIGHT_THRUST_THRESHOLD) {
		lastFlightCmd = osTick;
		if (!quadIsFlying) {
			takeoffTime = lastFlightCmd;
		}
	}
	quadIsFlying = (osTick-lastFlightCmd) < IN_FLIGHT_TIME_THRESHOLD;

	kalmanCorePredict(&coreData, &accAverage, &gyroAverage, dt, quadIsFlying);

	return true;
}


static bool updateQueuedMeasurments(const Axis3f *gyro, const uint32_t tick) {
	bool doneUpdate = false;
	/**
	 * Sensor measurements can come in sporadically and faster than the stabilizer loop frequency,
	 * we therefore consume all measurements since the last loop, rather than accumulating
	 */

	tofMeasurement_t tof;
	while (stateEstimatorHasTOFPacket(&tof))
	{
		kalmanCoreUpdateWithTof(&coreData, &tof);
		doneUpdate = true;
	}

	yawErrorMeasurement_t yawError;
	while (stateEstimatorHasYawErrorPacket(&yawError))
	{
		kalmanCoreUpdateWithYawError(&coreData, &yawError);
		doneUpdate = true;
	}

	heightMeasurement_t height;
	while (stateEstimatorHasHeightPacket(&height))
	{
		kalmanCoreUpdateWithAbsoluteHeight(&coreData, &height);
		doneUpdate = true;
	}

	distanceMeasurement_t dist;
	while (stateEstimatorHasDistanceMeasurement(&dist))
	{
		kalmanCoreUpdateWithDistance(&coreData, &dist);
		doneUpdate = true;
	}

	positionMeasurement_t pos;
	while (stateEstimatorHasPositionMeasurement(&pos))
	{
		kalmanCoreUpdateWithPosition(&coreData, &pos);
		doneUpdate = true;
	}

	poseMeasurement_t pose;
	while (stateEstimatorHasPoseMeasurement(&pose))
	{
		kalmanCoreUpdateWithPose(&coreData, &pose);
		doneUpdate = true;
	}

	tdoaMeasurement_t tdoa;
	while (stateEstimatorHasTDOAPacket(&tdoa))
	{
		kalmanCoreUpdateWithTDOA(&coreData, &tdoa);
		doneUpdate = true;
	}

	flowMeasurement_t flow;
	while (stateEstimatorHasFlowPacket(&flow))
	{
		kalmanCoreUpdateWithFlow(&coreData, &flow, gyro);
		doneUpdate = true;
	}

	sweepAngleMeasurement_t angles;
	while (stateEstimatorHasSweepAnglesPacket(&angles))
	{
		kalmanCoreUpdateWithSweepAngles(&coreData, &angles, tick);
		doneUpdate = true;
	}

	return doneUpdate;
}

// Called when this estimator is activated

void estimatorKalmanInit(void) {

	xQueueReset(tdoaDataQueue);


	xSemaphoreTake(dataMutex, portMAX_DELAY);
	accAccumulator = (Axis3f){.axis={0}};
	gyroAccumulator = (Axis3f){.axis={0}};


	accAccumulatorCount = 0;
	gyroAccumulatorCount = 0;


	xSemaphoreGive(dataMutex);

	kalmanCoreInit(&coreData);
}
void estimatorKalmanInit(void) {
	xQueueReset(distDataQueue);
	xQueueReset(posDataQueue);
	xQueueReset(poseDataQueue);
	xQueueReset(tdoaDataQueue);
	xQueueReset(flowDataQueue);
	xQueueReset(tofDataQueue);

	xSemaphoreTake(dataMutex, portMAX_DELAY);
	accAccumulator = (Axis3f){.axis={0}};
	gyroAccumulator = (Axis3f){.axis={0}};
	thrustAccumulator = 0;
	baroAslAccumulator = 0;

	accAccumulatorCount = 0;
	gyroAccumulatorCount = 0;
	thrustAccumulatorCount = 0;
	baroAccumulatorCount = 0;
	xSemaphoreGive(dataMutex);

	kalmanCoreInit(&coreData);
}

static bool appendMeasurement(xQueueHandle queue, void *measurement)
{
	portBASE_TYPE result;
	bool isInInterrupt = (SCB->ICSR & SCB_ICSR_VECTACTIVE_Msk) != 0;

	if (isInInterrupt) {
		portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
		result = xQueueSendFromISR(queue, measurement, &xHigherPriorityTaskWoken);
		if(xHigherPriorityTaskWoken == pdTRUE)
		{
			portYIELD();
		}
	} else {
		result = xQueueSend(queue, measurement, 0);
	}

	if (result == pdTRUE) {
		STATS_CNT_RATE_EVENT(&measurementAppendedCounter);
		return true;
	} else {
		STATS_CNT_RATE_EVENT(&measurementNotAppendedCounter);
		return true;
	}
}

bool estimatorKalmanEnqueueTDOA(const tdoaMeasurement_t *uwb)
{
	ASSERT(isInit);
	return appendMeasurement(tdoaDataQueue, (void *)uwb);
}

bool estimatorKalmanEnqueuePosition(const positionMeasurement_t *pos)
{
	ASSERT(isInit);
	return appendMeasurement(posDataQueue, (void *)pos);
}

bool estimatorKalmanEnqueuePose(const poseMeasurement_t *pose)
{
	ASSERT(isInit);
	return appendMeasurement(poseDataQueue, (void *)pose);
}

bool estimatorKalmanEnqueueDistance(const distanceMeasurement_t *dist)
{
	ASSERT(isInit);
	return appendMeasurement(distDataQueue, (void *)dist);
}

bool estimatorKalmanEnqueueFlow(const flowMeasurement_t *flow)
{
	// A flow measurement (dnx,  dny) [accumulated pixels]
	ASSERT(isInit);
	return appendMeasurement(flowDataQueue, (void *)flow);
}

bool estimatorKalmanEnqueueTOF(const tofMeasurement_t *tof)
{
	// A distance (distance) [m] to the ground along the z_B axis.
	ASSERT(isInit);
	return appendMeasurement(tofDataQueue, (void *)tof);
}

bool estimatorKalmanEnqueueAbsoluteHeight(const heightMeasurement_t *height)
{
	// A distance (height) [m] to the ground along the z axis.
	ASSERT(isInit);
	return appendMeasurement(heightDataQueue, (void *)height);
}

bool estimatorKalmanEnqueueYawError(const yawErrorMeasurement_t* error)
{
	ASSERT(isInit);
	return appendMeasurement(yawErrorDataQueue, (void *)error);
}

bool estimatorKalmanEnqueueSweepAngles(const sweepAngleMeasurement_t *angles)
{
	ASSERT(isInit);
	return appendMeasurement(sweepAnglesDataQueue, (void *)angles);
}

bool estimatorKalmanTest(void)
{
	return isInit;
}

void estimatorKalmanGetEstimatedPos(point_t* pos) {
	pos->x = coreData.S[KC_STATE_X];
	pos->y = coreData.S[KC_STATE_Y];
	pos->z = coreData.S[KC_STATE_Z];
}

void estimatorKalmanGetEstimatedRot(float * rotationMatrix) {
	memcpy(rotationMatrix, coreData.R, 9*sizeof(float));
}

// Temporary development groups
LOG_GROUP_START(kalman_states)
LOG_ADD(LOG_FLOAT, ox, &coreData.S[KC_STATE_X])
LOG_ADD(LOG_FLOAT, oy, &coreData.S[KC_STATE_Y])
LOG_ADD(LOG_FLOAT, vx, &coreData.S[KC_STATE_PX])
LOG_ADD(LOG_FLOAT, vy, &coreData.S[KC_STATE_PY])
LOG_GROUP_STOP(kalman_states)


// Stock log groups
LOG_GROUP_START(kalman)
LOG_ADD(LOG_UINT8, inFlight, &quadIsFlying)
LOG_ADD(LOG_FLOAT, stateX, &coreData.S[KC_STATE_X])
LOG_ADD(LOG_FLOAT, stateY, &coreData.S[KC_STATE_Y])
LOG_ADD(LOG_FLOAT, stateZ, &coreData.S[KC_STATE_Z])
LOG_ADD(LOG_FLOAT, statePX, &coreData.S[KC_STATE_PX])
LOG_ADD(LOG_FLOAT, statePY, &coreData.S[KC_STATE_PY])
LOG_ADD(LOG_FLOAT, statePZ, &coreData.S[KC_STATE_PZ])
LOG_ADD(LOG_FLOAT, stateD0, &coreData.S[KC_STATE_D0])
LOG_ADD(LOG_FLOAT, stateD1, &coreData.S[KC_STATE_D1])
LOG_ADD(LOG_FLOAT, stateD2, &coreData.S[KC_STATE_D2])
LOG_ADD(LOG_FLOAT, varX, &coreData.P[KC_STATE_X][KC_STATE_X])
LOG_ADD(LOG_FLOAT, varY, &coreData.P[KC_STATE_Y][KC_STATE_Y])
LOG_ADD(LOG_FLOAT, varZ, &coreData.P[KC_STATE_Z][KC_STATE_Z])
LOG_ADD(LOG_FLOAT, varPX, &coreData.P[KC_STATE_PX][KC_STATE_PX])
LOG_ADD(LOG_FLOAT, varPY, &coreData.P[KC_STATE_PY][KC_STATE_PY])
LOG_ADD(LOG_FLOAT, varPZ, &coreData.P[KC_STATE_PZ][KC_STATE_PZ])
LOG_ADD(LOG_FLOAT, varD0, &coreData.P[KC_STATE_D0][KC_STATE_D0])
LOG_ADD(LOG_FLOAT, varD1, &coreData.P[KC_STATE_D1][KC_STATE_D1])
LOG_ADD(LOG_FLOAT, varD2, &coreData.P[KC_STATE_D2][KC_STATE_D2])
LOG_ADD(LOG_FLOAT, q0, &coreData.q[0])
LOG_ADD(LOG_FLOAT, q1, &coreData.q[1])
LOG_ADD(LOG_FLOAT, q2, &coreData.q[2])
LOG_ADD(LOG_FLOAT, q3, &coreData.q[3])

STATS_CNT_RATE_LOG_ADD(rtUpdate, &updateCounter)
STATS_CNT_RATE_LOG_ADD(rtPred, &predictionCounter)
STATS_CNT_RATE_LOG_ADD(rtBaro, &baroUpdateCounter)
STATS_CNT_RATE_LOG_ADD(rtFinal, &finalizeCounter)
STATS_CNT_RATE_LOG_ADD(rtApnd, &measurementAppendedCounter)
STATS_CNT_RATE_LOG_ADD(rtRej, &measurementNotAppendedCounter)
LOG_GROUP_STOP(kalman)

PARAM_GROUP_START(kalman)
PARAM_ADD(PARAM_UINT8, resetEstimation, &coreData.resetEstimation)
PARAM_ADD(PARAM_UINT8, quadIsFlying, &quadIsFlying)
PARAM_GROUP_STOP(kalman)
