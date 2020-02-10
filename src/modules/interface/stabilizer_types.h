/**
 *    ||          ____  _ __
 * +------+      / __ )(_) /_______________ _____  ___
 * | 0xBC |     / __  / / __/ ___/ ___/ __ `/_  / / _ \
 * +------+    / /_/ / / /_/ /__/ /  / /_/ / / /_/  __/
 *  ||  ||    /_____/_/\__/\___/_/   \__,_/ /___/\___/
 *
 * Crazyflie control firmware
 *
 * Copyright (C) 2011-2012 Bitcraze AB
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
 * stabilizer.h: Stabilizer orchestrator
 */
#ifndef __STABILIZER_TYPES_H__
#define __STABILIZER_TYPES_H__

#include <stdint.h>
#include <stdbool.h>
#include "imu_types.h"
#include "lighthouse_geometry.h"

/* Data structure used by the stabilizer subsystem.
 * All have a timestamp to be set when the data is calculated.
 */

/** Attitude in euler angle form */
typedef struct attitude_s {
  uint32_t timestamp;  // Timestamp when the data was computed

  float roll;
  float pitch;
  float yaw;
} attitude_t;

/* x,y,z vector */
struct vec3_s {
  uint32_t timestamp; // Timestamp when the data was computed

  float x;
  float y;
  float z;
};

typedef struct vec3_s vector_t;
typedef struct vec3_s point_t;
typedef struct vec3_s velocity_t;
typedef struct vec3_s acc_t;
typedef struct vec3_s jerk_t;
typedef struct vec3_s d4p_t;


/* Orientation as a quaternion */
typedef struct quaternion_s {
  uint32_t timestamp;

  union {
    struct {
      float q0;
      float q1;
      float q2;
      float q3;
    };
    struct {
      float x;
      float y;
      float z;
      float w;
    };
  };
} quaternion_t;

typedef struct tdoaMeasurement_s {
  point_t anchorPosition[2];
  float distanceDiff;
  float stdDev;
} tdoaMeasurement_t;

typedef struct baro_s {
  float pressure;           // mbar
  float temperature;        // degree Celcius
  float asl;                // m (ASL = altitude above sea level)
} baro_t;

typedef struct positionMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  float stdDev;
} positionMeasurement_t;

typedef struct poseMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  quaternion_t quat;
  float stdDevPos;
  float stdDevQuat;
} poseMeasurement_t;

typedef struct distanceMeasurement_s {
  union {
    struct {
      float x;
      float y;
      float z;
    };
    float pos[3];
  };
  float distance;
  float stdDev;
} distanceMeasurement_t;

typedef struct zDistance_s {
  uint32_t timestamp;
  float distance;           // m
} zDistance_t;

typedef struct sensorData_s {
  Axis3f acc;               // Gs
  Axis3f gyro;              // deg/s
  Axis3f mag;               // gauss
  baro_t baro;
#ifdef LOG_SEC_IMU
  Axis3f accSec;            // Gs
  Axis3f gyroSec;           // deg/s
#endif
  uint64_t interruptTimestamp;
} sensorData_t;

typedef struct state_s {
  attitude_t attitude;      // deg (legacy CF2 body coordinate system, where pitch is inverted)
  quaternion_t attitudeQuaternion;
  point_t position;         // m
  velocity_t velocity;      // m/s
  acc_t acc;                // Gs (but acc.z without considering gravity)
  double dt;
  float w1;
  float w2;
  float w3;
} state_t;


typedef struct X_s{

	double x1;
	double x2;
	double x3;

	double x4;
	double x5;
	double x6;

	double x7;
	double x8;
	double x9;

	double x10;
	double x11;
	double x12;

	double x14;
	double x13;

	double phi;
	double theta;
	double psi;
	double dphi;
	double dtheta;
	double dpsi;
	double w1;
	double w2;
	double w3;
	double df;
	double f;
	double d2f;

	double dphidpsi;
	double dthetadpsi;
	double dt;
	double d4x;
	double d4y;
	double d4z;
	double d2psi;
	int Morceau_traj;
	int isInit;
 	double currenttime;

 	double accbx;
 	double accby;
 	double accbz;
	int start;




}X_t;
typedef struct X_log_s{

	float x1;
	float x2;
	float x3;

	float x4;
	float x5;
	float x6;

	float x7;
	float x8;
	float x9;

	float x10;
	float x11;
	float x12;

	float x14;
	float x13;

	float phi;
	float theta;
	float psi;
	float dphi;
	float dtheta;
	float dpsi;
	float w1;
	float w2;
	float w3;
	float df;
	float f;
	float d2f;

float dphidpsi;
float dthetadpsi;
	float dt;
	float d4x;
	float d4y;
	float d4z;
	float d2psi;
	int Morceau_traj;
 	float currenttime;
	int start;


}X_log_t;

typedef struct ctr_traj_s{
	float t1;
}ctr_traj_t;
typedef struct control_s {
  int16_t roll;
  int16_t pitch;
  int16_t yaw;
  float thrust;
} control_t;
typedef struct commande_s{
	double c1;
	double c2;
	double c3;
	double c4;
	double w1;
	double w2;
	double w3;
	double w4;
	double w1bc;
	double w2bc;
	double w3bc;
	double w4bc;
	double currenttime;
	int start;
}commande_t;
typedef struct Logcommande_s{
	float c1;
	float c2;
	float c3;
	float c4;
	float w1;
	float w2;
	float w3;
	float w4;
	float w1bc;
	float w2bc;
	float w3bc;
	float w4bc;
	float currenttime;
	int start;

}Logcommande_t;
typedef enum mode_e {
  modeDisable = 0,
  modeAbs,
  modeVelocity
} stab_mode_t;


typedef struct setpoint_s {
  uint32_t timestamp;

  attitude_t attitude;      // deg
  attitude_t attitudeRate;  // deg/s
  quaternion_t attitudeQuaternion;
  float thrust;
  point_t position;         // m
  velocity_t velocity;      // m/s
  acc_t acceleration;       // m/s^2
  jerk_t jerk;
  d4p_t d4p;
  float psi;
  float dpsi;
  float d2psi;

  bool velocity_body;       // true if velocity is given in body frame; false if velocity is given in world frame

  struct {
    stab_mode_t x;
    stab_mode_t y;
    stab_mode_t z;
    stab_mode_t roll;
    stab_mode_t pitch;
    stab_mode_t yaw;
    stab_mode_t quat;
  } mode;
  int TypeTraj;
} setpoint_t;

/** Estimate of position */
typedef struct estimate_s {
  uint32_t timestamp; // Timestamp when the data was computed

  point_t position;
} estimate_t;

/** Setpoint for althold */
typedef struct setpointZ_s {
  float z;
  bool isUpdate; // True = small update of setpoint, false = completely new
} setpointZ_t;

/** Flow measurement**/
typedef struct flowMeasurement_s {
  uint32_t timestamp;
  union {
    struct {
      float dpixelx;  // Accumulated pixel count x
      float dpixely;  // Accumulated pixel count y
    };
    float dpixel[2];  // Accumulated pixel count
  };
  float stdDevX;      // Measurement standard deviation
  float stdDevY;      // Measurement standard deviation
  float dt;           // Time during which pixels were accumulated
} flowMeasurement_t;


/** TOF measurement**/
typedef struct tofMeasurement_s {
  uint32_t timestamp;
  float distance;
  float stdDev;
} tofMeasurement_t;

/** Absolute height measurement */
typedef struct heightMeasurement_s {
  uint32_t timestamp;
  float height;
  float stdDev;
} heightMeasurement_t;

/** Yaw error measurement */
typedef struct {
  uint32_t timestamp;
  float yawError;
  float stdDev;
} yawErrorMeasurement_t;

/** Sweep angle measurement */
typedef struct {
  uint32_t timestamp;
  vec3d* baseStationPos;
  mat3d* baseStationRot;     // Base station rotation matrix
  mat3d* baseStationRotInv;  // Inverted base station rotation matrix
  float angleX;
  float angleY;
  float stdDevX;
  float stdDevY;
  vec3d* sensorPos;
} sweepAngleMeasurement_t;

// Frequencies to bo used with the RATE_DO_EXECUTE_HZ macro. Do NOT use an arbitrary number.
#define RATE_1000_HZ 1000
#define RATE_500_HZ 500
#define RATE_250_HZ 250
#define RATE_100_HZ 100
#define RATE_50_HZ 50
#define RATE_25_HZ 25

#define RATE_MAIN_LOOP RATE_1000_HZ
#define ATTITUDE_RATE RATE_500_HZ
#define POSITION_RATE RATE_100_HZ

#define RATE_DO_EXECUTE(RATE_HZ, TICK) ((TICK % (RATE_MAIN_LOOP / RATE_HZ)) == 0)

#endif
