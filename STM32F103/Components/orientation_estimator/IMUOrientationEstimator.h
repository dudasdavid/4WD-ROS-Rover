#ifndef COMPONENT_IMU_ORIENTATION_ESTIMATOR_H_
#define COMPONENT_IMU_ORIENTATION_ESTIMATOR_H_

#ifndef COMPONENT_TYPES_IMU_ORIENTATION_ESTIMATOR_H_
#define COMPONENT_TYPES_IMU_ORIENTATION_ESTIMATOR_H_

#include <float.h>
#include <stdint.h>

#define M_PI     3.14159f
#define M_PI_2   1.570795f

typedef struct {
    float x;
    float y;
    float z;
} Vector3D_t;

typedef struct {
    float pitch;
    float roll;
    float yaw;
} Orientation3D_t;

typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} Quaternion_t;

#endif /* COMPONENT_TYPES_IMU_ORIENTATION_ESTIMATOR_H_ */

void IMUOrientationEstimator(float sampleTime, const Vector3D_t* inAcc, const Vector3D_t* inGyro, const Vector3D_t* inMag, Orientation3D_t* outEuler, Quaternion_t* outQuat);
float deg_to_rad(float angle);
float rad_to_deg(float rad);
#endif /* COMPONENT_IMU_ORIENTATION_ESTIMATOR_H_ */
