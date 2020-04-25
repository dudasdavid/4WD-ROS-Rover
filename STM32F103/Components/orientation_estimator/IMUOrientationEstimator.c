#include "IMUOrientationEstimator.h"
#include <math.h>

static Quaternion_t orientation = {1.0f, 0.0f, 0.0f, 0.0f};

static int32_t nTurns = 0;
static float lastYaw = 0.0f;

float deg_to_rad(float angle)
{
    return angle * (float) M_PI / 180.0f;
}

float rad_to_deg(float rad)
{
    return rad * 180.0f / (float) M_PI;
}

static Orientation3D_t to_euler_angles(const Quaternion_t orientation)
{
    Orientation3D_t angles;

    // roll (x-axis rotation)
    float sinr_cosp = 2.0f * (orientation.q0 * orientation.q1 + orientation.q2 * orientation.q3);
    float cosr_cosp = 1.0f - 2.0f * (orientation.q1 * orientation.q1 + orientation.q2 * orientation.q2);
    angles.roll = atan2f(sinr_cosp, cosr_cosp);

    // pitch (y-axis rotation)
    float sinp = 2.0f * (orientation.q0 * orientation.q2 - orientation.q3 * orientation.q1);
    if (fabsf(sinp) >= 1.0f)
    {
        angles.pitch = copysignf((float)M_PI_2, sinp); // use 90 degrees if out of range
    }
    else
    {
        angles.pitch = asinf(sinp);
    }

    // yaw (z-axis rotation)
    float siny_cosp = 2.0f * (orientation.q0 * orientation.q3 + orientation.q1 * orientation.q2);
    float cosy_cosp = 1.0f - 2.0f * (orientation.q2 * orientation.q2 + orientation.q3 * orientation.q3);
    angles.yaw = atan2f(siny_cosp, cosy_cosp);

    return angles;
}

static Quaternion_t madgwick_imu(const float sampleTime, const Vector3D_t acceleration, const Vector3D_t angularSpeed, const Quaternion_t previous)
{
    float q0 = previous.q0;
    float q1 = previous.q1;
    float q2 = previous.q2;
    float q3 = previous.q3;

    // Rate of change of quaternion from gyroscope
    float qDot1 = -q1 * angularSpeed.x - q2 * angularSpeed.y - q3 * angularSpeed.z;
    float qDot2 = q0 * angularSpeed.x + q2 * angularSpeed.z - q3 * angularSpeed.y;
    float qDot3 = q0 * angularSpeed.y - q1 * angularSpeed.z + q3 * angularSpeed.x;
    float qDot4 = q0 * angularSpeed.z + q1 * angularSpeed.y - q2 * angularSpeed.x;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    if (acceleration.x != 0.0f || acceleration.y != 0.0f || acceleration.z != 0.0f)
    {
        // Normalise accelerometer measurement
        float invLength = 1.0f / sqrtf(acceleration.x * acceleration.x + acceleration.y * acceleration.y + acceleration.z * acceleration.z);
        float x = acceleration.x * invLength;
        float y = acceleration.y * invLength;
        float z = acceleration.z * invLength;

        // Auxiliary variables to avoid repeated arithmetic
        float q0q0_p_q3q3 = q0 * q0 + q3 * q3;
        float q1q1_p_q2q2 = q1 * q1 + q2 * q2;

        // Gradient descent algorithm corrective step
        float s0 = 2.0f * q0 * (q1q1_p_q2q2) + (q2 * x - q1 * y);
        float s1 = 2.0f * q1 * (2.0f * q1q1_p_q2q2 + q0q0_p_q3q3 + z - 1.0f) - (q3 * x + q0 * y);
        float s2 = 2.0f * q2 * (2.0f * q1q1_p_q2q2 + q0q0_p_q3q3 + z - 1.0f) + (q0 * x - q3 * y);
        float s3 = 2.0f * q3 * (q1q1_p_q2q2) - (q1 * x + q2 * y);

        // Apply feedback step
        const float beta = 0.2f; // < TODO: tune if necessary
        float scaling = beta / sqrtf(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);
        qDot1 -= s0 * scaling;
        qDot2 -= s1 * scaling;
        qDot3 -= s2 * scaling;
        qDot4 -= s3 * scaling;
    }

    // Integrate rate of change of quaternion to yield quaternion
    q0 += qDot1 * 0.5f * sampleTime;
    q1 += qDot2 * 0.5f * sampleTime;
    q2 += qDot3 * 0.5f * sampleTime;
    q3 += qDot4 * 0.5f * sampleTime;

    // Normalise quaternion
    float norm = 1.0f / sqrtf(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    return (Quaternion_t) {
        q0 * norm,
        q1 * norm,
        q2 * norm,
        q3 * norm
    };
}

void IMUOrientationEstimator(float sampleTime, const Vector3D_t* inAcc, const Vector3D_t* inGyro, const Vector3D_t* inMag, Orientation3D_t* outEuler, Quaternion_t* outQuat)
{
    Vector3D_t angularSpeedRad = {
        .x = deg_to_rad(inGyro->x),
        .y = deg_to_rad(inGyro->y),
        .z = deg_to_rad(inGyro->z)
    };
    orientation = madgwick_imu(sampleTime, *inAcc, angularSpeedRad, orientation);

    const Orientation3D_t euler = to_euler_angles(orientation);

    /* track yaw angle past the 360° mark */
    float yaw = rad_to_deg(euler.yaw);

    if (yaw - lastYaw > 180)
    {
        nTurns -= 1;
    }
    else if (yaw - lastYaw < -180)
    {
        nTurns += 1;
    }

    lastYaw = yaw;

    *outEuler = (Orientation3D_t) {
        .pitch = euler.pitch,
        .roll = euler.roll,
        .yaw = deg_to_rad(yaw + nTurns * 360)
    };
    *outQuat = orientation;
}
