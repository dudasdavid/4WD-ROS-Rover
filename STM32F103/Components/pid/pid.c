#include "pid.h"
#include "functions.h"

void pid_initialize(PID_t* controller)
{
    controller->config.P = 0.0f;
    controller->config.I = 0.0f;
    controller->config.D = 0.0f;
    controller->config.LowerLimit = 0.0f;
    controller->config.UpperLimit = 0.0f;

    controller->state.previousError = 0.0f;
    controller->state.previousOutput = 0.0f;
    controller->state.previousFeedback = 0.0f;
}

void pid_reset(PID_t* controller)
{
    controller->state.previousError = 0.0f;
    controller->state.previousOutput = 0.0f;
    controller->state.previousFeedback = 0.0f;
}

float pid_update(PID_t* controller, float refSignal, float feedback)
{
    float error = refSignal - feedback;
    float dError = error - controller->state.previousError;

    float p = error * controller->config.P;
    float d = dError * controller->config.D;

    float u1k = p + d;
    float u2k = (1 - controller->config.I) * controller->state.previousFeedback + controller->config.I * controller->state.previousOutput;
    float uk = constrain_f32(u1k + u2k, controller->config.LowerLimit, controller->config.UpperLimit);

    if (error == 0)
    {
        u2k = 0;
    }

    controller->state.previousError = error;
    controller->state.previousFeedback = u2k;
    controller->state.previousOutput = uk;

    return uk;
}
