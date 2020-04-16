#ifndef PID_H_
#define PID_H_

typedef struct
{
    struct {
        float P;
        float I;
        float D;
        float LowerLimit;
        float UpperLimit;
    } config;

    struct {
        float previousOutput;
        float previousFeedback;
        float previousError;
    } state;
} PID_t;

void pid_initialize(PID_t* controller);
void pid_reset(PID_t* controller);
float pid_update(PID_t* controller, float refSignal, float feedback);

#endif /* PID_H_ */