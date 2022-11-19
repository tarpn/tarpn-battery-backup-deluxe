#ifndef PID_H
#define PID_H

typedef struct {
  float pid_k_error;
  float pid_k_integral;

  float pid_error;
  float pid_integral;
  float last_measurement;
  float pid_output;
} PID;

void PID_Init(PID * pid, float k_error, float k_integral);
void PID_Update(PID * pid, unsigned long time_ms, float set_point, float measurement);

#endif