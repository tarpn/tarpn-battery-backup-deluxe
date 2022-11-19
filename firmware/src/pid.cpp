#include "pid.h"
#include "Arduino.h" // For min and max

void PID_Init(PID * pid, float k_error, float k_integral) {
  pid->pid_k_error = k_error;
  pid->pid_k_integral = k_integral;
  
  pid->pid_error = 0.0f;
  pid->pid_integral = 0.0f;
  pid->last_measurement = 0.0f;
}

void PID_Update(PID * pid, unsigned long dt_ms, float set_point, float measurement) {
  float error = pid->pid_k_error * (set_point - measurement);
  
  pid->pid_integral = pid->pid_integral + pid->pid_k_integral * dt_ms * (error + pid->pid_error);
  pid->pid_error = error;
  pid->last_measurement = measurement;    
  
  // Clamp the integrator term, no more than 5% of system
  pid->pid_integral = min(6.0, max(-6.0, pid->pid_integral)); 
  
  // Clamp the control output
  pid->pid_output = min(255, max(0, pid->pid_output + pid->pid_error + pid->pid_integral));

}