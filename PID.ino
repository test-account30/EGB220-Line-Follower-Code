void setupPID(PID *pid, float kp, float ki, float kd, float min_i, float max_i) {
  // Initialize PID parameters
  pid->kp = kp;                   // Proportional gain
  pid->ki = ki;                   // Integral gain
  pid->kd = kd;                   // Derivative gain
  pid->previous_error = 0;        // Initialize previous error
  pid->integral = 0;              // Initialize integral term
  pid->start = TCNT1;             // Start time for calculating dt
  pid->min_i = min_i;             // Minimum value for integral wind-up prevention
  pid->max_i = max_i;             // Maximum value for integral wind-up prevention
}

float calculatePID(PID *pid, float setpoint, float measured_value, int16_t max, int16_t min) {
  // Calculate time difference since last call
  uint16_t delta = ((TCNT1 - pid->start) < 0) ? ((TCNT1 - pid->start) + 65535) : (TCNT1 - pid->start);
  float dt = (float)(delta) * 0.0000032;
  pid->start = TCNT1;             // Update start time

  // Calculate PID components
  float currentError = setpoint - measured_value;
  float proportional = pid->kp * currentError;
  pid->integral += currentError * dt * pid->ki; // Integral term with anti-windup

  // Limit integral term to prevent wind-up
  if (pid->integral < pid->min_i){
    pid->integral = pid->min_i;
  } else if (pid->integral > pid->max_i){
    pid->integral = pid->max_i;
  }

  // Derivative term
  float derivative = (currentError - pid->previous_error) / dt;
  float output = proportional + pid->integral + pid->kd * derivative;
  pid->previous_error = currentError;  // Update previous error

  // Limit output within defined range
  if (output > max){
    output = max;
  } else if (output < min){
    output = min;
  }

  return output;  // Return PID output
}
