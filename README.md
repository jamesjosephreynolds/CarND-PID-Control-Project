# PID Controls for Steering Angle and Vehicle Speed Project
This project is originally forked from https://github.com/udacity/CarND-PID-Control-Project.  This repository includes starter code, that is used herein.

## Controller Structure
For this project I implemented two PID controllers: one for the steering angle and one for the vehicle speed.  They are both instances of the class `PID`.

### Controller Setpoints and Error
For the steering angle controller, the setpoint is `0.0` meters from center, and the error is simply the cross-track error (CTE).
```C++
// Use pid to calculate the steering angle
pid.UpdateError(cte);
steer_value = pid.TotalError();
if (steer_value > 1.0f) {
  steer_value = 1.0f;
} else if (steer_value < -1.0f) {
  steer_value = -1.0f;
}
```

For the vehicle speed controller, the setpoint is more complicated.  The general idea is that the target speed decreases as the cross-track error increases.  So, as the vehicle deviates from the center of the lane, the car should slow down.
```C++
// Use CTE and angle to calculate the target speed
double cte_lim = 1.5;
double steer_lim = 0.5;
double tgt_spd;
if (fabs(cte) > cte_lim) {
  // If CTE is too large, limit target speed to min value
  tgt_spd = 10;
} else if (fabs(steer_value) > steer_lim) {
  // If steering angle is too large, limit target speed to min value
  tgt_spd = 10;
} else {
  // Target speed is a linearly decreasing function of CTE absolute value
  // As CTE increases, the target speed decreases
  tgt_spd = 30*fabs((fabs(cte) - cte_lim)) + 10;
}
```

The error, then, is the difference between the target speed and the actual vehicle speed.
```C++
// Use pid_sp to calculate the throttle value
pid_sp.UpdateError(-(tgt_spd-speed));
double feedforward = tgt_spd/10; // Feedforward throttle term
double throttle_value = feedforward + pid_sp.TotalError();
if (throttle_value > 0.5f) {
  throttle_value = 0.5f;
} else if (steer_value < -0.2f) {
  // Inhibit excessive braking
  throttle_value = -0.2f;
}
```
