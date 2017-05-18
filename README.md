# PID Controls for Steering Angle and Vehicle Speed Project
This project is originally forked from https://github.com/udacity/CarND-PID-Control-Project.  This repository includes starter code, that is used herein.

## Important Note
I am running the simulator on MacOS, with Screen Resolution = 1024x768 and Graphics Quality = "Fastest".

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
} else if (throttle_value < -0.2f) {
  // Inhibit excessive braking
  throttle_value = -0.2f;
}
```

### Controller Modifications
In order to alleviate heavy oscillations very near the center of the lane, I implemented a few modifications to the base PID scheme.

First, I added a deadzone for the P term error.  This way, the control signal drops out within some band around `cte = 0.0`.
```C++
// Proportional
p_error = cte;
  
// Smooth deadzone near centerline
if (fabs(p_error) < 0.1) {
  p_error = 0.0;
} else if (p_error < 0.0) {
  p_error += 0.1;
} else {
  p_error -= 0.1;
}
```

Next, I added an anti-windup for the I term error.  This prevents the integrator from continuing to get bigger, when the I term has reached a practical limit.

```C++
// Integral
i_error += cte;
  
// Anti-windup for integral too large
double i_term_max = 0.1;
if (N > N_min) {
  if ((Ki*i_error) > i_term_max) {
    i_error = i_term_max / Ki;
  } else if ((Ki*i_error) < -i_term_max) {
    i_error = -i_term_max / Ki;
  }
} else {
  i_error = 0.0;
}
```

Third, I added a crude low-pass filter to the calculated derivative of `cte` in order to smooth the D term.

```C++
// Low pass filter for derivative smoothing
// y(t) = k*y(t-1) + (1-k)*x(t)
d_error = 0.2*d_error + 0.8*(cte - d_error_old);
```

Lastly, for the P term and the D term I included gain scheduling.  This has the effect of increasing the `Kp` and `Kd` gains as `cte` and the derivative of `cte`, respectively, get bigger.  Similar to the deadzone above, the goal is the not create a large steering angle when the car is very near the target.

```C++
double Kp_sched; // Use gain scheduling
if (fabs(p_error) > 1.5) {
  Kp_sched = Kp;
} else {
  Kp_sched = Kp*(p_error+0.5)/1.0;
}
double p_term = -Kp_sched*p_error;

double i_term = -Ki*i_error;
 
double Kd_sched; // Use gain scheduling
if (fabs(d_error) > 0.12) {
  Kd_sched = Kd;
} else {
  Kd_sched = Kd*(d_error+0.2)/0.12;
}
double d_term = -Kd*d_error;
  
// Signs are flipped due to steering angle definition
double control = p_term + i_term + d_term;
```

### Parameter Tuning
For the parameter tuning I used a mixture of approaches.  For the PID controller responsible for the vehicle speed, I manually tuned the P and I gains very crudely.  The target speed isn't a critical parameter, it's merely important that the vehicle slow down when turning.

For the PID controller responsible for the steering angle, I started by manually tuning the parameters such that the car could traverse the whole track without going off the road.

Then, I used the method `PID.Twiddle()` to search for the best possible parameters.  For the cost function that the Twiddle algorithm used to compare runs, I did the following:
1. If the car stops, make cost exceedingly high and fixed value
2. If the car goes off the road, make cost accumulate very quickly
3. Otherwise, value both a small error `cte` and a small steering `angle`

In this way, the cost not only emphasizes keeping the car near the center line, but also discourages it from simply swerving madly around the center of the lane.

```C++
if (speed < 2){
  cost = 2000000;
} else if (cost < 2000000) {
  if (fabs(cte) > 2.5) {
    cost += 150;
  } else if (fabs(cte) > 0.1) { // no cost if CTE < 0.1
    cost += pow((10*cte),4)/pow(10,4) + pow((angle),4)/pow(10,4); // scale up by 10 so small CTE has smaller cost
  }
}
```

## Performance
This [video](https://youtu.be/oZKl0-CHfHc) shows my solution in action.
 
## Reflections
My initial approach was to implement the Nelder Mead algorithm for parameter optimization.  This is a technique I've used successfully for other applications.  However, when I implemented this in object-oriented manner, I struggled to root out some segmentation faults.  This was likely due to my implementation and an associated array of structures, but I was unable to solve it.  Nelder-Mead does not lend itself well to functional-style code, so I abandoned this approach and instead implemented Twiddle.

In general, even with much tuning, I was not able to successfully get the vehicle to drive "well".  I think this is mainly due to two things:
1. Controlling to the `CTE` at time `t` is not a valid control approach.  The idea instead should be to either control to a trajectory that will reach `CTE = 0.0` over some finite time horizon, or to control the trajectory of the vehicle.  No driver will immediately try to center a vehicle unless avoiding a collision, otherwise, the driver will simply adjust the angle to allow the car to "fall" into the center of the lane again.
2. PID control may not be well suited to this type of system, or the control loop rate may be too small.  Based on my observing the simulator, there is a significant amount delay between when the vehicle crosses the center line and when the steering happens.  This may be due to where in the vehicle is the reference point (is it at the front of the car or the center of mass), or because of the system dynamics (time delay between turning the steering wheel and tires turning, time delay between turning tires, and vehicle turning, etc)

Ideally I would like to continue working on modifications to my `PID` class to make this work, but I have spent a lot of time and believe that PID is just not suited for this work.  It feels a bit like failure, but maybe that is point of this project, versus the "Model Predictive Control" project, up next.
