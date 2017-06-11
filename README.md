# CarND-MPC-Project
Self-Driving Car Engineer Nanodegree Program

## References and Acknowdlegements

1. Udacity Classroom lessons and quizzes :  https://github.com/udacity/CarND-MPC-Quizzes/tree/master/mpc_to_line
2. Udacity discussion forums
3. Slack channels
4. @smasoudn, @sharanyad

## The Model

I chose to implement the MPC using the Kinematic model with the following state, actuators and update equations.

### State 
The state of vehicle is represented using the tuple [x,y, psi,v] where 
x,y is the position of the vehicle
psi is the orientation expressed in degrees measured anticlockwise from the x axis
v is the velocity

### Actuator
Each actuator control value is representated as a tuple [delta, acceleration] where
delta is the steering angle
acceleration is the throttle(positive value) or brake applied(negative value)

### Update Equations

The following equations are used to update the state of the vehicle over time with the actuator control inputs accounted for.

x[​t+1] ​​= x[​t] ​​+ v[t] ​​∗ cos(psi[​t]​​) ∗ dt

y[​t+1]​ ​= y[t]​ ​+ v​[t]​​ ∗ sin(psi[t]​​) ∗ dt

psi[t+1] ​​= psi[​t]​​ + ​​​​​v[​t] / L ​​​​∗ delta ∗ dt

v​[t+1] ​​= v[​t] ​​+ accelration[​t] ​​∗ dt

## Timestep Length and Elapsed Duration (N & dt)

I chose a value of N = 25 (length of the timestep) and dt = 0.05 (elapsed duration). I chose this value as the the baseline from references in the quiz in the last lesson in Udacity.(https://github.com/udacity/CarND-MPC-Quizzes/tree/master/mpc_to_line). I preserved these values and instead worked on fine-tuning the cost/weights of other parameters like cross-track error (10.0), orientation error(50.0), steering angle(10.0) and throttle(3000.0). And eventually I was able to achieve a smooth drive. These parameters were manually tuned. If the cte weight was reduced to 1 or the orientation error was reduced to 25, the car's path was offset from the yellow line. If the weights of steering angle/throttle were reduced, then the car started to go off the track. Hence arrived at this final numbers. The baseline value for these weights to start with were chosen from slack channel discussions and peer solutions.


## Polynomial Fitting and MPC Preprocessing
We can fit the x and y co-ordinates in a third degree polynomial to derive four co-efficients. These co-efficients are fed into the MPC and factored in the cross track error (cte) and orientation error (epsi) values. I convert the state values from the global co-orindates to the vehicle co-orindate space to simplify the computations. I do this by the following equations
a) state(x) = (state(x)-x) * cos(psi) + (state(y)-y) * sin(psi)
b) state(y) = (state(y)-y) * cos(psi) - (state(x)-x) * sin(psi)
c) state(psi) = state(psi) - psi

The following equations from the lesson/quiz were used to calculate errors.

cross track error(cte):
cte[t+1] = f(x[t]) - y[t] + v[t] * sin(epsi[t]) * dt

orientation error(epsi):
epsi[t+1] = psi[t] - psides[t] + v[t] * delta[t] / Lf * dt

In the above equations,

f(x[t]) and psides[t] are computed using the co-efficents from the third degree polynomial.

f(x[t]) = coeffs[0] + coeffs[1] * x0 + coeffs[2] * CppAD::pow(x0,2) + coeffs[3] * CppAD::pow(x0,3);
psides[t] = CppAD::atan(coeffs[1] + 2* coeffs[2] * x0 + 3 * coeffs[3] * CppAD::pow(x0,2));


## Model Predictive Control with Latency
The purpose is to mimic real driving conditions where the car does actuate the commands instantly. A latency of 100 milliseconds(by putting the thread to sleep) is used in the implementation while updating the state. I also account for a delay of 10 milliseconds in the state update equations by multiplying the x,y,psi,v values by 10 milliseconds. This takes care do the latency we introduced to mimic real driving conditions.


The output video can be found at https://youtu.be/eEuHp28s2Sw
