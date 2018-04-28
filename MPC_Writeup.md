## Writeup Report

---

**MPC Controller Project**

(Note - Hi , The visualization for the simulator on my machine is available at - https://youtu.be/Sej0m-YtBc0)

The broad goals / steps of this project are the following:

* Set N and dt.
* Fit the polynomial to the waypoints.
* Calculate initial cross track error and orientation error values.
* Define the components of the cost function (state, actuators, etc).
* Define the model constraints.
* Calculate steering angle and throttle using MPC.


## [Rubric](https://review.udacity.com/#!/rubrics/896/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### MPC Controller

#### 1. Student describes their model in detail. This includes the state, actuators and update equations.

The MPC controller converts the problem of predicting the correct trajectory for the robot into an optimization problem. A 'Cost function' is defined in an MPC Controller and the aim is to come up with a trajectory that more or less follows the reference trajectory i.e. has the minimum cost with respect to the reference trajectory. For finding the cost we also take into account the various actuator inputs(acceleration, steering etc.) along with the state of the car. The trajectory that has the minimum cost is the optimal trajectory. Once this 'optimal' trajectory is determined , only the first actuator commands are executed and the remaining trajectory is not used as the there is always some error in predicting the trajectory that might be small at this point in time but might be large in the long run. So to counter this effect we calculate the new trajectory at each time step using the current state of the car. Thus, MPC constantly re-evaluates the inputs to find the optimal trajectory.

The equations used to implement this continuous re-evaluation use the vehicle's state. The vehicles state is defined as below.
state = [x, y, psi, v, cte, epsi]
where, 
x = x coordinate of the position of the car
y = y coordinate of the position of the car
psi = orientation of the car
v = velocity of the car
cte = cross track error
epsi = error in orientation of the car

State Update Equations:
```
 x_[t] = x[t-1] + v[t-1] * cos(psi[t-1]) * dt
 y_[t] = y[t-1] + v[t-1] * sin(psi[t-1]) * dt
 psi_[t] = psi[t-1] + v[t-1] / Lf * delta[t-1] * dt
 v_[t] = v[t-1] + a[t-1] * dt
 cte[t] = f(x[t-1]) - y[t-1] + v[t-1] * sin(epsi[t-1]) * dt
 epsi[t] = psi[t] - psides[t-1] + v[t-1] * delta[t-1] / Lf * dt
``` 

An additional point I'd like to mention is that the equations for orientation and orientation error had to be modified with a negative sign. Because as discussed in the classroom, in the simulator , a positive value implies a right turn and a negative value implies a left turn.

I have mostly followed the instructions given in the project Q&A video linked in the project module, whenever I got stuck about how to implement a particular equation and most of the code is adapted and taken from the MPC - 'Mind the Line' quiz and the project Q&A session.

#### 2. Student discusses the reasoning behind the chosen N (timestep length) and dt (elapsed duration between timesteps) values. Additionally the student details the previous values tried.

The value chosen for N and dt ultimately used are 10 and 0.1. These were the values I started with after going through the project Q&A video , however I tried two other combinations-
1. N-25, dt-0.05 - I picked this value from 'Mind the Line' quiz in the classroom but it made the processing slow (as is expected) (also I don't have very good hardware), so ultimately I have not used it.
2. N-20, dt-0.1 

I think , and also as explained in the Q&A , 10 and 0.1 just work because the time step is not so large that it slows down the MPC neither is it so small that it doesn't give a good prediction. I guess it is just a very optimal combination for the problem. Technically speaking the MPC considers a 1 second duration for correcting trajectory.

#### 3. A polynomial is fitted to waypoints. If the student preprocesses waypoints, the vehicle state, and/or actuators prior to the MPC procedure it is described.

I have transformed the reference trajectory and the car coordinates to the car coordinate system by orienting the x axis along the reference trajectory of the car so as to make the calculations easier and for the equations mentioned in the classroom hold valid.  

#### 4. The student implements Model Predictive Control that handles a 100 millisecond latency. Student provides details on how they deal with latency.

As described by my mentor I have used the following way to account for the 100 ms latency in the calculation if the steering and the throttle value and its actual application to the car in the simulator.
```
	    double latency =0.125;
            px = px + v * cos(psi) * latency;
            py = py + v * sin(psi) * latency;
            psi = psi - v* steer_value/Lf * latency;
            v += v + throttle_value * latency;
```

### Discussion

#### 1. Briefly discuss what could you do to make the implementation more robust?

I tried a number of times to make the vehicle simulation at a reference speed of 100mph. But my implementation always failed at the point after the bridge. I would continue to tune the weights for CTE and orientation angle error (which are currently at 3333) to see if I can make it work at 100mph.
