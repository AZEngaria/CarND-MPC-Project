# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

---

## Reflections

### Intro

This repository contains my solution to the Udacity SDCND MPC Project. The goal of this project is to navigate a track in a Udacity-provided simulator, which communicates telemetry and track waypoint data via websocket, by sending steering and acceleration commands back to the simulator. The solution must be robust to 100ms latency, as one may encounter in real-world application.

This solution, as the Nanodegree lessons suggest, makes use of the IPOPT and CPPAD libraries to calculate an optimal trajectory and its associated actuation commands in order to minimize error with a third-degree polynomial fit to the given waypoints. The optimization considers only a short duration's worth of waypoints, and produces a trajectory for that duration based upon a model of the vehicle's kinematics and a cost function based mostly on the vehicle's cross-track error (roughly the distance from the track waypoints) and orientation angle error, with other cost factors included to improve performance.

### Implementation Points

#### The Model

The kinematic model includes the vehicle's x and y coordinates, orientation angle (psi), and velocity, as well as the cross-track error and psi error (epsi). Actuator outputs are acceleration and delta (steering angle). The model combines the state and actuations from the previous timestep to calculate the state for the current timestep based on the equations below:

![equations](./eqns.png)

#### Timestep Length and Elapsed Duration (N & dt)

The values chosen for N and dt are 10 and 0.1, respectively. From the quiz I found out that if N is too small, the MPC will not be able to predict far enough into the future and will not predict an accurate current state and controls to prevent system overshoot or instabilities. However, a large N predicts unnecessary data for the future. Since MPC attempts to approximate a continuous reference trajectory by means of discrete paths between actuations, larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory which is called "discretization error". Adjusting either N or dt (even by small amounts) often produced erratic behavior. Other values tried include 20 / 0.05, 8 / 0.125, 6 / 0.15, and many others. 

#### Polynomial Fitting and MPC Preprocessing

The waypoints are preprocessed by transforming them to the vehicle's perspective. This simplifies the process to fit a polynomial to the waypoints because the vehicle's x and y coordinates are now at the origin (0, 0) and the orientation angle is also zero. 

``
x_transformed = x_old * cos(psi) + y_old * sin(psi);``

``y_transformed = y_old * cos(psi) - x_old * sin(psi);``

where is trans_x and trans_y are translated points i.e. (x0 - xp) & (y0 - yp) denoting (x0,y0) as position of observation(waypoint) and (xp,yp) as position of the car both in world/map coordinates.

Once the path trajectory points have been converted into vehicle centric coordinates, a line of best fit is calculated in the form of a quadratic equation. This equation is feed into the MPC for time stepped path prediction.

#### Model Predictive Control with Latency - 100ms

In order to account for the system latency, the actuators values, are taken one-time step in advance or at time step t1 from the current state. This means that as the MPC predicts the system from the current state, we act as if the state was in the past when the calculation was first performed. This prevents the model from lagging the system as we already one step ahead.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1(mac, linux), 3.81(Windows)
  * Linux: make is installed by default on most Linux distros
  * Mac: [install Xcode command line tools to get make](https://developer.apple.com/xcode/features/)
  * Windows: [Click here for installation instructions](http://gnuwin32.sourceforge.net/packages/make.htm)
* gcc/g++ >= 5.4
  * Linux: gcc / g++ is installed by default on most Linux distros
  * Mac: same deal as make - [install Xcode command line tools]((https://developer.apple.com/xcode/features/)
  * Windows: recommend using [MinGW](http://www.mingw.org/)
* [uWebSockets](https://github.com/uWebSockets/uWebSockets)
  * Run either `install-mac.sh` or `install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.

* **Ipopt and CppAD:** Please refer to [this document](https://github.com/udacity/CarND-MPC-Project/blob/master/install_Ipopt_CppAD.md) for installation instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.
