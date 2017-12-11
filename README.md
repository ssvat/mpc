# CarND-Model Predicitve Control
Self-Driving Car Engineer Nanodegree Program

## Introduction

The main goal of the project is to implement the Model Predictive Control in C++ to drive the car and finish the track, by using the Udacity Starter Code and Simulator as well.

## Model

The parameters include: position (x,y), heading (ψ) and velocity (v) form the vehicle state vector [x,y,ψ,v]. There are two actuators: Stearing angle (δ) and singular actuator (a). The former ranges between -25 and 25 degrees while the later ranges from -1 to 1. For simplicity the throttle and brake represented as a singular actuator (a), with negative values signifying braking and positive values signifying acceleration. 

There are also two kinds of errors: cross track error (cte) and ψ error (eψ). These error were used to build the cost function for the MPC and updated at the next time step. The parameters of the cost function and other parameters for the Model Predictive Controller are tuned for optimization.

First of all, data about waypoints was transformed into the vehicle space and a 3d order polynomial was fitted to the data. Actual state of the vehicle was "shifted" into the future by 100 ms latency. It helps to reduce negative effects of the latency and increase stability of the controller. The latency was introduced to simulate real delay of a human driver or physical actuators in case of a self driving car. Cross track error and orientation error were calculated, is then they were passed into the MPC routine.

The time horizon (T) was chosen to 2 s after experiments. It was shown that the MPC could drive safely around the track with T = 1 s, but on a slow speed. Higher speed requires more future information to make smart decisions in serial turns. Time step duration (dt) was setted equal to the latancy of the simulation (0.1 s), hense, 20 time steps (N) was used.

The cost function parameters were tuned by try-and-error method. All these parameters are stored in the src/MPC.h file. They were tuned in order to reach maximal speed and agressive race style with use of the whole width of the road and breaking before turns.

Note: To obtain relatively slow, but safe behavior use the following parameters in MPC.h:

## Result

The video of result is being placed at:
https://youtu.be/TtvLLuImBD8

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
