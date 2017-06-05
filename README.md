# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program

## Kinematic Model

Kinematic model is leveraged here for implementing a model predictive control. 

A vehicle's state is established using [x(coordinate),y (corodinate),ψ (orientation),v (velocity)]

Actuator inputs allow us to control the vehicle state, δ for steering angle and a for acceleration (throttle/brake combined)

The model looks like this:
State: [x,y,ψ,v]
Actuators: [δ,a]

The equations define the state, actuators and how the state changes over time based on the previous state and current actuator inputs. These are used to implement the Kinematic model.

x[​t+1] ​​= x[​t] ​​+ v[t] ​​∗ cos(ψ[​t]​​) ∗ dt

y[​t+1]​ ​= y[t]​ ​+ v​[t]​​ ∗ sin(ψ​[t]​​) ∗ dt

ψ​[t+1] ​​= ψ[​t]​​ + ​​​​​v[​t] / L ​​​​∗ δ ∗ dt

v​[t+1] ​​= v[​t] ​​+ a[​t] ​​∗ dt

## Errors
To minimize the error between the reference trajectory and vehicles actual path, first we predict the vehicle’s actual path and then adjust the actuators to minimize the difference between the prediction and the reference trajectory.

We can capture how the errors we are interested in change over time by deriving our kinematic model around these errors as our new state vector.

The new state is [x,y,ψ,v,cte,eψ]

cross track error(cte):
cte[t+1] = cte[t]+v[t]+sin(eψ[t])*dt

orientation error(eψ):
eψ[t+1] = eψ[t] + v[t]/L[f]*δ*dt

## Cost Function
Cost should be a function of how far errors(cte and eψ) are from 0. A few considerations used here are velocity error, the euclidean distance between the current position of the vehicle and the destination, change rate of the control input.

## Polynomial Fitting and MPC Preprocessing

Transform map coordinates into the vehicle coordinates and then fit the transformed waypoints with 3rd order polynomial. And use the fitted polynomial with x=0, y=0 to find initial cte and eψ.

## Timestep length and Frequency
N is the number of timesteps in the horizon. dt is how much time elapses between actuations. 25 (N) timesteps with duration of each step 50 (dt) milliseconds have been leveraged in the current implementation.
Larger values of dt result in less frequent actuations, which makes it harder to accurately approximate a continuous reference trajectory, as the values were increased to 1 second the vehicle went off track. Higher acceleration with larger dt values caused the vehicle to go off track on sharp turns.
N determines the number of variables the optimized by MPC. This is also the major driver of computational cost, a few values tried were 10 and 15.

## Latency handling
In a real car, an actuation command won't execute instantly - there will be a delay as the command propagates through the system. A realistic delay might be on the order of 100 milliseconds.

This is a problem called "latency", and it's a difficult challenge for some controllers to overcome. But a Model Predictive Controller can adapt quite well because we can model this latency in the system.

100 milliseconds delay is incorporated in the current model using sleep, to accomodate for latency. This means the state is predicted after 100ms.

---

## Dependencies

* cmake >= 3.5
 * All OSes: [click here for installation instructions](https://cmake.org/install/)
* make >= 4.1
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
* Fortran Compiler
  * Mac: `brew install gcc` (might not be required)
  * Linux: `sudo apt-get install gfortran`. Additionall you have also have to install gcc and g++, `sudo apt-get install gcc g++`. Look in [this Dockerfile](https://github.com/udacity/CarND-MPC-Quizzes/blob/master/Dockerfile) for more info.
* [Ipopt](https://projects.coin-or.org/Ipopt)
  * Mac: `brew install ipopt`
  * Linux
    * You will need a version of Ipopt 3.12.1 or higher. The version available through `apt-get` is 3.11.x. If you can get that version to work great but if not there's a script `install_ipopt.sh` that will install Ipopt. You just need to download the source from the Ipopt [releases page](https://www.coin-or.org/download/source/Ipopt/) or the [Github releases](https://github.com/coin-or/Ipopt/releases) page.
    * Then call `install_ipopt.sh` with the source directory as the first argument, ex: `bash install_ipopt.sh Ipopt-3.12.1`. 
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [CppAD](https://www.coin-or.org/CppAD/)
  * Mac: `brew install cppad`
  * Linux `sudo apt-get install cppad` or equivalent.
  * Windows: TODO. If you can use the Linux subsystem and follow the Linux instructions.
* [Eigen](http://eigen.tuxfamily.org/index.php?title=Main_Page). This is already part of the repo so you shouldn't have to worry about it.
* Simulator. You can download these from the [releases tab](https://github.com/udacity/self-driving-car-sim/releases).
* Not a dependency but read the [DATA.md](./DATA.md) for a description of the data sent back from the simulator.


## Basic Build Instructions


1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./mpc`.

## Tips

1. It's recommended to test the MPC on basic examples to see if your implementation behaves as desired. One possible example
is the vehicle starting offset of a straight line (reference). If the MPC implementation is correct, after some number of timesteps
(not too many) it should find and track the reference line.
2. The `lake_track_waypoints.csv` file has the waypoints of the lake track. You could use this to fit polynomials and points and see of how well your model tracks curve. NOTE: This file might be not completely in sync with the simulator so your solution should NOT depend on it.
3. For visualization this C++ [matplotlib wrapper](https://github.com/lava/matplotlib-cpp) could be helpful.

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

Note: regardless of the changes you make, your project must be buildable using
cmake and make!

More information is only accessible by people who are already enrolled in Term 2
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/b1ff3be0-c904-438e-aad3-2b5379f0e0c3/concepts/1a2255a0-e23c-44cf-8d41-39b8a3c8264a)
for instructions and the project rubric.

## Hints!

* You don't have to follow this directory structure, but if you do, your work
  will span all of the .cpp files here. Keep an eye out for TODOs.

## Call for IDE Profiles Pull Requests

Help your fellow students!

We decided to create Makefiles with cmake to keep this project as platform
agnostic as possible. Similarly, we omitted IDE profiles in order to we ensure
that students don't feel pressured to use one IDE or another.

However! I'd love to help people get up and running with their IDEs of choice.
If you've created a profile for an IDE that you think other students would
appreciate, we'd love to have you add the requisite profile files and
instructions to ide_profiles/. For example if you wanted to add a VS Code
profile, you'd add:

* /ide_profiles/vscode/.vscode
* /ide_profiles/vscode/README.md

The README should explain what the profile does, how to take advantage of it,
and how to install it.

Frankly, I've never been involved in a project with multiple IDE profiles
before. I believe the best way to handle this would be to keep them out of the
repo root to avoid clutter. My expectation is that most profiles will include
instructions to copy files to a new location to get picked up by the IDE, but
that's just a guess.

One last note here: regardless of the IDE used, every submitted project must
still be compilable with cmake and make./
