# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

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
* [uWebSockets](https://github.com/uWebSockets/uWebSockets) == 0.13, but the master branch will probably work just fine
  * Follow the instructions in the [uWebSockets README](https://github.com/uWebSockets/uWebSockets/blob/master/README.md) to get setup for your platform. You can download the zip of the appropriate version from the [releases page](https://github.com/uWebSockets/uWebSockets/releases). Here's a link to the [v0.13 zip](https://github.com/uWebSockets/uWebSockets/archive/v0.13.0.zip).
  * If you run OSX and have homebrew installed you can just run the ./install-mac.sh script to install this
* Simulator. You can download these from the [project intro page](https://github.com/udacity/CarND-PID-Control-Project/releases) in the classroom.

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

## Editor Settings

We've purposefully kept editor configuration files out of this repo in order to
keep it as simple and environment agnostic as possible. However, we recommend
using the following settings:

* indent using spaces
* set tab width to 2 spaces (keeps the matrices in source code aligned)

## Code Style

Please (do your best to) stick to [Google's C++ style guide](https://google.github.io/styleguide/cppguide.html).

## Project Instructions and Rubric

### Describe the effect each of the P, I, D components had in your implementation

* The P component produces an output that is propotional to the current error, CTE. It is calculated by multiplying the error by a constant Kp, the proportional gain. For a car, the steering angle should be propotional to CTE, the car makes a sharper turn when CTE is larger. However, the car should avoid sharp turn while in high speed. This is intuitive since when car is running higher speed, it could reduce CTE with smaller sterring angle. We calculate the P component by multipling the proportional gain Kp to CTE divided by current car speed (Kp*(CTE/speed)). However, using P component along, the CTE can be reduced to zero quickly with a large Kp value, but it also results in oscillation in the track.
* The I component is the sum of the instantaneous error over time and gives the accumulated offset that should have been corrected previously. This term should accelerates the movement of the car to intended trajectory and eliminates the residual steady-state error. We calculate this term by summing all CTE and multiply it with a constant Ki. Add this component to controller enables the car to move to trajectory point quicker with the same P term.
* The D component is proportianl to the rate of change of CTE. This component can compensate for a changing CTE, thus inhibit more rapid changes of CTE due to propotinal term. To enable the car to follow trajectory, especially the sharp turn curve, we need to increase the P component. But increase the P component alone will result in unstable behavior, oscillating around the trajectory. By adding D component, we are able to stablize the car to keep on track.  We calculate D component by multiple the change of current CTE from previous CTE to a constant Kd.

### Describe how the final hyperparameters were chosen.

I manually tuned the PID coefficients to achieve the intended result. First I set Ki and Kd to zero, and start the Kd from 0.1 and increase it until it can make the first turn. This happen when Kd goes to about 0.7. But the CTE is not converge to zero. I add Ki term to make sure the CTE converge to zero before first 50 steps. Then I increase the Kd to try to make the car to keep the second turn after the bridge. The car starts oscillating when Kd is larger than 1.0. To combat oscillation, I start adding D component, start by setting Kd to 1.0 and increase until the car stablizes. From here, I iterated the Kp and Kd, increasing Kp to try to have the car stay on track, especial on turns, while increasing Kd to make sure it did not oscillate, until the car was able to complete the track.

The final hyperparameters I choose are Kp = 4.5, Ki = 0.001, Kd = 12.5

### Result and Reflection

Here is a [link](https://youtu.be/rzDsFfvd9T0) to my final video output:

<p align="center">
    <a href="https://www.youtube.com/watch?v=rzDsFfvd9T0">
        <img src="https://img.youtube.com/vi/rzDsFfvd9T0/0.jpg" alt="video output">
    </a>
</p>

Possible improvements for this implementation include:

* The vehicle still has large CTEs even it can keep on track. This may indicate those hyperparameters are not optimal. Techniques such as Twiddle may be used to find the optimal hyparameters. We'll need to be able to control the simulator in order to implement such technique.
* The car is a little shaky. The steering angle seems changing too often. This may also an indication of suboptimal hyparameters.
* I used a constant throttle in the implementation. Adding a speed PID may enable the car to run faster and smoother. I tried but failed to find a simple implementation of speed PID. This is one improvement I need to play around.
* Without trajectory information, it could be hard to optimize the controller. Maybe we can estimate the trajectory from CTE and car heading?

