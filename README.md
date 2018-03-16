# CarND-Controls-PID
Self-Driving Car Engineer Nanodegree Program

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
  * Run either `./install-mac.sh` or `./install-ubuntu.sh`.
  * If you install from source, checkout to commit `e94b6e1`, i.e.
    ```
    git clone https://github.com/uWebSockets/uWebSockets 
    cd uWebSockets
    git checkout e94b6e1
    ```
    Some function signatures have changed in v0.14.x. See [this PR](https://github.com/udacity/CarND-MPC-Project/pull/3) for more details.
* Simulator. You can download these from the [project intro page](https://github.com/udacity/self-driving-car-sim/releases) in the classroom.

There's an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3)

## Basic Build Instructions

1. Clone this repo.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d)

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
of CarND. If you are enrolled, see [the project page](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/f1820894-8322-4bb3-81aa-b26b3c6dcbaf/lessons/e8235395-22dd-4b87-88e0-d108c5e5bbf4/concepts/6a4d8d42-6a04-4aa6-b284-1697c0fd6562)
for instructions and the project rubric.

## How to write a README
A well written README file can enhance your project and portfolio.  Develop your abilities to create professional README files by completing [this free course](https://www.udacity.com/course/writing-readmes--ud777).

## Writeup

1.The effect each of the P, I, D components had in this implementation.


The algorithm update the steering value based on cte received from simiulator, uses a strategy that combining the P, I and D components.

The strategy is :

steer_value_ = -K_P*cte -K_D*(cte-prev_cte_) -K_I*sum_cte_;

In my opinion, K_P should be most effect the steering value, it effects the steering directly. I suggest if manually tune the parameters, K_P could be set between 2 and 3 initially.


K_I is used to solve the systematic bias, according the integration of the previous ctes. I simply sum the previoud cte as the input of I component.

But since sum_cte_ could be very large if the car bias from the road center for some time, so the K_I must be chosen very small, I suggest 0.05 if you tune it manually.

For the K_d, it could let the car not oscillate too much, the value I suggest set as 0.5 firstly when tunning manually.

2. How the final hyperparameters were chosen.

Actually I use twiddle in this project to adjust the parameters. But the twiddle process taught in course can't be used directly in this project.
Becuase based on current project code structure, I can't run a whole loop to see the best error, adjust parameters, then run a whole loop again....

So I choose to seperate the twiddle process into 3 steps (p vector represent the P, D and I):

step1: p[i] += pd[i]

step2: caculator new error, then update best error or p[i] -= 2*pd[i]

step3: caculator new error again, then update best error or pd[i] *= 0.9


The error are caculated by the function TotalError(), using previous ctes in 5 cycles.


Each step I call UpdateError(), in the function it will decide the twiddle step and call twiddle function.

The final parameters are K_P =2.81, K_D = 0.389, K_I = 0.015. 


I regulate the throttle to 0.1, because the higher the speed is , the more oscillation the car performs. I didn't find a way to find a parameter combination to perform well in high speed, even using twiddle. This is what to refine in the future.
