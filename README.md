# PID Control of Steering and Throttle
Self-Driving Car Engineer Nanodegree Programme

---

## Objective

The goal of this project is to control the steering angle and throttle inputs of the car based on the cross track error (CTE = _d_<sub>ref</sub> - _d_) and speed error relative to the commanded speed, respectively.

After allowing for some transient phenomena such as overshoot and some settling time prior to converging, the CTE is expected to tend towards zero when driving in a straight line. Ultimately, the car must be able to complete multiple laps in succession without departing the side markers along the track, and these criteria must ideally be satisfied indefinitely.

## Control algorithm performance

The performance of the implemented algorithm can be observed in the video below. It can be seen that the car successfully completes two full laps round the track. In theory, starting with the second lap, the car is expected to settle into a repeating sequence of controlled manoeuvres; consequently, lap numbers 3, 4, 5 and so forth are expected to exhibit performance which is identical to lap number 2, and hence, the car should be able to drive round the loop indefinitely.

[![Car running laps autonomously](https://img.youtube.com/vi/73V1IrIFXyY/0.jpg)](https://www.youtube.com/watch?v=73V1IrIFXyY)

The PID gains for the steering angle controller were chosen to be _K<sub>P</sub>_ = 0.25, _K<sub>I</sub>_ = 0.005 and _K<sub>D</sub>_ = 5.5, while those for the throttle control were _K<sub>P</sub>_ = 0.6, _K<sub>I</sub>_ = 0 and _K<sub>D</sub>_ = 0.1, respectively. It should be noted that these values are specific to the author's hardware setup and software environment, and may not work as expected on another machine -- this is why the video evidence has been provided above.

## Implementation details

### Steering angle control

The control of the car's steering angle is based on the cross track error defined as CTE = _d_<sub>ref</sub> - _d_. The steering control signal comprises three terms which are added together; namely, a term proportional to the CTE itself, a term proportional to the discrete time integral of the CTE, and finally, a term proportional to the discretised derivative of the CTE (see the [PID::UpdateError](https://github.com/shahid-n/pid-control/blob/master/src/PID.cpp#L26) method).

### Speed control

The car's speed is modulated by first generating a speed reference signal, and then regulating it by means of a PID controller whose input is the speed error, defined as the difference between the reference signal and the actual speed of the car.

#### Speed reference generation

When travelling in a straight line, the car can drive quite fast, perhaps up to 100 mph, if not more. However, when negotiating a curve, and taking into account the transient response of the steering controller which could lead to some oscillations, it would be prudent to reduce the speed for non-zero steering angles. Consequently, the speed reference signal is computed by subtracting from the `MAX_SPEED` parameter (set to 100 mph), a term which is proportional to the absolute product of steering angle and CTE -- the exact mathematical details can be found in [line 90 of main.cpp](https://github.com/shahid-n/pid-control/blob/master/src/main.cpp#L90).

Consequently, the reference speed is reduced any time either the magnitude of steering angle or the magnitude of CTE increases.

#### Throttle and braking control

The combined control signal which commands the car either to accelerate or apply the brake is computed in analogous fashion to the steering control, by first finding the difference between the reference speed and the car's actual speed, and feeding it into another instance of the PID controller (see [lines 91 and 92 of main.cpp](https://github.com/shahid-n/pid-control/blob/master/src/main.cpp#L91)).

## Concluding remarks and future work

### Effect of each PID gain parameter

- _K<sub>P</sub>_ : the proportional gain has a direct impact on the amount of steering applied in response to any non-zero CTE, and can thus potentially impact transient behaviours such as rise time as well as settling time, not to mention overall stability of the controller
- _K<sub>D</sub>_ : the derivative gain can help reduce settling time and can also provide damping
- _K<sub>I</sub>_ : the integral gain helps to eliminate any steady state error, but it comes at the cost of degraded transient performance, particularly in the form of increased overshoot, a phenomenon known as integrator wind-up

It is worth noting that the interaction between _K<sub>I</sub>_ and _K<sub>D</sub>_ also affects the frequency of oscillations -- on the one hand, increasing _K<sub>D</sub>_ can help to mitigate any increase in overshoot caused by a larger _K<sub>I</sub>_, but it also has the side effect of increasing the frequency of oscillations, so it is prudent not to crank up this pair of parameters too high.

Last but not least, in real world applications, noisy signals are a fact of life. Whilst the derivative gain _K<sub>D</sub>_ appears to be very useful in a simulation environment, it actually tends to amplify any noise present in the measured signals because it is essentially a high-pass filter, as can be seen upon performing a frequency domain analysis of its behaviour. Consequently, in many applications, derivative control should be used sparingly and always with a small gain value, or perhaps not at all -- in other words, this gain should be set to zero for best results in the presence of significant measurement noise.

### A comment on controller performance

The PID gains of both the steering and throttle controllers were tuned manually for reasons to be explained shortly. As such, it was considerably easier to obtain good performance of the throttle controller in response to the commanded speed. In the case of the steering angle controller, on the other hand, it is quite evident from the video above that its performance is sub-optimal, especially in light of the prominent transient phenomena such as exaggerated overshoots and frequent oscillations, not to mention a somewhat long settling time.

Whilst an auto-tuning Twiddle algorithm was implemented for both controllers, whenever it was invoked, and every time it subsequently finished an iteration and sent a reset request, the simulator on my computer froze up. Consequently, the tuner was never able to run to completion -- if this were possible, the final set of controller parameters would no doubt have resulted in improved performance of the car's steering in particular.

### Additional improvements

The controller performance could be improved even further by analysing the bicycle model used by the simulator to drive the car, and developing a steering controller by means of the standard model-based design techniques of control theory. Moreover, if the bicycle model contains nonlinearities, then the controller could be designed in accordance with nonlinear and robust control techniques in order to properly address the observed performance issues.

An added benefit would be that the control scheme could be made robust with respect to both external disturbances (such as the impact of buffeting wind or an uneven road surface in real world scenarios) and measurement noise in the speed and steering angle signals.

Last but not least, in light of the need to develop a fully integrated control solution in the final class project, the natural next step would be to interface a computer vision algorithm with a dynamic controller and develop a system capable of driving on unfamiliar roads or paths. The approach could focus exclusively on analytical techniques based on robotics, image processing and control theory, or it could be a hybrid of traditional robotics and machine learning based approaches. Irrespective of which method is ultimately chosen, the system has to be optimised to run on an embedded platform with certain computational and resource limitations, and be able to execute predictably and reliably in real time.

## Basic build instructions

1. Clone this repository.
2. Make a build directory: `mkdir build && cd build`
3. Compile: `cmake .. && make`
4. Run it: `./pid`. 

Tips for setting up your environment can be found [here](https://classroom.udacity.com/nanodegrees/nd013/parts/40f38239-66b6-46ec-ae68-03afd8a601c8/modules/0949fca6-b379-42af-a919-ee50aa304e6a/lessons/f758c44c-5e40-4e01-93b5-1a82aa4e044f/concepts/23d376c7-0195-4276-bdf0-e02f1f3c665d).

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

Fellow students have put together a guide to Windows set-up for the project [here](https://s3-us-west-1.amazonaws.com/udacity-selfdrivingcar/files/Kidnapped_Vehicle_Windows_Setup.pdf) if the environment you have set up for the Sensor Fusion projects does not work for this project. There's also an experimental patch for windows in this [PR](https://github.com/udacity/CarND-PID-Control-Project/pull/3).
