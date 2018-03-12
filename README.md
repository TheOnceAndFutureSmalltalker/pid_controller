# PID Controller

This project is a C++ implementation of a PID controller for controlling steering angle of a simulated car.  The car simulation program provides speed, steering angle, and CTE - cross track error - as inputs to the controlling program.  CTE is the distance the car is from the center line.  The simulation accepts speed and steering angle back from the controlling program.  This communication with the program controlling the car is via a raw socket where the program acts as the server and accepts requests for commands from the simulation.  It is the job of the controlling C++ program to accept the inputs and formulate a correct steering angle so that the car stays as close to the center line as possible.  


<br /><br />
<p align="center">
<img src="https://github.com/TheOnceAndFutureSmalltalker/pid_controller/blob/master/img/simulator.JPG" width="802px" /><br /><b>Car on Track Simulator</b></p>
<br />


## PID Controller

The program implements a PID controller to keep the CTE value as close to zero as possible.  This is to keep the car centered on the track.  The CTE variable is called the process variable.  The desired zero CTE value is known as the set point.  It is the job of the program to calculate a steering angle from current (and previous) CTE values which will return the car back to the center.  The steering angle is called the control variable.  The function for calculating the steering angle is a standard PID controller equation and consists of three terms:  a proportional term, an integral term, and a differential term.  The standard PID equation is shown below.

<br /><br />
<p align="center">
<img src="https://wikimedia.org/api/rest_v1/media/math/render/svg/69072d4013ea8f14ab59a8283ef216fb958870b2" width="802px" /><br /><b>PID Controller Equation (from Wiikipedia)</b></p>
<br />

The proportional term seeks to move the process variable (CTE in our case) back to the set point (zero CTE).  It does this in direct proportion to how far off the process variable is form its set point.  If the car is way off center, then we need to steer hard back in the other direction.  The parameter Kp is the proportionality parameter.  

The integral term seeks to adjust for long term bias that is in one direction or the other and therefore is a proportion, Ki, of the current sum of errors. For example, a car is running on a track in a counter clockwise direction, then it will normally show a bias to be on the right side of the centerline - swinging wide on the curves.  

The differential term seeks to dampen over-correction of the proportional term.  The differential term itslef is proportional, by the parameter Kd, to the current <i>change</i> in process variable. A large change in process variable (CTE) means we might be over-correcting and need to pull back a bit.  

<i>Tuning</i> a PID controller is the search for the proper values of Kp, Ki, and Kd that minimize the error in process variable from the set point.  Programming a PID controller is easy.  Finding the proper gain parameter values is hard!




