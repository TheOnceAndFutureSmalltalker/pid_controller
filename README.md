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

<i>Tuning</i> a PID controller is the search for the proper values of Kp, Ki, and Kd that minimize the error in process variable from the set point.  Programming a PID controller equation in C++ is easy.  Finding the proper gain parameter values is hard!


## Some Observations

After a few hours of trying to set good parameters to control the car and failing I came to some realizations.

1) A PID controller must be tuned to its particular system (car steering, temperature control, etc.) because each of these systems has very different dynamics!  I cannot tune my controller to anything else other than the simulation program.  More to the point, I cannot write my own simulation program and tune my parameters to that - it won't be the same.

2) A given car on a given track under given conditions traveling at 5 mph and same car same, on the same track, under same conditions traveling at 50 mph are 2 very different systems!  Primarily because the faster car is going to overcompensate much more quickly and in general, will need, smaller, more sensitive PID parameters.  The slower car will have more time/observations to make adjustments and can generally have more aggressive PID parameters.  This means, PID parameters are a function of speed!  This is called gain scheduling in the PID literature.  This is a common problem in control systems because a system just starting up (a few miles per hour in our case), and that same system at half load (30 mph let’s say), and then again at full load (70 mph or so) exhibit very different dynamics and requires different PID parameters.

3) To adequately evaluate controller settings, I need to cover both straightaways and curves.  The first 10-15% of track is mostly a straight away so I can't just rely on this in my evaluation.

4) In order to generate reliable statistical measures, I need enough iterations to get a confidence level of current parameters.  This I determined to be around 1000 iterations. 

5) I needed a way to start measuring error only once the car got to a certain speed - the speed of iterest for the test.  

6) The simulation tests are not repeatable.  The simulation adds randomness to its operation, so 2 runs under identical conditions yield 2 different results.  This makes it difficult to compare runs with different parameters for parameter tuning.  I can't be sure if the difference in results is due to the change in parameters or just randomness from simulation.  Therefore, for a given set of parameters, I had make multiple runs to get a small sample for each parameter set.  And instead of comparing runs, I compared to samples of runs.  This made it more time consuming.

7) Also because of the noisy outputs of simulation explained above, I could not tune the parameters more than two significant digits.

8) I learned to ignore the first 10 iterations of the simulation at start up as transient noise.  The car is spinning its wheels here and will record erroneous speeds.  That is, the reading may be up to 8 mph, but the car is not going anywhere, just staying in place spinning its wheels.

9) I did not find a way to launch the simulation from command line and could only run it via the user interface.  This made testing more time consuming.



## Approach

I decided to create a schedule of gain parameters.  This is a different set of parameters optimized for a given speed range.  I arbitrarily chose 10 mph speed ranges.  That is, one set of parameter values for 0-10 mph, another set of paramter values for 10-20 mph, etc.  

I added the ability to set gain parameters at the command line to speed things up.  I created a way to start collecting statistics once a given threshold speed was achieved and then exit after a fixed number of iterations.  I also added a printout of all values with each iteration.  

From here, I just startded incrementing/decrementing values of Kd, Ki, and Kp trying to minimized the mean squared error.  Again, this process was repeated for each speed range.

I also tried a few other techniques.  One was trying to figure out car angle relative to centerline from previous values, but I was unable to do this.  I also tried scaling Kd so that it had more effect the smaller CTE was and less effect for larger CTE values.  This did not prove effective however.

## Solution

I ended up with a schedule of parameters as follows:

| lower speed | upper speed | Kp | Ki | Kd |
|-------------|-------------|----|----|---|
| -1000 | 10 | 0.6, 0.002, 0.25 |
| 10 | 20 | 0.4 | 0.002 | 0.2 |
| 20 | 30 | 0.2 | 0.01, 0.15 |
| 30 | 40 | 0.12 | 0.01 | 0.10 |

I succesfully ran the track up to 40 mpg with fair success but reset the submitted code back to 30 mph setting which actually will run up to 33 mph or so.

## Final Thoughts

Controlling steering from just CTE and speed is a very difficult problem.  I even glanced over some research papers that try to address this problem.  One helpful change would be to have more frequent observations.  But still, as speed increases, the faster observation rate becomes inadequate as well.  We just can't observing faster and faster to keep up with increasing speed.  The real underlying problem here is that we do not know the car's angle of orientation with the center line.  This is not the same as steering angle.  If we knew the car's orientation angle with center line, we could come up with a fourth term, similar to the differential term, one that would help mitigate over correction.  With such information, I am confident I could make a very smooth and low error drive at just about any speed.

Finally, I realize for the purposes of the project that we need a set point, but I would like to challenge a bit the goal of keepingthe car on the center line. In the scenario of the simulation, driving fast around a track, staying on the center line is not optimal.  For example, heading into a left turn, you would want to position the car on the right hand side of the lane.  I don't feel that the center of the lane is optimal for driving fast around the track.


