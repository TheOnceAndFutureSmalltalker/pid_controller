# PID Controller

C++ implementation of a PID controller for controlling steering angle of a simulated car.  The car simulation program provides speed, steering angle, and CTE - cross track error - as outputs.  CTE is the distance the car is from the center line.  The simulation accepts speed and steering angle as inputs.  This communication with the program controlling the car is via a raw socket where the program acts as the server and accepts requests for commands from the simulation.  It is the job of the controlling C++ program to accept the inputs and formulate a correct steering angle so that the car stays as close to the center line as possible.  

<br /><br />
<p align="center">
<img src="https://github.com/TheOnceAndFutureSmalltalker/kidnapped_vehicle_particle_filter/blob/master/images/kidnapped_car_simulator.JPG" width="802px" /><br /><b>Runaway Vehicle with Landmarks Simulator</b></p>
<br />
