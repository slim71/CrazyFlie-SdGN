# CrazyFlie-SdGN
Repo for the project of the course Navigation and Guidance systems.

All useful documentation is in the [docs folder](docs).

## Recap
We developed some script to use the __Crazyflie__ drone along with a __Wand__ 
in a __Vicon__ indoor motion capture system.

The main files are:
- wand_thread_onpos.py
- wand_obstacle.py

### wand_thread_onpos
The Wand is used as a position-recording reference. The user will first move
the Wand around for some time, then the quadcopter will take off and repeat
the same trajectory. 

### wand_obstacle
This time the Wand works as an obstacle for the drone, tracked by Vicon
thanks to the on-board LEDs. The drone will try to fly around a given setpoint,
but if the obstacle is too close it will adjust its flight to avoid collision.
If the obstacle is removed, the Crazyflie will once again travel towards the
initial setpoint.