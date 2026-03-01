# ORION
Orion is a custom rocket flight system. It includes a ground station and a flight controller that work together to transmit telemetry, monitor the rocket’s status, and perform active control and data logging during flight.

# How to use?
To use Orion, you connect the ground station to a laptop. The UI runs in the Processing IDE, and the main board plus ground station firmware is written and uploaded using PlatformIO. From the interface, you can see the rocket’s current state, switch between modes (for example, test and launch), command servo tests, read raw sensor data, and run calibration routines.

# Why?
I built Orion because ever since I started rocketry in 4th grade, I’ve been inspired by how SpaceX monitors and controls their rockets in real time. I wanted a similar system for my own launches, where I can see how the rocket is performing live, get data during the flight, and not just read everything after recovery.
