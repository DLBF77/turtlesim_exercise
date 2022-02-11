Conceptual use for the turtle: 

The turtle moves as a rover would, it always moves "forward" or "backward", and if asked to move to a new location in the sandbox it will first rotate to that angle, and move in a straight line. The user is free to ask it to move to a specific location, move it forwards / backwards a certain distance, or ask it to rotate on the spot without any forwards or backwards movement.



Design Discussion

Primary issue discovered while using the turtle was the inaccuracy of the hardware/OS running the code when it comes to issuing delays. From experimentation, Win10 is not able to provide precise timing whatsoever at a 10us rate let alone a 1ms rate. The time slice for windows is approx 15ms and even then it will fluctuate at least +/- 15%. Since all movement commands are at a timed rate, if we issue a command to move at Xm/s and then delay for 1s, the turtle will never actually go Xm (same for rotation in rad/s). Furthermore, the faster the turtle moves/rotates, the greater this error margin becomes.

Ideally, this type of system would be loaded onto an embedded SOC device with an interrupt level timer capable of issuing interrupts in a Xusec granularity, or at the very least, be run on a dedicated RTOS system with at a bare minimum 1ms timing slices.

To reduce the errors accumulated from each move, this system has been designed to:

1) perform the move at a rate of 1m/s and rotate at a rate of 180degrees/s
2) once the initial "fast" move is completed, the pose of the turtle will be checked to see how much of an over/undershoot there was and will readjust the movement at a half speed rate. This action is repeated up to 4 times, each time, the movement rate is halved. The target is a tolerance of 1mm (0.001) or 0.001 radians


Setup Guide

1. The ROS environment must be installed on target system as well as Python v3+
2. Create a new folder in the \\ws directory called "turtle_ws", copy the directory structure found in turtle_demo.zip to this directory.
3. Launch an ROS terminal window and run the command <roscore>
4. Launch a second ROS terminal window and run the command <rosrun turtlesim turtlesim_node>
5. Launch a third ROS terminal window, one by one, enter the following commands for linux:
	cd turtle_ws
	catkin_make
	source devel/setup.bash
	rosrun turtlesim_exercise turtle_server.py
5.1 If using windows, the commands are slightly different, do the following:
	cd turtle_ws
	catkin_make
	.\devel\setup.bat
	rosrun turtlesim_exercise .\src\turtlesim_exercise\src\turtle_server.py


Users Guide:

The sandbox the turtle has access to is setup as a grid of 11m wide by 11m high. The coordinates available on the grid are (0,0) to (11,11)

Once the program has launched, the turtle will spawn at "home", location (5.5,5.5) and will be facing "up" along the y axis, this is defined as 90 degrees.

all calls are made to a local browser with the following syntax:

localhost:8080/<command name>

replace <command name> (including the < and > characters) by any commands below to make the turtle do your bidding

Following functionality is available for the user:

<command name>: moveto/x/y
- move the turtle to the target grid location, recall that the grid is an 11x11 square.
- x and y values must be integer or floating point format (a floating point value has numbers after the decimal, i.e., 4.123)
- the turtle will first turn to the target direction, then proceed upon its way!
- the turtle will not move outside the box

<command name>: moveforward/x
- moves the turtle forward <x> meters from its current location
- the turtle will not move forward if the destination would hit or pass the "wall" of the box

<command name>: movebackward/x
- moves the turtle backwards <x> meters from its current location
- the turtle will not move backward if the destination would hit or pass the "wall" of the box

<command name>: turnto/x
- turns the turtle to face X degrees

<command name>: clear
- the turtle will dissapear as well as all traces left by the turtle. A new turtle will appear in the center of the sandbox
- facing 90 degrees (straight "up")

<command name>: clear/x/y/z
- the turtle will disappear as well as all traces left by the turtle. A new turtle will appear at coordinates x,y. The turtle will face z degrees where 0 degrees is east / right.

<command name>: toggledebug
- used to toggle debug traces from the console window ON/OFF.
- by default, debug traces are OFF

<command name>: dump
- dump the current location (taken from the Pose message) to the console window, used for debug purposes

Unit Testing:

toggledebug:
1. Functions as intended

clear commands:
1. Verify functionality
2. Verify that input parameters are protected (can't pass a string instead)
3. Verify that an attempt to respawn outside of the box is rejected

turnto command:
1. Verify basic functionality, turtle will turn to the desired direction turning the shortest distance available.
2. Verify input parameters are protected (can't pass invalid data type)
3. Verify special cases: turning from 1deg to 359deg, turtle turns a 2 degree arc and not a 358 degree arc.

moveforward / movebackwards commands:
1. Verify basic functionality
2. Verify input parameters are protected (can't pass invalid data types)
3. Verify that the turtle will reject movement commands that will end up outside the box

moveto command:
1. Verify basic functionality
2. Verify input parameters are protected
3. Verify that turtle ignores destinations which are outside of the box
4. Verify that turtle first orients itself to the desired direction then moves in a straight line until the destination is reached.

general testing:
1. Using a combined mix of the 3 move commands, with additional turnto commands mixed in, cause no issues. The turtle does what it is supposed to
2. Clearing and resetting the turtle causes no loss in functionality for subsequent commmands.



