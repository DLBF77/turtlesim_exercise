#TurtleSim Exercise

TurtleSim exercise contains a simple server written in python to control a turtle in ROS's turtlesim environment. 

## Dependencies

 - ROS installed
 - python dependencies

## Setup
  in a terminal: 
    - `roscore`
  in another terminal:
    - rosrun turtlesim turtlesim_node
  in a third terminal
    - `cd turtle_ws`
    - `catkin_make`
    - `source devel/setup.bash`
    - `rosrun turtlesim_exercise turtle_server.py`
  in a browser, navigate to `localhost:8080/turn`, and see the turtle turn!

