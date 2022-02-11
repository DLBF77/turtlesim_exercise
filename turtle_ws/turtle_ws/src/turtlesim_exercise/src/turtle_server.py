#!/usr/bin/env python
import rospy
# TurtleSim imports
# currently only using TWIST fn from geometry_msgs
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from std_srvs.srv import Empty as EmptySrvCall
# WSGI interface
from bottle import route, run
# math for degrees to radians conversion
from math import radians, degrees, atan, pi, sin, cos, sqrt, fmod

# error handling
import pdb

####################
#   Constants
####################
# there are no actual const's in python..... *sigh*
widthmax = 11
widthmin = 0
heightmax = 11
heightmin = 0
tolerance = 0.001
max_angular_speed = 1
max_linear_speed = 1

# ##################
# ## Global Vars
# ##################
# turtle actual location, updated constantly by the pose subscriber
my_loc = Pose()
# debug flag
debug = True 

############################################################
## Helper Functions
############################################################

# if use_current is TRUE: starting at my_loc, determine the distance required to travel to the target point (tx,ty) 
# if use_current is FALSE: starting at ox, oy, determine the distance required to travel to tx,ty
def determine_linear_distance(tx,ty, ox = 0, oy = 0, use_current = True):
    # declare access to global my_loc Pose object
    global my_loc
    t_loc = Pose()
    if (use_current):
        t_loc = my_loc
    else:
        t_loc.x = ox
        t_loc.y = oy 

    # return the distance to travel using good old pythagoras
    retval = sqrt(((tx-t_loc.x)**2 + (ty - t_loc.y)**2))
    # if (debug):
    #     print(f"determine_linear_distance\ntx:{tx} ty{ty}\nox:{ox} oy{oy}\ntlocx:{t_loc.x} tlocy:{t_loc.y}\nretval:{retval}")
    return retval 

# starting at my_loc, determine the angle required to get to our target of tx,ty
# return value is in RADIANS if process was successful, will return -1 if something went wrong
# note, that while -1 is technically a valid radian movement, all movement here is calculated as "positive" (0 to 2pi)
def determine_target_angle(tx,ty):
    # declare access to global my_loc Pose object
    global my_loc    

    # determine which quadrant are we working in Q1(+,+), Q2(-,+), Q3(-,-), Q4(+,-) and perform the math required to
    # calculate the target angle (note, python trig functions return values in rads already)
    # if we are moving in a cardinal direction simply return the angle required to get there
    if (tx == my_loc.x):
        if (ty == my_loc.y):
            # no movement required
            return my_loc.theta
        elif (ty > my_loc.y):
            return radians(90.0)
        else:
            return radians(270.0)
    elif (ty == my_loc.y):
        if (tx == my_loc.x):
            #no movement required
            return my_loc.theta
        elif(tx > my_loc.x):
            return radians(0.0)
        else:
            return radians(180.0)
    else:
        if (tx > my_loc.x) and (ty > my_loc.y):            
            return atan(abs((ty - my_loc.y)/(tx - my_loc.x)))
        elif (tx < my_loc.x) and (ty > my_loc.y):
            return atan(abs((tx - my_loc.x)/(ty - my_loc.y))) + radians(90)
        elif (tx < my_loc.x) and (ty < my_loc.y):
            return atan(abs((ty - my_loc.y)/(tx - my_loc.x))) + radians(180)
        elif (tx > my_loc.x) and (ty < my_loc.y):
            return atan(abs((tx - my_loc.x)/(ty - my_loc.y))) + radians(270)
        else:
            # realllllly shouldn't be able to get here, throw a printf and return false
            print("determine_angle screwed up....")
            return -1

    #blanket return even though it's impossible
    print("determine exited wrong....")
    return -1


# return the shortest distance between two angles
def radial_delta(r1, r2):
    if r1 > r2:
        rawdiff = r1 - r2
    else:
        rawdiff = r2 - r1

    moddiff = fmod(rawdiff, 2*pi)

    if moddiff > pi:
        retval = (2*pi) - moddiff
    else:
        retval = moddiff

    if (debug):
        print(f"radial delta: diff between {r1} and {r2} is {retval}")

    return retval




# used to rotate the turtle from where we are currently pointing to where we want to go as defined by points x,y OR
# if ANGLE /= -1, then simply turn to that angle.
# angle is in RAD
# return True if rotation performed successfully and False otherwise (debug traces will be thrown to console window)
# input values have already been validated
def rotate(x, y, angle = -1):
    global my_loc
    vel_msg = Twist()
    # ensure the object is zero'ed
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    # safety checker incase we can't hit tolerance so we don't get stuck in the loop
    runs = 0

    if (angle == -1):
        target_angle = determine_target_angle(x,y)
    else:
        target_angle = angle

    # determine target angle returns -1 on error
    if (target_angle == -1):
        return False

    # start at full speed
    angular_speed = max_angular_speed

    if (debug):
        print(f"starting rotation from {(my_loc.theta):.4f} to {target_angle:.4f} rads")

    while(abs(radial_delta(my_loc.theta, target_angle)) >= tolerance):
        runs += 1
        if (runs>4):
            print("Cannot acheive desired tolerance, halting motion")
            break

        target_travel = abs(radial_delta(my_loc.theta, target_angle))            
        # moving at ANGULAR_SPEED, how long do we have to move to go from MY_LOC.THETA to TARGET_ANGLE
        # angular_speed is rad/sec, distance is in rads, divide rad / rad/sec
        duration = abs(radial_delta(my_loc.theta, target_angle) / angular_speed)

        # are we moving counterclockwise (+ve speed) or clockwise to get there (-ve)?

        if (target_angle > my_loc.theta):
            if ((target_angle - my_loc.theta) > pi):
                angular_speed *= -1.0
        elif ((my_loc.theta - target_angle) < pi):
            angular_speed *= -1.0

        vel_msg.angular.z = angular_speed

        if (debug):
            print(f"issuing rotate command for {duration} seconds with speed {angular_speed}")

        # need to pump velocity messages otherwise turtle stops early
        t0 = rospy.Time.now().to_sec()
        angle_travelled = 0
        r = rospy.Rate(50) #50hz / 20ms

        while(angle_travelled < target_travel):
            velocity_publisher.publish(vel_msg)
            r.sleep()
            t1 = rospy.Time.now().to_sec()
            angle_travelled = abs(angular_speed*(t1-t0))

        if (debug):
            print(f"elapsed time {t1-t0}, target_angle {target_angle}, angle_travelled {angle_travelled}")

        # halt motion
        vel_msg.angular.z = 0
        velocity_publisher.publish(vel_msg)
        # small sleep to ensure we get a new pose message
        rospy.sleep(0.1)
        # assuming we will loop, and that we need to keep approaching the target, adjust angular speed
        angular_speed = abs(angular_speed/2.0)

        if (debug):
            print(f"in moving to {degrees(target_angle):.4f}, ended up at {degrees(my_loc.theta):.4f}")
            print(f"reducing angular speed to {angular_speed:.4f}")

    print(f"completed rotation, final angle is {my_loc.theta:.4f}, target_angle was {target_angle:.4f}")

    return True 

# used to move forwards/backwards a desired amount 'h'. Will do microadjustments until the target location is within tolerance
def movex(h):
    global my_loc
    vel_msg = Twist()
    runs = 0

    #reverse multiplier for linear speed
    if (h >= 0):
        reverse = 1.0
    else:
        reverse = -1.0

    #snapshot of original location to ensure we went the distance we needed to
    start_loc = Pose()
    start_loc.x = my_loc.x
    start_loc.y = my_loc.y 
    start_loc.theta = my_loc.theta

    #zero out the object for safety
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0

    linear_speed = max_linear_speed

    print(f"Starting linear motion from {my_loc.x},{my_loc.y}")

    while (abs(abs(h) - determine_linear_distance(my_loc.x, my_loc.y, start_loc.x, start_loc.y, False)) >= tolerance):
        # if we've done this 3 times already, just break out and report
        runs += 1 
        if (runs>4):
            print("Cannot acheive desired tolerance, halting motion")
            break

        # moving at LINEAR_SPEED, how long do we have to move to go 'h' meters
        # linear_speed is m/sec, distance is in m, divide m / m/sec

        duration = abs(abs(h) - determine_linear_distance(my_loc.x, my_loc.y, start_loc.x, start_loc.y, False)) / linear_speed

        # do we move forwards or backwards, this is always positive on the first loop, but if we overshoot, then this will
        # back us up if need be
        if ((abs(h) - determine_linear_distance(my_loc.x, my_loc.y, start_loc.x, start_loc.y, False)) < 0):
            #we've moved too far
            linear_speed *= -1.0

        vel_msg.linear.x = linear_speed * reverse

        target_distance = abs(abs(h) - determine_linear_distance(my_loc.x, my_loc.y, start_loc.x, start_loc.y, False))
        if (debug):
            print(f"issuing movex command for {duration} seconds at speed {linear_speed} to go {target_distance}m")

        # need to pump velocity messages otherwise turtle stops early
        t0 = rospy.Time.now().to_sec()
        distance_travelled = 0
        r = rospy.Rate(50) #50hz / 20ms

        while(distance_travelled < target_distance):
            velocity_publisher.publish(vel_msg)
            r.sleep()
            t1 = rospy.Time.now().to_sec()
            distance_travelled = abs(linear_speed*(t1-t0))

        # stop the motion
        vel_msg.linear.x = 0
        velocity_publisher.publish(vel_msg)
        #tiny sleep to make sure our pose message is updated
        rospy.sleep(0.1)
        # assuming we will loop to make adjustments, reduce the linear speed. Also resets the direction to "forwards"
        linear_speed = abs(linear_speed/2.0)

        if (debug):
            print(f"currently at {my_loc.x},{my_loc.y}")
            print(f"trying to move {abs(h)}m, moved: {determine_linear_distance(my_loc.x, my_loc.y, start_loc.x, start_loc.y, False)}m")
            print(f"reducing linear speed to {linear_speed:.4f}")

    if (debug):
        print (f"moveto completed. Desired distance was {abs(h)}m, moved: {determine_linear_distance(my_loc.x, my_loc.y, start_loc.x, start_loc.y, False)}m")

    return (f"moveto completed. Desired distance was {abs(h)}m, moved: {determine_linear_distance(my_loc.x, my_loc.y, start_loc.x, start_loc.y, False)}m")


def movelinear(h):
    # move the robot forward or backward, for backwards movement, h will be negative
    global my_loc   

    # ensure that we dont fall out of range due to the linear move, do sohcahtoa!
    if (degrees(my_loc.theta) >=0) and (degrees(my_loc.theta) < 90):
        #Q1
        dx = h*(cos(my_loc.theta))
        dy = h*(sin(my_loc.theta))
    elif (degrees(my_loc.theta) >= 90) and (degrees(my_loc.theta) < 180):
        #Q2
        dx = -1.0 * h*(sin(my_loc.theta - radians(90)))
        dy = h*(cos(my_loc.theta - radians(90)))
    elif (degrees(my_loc.theta) >= 180) and (degrees(my_loc.theta) < 270):
        #Q3
        dx = -1.0 * h*(cos(my_loc.theta - radians(180)))
        dy = -1.0 * h*(sin(my_loc.theta - radians(180)))
    elif (degrees(my_loc.theta) >= 270) and (degrees(my_loc.theta < 360)):
        #Q4
        dx = h*(sin(my_loc.theta - radians(270)))
        dy = -1.0 * h*(cos(my_loc.theta - radians(270)))
    else:
        # should not happen, return error message
        return (f"Something went wrong calculating dx/dy in movelinear\nmy angle is:\n(deg):{degrees(my_loc.theta)} (rad):{my_loc.theta}")

    # check to see if we are still in the box after this move
    if ((my_loc.x + dx) > widthmax) or ((my_loc.x + dx) < widthmin) or ((my_loc.y + dy) > heightmax) or ((my_loc.y + dy) < heightmin):
        #out of range
        return("Illegal movement, will take me out of my sandbox")

    # we can make this move. movex returns a string indicating success or not
    return movex(h)

#################################################################
## Web Server Functions
#################################################################

# move forward X meters
@route('/moveforward/<x>')
def moveforward(x):
    # verify input variable
    try:
        lx = float(x)
    except:
        return "Invalid input type, provide number"

    if (lx<0):
        return "Invalid input value, provide number greater than 0"

    return movelinear(lx)

# move backwards X meters
@route('/movebackward/<x>')
def movebackward(x):
    # verify input variable
    try:
        lx = float(x)
    except:
        return "Invalid input type, provide number"

    if (lx<0):
        return "Invalid input value, provide number greater than 0"

    return movelinear(-lx)

# move to a specified grid point
@route('/moveto/<x>/<y>')
def moveto(x, y):
    global my_loc

    # verify input variables
    try:
        lx = float(x)
        ly = float(y)
    except:
        return "Invalid input type, provide numbers"

    if (lx < widthmin) or (lx > widthmax) or (ly < heightmin) or (ly > heightmax):
        return "Invalid target location, must be between (0,0) and (11,11)"

    # everything looks good, call internal rotate then move.
    if (rotate(lx,ly)):
        return movex(determine_linear_distance(lx,ly))
    else:
        return "Failed to move, see console for decriptive traces"

# turn to a specific positive angle
@route('/turnto/<x>')
def turnto(x):
    # validate input variable
    try:
        langle = float(x)
    except:
        return "Invalid input type, provide number"

    if (langle<0):
        return "Please enter a positive angle, turtle knows how to turn efficiently"

    print(f"Calling rotate with {x}, {langle}, modded {langle %360}, rads {radians(langle%360)}")
    if (rotate(0,0,radians(langle % 360))):
        return "Turtle has turned!"
    else:
        return "Error turning turtle, please see logs"



#target reset, screen clears and turtle goes to desired spot
@route('/clear/<x>/<y>/<z>')
def clear(x,y,z):

    # make sure input variables are safe to use
    try:
        lx = float(x)
        ly = float(y)
        lz = float(z)
    except:
        return "Invalid input parameters, please use numbers"

    # verify input parameters are within the range of the box
    if (lx > widthmax) or (lx < widthmin) or (ly > heightmax) or (ly < heightmin):
        return "Input parameters not within range (x: 0 to 11 / y: 0 to 11)"

    # used to kill the turtle, clear the screen, and spawn a new turtle1 at x,y with rotation z (degrees)
    reset_sim()
    # reset sim brings turtle back at 0deg, I want it at 90deg
    kill_turtle('turtle1')
    spawn_turtle(lx, ly, radians(lz), 'turtle1')

    # do a tiny sleep to ensure that pose subscriber is getting fresh data
    rospy.sleep(0.05)

    return "Turtle Reset!"


# full reset, turtle goes home and trail is cleared
@route('/clear')
def clear():
    clear(5.5,5.5,90)

@route('/toggledebug')
def toggledebug():
    global debug 
    debug = not debug
    return (f"Debug flag is now: {debug}")

@route('/dump')
def dump():
    global my_loc
    global theoretical_loc

    print("*****************\nDumping debug data\n*****************")
    print(f"Actual location:\nx:{my_loc.x:.4f}\ny:{my_loc.y:.4f}\ntheta:{degrees(my_loc.theta):.4f}")
    print("*****************")



##################################################
# subscriber handler functions
##################################################
def update_pose(msg):
    global my_loc
    global theoretical_loc

    my_loc.x = round(msg.x, 4)
    my_loc.y = round(msg.y, 4)
    # NO NEGATIVE ANGLES
    if (msg.theta < 0):
        my_loc.theta = round( (2*pi) - abs(msg.theta),4)
    else:
        my_loc.theta = round(msg.theta,4)

    # also, I start at 90deg, rotate left 90deg leaving me at 180deg, and ROS tells me that I'm now at 540deg???????
    # normalize all angles to be between 0 and 2pi / 0 and 360
    my_loc.theta = my_loc.theta % (2*pi)
        
    # print(f"received pose msg, new coords:\nx: {my_loc.x}\ny: {my_loc.y}\ntheta: {my_loc.theta}")


if __name__ == '__main__':
    # part of turtlesim base package, init the server and declare a mailbox
    rospy.init_node('turtle_server', anonymous=True)

    rospy.wait_for_service('reset')
    reset_sim = rospy.ServiceProxy('reset', EmptySrvCall)
    reset_sim()
    # kill the default turtle and spawn one at "home" (0,0,90deg)
    kill_turtle = rospy.ServiceProxy('kill', Kill)
    try:
        kill_turtle('turtle1')
    except:
        # no turtle to kill, just ignore...
        pass
    # spawn our actual starting turtle, get the service
    spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
    spawn_turtle(5.5, 5.5 ,radians(90.0), 'turtle1')
    # get our service for clear


    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, update_pose)

    try:
        run(host='localhost', port=8080, debug=True)

        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        print(f"main() failure: {e.__name__}")
        pdb.post_mortem()


