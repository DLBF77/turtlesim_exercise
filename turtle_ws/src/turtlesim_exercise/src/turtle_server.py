#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from bottle import route, run

@route('/turn')
def turn():
    vel_msg = Twist()
    vel_msg.linear.x = 0
    vel_msg.linear.y = 0
    vel_msg.linear.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0
    vel_msg.angular.x = 0
    vel_msg.angular.y = 0
    vel_msg.angular.z = 0.393

    velocity_publisher.publish(vel_msg)
    d = rospy.Duration(1, 0)
    rospy.sleep(d)
    vel_msg.angular.z = 0
    velocity_publisher.publish(vel_msg)
    return "Done Turning!"

if __name__ == '__main__':
    rospy.init_node('turtle_server', anonymous=True)
    velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    try:
        run(host='localhost', port=8080, debug=True)
    except rospy.ROSInterruptException: pass
