#!/usr/bin/env python
# I will add feedforwardterm later
import rospy
import math
from sympy import Derivative, symbols
from std_msgs.msg import Float32MultiArray
from std_msgs.msg import Float32 
from nav_msgs.msg import Odometry

max_rad=0.523598 #I added margin
min_rad=-0.523598
soft_term=0.01
global velocity
velocity=0
# Please change gain here for your system
gain=0.5

rospy.init_node('stanley')

def error(msg):
    global velocity
    x = symbols('x')
    fx = msg.data[0] * x ** 3 + msg.data[1] * x ** 2 +  msg.data[2] * x ** 1 + msg.data[3]
    fprime = Derivative(fx, x).doit()
    n = fprime.subs({x: 2.6})
    cte=fx.subs({x: 2.6}) # check this today
    crosstrack_error_term=math.atan((gain*cte)/(velocity+soft_term))
    heading_error_term=math.atan(n)
    #print("heading_error term: ",heading_error_term )
    #print("crosstrack_error_term: ",crosstrack_error_term)
    delta= crosstrack_error_term+ heading_error_term
    if max_rad < delta:
        delta=max_rad
    elif delta< min_rad: 
        delta=min_rad

    pub.publish(delta)

def get_odom_cb(msg):
    global velocity
    velocity=msg.twist.twist.linear.x
 
if __name__== '__main__':
    pub=rospy.Publisher('delta_wheel',Float32,queue_size=10)
    rospy.Subscriber('coefficients',Float32MultiArray,error)
    rospy.Subscriber("Odometry/ekf_estimated",Odometry,get_odom_cb)
    rospy.spin() 
