#!/usr/bin/env python
# I will add feedforwardterm later
# I will high speed damping term too
import rospy
import math
from sympy import Derivative, symbols
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32 
from nav_msgs.msg import Odometry

wheel_base=2.645
steering_ratio = 22.462
steering_vel_constraint = 120
#max_rad=0.28 #I added margin
#min_rad=-0.28
max_rad=0.55 #I added margin maixmum is 0.6526438369 [rad]
min_rad=-0.55
soft_term=0.2
global velocity
velocity=0
# Please change gain here for your system
p_gain=0.18#0.06#0.15#0.06
h_gain =0.15#0.15
f_gain=0.3
dt=0.025#0.025
global prev_delta
prev_delta=0
rospy.init_node('stanley')

def error(msg):
    global velocity
    global prev_delta
    x = symbols('x')
    fx = msg.data[0] * x ** 3 + msg.data[1] * x ** 2 +  msg.data[2] * x ** 1 + msg.data[3]
    fprime = Derivative(fx, x).doit()
    f_2_prime=Derivative(fprime, x).doit()
    n = fprime.subs({x:7})
    n_2=f_2_prime.subs({x: 7})
    radius=((pow(n,2)+1)*(math.sqrt(pow(n,2)+1))) / n_2
    feedforward_term=math.atan(wheel_base/radius)
    cte=1*fx.subs({x: 7}) # check this today
    print("cte: ", cte)
    crosstrack_error_term=math.atan((p_gain*cte)/(velocity+soft_term))
    heading_error_term=1*math.atan(n)
    delta= crosstrack_error_term+ h_gain * heading_error_term+f_gain*feedforward_term
    if max_rad < delta:
        delta=max_rad
    elif delta< min_rad: 
        delta=min_rad

 
    if ((((delta-prev_delta)* steering_ratio * 180) / math.pi)/dt > steering_vel_constraint) :
        delta=prev_delta + steering_vel_constraint * dt * math.pi / 180 / steering_ratio
    elif  ((((delta-prev_delta)* steering_ratio * 180) / math.pi)/dt < -steering_vel_constraint):
        delta=prev_delta - steering_vel_constraint * dt * math.pi / 180 / steering_ratio

    #print("angular_velocity:",((((delta-prev_delta)* steering_ratio * 180) / math.pi)/dt))
    print("------------------------------------")
    print("Steering_angle: ",delta * steering_ratio * 180 / math.pi)
    print("Cross Track Eror: ", cte, "Position Command: ", crosstrack_error_term  * steering_ratio * 180 / math.pi)
    print("Heading Error Term: ", h_gain*heading_error_term *  steering_ratio * 180 / math.pi)
    #print("angular_velocity:",(((delta-prev_delta) / math.pi)/dt))
    prev_delta=delta
    #pub.publish(delta)

    commandOut = AckermannDriveStamped()
    commandOut.header.stamp = rospy.Time.now()
    commandOut.header.frame_id = 'base_link'
    commandOut.drive.steering_angle = delta * 22.462 * 180 / math.pi
    pubAckermann.publish(commandOut)
    
def get_odom_cb(msg):
    global velocity
    velocity=msg.twist.twist.linear.x
 
if __name__== '__main__':
  #  pub=rospy.Publisher('delta_wheel',Float32,queue_size=10)
    pubAckermann = rospy.Publisher("/Ackermann/command/auto", AckermannDriveStamped, queue_size=10)
    rospy.Subscriber('coefficients',Float32MultiArray,error)
    rospy.Subscriber("Odometry/ekf_estimated",Odometry,get_odom_cb)
    rospy.spin() 
