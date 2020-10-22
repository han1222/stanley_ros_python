#!/usr/bin/env python
# I will add feedforwardterm later
# I will high speed damping term too
import rospy
import math
import matplotlib.pyplot as plt
from sympy import symbols #Derivative
from std_msgs.msg import Float32MultiArray
from ackermann_msgs.msg import AckermannDriveStamped
from std_msgs.msg import Float32 
from nav_msgs.msg import Odometry


wheel_base=2.645
steer_angle=[]
cte_list=[]
cte_term_list=[]
angular_velocity_list=[]
heading_error_term_list=[]
feedforward_term_list=[]
steering_ratio = 22.462
steering_vel_constraint = 500
max_rad=0.55 #I added margin maixmum is 0.6526438369 [rad]
min_rad=-0.55
soft_term=2
global velocity
velocity=0

# Please change gain here for your system
p_gain=0.2
h_gain=0.3
f_gain=0.3
dt=0.025
global prev_delta
prev_delta=0
rospy.init_node('stanley')

def error(msg):
    global velocity
    global prev_delta
    x = symbols('x')
    fx = msg.data[0] * x ** 3 + msg.data[1] * x ** 2 +  msg.data[2] * x ** 1 + msg.data[3]
    # fprime = Derivative(fx, x).doit()
    fprime=3*msg.data[0]*x**2+2*msg.data[1]*x+msg.data[2]
    # f_2_prime=Derivative(fprime, x).doit()
    f_2_prime=6*msg.data[0]*x+2*msg.data[1]
    n = fprime.subs({x:2.5})
    n_2=f_2_prime.subs({x: 2.5})
    radius=((pow(n,2)+1)*(math.sqrt(pow(n,2)+1))) / n_2
    feedforward_term=f_gain*math.atan(wheel_base/radius)
    feedforward_term_list.append(feedforward_term * steering_ratio * 180 / math.pi)
    cte=fx.subs({x: 7}) 

    if cte >3 :
        cte=3
    elif cte<-3:
        cte=-3
    cte_list.append(cte)
    
    crosstrack_error_term=p_gain*math.atan((p_gain*cte)/(velocity+soft_term))
    
    cte_term_list.append(crosstrack_error_term * steering_ratio * 180 / math.pi)
    heading_error_term=h_gain*math.atan(n)
    heading_error_term_list.append(heading_error_term * steering_ratio * 180 / math.pi)
    delta= crosstrack_error_term+ heading_error_term+feedforward_term
    
    if max_rad < delta:
        delta=max_rad
    elif delta< min_rad: 
        delta=min_rad

    if ((((delta-prev_delta)* steering_ratio * 180) / math.pi)/dt > steering_vel_constraint) :
        delta=prev_delta + steering_vel_constraint * dt * math.pi / 180 / steering_ratio
    elif  ((((delta-prev_delta)* steering_ratio * 180) / math.pi)/dt < -steering_vel_constraint):
        delta=prev_delta - steering_vel_constraint * dt * math.pi / 180 / steering_ratio
        
    #low pass filter
    low_pass_gain = 0.0 # The closer to 0, the more filtering is performed and the speed is slower. The closer to 1, the less filtering is performed and the faster the speed.
    #delta = (1-low_pass_gain) * delta + low_pass_gain * prev_delta
    delta=(1-low_pass_gain)*prev_delta +delta*low_pass_gain
    steer_angle.append(delta * steering_ratio * 180 / math.pi)    
   
    print("------------------------------------")
    print("angular_velocity:",((((delta-prev_delta)* steering_ratio * 180) / math.pi)/dt))
    print("Steering_angle: ",delta * steering_ratio * 180 / math.pi)
    print("Cte: ", cte, "Cross Track Error Term: ",crosstrack_error_term  * steering_ratio * 180 / math.pi)
    print("Heading Error Term: ",heading_error_term *  steering_ratio * 180 / math.pi)
    print("feedforwardterm : ",feedforward_term*  steering_ratio * 180 / math.pi)
    angular_velocity_list.append(((((delta-prev_delta)* steering_ratio * 180) / math.pi)/dt))
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
  # pub=rospy.Publisher('delta_wheel',Float32,queue_size=10)
    pubAckermann = rospy.Publisher("/Ackermann/command/auto", AckermannDriveStamped, queue_size=10)
    rospy.Subscriber('coefficients',Float32MultiArray,error)
    rospy.Subscriber("Odometry/system",Odometry,get_odom_cb)
    rospy.spin()
    plt.plot(steer_angle, label="Steering_angle")
    plt.plot(cte_list, label="cte")
    plt.plot(cte_term_list, label="cte_term_list")
    plt.plot(heading_error_term_list, label="heading_error_term_list")
    plt.plot(feedforward_term_list, label="feedforward_term_list")
    # # plt.plot(angular_velocity_list, label="angular_velocity_list")
    plt.ylim(-100,100)
    plt.legend(loc=2)
    plt.show()
