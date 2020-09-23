#!/usr/bin/env python
import roslib
import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Path
from geometry_msgs.msg import Point,PoseStamped
import rviz_tools_py as rviz_tools
import matplotlib.pyplot as plt
rospy.init_node('polyfitting_coeffcients', anonymous=True)
number=20
def pathCallback (msg):
    path = Path() 
    x=[]
    y=[]
    new_y=[]                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
    for i in range (0,len(msg.poses)):
        x.append(msg.poses[i].pose.position.x)
        y.append(msg.poses[i].pose.position.y)
    coefs = np.polyfit(x[0:number], y[0:number], 3)
    ffit = np.polyval(coefs, x[0:number])
    path.header.frame_id = "base_link"
    path.header.stamp=rospy.Time.now()
    for element in ffit:
        new_y.append(element)
    for x_i,y_i in zip(x,new_y):
        pose = PoseStamped()
        pose.header.seq = path.header.seq + 1
        pose.header.frame_id = "base_link"
        pose.header.stamp = path.header.stamp
        pose.pose.position.x = float(x_i)
        pose.pose.position.y = float(y_i)
        path.poses.append(pose)
    pub2.publish(path)
    coefficients=Float32MultiArray(data=coefs)
    pub.publish(coefficients)
   
if __name__== '__main__':  
    rospy.Subscriber("/Path/LocalWaypoint/OnBody/", Path, pathCallback)
    pub=rospy.Publisher('coefficients',Float32MultiArray,queue_size=10)
    pub2 = rospy.Publisher('/path/polyfit', Path, queue_size=1)
    rospy.spin()
    
    
   
