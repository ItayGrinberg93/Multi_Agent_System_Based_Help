#!/usr/bin/env python

from turtle import position
import rospy
import gazebo_msgs.msg
import geometry_msgs.msg
import time
import pdb
import numpy as np
from std_msgs.msg import String
global box_positions
box_positions = {
        "box_1" : []
        ,"box_2" : []
        , "box_3" : []
    }

#rospy.init_node('gazebo_block')
#publish_frequency = rospy.get_param("publish_frequency", 1)
#pub = rospy.Publisher('box_positions', String, queue_size=10)
def callback(data):
    for object in data.name:
        ind = data.name.index(object)
        if 'box' in object:
            #print([data.pose[ind].position.x,data.pose[ind].position.y,data.pose[ind].position.z])
            pos = [float(data.pose[ind].position.x),float(data.pose[ind].position.y),float(data.pose[ind].position.z)]
            #print(pos)
            box_positions[object] = pos
        


    #my_msg = str(box_positions)
    #pub.publish(my_msg)
    #print(type(box_positions[0][0]))
    #positions = box_positions
    
            
    return 


def get_block_pos():
    #temp = rospy.init_node('gazebo_block') ### if running file on its own -> add node
    publish_frequency = rospy.get_param("publish_frequency", 1)
    #pub = rospy.Publisher('box_positions', String, queue_size=10)
    sub = rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, callback)
    rospy.sleep(1)
    while box_positions["box_1"]==[] :
        pass
    #print(box_positions)
    return box_positions




#rospy.spin()
