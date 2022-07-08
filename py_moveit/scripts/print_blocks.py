#!/usr/bin/env python

import rospy
import gazebo_msgs.msg
import geometry_msgs.msg
import time
import pdb
import numpy as np
from std_msgs.msg import String
import sys, time

class block_pos:

    def __init__(self):
    
        self.pub = rospy.Publisher('box_positions', String, queue_size=10)
        self.sub = rospy.Subscriber("/gazebo/model_states", gazebo_msgs.msg.ModelStates, self.callback,queue_size=1)
        print("sub to blocks")
        self.points = final_pos
    
    def callback(self,data):
        print('in callback')
        my_msg = String
        box_positions = []
        num_of_objs = len(data.name)
        counter = 0
        

        for object in data.name:
            ind = data.name.index(object)
            
            counter+=1


            if 'box' in object:
                #print([data.pose[ind].position.x,data.pose[ind].position.y,data.pose[ind].position.z])
                pos = [float(data.pose[ind].position.x),float(data.pose[ind].position.y),float(data.pose[ind].position.z)]
                #print(pos)
                box_positions.append(pos)
            


        my_msg = str(np.around(box_positions,3))
        self.pub.publish(my_msg)
        print(np.around(box_positions,3))
        rospy.sleep(5) #NEED TO LOWER
        global final_pos
        final_pos = box_positions
        box_positions = []
                 

def main(args):
    rospy.init_node('gazebo_block')
    print('hello')
    blocks = block_pos()
    rospy.spin()
    
    

if __name__ == '__main__':
    main(sys.argv)


