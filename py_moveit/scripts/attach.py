#!/usr/bin/env python

import rospy
from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse


def attach(model_1, link_1,model_2,link_2):
    #rospy.init_node('demo_attach_links')
    rospy.loginfo("Creating ServiceProxy to /link_attacher_node/attach")
    attach_srv = rospy.ServiceProxy('/link_attacher_node/attach',
                                    Attach)
    attach_srv.wait_for_service()
    rospy.loginfo("Created ServiceProxy to /link_attacher_node/attach")

    # Link them
    print("Attaching cubes",model_1, model_2)
    req = AttachRequest()
    req.model_name_1 = model_1
    req.link_name_1 = link_1
    req.model_name_2 = model_2
    req.link_name_2 = link_2

    attach_srv.call(req)

    return 
    # From the shell:
    """
rosservice call /link_attacher_node/attach "model_name_1: 'cube1'
link_name_1: 'link'
model_name_2: 'cube2'
link_name_2: 'link'"
    """

    # rospy.loginfo("Attaching cube2 and cube3")
    # req = AttachRequest()
    # req.model_name_1 = "cube2"
    # req.link_name_1 = "link"
    # req.model_name_2 = "cube3"
    # req.link_name_2 = "link"

    # attach_srv.call(req)

    # rospy.loginfo("Attaching cube3 and cube1")
    # req = AttachRequest()
    # req.model_name_1 = "cube3"
    # req.link_name_1 = "link"
    # req.model_name_2 = "cube1"
    # req.link_name_2 = "link"

    # attach_srv.call(req)
