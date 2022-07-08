#!/usr/bin/env python

import rospy
import sys
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
import numpy as np
from std_msgs.msg import String
from get_blocks import get_block_pos

positions = np.around(get_block_pos(),3)

print(positions)