#!/usr/bin/env python
import rospy
from rospkg import RosPack
import os

def load_ur5e_2f85_params(robot_name, sim):
    mounting_params = rospy.get_param('/mounting_params/' + robot_name)

    x = mounting_params['base_x']
    y = mounting_params['base_y']
    z = mounting_params['base_z']
    R = mounting_params['base_R']
    P = mounting_params['base_P']
    Y = mounting_params['base_Y']

    if(sim):
    	# Load the default kinematics_config file
    	rp = RosPack()
    	kinematics_file = os.path.join(rp.get_path('ur_e_description'), 'config/ur5e_default.yaml')
    else:
    	kinematics_file = mounting_params['kinematics_file']

    return x, y, z, R, P, Y, kinematics_file
