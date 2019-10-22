#!/usr/bin/env python
import rospy


def load_ur5e_2f85_params(robot_side):
    mounting_params = rospy.get_param('/mounting_params/' + robot_side)

    x = mounting_params['base_x']
    y = mounting_params['base_y']
    z = mounting_params['base_z']
    R = mounting_params['base_R']
    P = mounting_params['base_P']
    Y = mounting_params['base_Y']

    return x, y, z, R, P, Y
