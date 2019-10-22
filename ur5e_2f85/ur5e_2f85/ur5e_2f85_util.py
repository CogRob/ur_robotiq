#!/usr/bin/env python
import rospy


def load_mounting_params(robot_side):
    mounting_params = rospy.get_param('/mounting_params/' + robot_side)
    robot_description = rospy.get_param('/left/robot_description')

    print(robot_description)

    x = mounting_params['base_x']
    y = mounting_params['base_y']
    z = mounting_params['base_z']
    R = mounting_params['base_R']
    P = mounting_params['base_P']
    Y = mounting_params['base_Y']

    return x, y, z, R, P, Y
