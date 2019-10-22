#!/usr/bin/env python
import sys
import rospy
import xacro
import subprocess
import rospkg
import argparse
import os

from ur5e_2f85_util import load_ur5e_2f85_params


def upload_ur5e_2f85(robot_side, limited=True):
    x, y, z, R, P, Y = load_ur5e_2f85_params(robot_side)

    rospack = rospkg.RosPack()
    ur5e_2f85_description_path = rospack.get_path('ur5e_2f85_description')
    print(ur5e_2f85_description_path)

    if(not limited):
        urdf_path = 'urdf/ur5e_2f85_robot.urdf.xacro'
    else:
        urdf_path = 'urdf/ur5e_2f85_joint_limited_robot.urdf.xacro'

    model_filepath = os.path.join(
        ur5e_2f85_description_path, urdf_path)

    # print(model_filepath)

    try:
        # Build the URDF from xacro
        command_string = "rosrun xacro xacro {} base_x:={} base_y:={} base_z:={} base_R:={} base_P:={} base_Y:={}".format(
            model_filepath, x, y, z, R, P, Y)
        # Extact the output URDF as robot_description
        robot_description = subprocess.check_output(
            command_string, shell=True, stderr=subprocess.STDOUT)
    except subprocess.CalledProcessError as process_error:
        rospy.logfatal(
            'Failed to run xacro command with error: \n%s', process_error.output)
        sys.exit(1)

    # Update robot_description on ROS param server
    rospy.set_param('/' + robot_side + "/robot_description", robot_description)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(
        description='Loads UR5e_2f85 to the param server')
    parser.add_argument('--robot_side', type=str, default='',
                        help='the robot namespace')
    parser.add_argument('--limited', type=bool, default='True',
                        help='limits joints to -pi -> pi if True')

    args, unknown = parser.parse_known_args()

    try:
        rospy.init_node('load_' + args.robot_side +
                        '_ur5e_2f85', anonymous=True)
        upload_ur5e_2f85(args.robot_side, args.limited)
    except rospy.ROSInterruptException:
        pass
