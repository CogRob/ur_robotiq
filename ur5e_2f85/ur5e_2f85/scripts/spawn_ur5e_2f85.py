#!/usr/bin/env python

import rospy
import sys
from gazebo_msgs.srv import SpawnModel, SpawnModelRequest
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_from_euler
from ur5e_2f85_util import load_ur5e_2f85_params

"""Spawns the UR5e with 2F85 gripper on side 'robot_side'.
This involves spawning the model into the Gazebo world.
"""


def spawn_ur5e_2f85(robot_side, sim):
    x, y, z, R, P, Y, __ = load_ur5e_2f85_params(robot_side, sim)
    robot_description = rospy.get_param(
        '/' + robot_side + '/robot_description')

    q = quaternion_from_euler(R, P, Y)

    req = SpawnModelRequest()
    rospy.loginfo("Waiting for gazebo/spawn_urdf_model...")
    rospy.wait_for_service('gazebo/spawn_urdf_model')
    rospy.loginfo("gazebo/spawn_urdf_model is active")
    try:
        spawn_model_proxy = rospy.ServiceProxy(
            'gazebo/spawn_urdf_model', SpawnModel)
    except rospy.ServiceException as e:
        rospy.logerr("Service call to SpawnModel failed!")

    robot_position = Point(x=x, y=y, z=z)

    robot_orientation = Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

    print(robot_position)
    print(robot_orientation)

    robot_pose = Pose(position=robot_position, orientation=robot_orientation)

    try:
        robot_name = robot_side + '_ur5e_2f85'
        frame_name = ''
        rospy.logdebug("Spawning robot with name: {}".format(robot_name))
        rospy.logdebug("Frame name: {}".format(frame_name))

        req.model_name = robot_name
        req.model_xml = robot_description
        req.initial_pose = robot_pose
        req.robot_namespace = robot_side
        req.reference_frame = ''

        spawn_model_proxy(req)

    except Exception as e:
        print(e)
        rospy.logerr("{} couldn't be spawned.".format(robot_name))
        return None


def main():
    rospy.init_node("spawn_ur5e_2f85", log_level=rospy.DEBUG)
    spawn_ur5e_2f85(sys.argv[1], sim=sys.argv[2])


if __name__ == '__main__':
    main()
