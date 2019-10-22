#!/usr/bin/env python
import rospy
import tf
import sys
import rospkg
import yaml
import os
from tf.transformations import quaternion_from_euler
from ur5e_2f85_util import load_ur5e_2f85_params


if __name__ == '__main__':
    rospack = rospkg.RosPack()

    # Parse robot_side input to determine which TF to generate
    if(len(sys.argv) > 1):
        robot_side = sys.argv[1]
        if(robot_side not in ['left', 'right', 'single']):
            raise ValueError(
                'Robot side should be either \'left\', \'right\' or \'single\'!, got {}'.format(robot_side))
    else:
        raise ValueError(
            'Make sure to pass in robot_side (\'left\', \'right\' or \'single\'!')

    rospy.init_node(
        robot_side + '_to_gazebo_origin_tf_broadcaster', log_level=rospy.DEBUG)

    x, y, z, R, P, Y = load_ur5e_2f85_params(robot_side)
    # Convert R, P, Y to quaternion
    q = quaternion_from_euler(R, P, Y)

    rospy.loginfo('Orientation quaternion: {}'.format(q))

    # Construct the transform broadcaster, publish at 10 Hz
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        # Send transform
        br.sendTransform((x, y, z),
                         (q[0], q[1], q[2], q[3]),
                         rospy.Time.now(),
                         '/' + robot_side + '/world',
                         '/gazebo_origin')

        rate.sleep()
