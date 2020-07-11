#!/usr/bin/python
# -*- coding: utf-8 -*-

import rosbag
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
import tf
import numpy as np

import argparse
import subprocess, yaml
import os
import tqdm
import copy


def inv(transform):
    "Invert rigid body transformation matrix"
    R = transform[0:3, 0:3]
    t = transform[0:3, 3]
    t_inv = -1 * R.T.dot(t)
    transform_inv = np.eye(4)
    transform_inv[0:3, 0:3] = R.T
    transform_inv[0:3, 3] = t_inv
    return transform_inv


parser = argparse.ArgumentParser(description='Odometry reset in rosbag')

parser.add_argument('-i', action='store', dest='input_rosbag', default='/media/psf/Home/Desktop/bags/map2.bag')
parser.add_argument('-r', action='store', dest='result_directory', default='result')
parser.add_argument('-odom_topic', action='store', dest='odom_topic', default='result')

args = parser.parse_args()


input_rosbag_name = args.input_rosbag
input_bag = rosbag.Bag(input_rosbag_name)

source_fn = os.path.splitext(os.path.split(input_rosbag_name)[-1])[0]
result_bagname = os.path.join(args.result_directory, source_fn + '_odom_reseted' + '.bag')
result_bag = rosbag.Bag(result_bagname, 'w')


base_odom = Pose()
get_base_odom = True

for topic, msg, t in tqdm.tqdm(input_bag.read_messages()):
	if topic == args.odom_topic:
		if get_base_odom:
			base_odom = copy.deepcopy(msg.pose.pose)

			footprint_q = (base_odom.orientation.x,
					       base_odom.orientation.y,
					       base_odom.orientation.z,
				    	   base_odom.orientation.w)
			footprint_orientation = tf.transformations.quaternion_matrix(footprint_q)

			footprint_orientation[0, 3] = base_odom.position.x
			footprint_orientation[1, 3] = base_odom.position.y
			footprint_orientation[2, 3] = base_odom.position.z

			get_base_odom = False


		q = (msg.pose.pose.orientation.x,
			 msg.pose.pose.orientation.y,
			 msg.pose.pose.orientation.z,
		     msg.pose.pose.orientation.w)
		orientation = tf.transformations.quaternion_matrix(q)

		orientation[0, 3] = msg.pose.pose.position.x
		orientation[1, 3] = msg.pose.pose.position.y
		orientation[2, 3] = msg.pose.pose.position.z


		new_orientation = np.dot(inv(footprint_orientation), orientation)

		msg.pose.pose.position.x = new_orientation[0, 3]
		msg.pose.pose.position.y = new_orientation[1, 3]
		msg.pose.pose.position.z = new_orientation[2, 3]

		odom_quat = tf.transformations.quaternion_from_matrix(new_orientation)
		msg.pose.pose.orientation.x = odom_quat[0]
		msg.pose.pose.orientation.y = odom_quat[1]
		msg.pose.pose.orientation.z = odom_quat[2]
		msg.pose.pose.orientation.w = odom_quat[3]

	output_bags[i_bag].write(topic, msg, t)