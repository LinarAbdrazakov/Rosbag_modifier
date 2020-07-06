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


parser = argparse.ArgumentParser(description='Rosbag splitter')

parser.add_argument('-i', action='store', dest='input_rosbag', default='/media/psf/Home/Desktop/bags/map2.bag')
parser.add_argument('-n', action='store', dest='n_output_bags', type=int, default=2)
parser.add_argument('-o', action='store', dest='result_directory', default='result')
parser.add_argument('-r', action='store', dest='reset_to_zero_odom', type=bool, default=True)

args = parser.parse_args()

input_rosbag_name = args.input_rosbag #'/media/psf/Home/Desktop/bags/map2.bag'
input_bag = rosbag.Bag(input_rosbag_name)

info_dict = yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', input_rosbag_name], stdout=subprocess.PIPE).communicate()[0])


start_t        = rospy.Time.from_sec(info_dict['start'])
duration_t     = rospy.Duration.from_sec(info_dict['duration'])
end_t          = rospy.Time.from_sec(info_dict['end'])
new_duration_t = rospy.Duration.from_sec(info_dict['duration'] / args.n_output_bags)


source_fn = os.path.splitext(os.path.split(input_rosbag_name)[-1])[0]
output_bags = []
for s in range(args.n_output_bags):
	result_bagname = os.path.join(args.result_directory, source_fn + '_part_' + str(s) + '.bag')
	output_bags.append(rosbag.Bag(result_bagname, 'w'))


i_bag = 0
new_start_t = start_t

base_odom = Pose()
update_base_odom = True

for topic, msg, t in tqdm.tqdm(input_bag.read_messages()):
	if t > new_start_t + new_duration_t and i_bag + 1 < args.n_output_bags:
		update_base_odom = True
		new_start_t += new_duration_t
		i_bag += 1

	if update_base_odom and topic == '/base_controller/odom':
		base_odom = copy.deepcopy(msg.pose.pose)
		update_base_odom = False

	if args.reset_to_zero_odom and topic == '/base_controller/odom':
		msg.pose.pose.position.x -= base_odom.position.x;
		msg.pose.pose.position.y -= base_odom.position.y;
		msg.pose.pose.position.z -= base_odom.position.z;

		q = (msg.pose.pose.orientation.x,
			 msg.pose.pose.orientation.y,
			 msg.pose.pose.orientation.z,
		     msg.pose.pose.orientation.w)
		rot = tf.transformations.quaternion_matrix(q)

		q_base_inv = (base_odom.orientation.x,
				      base_odom.orientation.y,
				      base_odom.orientation.z,
				      - base_odom.orientation.w)
		rot_base_inv = tf.transformations.quaternion_matrix(q_base_inv)

		rot = np.dot(rot, rot_base_inv)
		odom_quat = tf.transformations.quaternion_from_matrix(rot)

		msg.pose.pose.orientation.x = odom_quat[0]
		msg.pose.pose.orientation.y = odom_quat[1]
		msg.pose.pose.orientation.z = odom_quat[2]
		msg.pose.pose.orientation.w = odom_quat[3]

	output_bags[i_bag].write(topic, msg, t)


input_bag.close()
[b.close() for b in output_bags]