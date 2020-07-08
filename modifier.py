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


TIME_GAP = 0

def inv(transform):
    "Invert rigid body transformation matrix"
    R = transform[0:3, 0:3]
    t = transform[0:3, 3]
    t_inv = -1 * R.T.dot(t)
    transform_inv = np.eye(4)
    transform_inv[0:3, 0:3] = R.T
    transform_inv[0:3, 3] = t_inv
    return transform_inv


def get_time(msg):
	return msg.header.stamp.secs * 10**9 + msg.header.stamp.nsecs


parser = argparse.ArgumentParser(description='Rosbag splitter')

parser.add_argument('-i', action='store', dest='input_rosbag', default='/media/psf/Home/Desktop/bags/map2.bag')
parser.add_argument('-n', action='store', dest='n_output_bags', type=int, default=2)
parser.add_argument('-o', action='store', dest='result_directory', default='result')

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


# topics

#odom_topic = ["/base_controller/odom"]
#tf_topic   = ["tf"]
#other_topics = ["base_scan", "data_throttled_camera_info", "data_throttled_image/compressed", "data_throttled_image_depth/compressedDepth"]

odom_topic = ["/az3/base_controller/odom"]
tf_topic   = ["/tf"]
other_topics = ["/jn0/base_scan", "data_throttled_camera_info", "data_throttled_image/compressed", "data_throttled_image_depth/compressedDepth"]

# ---------------- write odometry msgs ------------------
i_bag = 0
new_start_t = start_t

base_odom = Pose()
update_base_odom = True

odometry_buffer = {}

for topic, msg, t in tqdm.tqdm(input_bag.read_messages(topics=odom_topic)):
	if t > new_start_t + new_duration_t and i_bag + 1 < args.n_output_bags:
		update_base_odom = True
		new_start_t += new_duration_t
		i_bag += 1

	if update_base_odom:
		base_odom = copy.deepcopy(msg.pose.pose)

		footprint_q = (base_odom.orientation.x,
				       base_odom.orientation.y,
				       base_odom.orientation.z,
			    	   base_odom.orientation.w)
		footprint_orientation = tf.transformations.quaternion_matrix(footprint_q)

		footprint_orientation[0, 3] = base_odom.position.x
		footprint_orientation[1, 3] = base_odom.position.y
		footprint_orientation[2, 3] = base_odom.position.z

		update_base_odom = False


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

	odometry_buffer[get_time(msg)] = copy.deepcopy(msg.pose.pose) # save new odometry in buffer for tf topic

	msg.header.stamp.secs += i_bag * TIME_GAP
	t += rospy.Duration.from_sec(i_bag * TIME_GAP)

	output_bags[i_bag].write(topic, msg, t)


# ------------------- write tf msgs ---------------------
i_bag = 0
new_start_t = start_t

for topic, msg, t in tqdm.tqdm(input_bag.read_messages(topics=tf_topic)):
	if t > new_start_t + new_duration_t and i_bag + 1 < args.n_output_bags:
		update_base_odom = True
		new_start_t += new_duration_t
		i_bag += 1

	for i in range(len(msg.transforms)):
		if msg.transforms[i].header.frame_id == "/odom" and msg.transforms[i].child_frame_id == "/base_footprint":
			if get_time(msg.transforms[i]) in odometry_buffer:

				odom = odometry_buffer[get_time(msg.transforms[i])]

				msg.transforms[i].transform.translation.x = odom.position.x
				msg.transforms[i].transform.translation.y = odom.position.y
				msg.transforms[i].transform.translation.z = odom.position.z

				msg.transforms[i].transform.rotation.x = odom.orientation.x
				msg.transforms[i].transform.rotation.y = odom.orientation.y
				msg.transforms[i].transform.rotation.z = odom.orientation.z
				msg.transforms[i].transform.rotation.w = odom.orientation.w

		msg.transforms[i].header.stamp.secs += i_bag * TIME_GAP

	t += rospy.Duration.from_sec(i_bag * TIME_GAP)

	output_bags[i_bag].write(topic, msg, t)


# ----------------- write others msgs -------------------
i_bag = 0
new_start_t = start_t

for topic, msg, t in tqdm.tqdm(input_bag.read_messages(topics=other_topics)):
	if t > new_start_t + new_duration_t and i_bag + 1 < args.n_output_bags:
		update_base_odom = True
		new_start_t += new_duration_t
		i_bag += 1

	msg.header.stamp.secs += i_bag * TIME_GAP
	t += rospy.Duration.from_sec(i_bag * TIME_GAP)

	output_bags[i_bag].write(topic, msg, t)



input_bag.close()
[b.close() for b in output_bags]
