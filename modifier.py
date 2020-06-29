#!/usr/bin/python
# -*- coding: utf-8 -*-

import rosbag
import rospy
import argparse
import subprocess, yaml
import os


parser = argparse.ArgumentParser(description='Rosbag splitter')

parser.add_argument('-i', action='store', dest='input_rosbag')
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


i_bag = 0
new_start_t = start_t
for topic, msg, t in input_bag.read_messages():
	if t > new_start_t + new_duration_t and i_bag + 1 < args.n_output_bags:
		new_start_t += new_duration_t
		i_bag += 1

	output_bags[i_bag].write(topic, msg, t)


input_bag.close()
[b.close() for b in output_bags]