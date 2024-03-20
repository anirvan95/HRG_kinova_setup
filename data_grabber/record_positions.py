#!/usr/bin/env python
import rospy
import moveit_commander
from tf import TransformListener
from robot_utils import transform_pose, load_yaml_file, write_yaml_file, pose_to_dict
import argparse
import sys

if __name__ == '__main__':

    parser = argparse.ArgumentParser(description='Record one robot pose.')
    # parser.add_argument('-r', type=str, dest='robot_name', help='Name of the robot to be recorded')
    parser.add_argument('-n', type=str, dest='pose_name', help='Name of the pose to be recorded')

    args = parser.parse_args()
    pos_name = args.pose_name
    robot_group_name = "arm"
    robot_name = "gen3_vistac"

    rospy.init_node('recorder_node')
    move_group = moveit_commander.MoveGroupCommander(robot_group_name)

    tf_listener = TransformListener()
    tf_listener.waitForTransform("/base_link", "/tool_frame", rospy.Time(0), rospy.Duration(4.0))

    pos_file_content = load_yaml_file('config/positions.yaml')

    current_pose = move_group.get_current_pose().pose
    eef_link = move_group.get_end_effector_link()
    print(eef_link)
    current_pose = transform_pose(eef_link, pos_file_content[robot_name]['frame'], current_pose, tf_listener)

    pos_file_content[robot_name][pos_name] = pose_to_dict(current_pose)

    write_yaml_file('config/positions.yaml', pos_file_content)

    print "> Saved new position {} for robot {}".format(pos_name, robot_name)