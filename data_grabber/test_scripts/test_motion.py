#!/usr/bin/env python
# Testing Precession motion- x,y cartesian motion, then veloctiy controller in z, and then precission motion
import sys
import rospy
import moveit_commander
import moveit_msgs.msg
from robot_utils import Plan, all_close, transform_pose, yaml_to_pose_msg, load_yaml_file
from tf import TransformListener
from collections import deque
import tf
from geometry_msgs.msg import PoseStamped, Pose
import kortex_driver.msg
import kortex_driver.srv
import math
import numpy as np


def handle_exit():
    """
    Handles the termination of the texture data collection node
    :return:
    """
    print("Shutting down compliance data collection !!")


def external_wrench_data_callback(base_data):
    """
    accelerometer data subscriber
    :param acc_data:
    :return:
    """
    global external_wrench_data_buffer
    global external_wrench_received
    external_wrench_received = True
    # print(acc_data.data)
    # print(base_data)
    external_wrench_data_buffer.append(base_data.base.tool_external_wrench_force_z)


class MoveItInterface():
    """
    This handles the MoveIt communication with the Kinova
    """
    def __init__(self, config):
        self.config = config['my_gen3']
        moveit_commander.roscpp_initialize(sys.argv)
        try:
            # self.is_gripper_present = rospy.get_param(rospy.get_namespace() + "is_gripper_present", False)
            self.is_gripper_present = True
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(rospy.get_namespace() + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            move_group_name = "arm"
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.move_group = moveit_commander.MoveGroupCommander(move_group_name)

            self.display_trajectory_publisher = rospy.Publisher(rospy.get_namespace() + 'move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)

            self.tf_listener = TransformListener()
            self.tf_listener.waitForTransform("/base_link", "/tool_frame", rospy.Time.now(), rospy.Duration(4.0))

            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=rospy.get_namespace())

            rospy.loginfo("Initializing node in namespace " + rospy.get_namespace())
        except Exception as e:
            print(e)
            self.is_init_success = False
        else:
            self.is_init_success = True

    def robot_at_pose(self, pose, frame):
        """
        Returns true if the robot is at this position and false if not
        """
        current_pose = self.move_group.get_current_pose().pose
        transformed_pose = transform_pose(frame, self.config['planning_frame'], pose, self.tf_listener)

        return all_close(current_pose, transformed_pose, self.config['pose_tolerance'])

    def move_to_pose(self, pose, frame, velocity_scaling=None, async=False, confirm=True):
        """
        Moves robot on straight line to this pose
        """
        plan = Plan(self.move_group)
        t_pose = transform_pose(frame, self.config['planning_frame'], pose, self.tf_listener)

        plan.append(t_pose)
        res = plan.execute(async=async, confirm=confirm, velocity_scaling=velocity_scaling)

        if not async:
            plan.delete()
            return res, None
        else:
            return res, plan

        return res, plan

    def get_pose_value(self, x, y, z):
        """
        Computes a pose message of the planning frame
        """
        target_pose = Pose()
        target_pose.position.x = x
        target_pose.position.y = y
        target_pose.position.z = z

        # Transforms the world pose to sensor pose
        quat = tf.transformations.quaternion_from_euler(-math.pi, 0, -math.pi/2, 'rxyz')
        target_pose.orientation.x = quat[0]
        target_pose.orientation.y = quat[1]
        target_pose.orientation.z = quat[2]
        target_pose.orientation.w = quat[3]

        return target_pose

    def move_up(self, home_frame, home_pose, async=False, confirm=True):
        """
        Moves the robot up to given z_height of robot home in frame
        """
        plan = Plan(self.move_group)

        t_home_pose = transform_pose(home_frame, self.config['planning_frame'], home_pose, self.tf_listener)

        # Get current pose
        current_pose = self.move_group.get_current_pose().pose

        # Change z value to the z_height
        current_pose.position.z = t_home_pose.position.z

        plan.append(current_pose)

        res = plan.execute(async=async, confirm=confirm)

        if not async:
            plan.delete()
            return res, None
        else:
            return res, plan

    def reach_gripper_position(self, relative_position):
        gripper_group = self.gripper_group
        # We only have to move this joint because all others are mimic!
        gripper_joint = self.robot.get_joint(self.gripper_joint_name)
        gripper_max_absolute_pos = gripper_joint.max_bound()
        gripper_min_absolute_pos = gripper_joint.min_bound()
        try:
            val = gripper_joint.move(
                relative_position * (gripper_max_absolute_pos - gripper_min_absolute_pos) + gripper_min_absolute_pos,
                True)
            return val
        except:
            return False


class KortexController():
    """
    This handles the Direct controller communication with the Kinova
    """
    def __init__(self, config):
        rospy.wait_for_service('/base/send_joint_speeds_command')  # joint velocity control
        rospy.wait_for_service('/base/send_twist_command')  # cartesian velocity control
        rospy.wait_for_service('/base/send_wrench_command')  # torque control
        self.srv_joint_velocity_controller = rospy.ServiceProxy('/base/send_joint_speeds_command', kortex_driver.srv.SendJointSpeedsCommand)
        self.srv_twist_controller = rospy.ServiceProxy('/base/send_twist_command', kortex_driver.srv.SendTwistCommand)
        self.srv_wrench_controller = rospy.ServiceProxy('/base/send_wrench_command', kortex_driver.srv.SendWrenchCommand)
        self.config = config['my_gen3']

    def send_joint_velocity_command(self, t):
        speed_req = kortex_driver.srv.SendJointSpeedsCommandRequest()
        # joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        # NOTE: JointSpeed/value is in DEGREES per second.
        # cf. https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/Base/JointSpeed.md
        joint_speed = kortex_driver.msg.JointSpeed()
        joint_speed.joint_identifier = 6
        joint_speed.value = 3*math.sin(t)
        speed_req.input.joint_speeds.append(joint_speed)
        resp = self.srv_joint_velocity_controller.call(speed_req)

    def stop_joint_velocity_control(self):
        speed_req = kortex_driver.srv.SendJointSpeedsCommandRequest()
        joint_speed = kortex_driver.msg.JointSpeed()
        joint_speed.joint_identifier = 6
        joint_speed.value = 0.0
        speed_req.input.joint_speeds.append(joint_speed)
        self.srv_joint_velocity_controller.call(speed_req)

    def send_cartesian_velocity_command(self, x, y, z):
        twist_req = kortex_driver.srv.SendTwistCommandRequest()
        twist_req.input.reference_frame = 0
        twist_req.input.twist.angular_x = x
        twist_req.input.twist.angular_y = y
        twist_req.input.twist.linear_z = z
        twist_req.input.duration = 0
        resp = self.srv_twist_controller.call(twist_req)

    def stop_cartesian_velocity_command(self):
        twist_req = kortex_driver.srv.SendTwistCommandRequest()
        twist_req.input.reference_frame = 0 # TODO: Clarify
        twist_req.input.twist.angular_x = 0
        twist_req.input.twist.angular_y = 0
        twist_req.input.twist.linear_z = 0
        twist_req.input.duration = 0
        resp = self.srv_twist_controller.call(twist_req)

    def send_wrench_command(self, force):
        wrench_req = kortex_driver.srv.SendWrenchCommand()
        wrench = kortex_driver.msg.Wrench()
        wrench_req.reference_frame = 0 # TODO: Clarify
        wrench_req.mode = 0 # TODO: Clarify
        wrench.force_z = force
        wrench_req.wrench = wrench
        wrench_req.duration = 0
        resp = self.srv_wrench_controller.call(wrench_req)

    def stop_wrench_command(self):
        wrench_req = kortex_driver.srv.SendWrenchCommand()
        wrench = kortex_driver.msg.Wrench()
        wrench_req.reference_frame = 0  # TODO: Clarify
        wrench_req.mode = 0  # TODO: Clarify
        wrench.force_z = 0
        wrench_req.wrench = wrench
        wrench_req.duration = 0
        resp = self.srv_wrench_controller.call(wrench_req)


rospy.init_node('testing_compliance_trajectory')
rospy.on_shutdown(handle_exit)

config = load_yaml_file('config/config.yaml')

poses = load_yaml_file('config/positions.yaml')
pos_frame_robot = poses['my_gen3']['frame']
poses = yaml_to_pose_msg(poses)

# moveit_controller = MoveItInterface(config)
# moveit_controller.reach_gripper_position(1.0) # TODO: Fix the gripper, maybe not required when tactile sensor will be added
kortex_controller = KortexController(config)

external_wrench_data_buffer = deque(maxlen=3)
external_wrench_received = False

# base_feedback subscriber
base_data_sub = rospy.Subscriber('/base_feedback', kortex_driver.msg.BaseCyclic_Feedback, external_wrench_data_callback, queue_size=10)

'''
if not position_controller.robot_at_pose(poses['my_gen3']['home'], pos_frame_robot):
    res, plan = position_controller.move_to_pose(poses['my_gen3']['home'], pos_frame_robot, velocity_scaling=0.1, async=True, confirm=False)
    while not plan.move_group_succeeded:
        rospy.sleep(0.01)
'''
'''
for t in range(0, 5):
    velocity_controller.send_joint_velocity_command(t)
    rospy.sleep(1)

velocity_controller.stop_joint_velocity_control()
print("Done")
'''
'''
# Test target positions to give
x = 0.4
y = 0.0
z = 0.48
target_pose = position_controller.get_pose_value(x, y, z)

res, plan = position_controller.move_to_pose(target_pose, pos_frame_robot, velocity_scaling=0.1, async=True)
    
while not plan.move_group_succeeded:
    rospy.sleep(0.01)
'''

# Move down in velocity control mode till make contact
made_contact = False
kortex_controller.send_cartesian_velocity_command(0, 0, -0.008)
while not external_wrench_received:
    print('Waiting for wrench data')
    rospy.sleep(0.1)

while not made_contact:
    current_force_z = np.mean(external_wrench_data_buffer)
    print('Current Force Z: ', current_force_z)
    if abs(current_force_z) > 9:
        made_contact = True
        initial_contact_wrench = abs(current_force_z)

kortex_controller.stop_cartesian_velocity_command()
#external_wrench_data_buffer.clear()
#external_wrench_received = False

#kortex_controller.send_wrench_command(1)
time_traj = np.linspace(0, 12*math.pi, 1000)


for t in range(0, len(time_traj)):
    # print(time_traj[t])
    # kortex_controller.send_wrench_command(2)
    x = 5*math.cos(time_traj[t])
    y = 5*math.sin(time_traj[t])
    force_diff = (np.mean(external_wrench_data_buffer) - initial_contact_wrench)
    z = 0.001 * force_diff
    print(force_diff)
    kortex_controller.send_cartesian_velocity_command(x, y, z)
    rospy.sleep(0.01)

kortex_controller.stop_cartesian_velocity_command()
kortex_controller.stop_wrench_command()
print("Done")
rospy.spin()

