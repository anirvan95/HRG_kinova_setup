#!/usr/bin/env python
# Code to collect compliance experiment data
# Procedure -
# Robot moves to home position
# Captures the initial visual glimpse
# Moves to Exploratory position with some random fluctuation to debiase the position effect
# Makes contact with the object
# Starts recording data
# Performs palpation action
# Stop capturing the data
# Repeats

import time
import rospy
from robot_utils import MoveItInterface, KortexInterface, yaml_to_pose_msg, load_yaml_file, log_helper
from tf.transformations import euler_from_quaternion
from collections import deque
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import numpy as np
import tf
from fsr.msg import IntList
from fsr.srv import BiasRequest, BiasRequestResponse, BiasRequestRequest
import kortex_driver.msg
import kortex_driver.srv
import pickle


def handle_exit():
    """
    Handles the termination of the texture data collection node
    :return:
    """
    rospy.sleep(1)
    print("Shutting down compliance data collection !!")


def ft_data_callback(base_data):
    """
    external force and wrench + pose of tool_frame data subscriber
    :param base_data:
    :return:
    """
    global ft_data_buffer
    global ft_data_received
    global action_data_received
    global action_data_buffer
    global tf_listener
    ft_data_received = True
    ft_data = np.array([base_data.base.tool_external_wrench_force_x, base_data.base.tool_external_wrench_force_y, base_data.base.tool_external_wrench_force_z, base_data.base.tool_external_wrench_torque_x, base_data.base.tool_external_wrench_torque_y, base_data.base.tool_external_wrench_torque_z])
    action_data = np.array([base_data.base.tool_pose_x, base_data.base.tool_pose_y, base_data.base.tool_pose_z, base_data.base.tool_pose_theta_x, base_data.base.tool_pose_theta_y, base_data.base.tool_pose_theta_z, base_data.base.tool_twist_linear_x, base_data.base.tool_twist_linear_y, base_data.base.tool_twist_linear_z, base_data.base.tool_twist_angular_x, base_data.base.tool_twist_angular_y, base_data.base.tool_twist_angular_z])
    action_data_received = True
    ft_data_buffer.append(ft_data)
    action_data_buffer.append(action_data)


def depth_img_callback(depth_data):
    """
    Subscriber for depth image
    :return:
    """
    global process_depth
    if process_depth:
        global initial_depth_buffer
        depth_image = bridge.imgmsg_to_cv2(depth_data)
        initial_depth_buffer.append(depth_image)
        global depth_data_received
        depth_data_received = True


def rgb_img_callback(rgb_data):
    """
    Subscriber for rgb image callback
    :return:
    """
    global process_rgb
    if process_rgb:
        global initial_rgb_buffer
        rgb_image = bridge.imgmsg_to_cv2(rgb_data)
        initial_rgb_buffer.append(rgb_image)
        global rgb_data_received
        rgb_data_received = True


def fsr_data_callback(fsr_data):
    """
    tactile (fsr) data subscriber
    :param fsr_data:
    :return:
    """
    global tactile_data_buffer
    global tactile_data_received
    tactile_data_received = True
    tactile_data_buffer.append(fsr_data.data)

# ############################################### Main Code ###############################################
# Initialize ROS node
rospy.init_node('compliance_data_grabber')
rospy.on_shutdown(handle_exit)
bridge = CvBridge()
tf_listener = tf.TransformListener()

# Initialize robot controllers and load config files
config = load_yaml_file('config/config.yaml')
poses = load_yaml_file('config/positions.yaml')
pos_frame_robot = poses['gen3_vistac']['frame']
poses = yaml_to_pose_msg(poses)
moveit_controller = MoveItInterface(config)
log_helper('Ready', 'Kinova MoveIT interface')
kortex_controller = KortexInterface(config)
log_helper('Ready', 'Kinova Kortex interface')

# Initialize tracking variables and buffers to handle variable sampling rate data stream
tactile_data_received = False
ft_data_received = False
action_data_received = False
initial_image_received = False
process_depth = True
process_rgb = True
tactile_data_buffer = deque(maxlen=1000)
ft_data_buffer = deque(maxlen=1000)
action_data_buffer = deque(maxlen=1000)
initial_rgb_buffer = deque(maxlen=10)
initial_depth_buffer = deque(maxlen=10)


# Initialize ROS subscribers and services
fsr_sub = rospy.Subscriber('/FSRray/data', IntList, fsr_data_callback, queue_size=10)
rospy.wait_for_service('/bias_request')
fsr_bias_request_service = rospy.ServiceProxy('/bias_request', BiasRequest)

rgb_info_msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo)
depth_info_msg = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo)
depth_image_sub = rospy.Subscriber('/camera/depth/image_rect', Image, depth_img_callback, queue_size=1)
rgb_image_sub = rospy.Subscriber('/camera/color/image_rect_color', Image, rgb_img_callback, queue_size=1)

base_data_sub = rospy.Subscriber('/base_feedback', kortex_driver.msg.BaseCyclic_Feedback, ft_data_callback, queue_size=10)
tf_listener.waitForTransform("/base_link", "/tool_frame", rospy.Time(0), rospy.Duration(1.0))

log_helper('Everything initialized', 'DATA_GRABBER')

# Initialize the database structure & load the parameters from config file
experiment_config = load_yaml_file('config/experiment_config.yaml')
traj_keys = ['initial_image', 'params', 'fsr_data', 'action_data', 'ft_data']
scans = experiment_config['scans']
object_id = raw_input('Enter object name - ')  # Add input from the user
exploratory_pose = 'exp_1'
exploratory_actions = ['precision', 'pressing']
contact_vz = experiment_config['contact_velocity']
fsr_calibration_threshold = experiment_config['fsr_calibration_threshold']
fsr_contact_threshold = experiment_config['fsr_contact_threshold']
fsr_calibration_window = 15

sampling_freq = experiment_config['sampling_freq']  # Hz
total_time = experiment_config['exp_time']  # seconds total time of action, fixed for now
time_index = np.linspace(0, total_time, int(sampling_freq * total_time))
# Precision parameters
precision_angles = np.linspace(experiment_config['precision']['angle_min'], experiment_config['precision']['angle_max'], scans/2)
precision_qts = np.linspace(experiment_config['precision']['qt_min'], experiment_config['precision']['qt_max'], scans/2)
precision_angle_vals, precision_qt_vals = np.meshgrid(precision_angles, precision_qts)
precision_angle_vals = precision_angle_vals.reshape(-1)
precision_qt_vals = precision_qt_vals.reshape(-1)
precision_frequency_vals = 1.0 / (precision_qt_vals * 4.0)
precision_regulation_gain = 0.0001

# Pressing parameters
pressing_depths = np.linspace(experiment_config['pressing']['depth_min'], experiment_config['pressing']['depth_max'], scans/2)
pressing_qts = np.linspace(experiment_config['pressing']['qt_min'], experiment_config['pressing']['qt_max'], scans/2)
pressing_depth_vals, pressing_qt_vals = np.meshgrid(pressing_depths, pressing_qts)
pressing_depth_vals = pressing_depth_vals.reshape(-1)
pressing_qt_vals = pressing_qt_vals.reshape(-1)
pressing_frequency_vals = 1.0 / (pressing_qt_vals * 4.0)
pressing_force_max = 100

# Creating nested dict
db = {object_id: {}}
for exp_action in exploratory_actions:
    db[object_id][exp_action] = {}
    scan = 0
    while scan < scans:
        db[object_id][exp_action][scan] = {}
        for key in traj_keys:
            db[object_id][exp_action][scan][key] = []
        scan += 1
db_file_name = experiment_config['db_path']+'object_'+str(object_id)+'.pickle'

# TODO: add Restart option by adding the asking for scan id and load the existing database in the saved path

for exp_action in exploratory_actions:
    scan = 0
    while scan < scans:
        log_helper('Exp Action - '+str(exp_action)+' Scan ID - '+str(scan), 'DATA_GRABBER')
        tactile_obs = []
        ft_obs = []
        action_obs = []
        baseline_set = False
        # Move robot to Home position
        if not moveit_controller.robot_at_pose(poses['gen3_vistac']['home'], pos_frame_robot):
            res, plan = moveit_controller.move_to_pose(poses['gen3_vistac']['home'], pos_frame_robot,
                                                       velocity_scaling=0.1, async=True, confirm=False)
            log_helper('Moving to home position', 'Kinova MoveIT interface')
            while not plan.move_group_succeeded:
                rospy.sleep(0.01)

        # Wait for initial glimpse
        while len(initial_depth_buffer) < 5 and len(initial_rgb_buffer) < 5:
            rospy.sleep(0.01)

        rgb_info_msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo)
        depth_info_msg = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo)

        initial_image = {'rgb_image': initial_rgb_buffer[-1], 'depth_image': initial_depth_buffer[-1],
                         'rgb_info_msg': rgb_info_msg, 'depth_info_msg': depth_info_msg}
        initial_image_received = True
        log_helper('Initial image captured', 'DATA_GRABBER')
        process_depth = False
        process_rgb = False

        # Move robot to Exploratory position
        recorded_pose = poses['gen3_vistac'][exploratory_pose]
        # TODO: Add 5 mm noise in x and y direction

        res, plan = moveit_controller.move_to_pose(poses['gen3_vistac'][exploratory_pose], pos_frame_robot,
                                                   velocity_scaling=0.1, async=True, confirm=False)
        log_helper('Moving to exploratory position', 'Kinova MoveIT interface')
        while not plan.move_group_succeeded:
            rospy.sleep(0.01)

        # Set baseline of the fsr sensor
        while not baseline_set:
            resp = fsr_bias_request_service()
            assert resp
            tactile_data_received = False
            tactile_data_buffer.clear()
            while not tactile_data_received or len(tactile_data_buffer) < fsr_calibration_window:
                log_helper('Waiting for tactile data', 'DATA_GRABBER')
                rospy.sleep(0.1)
            re_calibrated_values = np.mean(tactile_data_buffer, axis=0)
            if max(re_calibrated_values) < fsr_calibration_threshold:
                baseline_set = True

        log_helper('Starting exploratory action', 'DATA_GRABBER')

        assert tactile_data_received
        assert action_data_received
        assert ft_data_received

        # Move down in velocity control mode till make contact
        made_contact = False
        cartesian_twists = [0, 0, -contact_vz, 0, 0, 0]
        kortex_controller.send_cartesian_twist_command(cartesian_twists)
        while not made_contact:
            current_force_z = np.mean(tactile_data_buffer[-1])
            if abs(current_force_z) > fsr_contact_threshold:
                log_helper('Made contact with Force Z ' + str(current_force_z), 'DATA_GRABBER')
                made_contact = True
                initial_contact_force = abs(current_force_z)

        kortex_controller.stop_cartesian_twist_command()

        while len(tactile_data_buffer) < 50 or len(action_data_buffer) < 50 or len(ft_data_buffer) < 50:
            rospy.sleep(0.001)

        # Start logging data - stores some samples of contact information while making touch
        for i in range(50, 1, -1):
            tactile_obs.append(tactile_data_buffer[-i])
            action_obs.append(action_data_buffer[-i])
            ft_obs.append(ft_data_buffer[-i])

        if exp_action == 'precision':
            # For precession motion
            theta_trajectory = precision_angle_vals[scan]*np.sin(2*np.pi*precision_frequency_vals[scan]*time_index)
            omega_y_trajectory = 2*np.pi*precision_frequency_vals[scan]*precision_angle_vals[scan]*np.cos(2*np.pi*precision_frequency_vals[scan]*time_index)
            for t in range(0, len(time_index)):
                omega_y = omega_y_trajectory[t]
                force_diff = (np.mean(tactile_data_buffer[-1]) - initial_contact_force)
                adaptive_vz = precision_regulation_gain * force_diff
                # adaptive_vz = 0
                kortex_controller.send_cartesian_twist_command([0, 0, adaptive_vz, 0, omega_y, 0])

                # Continue logging data
                tactile_obs.append(tactile_data_buffer[-1])
                action_obs.append(action_data_buffer[-1])
                ft_obs.append(ft_data_buffer[-1])

                rospy.sleep(1.0/sampling_freq)

            kortex_controller.stop_cartesian_twist_command()

        elif exp_action == 'pressing':
            # For pressing motion
            z_trajectory = pressing_depth_vals[scan] * np.sin(2 * np.pi * pressing_frequency_vals[scan] * time_index)
            vz_trajectory = 2 * np.pi * pressing_frequency_vals[scan] * pressing_depth_vals[scan] * np.cos(2 * np.pi * pressing_frequency_vals[scan] * time_index)

            for t in range(0, len(time_index)):
                force_diff = (np.mean(tactile_data_buffer[-1]) - initial_contact_force)
                '''
                if force_diff > pressing_force_max:
                    updated_max_depth = abs(z_trajectory[t - 1])
                    # update the trajectory to avoid to much pressing
                    vz_trajectory = 2 * np.pi * pressing_frequency_vals[scan] * updated_max_depth * np.cos(
                        2 * np.pi * pressing_frequency_vals[scan] * time_index)
                    v_z = 0
                else:
                    v_z = vz_trajectory[t]
                '''
                v_z = vz_trajectory[t]
                kortex_controller.send_cartesian_twist_command([0, 0, v_z, 0, 0, 0])
                # Continue logging data
                tactile_obs.append(tactile_data_buffer[-1])
                action_obs.append(action_data_buffer[-1])
                ft_obs.append(ft_data_buffer[-1])

                rospy.sleep(1.0/sampling_freq)

            kortex_controller.stop_cartesian_twist_command()


        time.sleep(2)
        log_helper('Moving up', 'Kinova MoveIT interface')
        res, plan = moveit_controller.move_up(poses['gen3_vistac']['home'], pos_frame_robot, velocity_scaling=0.1, confirm=False)
        while not plan.move_group_succeeded:
            rospy.sleep(0.1)

        # user_confirm = raw_input('Save the exploratory action ? (y/n/r) - ')
        user_confirm = 'y'
        if user_confirm == 'y':
            # traj_keys = ['initial_image', 'params', 'fsr_data', 'action_data', 'ft_data']
            # save the trajectory data into the database
            if exp_action == 'pressing':
                traj_param = {'exp_action': 'pressing', 'amplitude': pressing_depth_vals[scan], 'freq': pressing_frequency_vals[scan], 'baseline': re_calibrated_values}
            elif exp_action == 'precision':
                traj_param = {'exp_action': 'precision', 'amplitude': precision_angle_vals[scan], 'freq': precision_frequency_vals[scan], 'baseline': re_calibrated_values}
            db[object_id][exp_action][scan]['initial_image'] = initial_image
            db[object_id][exp_action][scan]['params'] = traj_param
            db[object_id][exp_action][scan]['fsr_data'] = tactile_obs
            db[object_id][exp_action][scan]['action_data'] = action_obs
            db[object_id][exp_action][scan]['ft_data'] = ft_obs
            scan += 1

            with open(db_file_name, 'wb') as f:
                pickle.dump(db, f)
            log_helper('Saved database', 'DATA_GRABBER')

        elif user_confirm == 'n':
            # Redo
            log_helper('Re-doing the last scan', 'DATA_GRABBER')
            rospy.sleep(0.01)
        else:
            # Restart experiment
            # TODO: fix database loading and restart script
            break

rospy.spin()
