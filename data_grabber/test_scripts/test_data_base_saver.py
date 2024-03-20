#!/usr/bin/env python
# Code to collect compliance experiment data
# Procedure -
# Robot moves to home position
# Captures the initial visual glimpse
# Moves to Exploratory position with some random fluctuation
# Starts capturing data
# Makes contact with the object
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
    external force and wrench data subscriber
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

    action_data_received = True
    trans, rot = tf_listener.lookupTransform('/base_link', '/tool_frame', rospy.Time())
    x = trans[0]
    y = trans[1]
    z = trans[2]
    roll, pitch, yaw = euler_from_quaternion(rot)
    action_data = np.array([x, y, z, roll, pitch, yaw])

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
    tactile data subscriber
    :param fsr_data:
    :return:
    """
    global tactile_data_buffer
    global tactile_data_received
    tactile_data_received = True
    tactile_data_buffer.append(fsr_data.data)


rospy.init_node('compliance_data_grabber')
rospy.on_shutdown(handle_exit)
bridge = CvBridge()
tf_listener = tf.TransformListener()

config = load_yaml_file('config/config.yaml')
experiment_config = load_yaml_file('config/experiment_config.yaml')
poses = load_yaml_file('config/positions.yaml')
pos_frame_robot = poses['gen3_vistac']['frame']
poses = yaml_to_pose_msg(poses)

moveit_controller = MoveItInterface(config)
log_helper('Ready', 'MoveItInterface')

kortex_controller = KortexInterface(config)
log_helper('Ready', 'KortexInterface')

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

# Add all the subscribers
fsr_sub = rospy.Subscriber('/FSRray/data', IntList, fsr_data_callback, queue_size=10)
rospy.wait_for_service('/bias_request')
fsr_bias_request_service = rospy.ServiceProxy('/bias_request', BiasRequest)

rgb_info_msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo)
depth_info_msg = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo)
depth_image_sub = rospy.Subscriber('/camera/depth/image_raw', Image, depth_img_callback, queue_size=1)
rgb_image_sub = rospy.Subscriber('/camera/color/image_raw', Image, rgb_img_callback, queue_size=1)

base_data_sub = rospy.Subscriber('/base_feedback', kortex_driver.msg.BaseCyclic_Feedback, ft_data_callback, queue_size=10)
tf_listener.waitForTransform("/base_link", "/tool_frame", rospy.Time(0), rospy.Duration(1.0))

log_helper('Everything initialized', 'DATA_GRABBER')

# Define the database structure
traj_keys = ['initial_image', 'params', 'fsr_data', 'action_data', 'ft_data']
scans = np.arange(0, experiment_config['scans'])
# object_id = raw_input('Enter object name - ')  # Add input from the user
object_id = 'I'
# ###################################### Start Experiment ##############################################################
# Define the parameters load from config file
exploratory_pose = 'exp_1'
exploratory_actions = ['pressing', 'precision']
vz = experiment_config['vz']
min_fsr_calibration = 15
fsr_threshold = 8

force_regulation_gain = 0.0001
pressing_threshold = 30

# Creating nested dict
db = {object_id: {}}
for exp_action in exploratory_actions:
    db[object_id][exp_action] = {}
    for scan in scans:
        db[object_id][exp_action][scan] = {}
        for key in traj_keys:
            db[object_id][exp_action][scan][key] = []

db_file_name = 'data/letters/object_'+str(object_id)+'.pickle'

# TODO: add Restart option by adding the last scan and load the existing database in the saved path

exp_action = 'precision'
scan = 0

tactile_obs = []
ft_obs = []
action_obs = []

# Move robot to Home position
if not moveit_controller.robot_at_pose(poses['gen3_vistac']['home'], pos_frame_robot):
    res, plan = moveit_controller.move_to_pose(poses['gen3_vistac']['home'], pos_frame_robot, velocity_scaling=0.1, async=True, confirm=False)
    log_helper('Moving to home position', 'Kinova MoveIT interface')
    while not plan.move_group_succeeded:
        rospy.sleep(0.01)

# Wait for initial glimpse
while len(initial_depth_buffer) < 5 and len(initial_rgb_buffer) < 5:
    rospy.sleep(0.01)

rgb_info_msg = rospy.wait_for_message('/camera/color/camera_info', CameraInfo)
depth_info_msg = rospy.wait_for_message('/camera/depth/camera_info', CameraInfo)

initial_image = {'rgb_image': initial_rgb_buffer[-1], 'depth_image': initial_rgb_buffer[-1], 'rgb_info_msg': rgb_info_msg, 'depth_info_msg': depth_info_msg}
initial_image_received = True
log_helper('Initial image captured', 'DATA_GRABBER')
process_depth = False
process_rgb = False

# Move robot to Exploratory position
recorded_pose = poses['gen3_vistac'][exploratory_pose]
# TODO: Add 5 mm noise in x and y direction

res, plan = moveit_controller.move_to_pose(poses['gen3_vistac'][exploratory_pose], pos_frame_robot, velocity_scaling=0.1, async=True, confirm=False)
log_helper('Moving to exploratory position', 'Kinova MoveIT interface')
while not plan.move_group_succeeded:
    rospy.sleep(0.01)

# Set baseline of the fsr sensor
log_helper('Setting Baseline of FSR', 'DATA_GRABBER')
while not baseline_set:
    resp = fsr_bias_request_service()
    assert resp
    tactile_data_received = False
    tactile_data_buffer.clear()
    while not tactile_data_received or len(tactile_data_buffer) < 10:
        rospy.sleep(0.005)
    re_calibrated_values = np.mean(tactile_data_buffer, axis=0)
    if max(re_calibrated_values) < min_fsr_calibration:
        baseline_set = True
tactile_data_received = False
tactile_data_buffer.clear()
action_data_buffer.clear()
ft_data_buffer.clear()

while not tactile_data_received or len(tactile_data_buffer) < 10:
    rospy.sleep(0.005)

log_helper('Starting exploratory action', 'DATA_GRABBER')

assert tactile_data_received
assert action_data_received
assert ft_data_received

# Move down in velocity control mode till make contact
made_contact = False
cartesian_twists = [0, 0, -vz, 0, 0, 0]
kortex_controller.send_cartesian_twist_command(cartesian_twists)
while not made_contact:
    current_force_z = np.mean(tactile_data_buffer[-1])
    if abs(current_force_z) > fsr_threshold:
        log_helper('Made contactac with Force Z '+ str(current_force_z), 'DATA_GRABBER')
        made_contact = True
        initial_contact_force = abs(current_force_z)

kortex_controller.stop_cartesian_twist_command()

while len(tactile_data_buffer) < 50 or len(action_data_buffer) < 50 or len(ft_data_buffer) < 50:
    rospy.sleep(0.001)
print('Have enough touch data')
# Start logging data - stores the contact information while making touch
for i in range(50, 1, -1):
    tactile_obs.append(tactile_data_buffer[-i])
    action_obs.append(action_data_buffer[-i])
    ft_obs.append(ft_data_buffer[-i])

print('Here here', len(tactile_obs))
if exp_action == 'precision':
    # For precession motion
    sampling_freq = 50  # Hz
    max_amplitude = 4  # max movement angle in degrees # TODO:Action parameter 2 maybe or maybe not
    quarter_time = 1  # seconds # amount of time to reach the max amplitude # TODO: Action parameter 1
    total_time = 10  # seconds total time of action, fixed for now
    palpation_frequency = 1.0/(quarter_time*4.0)
    time_index = np.linspace(0, total_time, int(sampling_freq*total_time))

    theta_trajectory = max_amplitude*np.sin(2*np.pi*palpation_frequency*time_index)
    omega_y_trajectory = 2*np.pi*palpation_frequency*max_amplitude*np.cos(2*np.pi*palpation_frequency*time_index)
    for t in range(0, len(time_index)):
        omega_y = omega_y_trajectory[t]
        force_diff = (np.mean(tactile_data_buffer[-1]) - initial_contact_force)
        adaptive_vz = force_regulation_gain * force_diff
        print(adaptive_vz)
        kortex_controller.send_cartesian_twist_command([0, 0, adaptive_vz, 0, omega_y, 0])

        # Continue logging data
        tactile_obs.append(tactile_data_buffer[-1])
        action_obs.append(action_data_buffer[-1])
        ft_obs.append(ft_data_buffer[-1])

        rospy.sleep(1.0/sampling_freq)

    kortex_controller.stop_cartesian_twist_command()

elif exp_action == 'pressing':
    v_z_trajectories = []
    # For pressing motion
    sampling_freq = 50  # Hz
    max_depth = 0.001  # max movement m
    quarter_time = 0.1  # seconds # amount of time to reach the max amplitude
    total_time = 10  # seconds total time of action, fixed for now
    palpation_frequency = 1.0 / (quarter_time * 4.0)
    time_index = np.linspace(0, total_time, int(sampling_freq * total_time))

    z_trajectory = max_depth * np.sin(2 * np.pi * palpation_frequency * time_index)
    vz_trajectory = 2 * np.pi * palpation_frequency * max_depth * np.cos(2 * np.pi * palpation_frequency * time_index)
    check_force = True
    stop_traj = False
    for t in range(0, len(time_index)):
        v_z_trajectories.append(vz_trajectory)
        force_diff = (np.mean(tactile_data_buffer[-1]) - initial_contact_force)
        if force_diff > pressing_threshold:
            updated_max_depth = abs(z_trajectory[t-1])
            del vz_trajectory
            vz_trajectory = 2 * np.pi * palpation_frequency * updated_max_depth * np.cos(
                2 * np.pi * palpation_frequency * time_index)
            v_z = 0
        else:
            v_z = vz_trajectory[t]
            check_force = True
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

'''
with open('test.npy', 'wb') as f:
    np.save(f, np.array(v_z_trajectories))
'''

user_confirm = raw_input('Save the exploratory action ? (y/n/r)')
if user_confirm == 'y':
    # traj_keys = ['initial_image', 'params', 'fsr_data', 'action_data', 'ft_data']
    # save the trajectory data into the database
    db[object_id][exp_action][scan]['initial_image'] = initial_image
    db[object_id][exp_action][scan]['params'] = experiment_config
    db[object_id][exp_action][scan]['fsr_data'] = tactile_obs
    db[object_id][exp_action][scan]['action_data'] = action_obs
    db[object_id][exp_action][scan]['ft_data'] = ft_obs

    with open(db_file_name, 'wb') as f:
        pickle.dump(db, f)
    log_helper('Saved database', 'DATA_GRABBER')

    scan += 1
elif user_confirm == 'n':
    # Redo
    print('Retrying the trajectory')
else:
    # Restart experiment TODO: fix database loading and restart script
    exit()

rospy.spin()
