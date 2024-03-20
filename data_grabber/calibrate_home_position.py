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
import matplotlib.pyplot as plt

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
    ft_data = np.array([base_data.base.tool_pose_x, base_data.base.tool_pose_y, base_data.base.tool_pose_z, base_data.base.tool_pose_theta_x, base_data.base.tool_pose_theta_y, base_data.base.tool_pose_theta_z])

    action_data_received = True
    trans, rot = tf_listener.lookupTransform('/base_link', '/tool_frame', rospy.Time())
    x = trans[0]
    y = trans[1]
    z = trans[2]
    roll, pitch, yaw = euler_from_quaternion(rot)
    action_data = np.array([x, y, z, roll, pitch, yaw])

    ft_data_buffer.append(ft_data)
    action_data_buffer.append(action_data)


rospy.init_node('calibrate_robot_position')
bridge = CvBridge()
tf_listener = tf.TransformListener()
tf_listener.waitForTransform("/base_link", "/tool_frame", rospy.Time(0), rospy.Duration(1.0))
ft_data_received = False
action_data_received = False
ft_data_buffer = deque(maxlen=1000)
action_data_buffer = deque(maxlen=1000)

base_data_sub = rospy.Subscriber('/base_feedback', kortex_driver.msg.BaseCyclic_Feedback, ft_data_callback, queue_size=10)

config = load_yaml_file('config/config.yaml')
experiment_config = load_yaml_file('config/experiment_config.yaml')
poses = load_yaml_file('config/positions.yaml')
pos_frame_robot = poses['gen3_vistac']['frame']
poses = yaml_to_pose_msg(poses)

moveit_controller = MoveItInterface(config)
log_helper('Ready', 'MoveItInterface')

kortex_controller = KortexInterface(config)
log_helper('Ready', 'KortexInterface')

while not ft_data_received:
    print('Waiting for data')
    rospy.sleep(0.1)

x = -0.211
y = -0.367
z = 0.2
target_pose = moveit_controller.get_pose_value(x, y, z)

res, plan = moveit_controller.move_to_pose(target_pose, pos_frame_robot, velocity_scaling=0.1, async=True, confirm=False)

while not plan.move_group_succeeded:
    rospy.sleep(0.01)

'''
sampling_freq = 50  # Hz
max_amplitude = 3  # max movement angle in degrees # TODO:Action parameter 2 maybe or maybe not
quarter_time = 0.75  # seconds # amount of time to reach the max amplitude # TODO: Action parameter 1
total_time = 10  # seconds total time of action, fixed for now
palpation_frequency = 1.0/(quarter_time*4.0)
time_index = np.linspace(0, total_time, int(sampling_freq*total_time))
'''

sampling_freq = 50  # Hz
max_depth = 0.0008  # max movement m
quarter_time = 0.5  # seconds # amount of time to reach the max amplitude
total_time = 10  # seconds total time of action, fixed for now
palpation_frequency = 1.0 / (quarter_time * 4.0)
time_index = np.linspace(0, total_time, int(sampling_freq * total_time))

#theta_trajectory = max_amplitude*np.sin(2*np.pi*palpation_frequency*time_index)
#omega_y_trajectory = 2*np.pi*palpation_frequency*max_amplitude*np.cos(2*np.pi*palpation_frequency*time_index)
z_trajectory = max_depth * np.sin(2 * np.pi * palpation_frequency * time_index)
vz_trajectory = 2 * np.pi * palpation_frequency * max_depth * np.cos(2 * np.pi * palpation_frequency * time_index)

z_traj = []
z_traj_mod = []
for t in range(0, len(time_index)):
    #omega_y = omega_y_trajectory[t]
    #kortex_controller.send_cartesian_twist_command([0, 0, 0, 0, omega_y, 0])
    v_z = vz_trajectory[t]
    kortex_controller.send_cartesian_twist_command([0, 0, v_z, 0, 0, 0])
    trans, rot = tf_listener.lookupTransform('/tool_frame', '/base_link', rospy.Time())
    x = trans[0]
    y = trans[1]
    z = trans[2]
    roll, pitch, yaw = euler_from_quaternion(rot)

    z_traj.append(z-0.2)
    tool_data = ft_data_buffer[-1]
    z_traj_mod.append(tool_data[2]-0.2)
    rospy.sleep(1/sampling_freq)


kortex_controller.stop_cartesian_twist_command()

plt.plot(np.array(z_traj), 'r')
plt.plot(np.array(z_traj_mod), 'b')
plt.plot(z_trajectory, 'k')
plt.show()
