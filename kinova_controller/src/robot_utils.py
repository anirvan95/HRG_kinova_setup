import numpy as np
import rospy
import tf
from geometry_msgs.msg import PoseStamped, Pose
import yaml
import moveit_commander
import sys
from moveit_commander.conversions import pose_to_list
from tf import TransformListener
import moveit_msgs
from moveit_msgs.msg import ExecuteTrajectoryActionFeedback, RobotTrajectory
import math
import kortex_driver.msg
import kortex_driver.srv
import time


class UserDeclinedError(Exception):
    def __init__(self):
        super(UserDeclinedError, self).__init__("User declined motion.")


class MoveItInterface():
    """
    This handles the MoveIt communication with the Kinova
    """
    def __init__(self, config):
        log_helper("Initializing MoveIT", 'MoveItInterface')
        self.config = config['gen3_vistac']
        moveit_commander.roscpp_initialize(sys.argv)
        try:
            self.degrees_of_freedom = rospy.get_param(rospy.get_namespace() + "degrees_of_freedom", 7)

            # Create the MoveItInterface necessary objects
            move_group_name = "arm"
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.move_group = moveit_commander.MoveGroupCommander(move_group_name)

            self.display_trajectory_publisher = rospy.Publisher(
                rospy.get_namespace() + 'move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory,
                queue_size=20)

            self.tf_listener = TransformListener()
            self.tf_listener.waitForTransform("/base_link", "/tactile_tool_frame", rospy.Time.now(), rospy.Duration(4.0))
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
        quat = tf.transformations.quaternion_from_euler(-math.pi, 0, -math.pi / 2, 'rxyz')
        target_pose.orientation.x = quat[0]
        target_pose.orientation.y = quat[1]
        target_pose.orientation.z = quat[2]
        target_pose.orientation.w = quat[3]

        return target_pose

    def move_up(self, home_pose, home_frame, velocity_scaling=None, async=True, confirm=True):
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

        res = plan.execute(async=async, confirm=confirm, velocity_scaling=velocity_scaling)

        if not async:
            plan.delete()
            return res, None
        else:
            return res, plan


class KortexInterface():
    """
    This handles the Low Level controller communication with the Kinova
    """
    def __init__(self, config):
        log_helper("Initializing Kortex", 'KortexInterface')
        rospy.wait_for_service('/base/send_joint_speeds_command')  # joint velocity control
        rospy.wait_for_service('/base/send_twist_command')  # cartesian velocity control
        rospy.wait_for_service('/base/send_wrench_command')  # torque control
        self.srv_joint_velocity_controller = rospy.ServiceProxy('/base/send_joint_speeds_command',
                                                                kortex_driver.srv.SendJointSpeedsCommand)
        self.srv_twist_controller = rospy.ServiceProxy('/base/send_twist_command', kortex_driver.srv.SendTwistCommand)
        self.srv_wrench_controller = rospy.ServiceProxy('/base/send_wrench_command',
                                                        kortex_driver.srv.SendWrenchCommand)
        self.config = config['gen3_vistac']

    def send_joint_velocity_command(self, joint_speeds):
        speed_req = kortex_driver.srv.SendJointSpeedsCommandRequest()
        # joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        # NOTE: JointSpeed/value is in DEGREES per second.
        # cf. https://github.com/Kinovarobotics/kortex/blob/master/api_cpp/doc/markdown/messages/Base/JointSpeed.md
        for i in range(0, len(joint_speeds)):
            joint_speed = kortex_driver.msg.JointSpeed()
            joint_speed.joint_identifier = i
            joint_speed.value = joint_speeds[i]
            speed_req.input.joint_speeds.append(joint_speed)
        resp = self.srv_joint_velocity_controller.call(speed_req)

    def stop_joint_velocity_control(self):
        # joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6', 'joint_7']
        speed_req = kortex_driver.srv.SendJointSpeedsCommandRequest()
        for i in range(0, 7):
            joint_speed = kortex_driver.msg.JointSpeed()
            joint_speed.joint_identifier = i
            joint_speed.value = 0
            speed_req.input.joint_speeds.append(joint_speed)
        resp = self.srv_joint_velocity_controller.call(speed_req)
        self.srv_joint_velocity_controller.call(speed_req)

    def send_cartesian_twist_command(self, cartesian_twists):
        twist_req = kortex_driver.srv.SendTwistCommandRequest()
        twist_req.input.reference_frame = 0
        twist_req.input.twist.linear_x = cartesian_twists[0]
        twist_req.input.twist.linear_y = cartesian_twists[1]
        twist_req.input.twist.linear_z = cartesian_twists[2]
        twist_req.input.twist.angular_x = cartesian_twists[3]
        twist_req.input.twist.angular_y = cartesian_twists[4]
        twist_req.input.twist.angular_z = cartesian_twists[5]
        twist_req.input.duration = 0
        resp = self.srv_twist_controller.call(twist_req)

    def stop_cartesian_twist_command(self):
        twist_req = kortex_driver.srv.SendTwistCommandRequest()
        twist_req.input.reference_frame = 0
        twist_req.input.twist.linear_x = 0
        twist_req.input.twist.linear_y = 0
        twist_req.input.twist.linear_z = 0
        twist_req.input.twist.angular_x = 0
        twist_req.input.twist.angular_y = 0
        twist_req.input.twist.angular_z = 0
        twist_req.input.duration = 0
        resp = self.srv_twist_controller.call(twist_req)

    def send_cartesian_wrench_command(self, cartesian_wrench):
        wrench_req = kortex_driver.srv.SendWrenchCommand()
        wrench = kortex_driver.msg.Wrench()
        wrench_req.reference_frame = 0  # TODO: Clarify
        wrench_req.mode = 0  # TODO: Clarify
        wrench.force_x = cartesian_wrench[0]
        wrench.force_y = cartesian_wrench[1]
        wrench.force_z = cartesian_wrench[2]
        wrench.torque_x = cartesian_wrench[3]
        wrench.torque_y = cartesian_wrench[4]
        wrench.torque_z = cartesian_wrench[5]
        wrench_req.wrench = wrench
        wrench_req.duration = 0
        resp = self.srv_wrench_controller.call(wrench_req)

    def stop_cartesian_wrench_command(self):
        wrench_req = kortex_driver.srv.SendWrenchCommand()
        wrench = kortex_driver.msg.Wrench()
        wrench_req.reference_frame = 0  # TODO: Clarify
        wrench_req.mode = 0  # TODO: Clarify
        wrench.force_x = 0
        wrench.force_y = 0
        wrench.force_z = 0
        wrench.torque_x = 0
        wrench.torque_y = 0
        wrench.torque_z = 0
        wrench_req.wrench = wrench
        wrench_req.duration = 0
        resp = self.srv_wrench_controller.call(wrench_req)


def yaml_to_pose_msg(yaml_poses):

    pose_msgs = {}

    for robot_name, poses in yaml_poses.items():

        if not robot_name in pose_msgs.values():
            pose_msgs[robot_name] = {}

        for pose_name, pose in poses.items():
            pose_msgs[robot_name][pose_name] = to_pose_msg(pose)

    return pose_msgs

def log_helper(msg, component_name):

    log_string = '({}) : '.format(component_name) + msg
    rospy.loginfo(log_string)


def all_close(goal, actual, tolerance):
      """
      Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
      @param: goal       A list of floats, a Pose or a PoseStamped
      @param: actual     A list of floats, a Pose or a PoseStamped
      @param: tolerance  A float
      @returns: bool
      """
      all_equal = True
      if type(goal) is list:
        for index in range(len(goal)):
          if abs(actual[index] - goal[index]) > tolerance:
            return False

      elif type(goal) is PoseStamped:
        return all_close(goal.pose, actual.pose, tolerance)

      elif type(goal) is Pose:
        return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

      return True


def pose_to_dict(pose):

    pose_dict = {}

    pose_dict['x'] = float(pose.position.x)
    pose_dict['y'] = float(pose.position.y)
    pose_dict['z'] = float(pose.position.z)
    pose_dict['o_x'] = float(pose.orientation.x)
    pose_dict['o_y'] = float(pose.orientation.y)
    pose_dict['o_z'] = float(pose.orientation.z)
    pose_dict['o_w'] = float(pose.orientation.w)

    return pose_dict

def load_yaml_file(file_path):

    with open(file_path, 'r') as f:
        content = yaml.safe_load(f)
    return content

def write_yaml_file(file_path, content):

    with open(file_path, 'w') as f:
        yaml.dump(content, f)


def transform_pose(from_frame, to_frame, pose, tf_listener):
    """
    Will transform the pose msg given in from_frame into to_frame
    """

    # Get Transform from from_frame to to_frame
    trans, rot = tf_listener.lookupTransform(from_frame, to_frame, rospy.Time())
    H_transform = tf_listener.fromTranslationRotation(trans, rot)

    # Convert pose to transform from world to from_frame
    p = [pose.position.x, pose.position.y, pose.position.z]
    r = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    H_pose = tf_listener.fromTranslationRotation(p, r)

    # Create transform from world to to_frame
    H_to_frame = np.matmul(H_pose, H_transform)

    return to_pose_msg(H_to_frame)

def to_pose_msg(pose):

    pose_msg = Pose()

    # Convert from pose in dict to pose_msg
    if isinstance(pose, dict):

        # Translation
        pose_msg.position.x = pose['x']
        pose_msg.position.y = pose['y']
        pose_msg.position.z = pose['z']

        # Rotation
        pose_msg.orientation.x = pose['o_x']
        pose_msg.orientation.y = pose['o_y']
        pose_msg.orientation.z = pose['o_z']
        pose_msg.orientation.w = pose['o_w']

        return pose_msg

    # Convert from 4x4 homogeneous transform to pose_msg
    elif isinstance(pose, np.ndarray):

        # Translation
        pose_msg.position.x = pose[0, 3]
        pose_msg.position.y = pose[1, 3]
        pose_msg.position.z = pose[2, 3]

        ## Rotation
        quat = tf.transformations.quaternion_from_matrix(pose)
        pose_msg.orientation.x = quat[0]
        pose_msg.orientation.y = quat[1]
        pose_msg.orientation.z = quat[2]
        pose_msg.orientation.w = quat[3]

        return pose_msg


class Plan:
    """
    This class serves as an intermediate class between robot_interface and moveit
    """

    def __init__(self, move_group):

        self.logging_name = "PLAN"
        self.poses = []
        self.move_group = move_group
        self.disp_traj_pub = rospy.Publisher('/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=20)
        self.move_group_prev_val = False
        self.move_group_sub = rospy.Subscriber('/execute_trajectory/feedback', ExecuteTrajectoryActionFeedback, self.move_group_status_callback)
        self.move_group_succeeded = True
        self.current_goal_id = None

    def delete(self):

        self.move_group_sub.unregister()
        self.disp_traj_pub.unregister()

    def append(self, pose):

        if isinstance(pose, list):
            self.poses += pose
        else:
            self.poses.append(pose)

    def execute(self, async=False, confirm=True, velocity_scaling=None):

        total_plan, _ = self.add_current_pose(self.poses)

        self.move_group.clear_pose_targets()
        plan, _ = self.plan_trajectory(total_plan, velocity_scaling)
        # plan = self.remove_redundant(plan)
        self._display_plan(plan)
        self.move_group_succeeded = False
        if confirm:
            confirmation = raw_input("Confirm trajectory [y/n]: ")
        else:
            confirmation = 'y'

        if confirmation == 'y':

            wait = True if async == False else False
            log_helper("Moving the robot with wait: {}".format(wait), self.logging_name)
            res = self.move_group.execute(plan, wait=wait)
            log_helper("Move Group returned: {}".format(res), self.logging_name)
            time.sleep(0.5)
            if not wait:
                self.move_group.clear_pose_targets()
                log_helper("Motion completed", self.logging_name)
            return res
        else:
            self.delete()
            self.move_group.clear_pose_targets()
            raise UserDeclinedError()

        return False

    def remove_redundant(self, plan):

        traj = plan.joint_trajectory
        for i in range(len(traj.points)-1):

            t1 = traj.points[i].time_from_start.to_sec()
            t2 = traj.points[i+1].time_from_start.to_sec()

            if t1 == t2:

                new_plan = RobotTrajectory()
                new_plan.joint_trajectory.header = traj.header
                new_plan.joint_trajectory.joint_names = traj.joint_names

                for j in range(len(traj.points)):

                    if j != i:
                        new_plan.joint_trajectory.points.append(traj.points[j])

                assert len(new_plan.joint_trajectory.points) == (len(traj.points) - 1)

                log_helper("Removed reduntant point from trajectory at position {}".format(i), self.logging_name)
                return new_plan

            elif t1 > t2:

                print plan
                raise ValueError("Cannot fix this plan, since t1>t2")

        return plan

    def move_group_status_callback(self, data):

        if data.feedback.state == "MONITOR" and self.current_goal_id is None:
            self.current_goal_id = data.status.goal_id

        if (data.feedback.state == "IDLE" and
            data.status.goal_id.id == self.current_goal_id.id):

            self.current_goal_id = None
            self.move_group_succeeded = True

    def add_current_pose(self, poses):

        first_pose = poses[0]

        if all_close(first_pose, self.move_group.get_current_pose().pose, 0.006):

            log_helper("Current pose not added to plan", self.logging_name)
            return poses, False

        else:
            current_pose = [self.move_group.get_current_pose().pose]
            total_poses = current_pose + poses
            log_helper("Current pose added to plan", self.logging_name)
            return total_poses, True

    def stop_execution(self):
        self.move_group.stop()
        self.move_group.clear_pose_targets()

    def _display_plan(self, plan):
        log_helper("Now displaying trajectory", self.logging_name)
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory.append(plan)
        self.disp_traj_pub.publish(display_trajectory)

    def plan_trajectory(self, waypoints, scaling_factor=None):

        plan, fraction = self.move_group.compute_cartesian_path(waypoints, 0.01, 0.0)
        if scaling_factor:
            # rescale to different velocity profile
            plan = self.move_group.retime_trajectory(self.move_group.get_current_state(), plan, velocity_scaling_factor=scaling_factor)
        return plan, fraction


def get_world(pose, frame, tf_listener):

    t_pose = transform_pose(frame, '/base_link', pose, tf_listener)
    return t_pose


def pose_wf_to_sensor_frame(x, y, z, theta, sensor_frame='contactile_sensor_0', block=False):
    # Has to go to utils file
    pose_msg = Pose()
    br = tf.TransformBroadcaster()
    pose_msg.position.x = x
    pose_msg.position.y = y
    pose_msg.position.z = z
    quat = None
    # TODO: Change to Alexis's sensor
    if sensor_frame == 'contactile_sensor_0':
        # print('Contactile frame detected')
        # Transforms the world pose to sensor pose
        quat = tf.transformations.quaternion_from_euler(-math.pi/2, -theta, 0, 'rxyz')
        pose_msg.orientation.x = quat[0]
        pose_msg.orientation.y = quat[1]
        pose_msg.orientation.z = quat[2]
        pose_msg.orientation.w = quat[3]

    else:
        print('Incorrect sensor frame selection')

    if block:
        while True:
            br.sendTransform((x, y, z), (quat), rospy.Time.now(), "/test_sensor_planning", "/world")

    return pose_msg

