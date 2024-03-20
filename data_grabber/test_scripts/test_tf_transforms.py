import roslib
import rospy
import tf
import math
import numpy as np

if __name__ == '__main__':
    rospy.init_node('fixed_tf_broadcaster')
    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)
    x = -0.211
    y = -0.367
    z = 0.135

    while not rospy.is_shutdown():
        # Test Xela Planning frame
        quat = tf.transformations.quaternion_from_euler(-math.pi, 0, 0, 'rxyz')
        br.sendTransform((x,y,z),
                         (quat),
                         rospy.Time.now(),
                         "/test_tool_frame",
                         "/base_link")
        rate.sleep()

        '''
        # Test Contactile Planning frame
        quat_1 = tf.transformations.quaternion_from_euler(math.pi, 0, math.pi/2, 'rxyz')
        quat_2 = tf.transformations.quaternion_from_euler(0, 0, (math.pi-theta), 'rxyz')
        quat_final = tf.transformations.quaternion_multiply(quat_1, quat_2)
        br.sendTransform((x,y,z),
                         (quat_final),
                         rospy.Time.now(),
                         "/test_contactile_planning",
                         "/world")
        rate.sleep()
        '''