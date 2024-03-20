#!/usr/bin/env python
import serial
import struct
import time
import rospy
import cv2
from sensor_msgs.msg import CompressedImage
from fsr.msg import IntList
import numpy as np
from fsr.srv import BiasRequest, BiasRequestResponse
from collections import deque


class FSRray():
    def __init__(self, width=16, verbose=False):
        self.array_width = width
        self.values = [0] * width * width
        self.dt = [0] * 2
        self.path = '/dev/ttyACM0'
        self.baud = 500000
        self.timeout = 3
        self.verbose = verbose
        self.pub_data = True
        self.pub_image = True
        self.data_received = False
        self.error_count = 0
        self.pub_rate = 50 # in Hz
        self.baseline_values = [0] * width * width
        self.data_buffer = deque(maxlen=10)

        # Set up connection
        self.arduino_comm = serial.Serial(self.path, self.baud, parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
                            bytesize=serial.EIGHTBITS, timeout=3)

        # Initialise ROS node
        rospy.init_node('FSRay_handler', anonymous=True, log_level=rospy.DEBUG)
        rospy.on_shutdown(self.handle_exit)
        self.r = rospy.Rate(self.pub_rate)

        # Set up services
        self.bias_request_srv = rospy.Service('bias_request', BiasRequest, self.handle_bias_request)

        # Set up publishers
        self.image_pub = rospy.Publisher("/FSRray/compressed", CompressedImage, queue_size=2)
        self.data_pub = rospy.Publisher("/FSRray/data", IntList, queue_size=2)

        rospy.loginfo("FSRray handler initialized")
        self.pub_data = True
        self.handle_data()

    def handle_exit(self):
        # Disconnect communication with arduino
        self.arduino_comm.close()

    def handle_bias_request(self, request):
        """
        Handles a Bias Request from the service, collects 10 frames of tactile data and then sets the baseline value
        """
        rospy.loginfo("Bias request received")
        # Stop publishing data temporarily
        self.pub_data = False
        resp = BiasRequestResponse()
        try:
            while len(self.data_buffer) < 10:
                if self.verbose:
                    rospy.loginfo("Sampling FSR data")
                while not self.data_received:
                    rospy.sleep(0.01)
                self.data_buffer.append(self.values)
                self.data_received = False

            self.baseline_values = np.mean(self.data_buffer, axis=0)
            resp.result = True
            rospy.loginfo("Baseline Set")
            self.pub_data = True
            self.data_buffer.clear()
        except Exception as ex:
            rospy.logerr("Baseline Set Failed")
            resp.result = False

        return resp

    def handle_data(self):
        rospy.loginfo("Starting Publishing data")
        time.sleep(5)
        while not rospy.is_shutdown():
            self.arduino_comm.write(bytearray([16]))
            if self.verbose:
                rospy.loginfo("Wrote 1 byte")
            try:
                for i in range(2):
                    self.dt[i] = struct.unpack('<I', self.arduino_comm.read(4))[0]
                for i in range(16 * 16):  # read the n*n values on 2 bytes each
                    self.values[i] = struct.unpack('<H', self.arduino_comm.read(2))[0]

                    if self.verbose:
                        rospy.loginfo("dt[{}] = {}".format(i, self.dt))
                        rospy.loginfo("values[{}] = {}".format(i, max(self.values)))
                self.data_received = True
            except Exception as ex:
                self.error_count += 1
                rospy.loginfo("Issue in FSR communication, trying again")
                rospy.loginfo("Error Count = [{}]".format(self.error_count))
                continue

            # Correct the baseline
            fsr_data = np.array(self.values) - self.baseline_values
            # print('Max Value: ', np.max(fsr_data))
            fsr_image = np.reshape(fsr_data, [16, 16])

            # Create message types
            image_msg = CompressedImage()
            data_msg = IntList()

            # Publish the Integer List for saving in database
            fsr_data = fsr_data.astype('int')
            if self.pub_data:
                data_msg.data = fsr_data.tolist()
                self.data_pub.publish(data_msg)

            # Publish the Image for visualization
            if self.pub_image:
                image_msg.header.stamp = rospy.Time.now()
                image_msg.format = "png"
                image_msg.data = np.array(cv2.imencode('.png', fsr_image)[1]).tostring()
                # print('Max Image: ', max(image_msg.data))
                self.image_pub.publish(image_msg)

            self.r.sleep()


if __name__ == "__main__":
    fsr_handler = FSRray()
