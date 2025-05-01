"""

MIT License

Copyright (c) 2025 Flight Systems and Control Laboratory

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

This is the ROS 1 node class definiting all publishers,
subscribers and Qt singals.
"""

import rospy
from PyQt5.QtCore import QObject, pyqtSignal, QThread
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QStringListModel
from PyQt5 import QtWidgets
from ros_node import *

class RosMotorStandControlNode(QObject):
    # signals used in Qt to do synchronization
    update_data = pyqtSignal(int)
    def __init__(self):
        super().__init__()
        # subscribers
        self.esp_encoder_sub = rospy.Subscriber(GroundStationRosTopics["esp_encoder"].topic,
                                                GroundStationRosTopics["esp_encoder"].data_type,
                                                callback=self.esp_msg_sub)
        # publishers
        self.stepper_motor_mod_pub = rospy.Publisher(GroundStationRosTopics["stepper_motor_mode_select"].topic,
                                                      GroundStationRosTopics["stepper_motor_mode_select"].data_type,
                                                      queue_size=10)
    
        self.stepper_vel_direct_pub = rospy.Publisher(GroundStationRosTopics["stepper_vel_direct"].topic,
                                                      GroundStationRosTopics["stepper_vel_direct"].data_type,
                                                      queue_size=10)

        self.stepper_pos_pub = rospy.Publisher(GroundStationRosTopics["stepper_pos"].topic,
                                               GroundStationRosTopics["stepper_pos"].data_type,
                                               queue_size=10)

        self.stepper_pos_pub = rospy.Publisher(GroundStationRosTopics["stepper_vel_auto"].topic,
                                               GroundStationRosTopics["stepper_vel_auto"].data_type,
                                               queue_size=10)

        # ros setup
        self.rate = rospy.Rate(ROS_FREQ)
        
        # local variable setup
        self.angleX = 0.0
        self.angleY = 0.0
        self.cable_len = 0.0
        self.angleX_vel = 0.0
        self.angleY_vel = 0.0
        self.cable_vel = 0.0

        # self.res = TwistStamped()
        # self.is_publishing = False
    
    # ### define signal connections to / from gui ###100
    # def connect_update_gui(self, callback):
    #     self.update_data.connect(callback)

    ### define callback functions from ros topics ###
    def esp_msg_sub(self, msg):
        # get orientation and convert to euler angles
        self.angleX = msg.twist.linear.x
        self.angleY = msg.twist.linear.y
        self.cable_len = msg.twist.linear.z
        self.angleX_vel = msg.twist.angular.x
        self.angleY_vel = msg.twist.angular.y
        self.cable_vel = msg.twist.angular.z

    def publish_motor_mode(self, mode):
        pass
        # for i in range(4):
        #     self.motor_msg.motors[i] = throttle
        # self.throttle_pub.publish(self.motor_msg)
    
    # main loop of ros node
    def run(self):
        while not rospy.is_shutdown():
            try:
                self.update_data.emit(0) # notify Qt in every iteration
                self.rate.sleep()
            except rospy.ROSInterruptException:
                print("ROS Shutdown Requested")
                break