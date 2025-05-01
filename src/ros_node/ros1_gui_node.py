import rospy
from PyQt5.QtCore import QObject, pyqtSignal, QThread
from PyQt5.QtCore import Qt
from PyQt5.QtWidgets import QMessageBox
from PyQt5.QtCore import QStringListModel
from PyQt5 import QtWidgets
from ros_node import *

ROS_FREQ = 30 # Hz, enough for display and control

"""
This is the ros node class definiting all publishers,
subscribers and Qt singals.
"""

class RosMotorStandControlNode(QObject):
    # define signals
    update_data = pyqtSignal(int)
    def __init__(self):
        super().__init__()
        # # define subscribers
        # self.ESC_RPM_sub = rospy.Subscriber('/kingfisher/betaflight/rpm', SensorRpm, callback=self.esc_rpm_sub)
        # # publisher to arm
        # self.arm_pub = rospy.Publisher('/kingfisher/cybros_autopilot/arm', Bool, queue_size=10)
        # self.throttle_pub = rospy.Publisher('/kingfisher/cybros_autopilot/feedthrough_command', QuadState, queue_size=10)
        # # publish to final data output
        # self.res_pub =  rospy.Publisher('/motor_stand/result', TwistStamped, queue_size=10)
        # # the message to the flight control unit
        # self.motor_msg = QuadState()
        # self.esc_rpm = SensorRpm()
        # # other
        # self.rate = rospy.Rate(ROS_FREQ)
        
        # self.throttle_cmd = 0

        # self.res = TwistStamped()
        # self.is_publishing = False
    
    # ### define signal connections to / from gui ###100
    # def connect_update_gui(self, callback):
    #     self.update_data.connect(callback)

    ### define callback functions from ros topics ###
    def esc_rpm_sub(self, msg):
        # get orientation and convert to euler angles
        self.esc_rpm = msg
    
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