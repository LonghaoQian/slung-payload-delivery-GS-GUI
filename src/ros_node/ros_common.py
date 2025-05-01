"""
Ros Parameter Definition

"""

from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Int32, Float32

ROS_FREQ = 30 # Hz, enough for display and control

class TopicInfo:
    def __init__(self, topic, data_type):
        self.topic = topic
        self.data_type = data_type

# a list of topics used in this GUI:
GroundStationRosTopics = {
    "esp_encoder" : TopicInfo('/encoder/encoder_raw', TwistStamped),
    "stepper_motor_mode_select" :  TopicInfo('stepper/mode_select', Int32),
    "stepper_vel_direct": TopicInfo('stepper/vel_direct', Float32),
    "stepper_pos": TopicInfo('stepper/pos', Float32),
    "stepper_vel_auto": TopicInfo('stepper/vel_auto', Float32),
}

class IMUinfo:
    def __init__(self, ax=0, ay=0, az=0) -> None:
        self.ax = ax
        self.ay = ay
        self.az = az