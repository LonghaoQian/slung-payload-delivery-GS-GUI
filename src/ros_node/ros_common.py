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

This file defines parameters and ros msg structures

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