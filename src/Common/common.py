#!/usr/bin/env python
import ROS_Node.ros_common as ros_common
from PyQt5.QtCore import QMutex
from scipy.spatial.transform import Rotation
import numpy as np

class CommonData(): # store the data from the ROS nodes
    def __init__(self):
        self.msg = ""
        self.current_Time = ""
        
        self.current_distance = ""
        self.total_distance = 0

        self.current_imu = ros_common.IMUinfo()
        self.current_global_pos = ros_common.GlobalPositionInfo()
        self.current_local_pos = ros_common.Vector3()
        self.current_vel = ros_common.Vector3()
        self.current_battery_status = ros_common.BatteryInfo()
        self.current_state = ros_common.StateInfo()
        self.current_attitude_target = ros_common.AttitudeTarget()
        self.indoor_mode = False

        # water sampling
        self.encoder_raw = ros_common.Vector3()
        self.payload_pos = ros_common.Vector3()

        self.lock = QMutex()

    def update_imu(self, x, y, z, w):
        euler = self.quat_to_euler(x, y, z, w)
    
        if not self.lock.tryLock():
            return
        self.current_imu.roll = euler[0]
        self.current_imu.pitch = euler[1]
        self.current_imu.yaw = euler[2]
        self.lock.unlock()
        return
  
    def update_global_pos(self, latitude, longitude, altitude):
        if not self.lock.tryLock():
            return
        self.current_global_pos.latitude = latitude
        self.current_global_pos.longitude = longitude
        self.current_global_pos.altitude = altitude
        self.lock.unlock()
        return

    def update_local_pos(self, x, y, z):
        if not self.lock.tryLock():
            return
        self.current_local_pos.x = x
        self.current_local_pos.y = y
        self.current_local_pos.z = z
        self.lock.unlock()
        return
    
    def update_vel(self, vx, vy, vz):
        if not self.lock.tryLock():
            return
        self.current_vel.x = vx
        self.current_vel.y = vy
        self.current_vel.z = vz
        self.lock.unlock()
        return
    
    def update_bat(self, percentage, voltage):
        if not self.lock.tryLock():
            return
        self.current_battery_status.percentage = percentage
        self.current_battery_status.voltage = voltage
        self.lock.unlock()
        return
    

    def update_state(self, connected, armed, manual_input, mode, seconds):
        if not self.lock.tryLock():
            return
        self.current_state.connected = connected
        self.current_state.armed = armed
        self.current_state.manual_input = manual_input
        self.current_state.mode = mode
        self.current_state.seconds = seconds
        self.lock.unlock()
        return
    
    def update_attitude_target(self, x, y, z, w, thrust):
        euler = self.quat_to_euler(x, y, z, w)
        if not self.lock.tryLock():
            return
        self.current_attitude_target.roll = euler[0]
        self.current_attitude_target.pitch = euler[1]
        self.current_attitude_target.yaw = euler[2]
        self.current_attitude_target.thrust = thrust
        self.lock.unlock()
        return
    
    def update_body_rate_target(self, x, y, z, thrust):
        if not self.lock.tryLock():
            return
        euler_rate = self.body_rate_to_euler_rate(x, y, z)
        self.current_attitude_target.roll = euler_rate[0]
        self.current_attitude_target.pitch = euler_rate[1]
        self.current_attitude_target.yaw = euler_rate[2]
        self.current_attitude_target.thrust = thrust
        self.lock.unlock()
        return
    
    def quat_to_euler(self, x, y, z, w):
        # guard from NaN or 0 quaternion
        if x == 0 and y == 0 and z == 0 and w == 0:
            return [0, 0, 0]
        
        r = Rotation.from_quat([x, y, z, w])
        euler = r.as_euler('xyz', degrees=True)

        # convert to 360 coordinates
        if euler[2] < 0:
            euler[2] = euler[2] + 360

        return euler
    
    def body_rate_to_euler_rate(self, x, y, z):
        phi = self.current_imu.roll
        theta = self.current_imu.pitch

        # if theta is 90 or -90, the transformation matrix will have a division by 0, return 0 in this case
        if abs(np.cos(theta)) < 1e-8:
            return [0, 0, 0]

        transformation_matrix = np.array([
            [1, np.sin(phi) * np.tan(theta), np.cos(phi) * np.tan(theta)],
            [0, np.cos(phi), -np.sin(phi)],
            [0, np.sin(phi) / np.cos(theta), np.cos(phi) / np.cos(theta)]
        ])
        body_rate = np.array([x, y, z])
        return np.dot(transformation_matrix, body_rate)
        
    
    def update_estimator_type(self, indoor_mode):
        if not self.lock.tryLock():
            return
        self.indoor_mode = indoor_mode
        self.lock.unlock()
        return
    
    ## water sampling tab

    def update_encoder_raw(self, x, y, z):
        if not self.lock.tryLock():
            return
        self.encoder_raw.x = x
        self.encoder_raw.y = y
        self.encoder_raw.z = z
        self.lock.unlock()
        return
    
    def update_payload_pos(self, x, y, z):
        if not self.lock.tryLock():
            return
        self.payload_pos.x = x
        self.payload_pos.y = y
        self.payload_pos.z = z
        self.lock.unlock()
        return
        
    

    
    


