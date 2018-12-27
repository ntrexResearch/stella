#! /usr/bin/python

#################################################################################
# Copyright 2018 NTREX CO.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#################################################################################

# Authors: Ji #

from mw_serial import IMUSerialThread
import rospy
import threading

from sensor_msgs.msg import Imu


def init_imu():
    global imu_data
    imu_data.header.frame_id = "imu_link"
    imu_data.orientation.x = 0
    imu_data.orientation.y = 0
    imu_data.orientation.z = 0

    imu_data.orientation_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]

    imu_data.angular_velocity.x = 0
    imu_data.angular_velocity.y = 0
    imu_data.angular_velocity.z = 0

    imu_data.angular_velocity_covariance = [0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02]

    imu_data.linear_acceleration.x = 0
    imu_data.linear_acceleration.y = 0
    imu_data.linear_acceleration.z = 0

    imu_data.linear_acceleration_covariance = [0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04]


def shutdownhook():
    global ctrl_c
    global serialThread
    ctrl_c = True
    serialThread.join()

if __name__ == "__main__":
    rospy.init_node("stella_imu_node")
    imu_data = Imu()
    init_imu()
    port = rospy.get_param("~serial_dev")
    rate = rospy.Rate(10)  # 10Hz
    rospy.on_shutdown(shutdownhook)

    publisher_mw_imu = rospy.Publisher("/imu", Imu, queue_size=5)

    serialThread = IMUSerialThread(2, "serialThread-2", imu_data, port)

    lock = threading.Lock()
    ctrl_c = False

    serialThread.start()
    while not ctrl_c:
        lock.acquire()
        try:
            publisher_mw_imu.publish(imu_data)

        finally:
            lock.release()
        rate.sleep()


