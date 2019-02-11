#! /usr/bin/python
#-*- coding: utf-8 -*-
# 한글 입력을 위한 엔코딩 명시 
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

import rospy
import Queue
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32, Int16
from mw_serial import MDSerialThread,  queue_handler
from math import sin, cos
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from definition import *

# 로봇 구동부에 관한 정보를 기재한다. 
wheel_to_wheel_d = 0.2835 # 바퀴와 바퀴 간 거리  [m]
distance_per_rev = 0.471 # 한바퀴 회전시 이동 거리 [m / rev]
pulse_per_rev = 14336 # 한바퀴 회전시 엔코더 펄스 카운트(이 값은 부착된 엔코더와 감속기를 고려해 정해진다.) [pulse / rev] 
pulse_per_distance = 30437.367 # 1m 이동시 엔코더 펄스 카운트 [pulse / m]
gear_ratio = 14 # 감속비
motor_scale_const = 1783.4395 # gear_ratio * 60 / distance_per_rev


fault_flag = False

command_list = [0x67, 0x7D]
index = 0


def init_odom():
    """
    이 함수는 ros topic odom 에 대한 정보를 초기화한다.
    :return: None
    """
    global x
    global y
    global th

    global vs
    # global vy
    global vth
    global left_velocity
    global right_velocity
    global left_encoder
    global right_encoder

    x = 0.0
    y = 0.0
    th = 0.0

    vs = 0.0
    # vy = 0
    vth = 0.0
    left_velocity = 0
    right_velocity = 0
    left_encoder = 0
    right_encoder = 0


def calculate_wheel_vel(linear, angular):
    """
    이 함수는 cmd_vel topic을 받아서 양쪽 바퀴의 속도를 계산한다.
    :param linear: 직진 속도를 의미한다.
    :param angular: 회전 속도를 의미한다.
    :return scaled_r_speed, scaled_l_speed: Moonwalker 모터드라이버에 필요한 왼쪽 오른쪽 바퀴의 속도 [rpm]를 리턴한다.
    """
    right_speed = linear + wheel_to_wheel_d / 2.0 * angular
    left_speed = linear - wheel_to_wheel_d / 2.0 * angular
    # Unit conversion to rpm
    scaled_right_speed = right_speed * motor_scale_const
    scaled_left_speed = left_speed * motor_scale_const
    return scaled_right_speed, scaled_left_speed


def make_text_command(command_type, arg1=0.0, arg2=0.0):
    """
    이 함수를 사용해 Moonwalker 모터드라이버에 들어가는 text 기반 명령어를 만든다.
    :param command_type: 모터드라이버를 위한 명령어의 종류를 스트링으로 받는다.
    :param arg1: dual channel중 channel 1에 해당하는 명령값을 받는다.
    :param arg2: dual channel중 channel 2에 해당하는 명령값을 받는다.
    :return: 문자열로 된 모터드라이버 명령어
    """
    if command_type == 'Control Velocity':
        return 'mvc=%0.3f,%0.3f\r\n' % (arg1, arg2)
    elif command_type == 'System Command':
        return 'co1=%d;co2=%d\r\n' % (arg1, arg2)
    elif command_type == 'Poll Status':
        return 'mpf\r\n'


def on_new_twist(data):
    """
    이 함수는 cmd_vel topic callback 함수이다.
    명령을 수신하면 Moonwalker 모터드라이버에 맞게 속도값이 조정되어 시리얼 전송 queue에 들어간다.
    :param data: cmd_vel topic 이 data로 수신된다.
    :return: None
    """
    global fault_flag
    global remote_tx_queue
    global vs
    global vth
    vs = data.linear.x
    vth = data.angular.z
    if not fault_flag:
        right_speed, left_speed = calculate_wheel_vel(data.linear.x, data.angular.z)
        remote_tx_queue.put(make_text_command('Control Velocity', left_speed, right_speed))
    else:
        remote_tx_queue.put(make_text_command('Control Velocity', 0.0, 0.0))


def on_new_cmd(data):
    """
    이 함수는 mw/command topic callback 함수이다.
    command 함수는 다음과 같은 기능을 수행한다.
    -- command 가 0이면 Motor control을 서보 오프시킨다.
    -- command 가 1이면 Motor control을 서보 온시킨다.
    -- command 가 2이면 Motor control에 발생한 fault를 clear한다.
    :param data: command topic 데이터이다.
    :return: None
    """
    global remote_tx_queue
    remote_tx_queue.put(make_text_command('System Command', data.data, data.data))


def shutdownhook():
    """
    이 함수는 모터 드라이버 노드 종료시 불려지는 함수이다.
    :return: None
    """
    global ctrl_c
    global remote_tx_queue
    global serialThread
    ctrl_c = True
    # Stop the wheels
    remote_tx_queue.put(make_text_command('Control Velocity', 0.0, 0.0))
    # Exit the serial thread
    serialThread.join()


if __name__ == "__main__":
    rospy.init_node("mw_drive_node") 
    tx_queue = Queue.Queue()
    remote_tx_queue = Queue.Queue()
    rx_queue = Queue.Queue()

    port = rospy.get_param("~serial_dev")
    serialThread = MDSerialThread(1, "Motor driver serial thread", remote_tx_queue, tx_queue, rx_queue, port)
    rate = rospy.Rate(20)

    publisher_mw_fault1 = rospy.Publisher("/mw/fault1", Int32, queue_size=10)
    publisher_mw_fault2 = rospy.Publisher("/mw/fault2", Int32, queue_size=10)
    subscriber_cmd = rospy.Subscriber("mw/command", Int32, on_new_cmd, queue_size=10)
    subscriber_twist = rospy.Subscriber("cmd_vel", Twist, on_new_twist, queue_size=10)
    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    init_odom()

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    ctrl_c = False
    left_encoder_prev = 0
    right_encoder_prev = 0
    vs = 0
    vth = 0
    serialThread.start()
    rospy.on_shutdown(shutdownhook)
    while not ctrl_c:
        current_time = rospy.Time.now()
        if tx_queue.empty():
            tx_queue.put(make_text_command('Poll Status'))
        while not rx_queue.empty():
            rx_message = queue_handler(rx_queue, False)
            message_type = rx_message[0]
            if message_type == "mpf":
                fault = int(rx_message[3])
                if fault != 0:
                    fault_flag = True
                    break
                else:
                    fault_flag = False
                    publisher_mw_fault1.publish(fault_flag & 0xff)
                    publisher_mw_fault2.publish((fault_flag >> 8) & 0xff)
                left_encoder = int(rx_message[1])
                right_encoder = int(rx_message[2])
                # Update odometry
                delta_left = left_encoder - left_encoder_prev
                delta_right = right_encoder - right_encoder_prev

                delta_s = (delta_left + delta_right) / 2.0 / pulse_per_distance
                delta_th = (delta_right - delta_left) / wheel_to_wheel_d / pulse_per_distance
                delta_x = delta_s * cos(th + delta_th / 2.0)
                delta_y = delta_s * sin(th + delta_th / 2.0)
                current_time = rospy.Time.now()
                x += delta_x
                y += delta_y
                th += delta_th
                odom_quat = tf.transformations.quaternion_from_euler(0, 0, th)

                odom_broadcaster.sendTransform(
                    (x, y, 0),
                    odom_quat,
                    current_time,
                    "base_footprint",
                    "odom"
                )

                odom = Odometry()
                odom.header.frame_id = "odom"
                odom.child_frame_id = "base_footprint"
                odom.header.stamp = current_time
                # set the position
                odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))
                # set the velocity
                odom.twist.twist = Twist(Vector3(vs, 0, 0), Vector3(0, 0, vth))
                # publish the message
                odom_pub.publish(odom)
                left_encoder_prev = left_encoder
                right_encoder_prev = right_encoder
        rate.sleep()
