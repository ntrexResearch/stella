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

import rospy
import Queue

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32, Int32, Int16

from mw_serial import MDSerialThread,  queue_handler
from math import sin, cos
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3


# Constant
wheel_to_wheel_d = 0.2835 #0.29 # unit m
distance_per_rev = 0.471 #unit m / 1 revolution
pulse_per_rev = 14336 #unit pulse/1 revolution
pulse_per_distance = 30437.367 # pulse_per_rev / distance_per_rev ~ 30437.3673,,,
gear_ratio = 14
motor_scale_const = 1783.4395 #gear_ratio * 60 / distance_per_rev

fault_flag = False

command_list = [0x67, 0x7D]
index = 0

def init_odom():
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

#
# This function calculates the left and right velocity in the unit of rpm.
#
def calculate_wheel_vel(linear, angular):
    # angular = (right - left) / wheel_to_wheel_d, which is the robot width.
    # linear = (right + left) / 2
    right_speed = linear + wheel_to_wheel_d / 2.0 * angular
    left_speed = linear - wheel_to_wheel_d / 2.0 * angular

    # Unit conversion to rpm
    scaled_right_speed = right_speed * motor_scale_const
    scaled_left_speed = left_speed * motor_scale_const

    return scaled_right_speed, scaled_left_speed


def on_new_twist(data):
    global fault_flag
    global remote_tx_queue
    global vs
    global vth
    vs = data.linear.x
    vth = data.angular.z
    if not fault_flag:
        right_speed, left_speed = calculate_wheel_vel(data.linear.x, data.angular.z)
        #print(right_speed)
        #print(left_speed)
        remote_tx_queue.put(['velocity_command', 'WriteRequest', 'Float32', left_speed, 1])
        remote_tx_queue.put(['velocity_command', 'WriteRequest', 'Float32', right_speed, 2])
    else:
        # On fault, stop the robot
        remote_tx_queue.put(['velocity_command', 'WriteRequest', 'Float32', 0, 2])
        remote_tx_queue.put(['velocity_command', 'WriteRequest', 'Float32', 0, 1])


def on_new_cmd(data):
    global remote_tx_queue
    remote_tx_queue.put(['command', 'WriteRequest', 'Int16', data.data, 1])
    remote_tx_queue.put(['command', 'WriteRequest', 'Int16', data.data, 2])


def shutdownhook():
    global ctrl_c
    global remote_tx_queue
    global serialThread
    ctrl_c = True
    # Stop the wheels 
    remote_tx_queue.put(['velocity_command', 'WriteRequest', 'Float32', 0, 1])
    remote_tx_queue.put(['velocity_command', 'WriteRequest', 'Float32', 0, 2])
    serialThread.join()


if __name__ == "__main__":
    rospy.init_node("mw_drive_node") 

    port = rospy.get_param("~serial_dev")
    tx_queue = Queue.Queue()
    remote_tx_queue = Queue.Queue()
    rx_queue = Queue.Queue()
    serialThread = MDSerialThread(1, "serialThread-1", remote_tx_queue,tx_queue, rx_queue, port)
    rate = rospy.Rate(50)
    # On shutdown stop the motor and close the serial port
    rospy.on_shutdown(shutdownhook)

    #publisher_mw_encoder1 = rospy.Publisher("/mw/encoder1", Int32, queue_size=10)
    publisher_mw_fault1 = rospy.Publisher("/mw/fault1", Int32, queue_size=10)
    #publisher_mw_encoder2 = rospy.Publisher("/mw/encoder2", Int32, queue_size=10)
    publisher_mw_fault2 = rospy.Publisher("/mw/fault2", Int32, queue_size=10)

    # publisher_mw_velocity1 = rospy.Publisher("/mw/left_velocity", Float32, queue_size=10)
    # publisher_mw_velocity2 = rospy.Publisher("/mw/right_velocity", Float32, queue_size=10)

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    init_odom()

    subscriber_cmd = rospy.Subscriber("mw/command", Int32, on_new_cmd, queue_size=10)

    subscriber_twist = rospy.Subscriber("cmd_vel", Twist, on_new_twist, queue_size=10)
    right_fault_flag = False
    left_fault_flag = False
    right_wheel_flag = False
    left_wheel_flag = False
    ctrl_c = False
    serialThread.start()
    left_encoder_prev = 0
    right_encoder_prev = 0
    vs = 0
    vth = 0
    while not ctrl_c:
        current_time = rospy.Time.now()
        # dt = (current_time - last_time).to_sec()
        #print(dt)
        # Here, first check the status of the robot
        if not rx_queue.empty():
            # print("What")
            while not rx_queue.empty():
                rx_message = queue_handler(rx_queue, False)
                message_type = rx_message[0]
                #
                if message_type == 112:
                    # Publish the wheel velocity
                    if rx_message[1] == 1:
                        actual_left_velocity = rx_message[2]
                        # publisher_mw_velocity1.publish(rx_message[2])
                    elif rx_message[2] == 2:
                        actual_right_velocity = rx_message[2]
                        # publisher_mw_velocity2.publish(rx_message[2])
                elif message_type == 103:
                    #	print("RX: Fault message received with motor id %d and speed %d "%(rx_message[1], rx_message[2]))

                    if rx_message[1] == 1:
                        publisher_mw_fault1.publish(rx_message[2])
                        if rx_message[2] == 1:
                            left_fault_flag = True
                        else:
                            left_fault_flag = False
                    elif rx_message[1] == 2:
                        publisher_mw_fault2.publish(rx_message[2])
                        if rx_message[2] == 1:
                            right_fault_flag = True
                        else:
                            right_fault_flag = False
                    if right_fault_flag == True or left_fault_flag == True:
                        fault_flag = True
                    else:
                        fault_flag = False
                elif message_type == 125:
                    #print("RX: Position message received with motor id %d and speed %d "%(rx_message[1], rx_message[2]))
                    if rx_message[1] == 1:
                        left_encoder = rx_message[2]
                        left_wheel_flag = True
                        #print('reasafdasf;')
                        #publisher_mw_encoder1.publish(rx_message[2])
                    elif rx_message[1] == 2:
                        right_encoder = rx_message[2]
                        right_wheel_flag = True
                        # publisher_mw_encoder2.publish(rx_message[2])
        if right_wheel_flag and left_wheel_flag:
            
            # Update odometry
            delta_left = left_encoder - left_encoder_prev
            delta_right = right_encoder - right_encoder_prev
        
            delta_s = (delta_left + delta_right) / 2.0 / pulse_per_distance
            delta_th = (delta_right - delta_left) / wheel_to_wheel_d / pulse_per_distance
            delta_x = delta_s * cos(th + delta_th / 2.0)  # vx * cos(th) * dt
            delta_y = delta_s * sin(th + delta_th / 2.0)  # vx * sin(th) * dt
            
            current_time = rospy.Time.now()
            step_time = (current_time - last_time).to_sec()
            #print("Print ", step_time)
            #velocity_l = delta_left / step_time / pulse_per_distance
            #velocity_r = delta_right / step_time / pulse_per_distance
            #vs = (velocity_l + velocity_r) / 2.0
            #print(velocity_l)
            #print(velocity_r)
            #print("vs = ",vs)
            #vth = (velocity_r - velocity_l) / wheel_to_wheel_d * 2.0
            #print("vth = ",vth)
            ## delta_th = vth * dt
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

            odom.header.stamp = current_time
            odom.header.frame_id = "odom"

            # set the position
            odom.pose.pose = Pose(Point(x, y, 0), Quaternion(*odom_quat))

            # set the velocity
            odom.child_frame_id = "base_footprint"
            odom.twist.twist = Twist(Vector3(vs, 0, 0), Vector3(0, 0, vth))

            # publish the message
            odom_pub.publish(odom)
            last_time = current_time
            left_encoder_prev = left_encoder
            right_encoder_prev = right_encoder
            right_wheel_flag = False
            left_wheel_flag = False
        if tx_queue.empty():
            # print(tx_queue.empty())
            index+=1
            index%=2
            #print("Working",index)
            tx_queue.put(['dual','ReadRequest','Int32', command_list[index], 0])
        # print(tx_queue.empty())
        rate.sleep()
