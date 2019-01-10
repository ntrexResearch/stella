#!/usr/bin/env python

import math
from math import sin, cos, pi

import rospy
import tf
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from std_msgs.msg import Int32

left_fault_flag = False
right_fault_flag = False


def init_odom():
    global x
    global y
    global th

    global vx
    # global vy
    global vth

    x = 0.0
    y = 0.0
    th = 0.0

    vx = 0.0
    # vy = 0
    vth = 0.0


def on_new_twist(data):
    # Update the current velocity
    global vx
    global vth
    vx = data.linear.x
    vth = data.angular.z


def on_new_fault1(data):
    global left_fault_flag
    if not (data == 0):
        left_fault_flag = True
    elif data == 0:
        left_fault_flag = False


def on_new_fault2(data):
    global right_fault_flag
    if not (data == 0):
        right_fault_flag = True
    elif data == 0:
        right_fault_flag = False


def shutdownhook():
    global ctrl_c
    ctrl_c = True
    # Stop the wheels


if __name__ == "__main__":

    rospy.init_node('mw_odometry_publisher_node')

    global ctrl_c

    odom_pub = rospy.Publisher("odom", Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()

    subscriber_twist = rospy.Subscriber('cmd_vel', Twist, on_new_twist, queue_size=10)

    subscriber_fault1 = rospy.Subscriber('mw/fault1', Int32, on_new_fault1, queue_size=10)
    subscriber_fault2 = rospy.Subscriber('mw/fault2', Int32, on_new_fault2, queue_size=10)

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()

    rate = rospy.Rate(10)

    ctrl_c = False
    # On shutdown action
    rospy.on_shutdown(shutdownhook)

    init_odom()

    while not ctrl_c:
        current_time = rospy.Time.now()
        dt = (current_time - last_time).to_sec()

        if not left_fault_flag or not right_fault_flag:
            delta_x = vx * cos(th) * dt
            delta_y = vx * sin(th) * dt
            delta_th = vth * dt

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
            odom.twist.twist = Twist(Vector3(vx, 0, 0), Vector3(0, 0, vth))

            # publish the message
            odom_pub.publish(odom)

        # Mark the time
        last_time = current_time
        rate.sleep()
