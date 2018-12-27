#! /usr/bin/env python

# Copyright (c) 2011, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#    * Neither the name of the Willow Garage, Inc. nor the names of its
#      contributors may be used to endorse or promote products derived from
#       this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

# 0.471m per revolution
STELLA_MAX_LIN_VEL = 2 # m/s
STELLA_MAX_ANG_VEL = 3.14 # rad/s


LIN_VEL_STEP_SIZE = 0.015
ANG_VEL_STEP_SIZE = 0.1

msg = """
Control Your Stella!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity 
a/d : increase/decrease angular velocity 

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""


def get_key():
	tty.setraw(sys.stdin.fileno())
	rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
	if rlist:
		key = sys.stdin.read(1)
	else:
		key = ''

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
	return key


def vels(target_linear_vel, target_angular_vel):
	return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)


def make_simple_profile(output, input, slop):
	if input > output:
		output = min( input, output + slop )
	elif input < output:
		output = max( input, output - slop )
	else:
		output = input

	return output


def constrain(input, low, high):
	if input < low:
		input = low
	elif input > high:
		input = high
	else:
		input = input
	return input


def check_linear_limit_velocity(vel):
	vel = constrain(vel, -STELLA_MAX_LIN_VEL, STELLA_MAX_LIN_VEL)

	return vel


def check_angular_limit_velocity(vel):
	vel = constrain(vel, -STELLA_MAX_ANG_VEL, STELLA_MAX_ANG_VEL)
	return vel


if __name__=="__main__":
	settings = termios.tcgetattr(sys.stdin)

	rospy.init_node('stella_teleop_node')
	pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
	rate = rospy.Rate(10)
	status = 0
	target_linear_vel   = 0.0
	target_angular_vel  = 0.0
	control_linear_vel  = 0.0
	control_angular_vel = 0.0

	try:
		print msg
		while(1):
			key = get_key()
			if key == 'w' :
				target_linear_vel = check_linear_limit_velocity(target_linear_vel + LIN_VEL_STEP_SIZE)
				status = status + 1
				print vels(target_linear_vel,target_angular_vel)
			elif key == 'x' :
				target_linear_vel = check_linear_limit_velocity(target_linear_vel - LIN_VEL_STEP_SIZE)
				status = status + 1
				print vels(target_linear_vel,target_angular_vel)
			elif key == 'a' :
				target_angular_vel = check_angular_limit_velocity(target_angular_vel + ANG_VEL_STEP_SIZE)
				status = status + 1
				print vels(target_linear_vel,target_angular_vel)
			elif key == 'd' :
				target_angular_vel = check_angular_limit_velocity(target_angular_vel - ANG_VEL_STEP_SIZE)
				status = status + 1
				print vels(target_linear_vel,target_angular_vel)
			elif key == ' ' or key == 's' :
				target_linear_vel   = 0.0
				control_linear_vel  = 0.0
				target_angular_vel  = 0.0
				control_angular_vel = 0.0
				print vels(target_linear_vel, target_angular_vel)
			else:
				if key == '\x03':
					break

			if status == 20 :
				print msg
				status = 0

			twist = Twist()

			control_linear_vel = make_simple_profile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
			twist.linear.x = control_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

			control_angular_vel = make_simple_profile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
			twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = control_angular_vel

			pub.publish(twist)
			rate.sleep()

	except:
		print e

	finally:
		twist = Twist()
		twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
		twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
		pub.publish(twist)

	termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

