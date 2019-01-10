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


import serial
import struct
import threading
from tf.transformations import quaternion_from_euler
import rospy
import sys
from definition import *
import re

imu_lock = threading.Lock()
md_lock = threading.Lock()


class MDSerialThread(threading.Thread):
    def __init__(self, thread_id, name, remote_tx_queue, tx_queue, rx_queue, port):
        threading.Thread.__init__(self)
        self.thread_id = thread_id
        self.name = name
        self.tx_queue = tx_queue
        self.remote_tx_queue = remote_tx_queue
        self.rx_queue = rx_queue
        self.serial = serial.Serial(port=port, baudrate=115200)
        self._is_running = False
        self._stopevent = threading.Event()
        self._sleepperiod = 1.0

    def run(self):
        rospy.loginfo("Motor driver serial thread started")
        self._is_running = True
        md_thread_func(self, self.name, self.serial, self.remote_tx_queue, self.tx_queue, self.rx_queue)
        # On exit
        self.serial.close()
        rospy.loginfo("Motor driver serial thread exited")

    def stop(self):
        self._is_running = False
        sys.exit(0)
        
    def join(self, timeout=None):
        self._stopevent.set()
        threading.Thread.join(self, timeout)



class IMUSerialThread(threading.Thread):
    def __init__(self, thread_id, name, imu_data, port):
        threading.Thread.__init__(self)
        self.thread_id = thread_id
        self.name = name
        self.imu_data = imu_data
        self.serial = serial.Serial(port=port, baudrate=115200)
        self._stopevent = threading.Event()
        self._sleepperiod = 1.0

    def run(self):  
        rospy.loginfo("IMU serial thread started")
        imu_thread_func(self, self.name, self.serial, self.imu_data)
        # On exit
        self.serial.close()
        rospy.loginfo("IMU serial thread exited")

    def join(self, timeout=None):
        self._stopevent.set()
        threading.Thread.join(self, timeout)


def checksum(data_list):
    return struct.pack('B', (sum(data_list) & 0xFF))


def encode_cmd(cmd_type, cmd_code, cmd_object, value, subindex=0x01):
    cmd = AccessCode_map[cmd_code] | Object_map[cmd_object]
    # Depending on the cmd_object type, encode the value differently
    # Append pads to match the packet structure
    if cmd_object == 'Int8':
        value_bytes = struct.pack('c', chr(value & 0xFF)) + b'\x00\x00\x00'
    elif cmd_object == 'Int16':
        value_bytes = struct.pack('h', value & 0xFFFF) + b'\x00\x00'
    elif cmd_object == 'Int32' or cmd_object == 'DoubleInt32':
        value_bytes = struct.pack('i', value)
    elif cmd_object == 'Float32':

        value_bytes = struct.pack('f', value)
    else:
        value_bytes = b'\x00\x00\x00\x00'

    cmd_array = DEVICE_ID + struct.pack('c', chr(cmd & 0xFF)) + \
                struct.pack('hc', MD_command_map[cmd_type], chr(subindex & 0xFF)) + value_bytes

    # Compute the checksum:
    cs = checksum(bytearray(cmd_array))
    serial_msg = STX + b'\x09' + cmd_array + cs + ETX
    return serial_msg


def verify_msg(msg_bytes):
    if msg_bytes[0] != 2:
        print("Received message is not starting with STX byte")
        return False
    if msg_bytes[1] != 9:
        print("Received message's length is not matched")
        return False
    if msg_bytes[2] != 1:
        print("Device ID is not matched")
        return False
    if msg_bytes[3] & 0xf0 == 0x80:
        print("Communication Error is detected")
        return False
    cs = checksum(msg_bytes[2:11])
    if msg_bytes[11] != ord(cs):
        print("Checksum is not matched")
        return False
    if msg_bytes[12] != 3:
        print("Received message is not ending with ETX byte")
        return False
    return True


# This function decodes the message received from the device
def decode_msg(msg_bytes):
    # In binary format
    cmd_code = msg_bytes[3] & 0xF0
    cmd_object = msg_bytes[3] & 0x0F
    temp_cmd_info = struct.unpack('HB', msg_bytes[4:7])
    cmd_type = temp_cmd_info[0]
    subindex = temp_cmd_info[1]

    # Check the command code
    if cmd_code == 0x80:
        # Command code is the error response
        error_code = struct.unpack('h', msg_bytes[4:5])
        print("The following error is detected {}".format(error_code))
        return b'\x80' + error_code

    elif cmd_code == 0xF0:
        # Command code is the sync response specifically for the IMU sensor
        # Decode the value based on the object type
        value_list = (b'\xF0',) + struct.unpack('c', chr(msg_bytes[4] & 0xFF)) + struct.unpack('hhh', msg_bytes[5:11])

        return value_list

    elif cmd_code == 0x20 or cmd_code == 0x40:
        # Command code is either ReadResponse, WriteResponse, or ErrorResponse
        # Leave a log msg
        # Decode the value based on the object type
        if cmd_object == 0x00:
            value = struct.unpack('c', msg_bytes[7])
        elif cmd_object == 0x04:
            value = struct.unpack('h', msg_bytes[7:9])
        elif cmd_object == 0x08:
            value = struct.unpack('i', msg_bytes[7:11])
        elif cmd_object == 0x0C:
            value = struct.unpack('f', msg_bytes[7:11])
        else:
            value = 0x00

        #print("The rx msg's command type is {} and the value is {}".format(cmd_type, value[0]))
        ret_list = (cmd_type, subindex,) + value + struct.unpack('c', chr(cmd_code & 0xFF))
        # print(ret_list)
        return ret_list

    else:
        # Unknown format of the received message
        return b'\x00'


def imu_thread_func(thread, thread_name, _serial, imu_data):
    try:
        st_flag = False
        rx_cnt = 0
        rx_msg = []
        while not thread._stopevent.isSet():
            # If the serial is not opened, break the loop    
            if not _serial.isOpen():
                break

            while  _serial.inWaiting() > 0:
                # Start reading
                data = _serial.read(1)
                if data == b'\x02':
                    st_flag = True
                if st_flag:
                    rx_cnt = rx_cnt + 1
                    rx_msg.append(ord(data))
                    
                    if rx_cnt == 13:
                        rx_msg = bytearray(rx_msg)
                        if not verify_msg(rx_msg):
                            print("Rx failed for the following message")
                            print(''.join('{:02x}'.format(x) for x in rx_msg))
                            _serial.reset_input_buffer()
                        else:
                            data = decode_msg(rx_msg)
                            data_type = ord(data[1])
                            # Lock the resource
                            imu_lock.acquire()
                            try:
                                if data_type == 0x35:  # AHRS_command_map['euler_data']:
                                    # Update the euler angle data
                                    quat = quaternion_from_euler(data[2] / 100.0, data[3] / 100.0, data[4] / 100.0)
                                    imu_data.orientation.x = quat[0]
                                    imu_data.orientation.y = quat[1]
                                    imu_data.orientation.z = quat[2]
                                    imu_data.orientation.w = quat[3]

                                elif data_type == 0x34:  # AHRS_command_map['ang_vel_data']:
                                    # Update the angular velocity data
                                    imu_data.angular_velocity.x = data[2] / 10.0
                                    imu_data.angular_velocity.y = data[3] / 10.0
                                    imu_data.angular_velocity.z = data[4] / 10.0

                                elif data_type == 0x33:  # AHRS_command_map['accel_data']:
                                    # Update the linear acceleration data

                                    imu_data.linear_acceleration.x = data[2] / 1000.0
                                    imu_data.linear_acceleration.y = data[3] / 1000.0
                                    imu_data.linear_acceleration.z = data[4] / 1000.0
                            finally:
                                imu_lock.release()

                        st_flag = False
                        rx_cnt = 0
                        rx_msg = []
                                    
            rospy.sleep(0.001)
    except KeyboardInterrupt:
        thread_name.exit()
        _serial.close()
        print("Interrupted")

def queue_handler(queue, action, cmd=[]):
    md_lock.acquire()
    try:
        if action:
            queue.put(cmd)
        if not action:
            item = queue.get()
    finally:
        md_lock.release()
    if action:
        return cmd
    if not action:
        return item

def make_text_command(command_set):
    command = command_set[0]
    if command == "mpf":
        return "mpf\r\n"
    elif command == "mvc":
        return command + '=' + str(command_set[1]) + ',' + str(command_set[2]) + '\r\n'
    elif command == "co":
        return command + '1=' + str(command_set[1]) + ';' + command + '2=' + str(command_set[1]) + '\r\n'
    else:
        return '\r\n'


# def encode_cmd(cmd_type, cmd_code, cmd_object, value, subindex = b'\x01'):
def md_thread_func(thread, thread_name, _serial, remote_tx_queue, tx_queue, rx_queue):
    try:
        if _serial.isOpen():
            _serial.read(_serial.inWaiting())
        else:
            print("Motor driver serial is not opened")
            return 
        remote_tx_queue.queue.clear()
        tx_queue.queue.clear()
        rx_queue.queue.clear()
        return_flag = False
        newline_flag = False
        rx_msg = ''
        pattern = r'([a-z]*)([0-9]*)=(-*[0-9]*),*(-*[0-9]*),*([0-9]*)(\r\n)'

        while not thread._stopevent.isSet():
                        
            if _serial.isOpen():
                if not remote_tx_queue.empty() and _serial.inWaiting() == 0:
                    # Make sure the velocity commands are executed asap
                    while not remote_tx_queue.empty():
                        tx_msg = remote_tx_queue.get()
                        # tx_msg = encode_cmd(cmd_info[0], cmd_info[1], cmd_info[2], cmd_info[3], cmd_info[4])
                        #tx_msg = make_text_command(command_set)
                        _serial.write(tx_msg)
                        #print(tx_msg)
                        #print("remote tx")
                elif not tx_queue.empty() and _serial.inWaiting() == 0:
                    tx_msg = tx_queue.get()
                    #print(tx_msg)
                    #print("regular tx")
                    #tx_msg = make_text_command(command_set)
                    #print(tx_msg)
                    _serial.write(tx_msg)

                while _serial.inWaiting() > 0:
                    data = _serial.read(1)
                    rx_msg += data
                    if data == '\r':
                        return_flag = True
                    elif data == '\n':
                        newline_flag = True

                    if return_flag and newline_flag:
                        rx_msg_regex = re.search(pattern, rx_msg)
                        return_flag = newline_flag = False
                        #print(rx_msg)

                        #print(rx_msg_regex.group(1), rx_msg_regex.group(3), rx_msg_regex.group(4), rx_msg_regex.group(5))
                        if rx_msg_regex:
                            #print(rx_msg_regex.group(1), rx_msg_regex.group(3), rx_msg_regex.group(4), rx_msg_regex.group(5))
                            queue_handler(rx_queue, True, [rx_msg_regex.group(1),rx_msg_regex.group(3), rx_msg_regex.group(4), rx_msg_regex.group(5)]) 
                        rx_msg = ''
            rospy.sleep(0.01)
                                           
    except KeyboardInterrupt:
        thread_name.exit()
        _serial.close()
        print('Interrupted')
