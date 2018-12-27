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

STX = b'\x02'
ETX = b'\x03'
DEVICE_ID = b'\x01'
LENGTH = 13

AccessCode_map = {
    'WriteRequest'      : 0x10,
    'WriteResponse'     : 0x20,
    'ReadRequest'       : 0x30,
    'ReadResponse'      : 0x40,
    'ErrorResponse'     : 0x80,
    # For AHRS imu sensor
    'SyncRequest'       : 0xF0

}

Object_map = {
    'Int8'              : 0x00,
    'Int16'             : 0x04,
    'Int32'             : 0x08,
    'Float32'             : 0x0C,
    'DoubleInt32'	: 0x0A
}

AHRS_command_map = {
    "vendor_id"         : 0x01,
    "product_id"        : 0x02,
    "hardware_version"  : 0x04,
    "software_version"  : 0x03,

    "system_command"    : 0x07,
    "device_id"         : 0x0B,
    "serial_bps"        : 0x0D,

    "set_sync_data"     : 0x15,
    "set_sync_rate"     : 0x18,
    "set_data_type"     : 0x19,

    "accel_data"        : 0x33,
    "ang_vel_data"      : 0x34,
    "euler_data"        : 0x35,
    "magnet_data"       : 0x36,
    "temperature_data"  : 0x39

    # "SyncAccelData": 0x33,
    # "SyncAngVelData"  : 0x34,
    # "SyncEulerAngle"  : 0x35,
    # "SyncSet"      : 0x15,
    # "SystemCmd"    : 0x07,
    # "SyncRateSet"  : 0x18
}

AHRS_inverse_command_map = dict((v, k) for k, v in AHRS_command_map.iteritems())

AHRS_system_cmd_code = {
    "flash_write"       : 0x01,
    "factory_reset"     : 0x02,
    "accel_calib"       : 0x03,
    "magne_calib"       : 0x04,
    "euler_reset"       : 0x05,
    "calib_reset"       : 0x09,
    "software_restart"  : 0x63
}

AHRS_fault_code = {
    "undefined_name"    : 0x01,
    "packet_format"     : 0x02,
    "object_access"     : 0x03,
    "value_assignment"  : 0x04
}

MD_command_map = {
    "vendor_id"         : 0x01,
    "product_id"        : 0x02,
    "software_version"  : 0x03,
    "system_command"    : 0x07,
    "device_id"         : 0x0B,
    "serial_bps"        : 0x0D,
    "command"           : 0x65,
    "status"            : 0x66,
    "fault"             : 0x67,
    "position_command"  : 0x6F,
    "velocity_command"  : 0x70,
    "current_command"   : 0x71,
    "voltage_command"   : 0x72,
    "temperature"       : 0x79,
    "voltage"           : 0x7A,
    "current"           : 0x7B,
    "velocity"          : 0x7C,
    "position"          : 0x7D,
    "hall_count"        : 0x7E,

    "max_current"       : 0x97,
    "max_voltage"       : 0x98,
    "max_velocity"      : 0x99,
    "acceleration"      : 0x9A,
    "deceleration"      : 0x9B,
    "overheat_limit"    : 0xA1,
    "overcurrent_limit" : 0xA2,
    "stall_detection"   : 0xA7,
    "vel_error_detection": 0xA8,
    "pos_error_detection": 0xA9,
    "profile_mode"      : 0xAC,
    "startup_power_on"  : 0xAD,
    "direction"         : 0xAE,
    "cc_kp"             : 0xB5,
    "cc_ki"             : 0xB6,
    "vc_kp"             : 0xBA,
    "vc_ki"             : 0xBB,
    "pp_kp"             : 0xBF,
    "pp_ki"             : 0xC0,
    "pp_kd"             : 0xC1,

    "dual"		: 0x61,

}

MD_inverse_command_map = dict((v, k) for k, v in MD_command_map.iteritems())

MD_system_cmd_code = {
    "torque_on"         : 0x00,
    "torque_off"        : 0x01,
    "clear_faults"      : 0x02,
    "slow_stop"         : 0x03,
    "quick_stop"        : 0x04
}

MD_fault_code = {
    "overcurrent"       : 0x01,
    "overvoltage"       : 0x02,
    "undervoltage"      : 0x04,
    "overheat"          : 0x08,
    "short_circuit"     : 0x10,
    "stall_detection"   : 0x20,
    "velocity_error"    : 0x40,
    "position_error"    : 0x80
}

MD_comm_error_code = {
    "no_error"          : 0x00,
    "undefined_index"   : 0x01,
    "wrong_format"      : 0x02,
    "access_error"      : 0x03
}

