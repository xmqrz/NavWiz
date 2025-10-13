#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_modbus_client.client import ModbusClient
from agv05_msgs.msg import ModbusReply
from agv05_msgs.msg import ModbusRequest
from dynamic_reconfigure.server import Server
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import UInt16MultiArray
import agv05_modbus_client.cfg.ModbusClientConfig
import diagnostic_updater
import rospy
import threading

# configuration for the node
modbus_count = 10

# callback function


def callbackConfig(config, level):
    rospy.loginfo("Config received")

    timeout = config.modbus_communication_timeout
    communication_timeout_pub.publish(timeout)

    for i in range(1, modbus_count + 1):
        enable = config['modbus_%d_enable_' % i]
        ip_address = config['modbus_%d_ip_address_' % i]
        port = config['modbus_%d_port_' % i]
        slave_id = config['modbus_%d_slave_id_' % i]
        if enable:
            devices[i] = ModbusClient(ip_address, port, timeout, slave_id)
        else:
            devices[i] = None
    return config


def callbackModbusRequest(data):
    # get information from callback
    modbus_ = data.modbus
    request_id_ = data.request_id
    type_ = data.type
    address_ = data.address
    data_ = data.data
    data_mask_ = data.data_mask

    # diagnostic
    global diagnostic_request_modbus_, diagnostic_request_id_, \
        diagnostic_request_type_, diagnostic_request_address_, \
        diagnostic_request_data_, diagnostic_request_data_mask_
    diagnostic_request_modbus_ = modbus_
    diagnostic_request_id_ = request_id_
    diagnostic_request_type_ = type_
    diagnostic_request_address_ = address_
    diagnostic_request_data_ = data_
    diagnostic_request_data_mask_ = data_mask_
    diagnostic_updater_.update()

    # create worker according to request type
    if type_ < ModbusRequest.TYPE_WRITE_COIL:
        threading.Thread(target=workerRead, args=(request_id_, modbus_, type_, address_)).start()

    elif type_ <= ModbusRequest.TYPE_MASK_WRITE_REGISTER:
        threading.Thread(target=workerWrite, args=(
            request_id_, modbus_, type_, address_, data_, data_mask_)).start()


def workerRead(request_id_, modbus_, type_, address_):
    # exit if modbus is not valid
    if modbus_ > modbus_count:
        rospy.logerr('Read request for invalid Modbus %d' % modbus_)
        return
    mb = devices[modbus_]
    if mb is None:
        rospy.logerr('Read request for invalid Modbus %d' % modbus_)
        return

    fn = (mb.read_coil, mb.read_discrete_input, mb.read_holding_register, mb.read_input_register)[type_]
    result = fn(address_)
    if result == -1:
        rospy.logerr('Modbus %d read input error: %s' % (modbus_, mb.error()))
    else:
        publishModbusReply(request_id_, result)


def workerWrite(request_id_, modbus_, type_, address_, data_, data_mask_):
    # exit if Device is not valid
    if modbus_ > modbus_count:
        rospy.logerr('Write request for invalid Modbus %d' % modbus_)
        return
    mb = devices[modbus_]
    if mb is None:
        rospy.logerr('Write request for invalid Modbus %d' % modbus_)
        return

    if type_ == ModbusRequest.TYPE_WRITE_COIL:
        result = mb.write_coil(address_, data_)
    elif data_mask_ == 0xffff:
        result = mb.write_register(address_, data_)
    else:
        result = mb.mask_write_register(address_, data_, data_mask_)
    if result == -1:
        rospy.logerr('Modbus %d write output error: %s' % (modbus_, mb.error()))
    else:
        publishModbusReply(request_id_, 0)


def publishModbusReply(request_id, data):
    # diagnostic
    global diagnostic_reply_id_, diagnostic_reply_data_
    diagnostic_reply_id = request_id
    diagnostic_reply_data_ = data
    diagnostic_updater_.update()

    # publish reply
    result = ModbusReply()
    result.request_id = request_id
    result.data = data
    modbus_reply_pub.publish(result)


def init():
    # device
    global devices
    devices = [None] * (modbus_count + 1)

    # timeout publisher
    global communication_timeout_pub
    communication_timeout_pub = rospy.Publisher(
        'agv05/peripheral/modbus/communication_timeout',
        Float32, latch=True, queue_size=1)

    # dynamic reconfigure
    global dyncfg_server
    dyncfg_server = Server(agv05_modbus_client.cfg.ModbusClientConfig, callbackConfig)

    # reply publisher
    global modbus_reply_pub
    modbus_reply_pub = rospy.Publisher(
        'agv05/peripheral/modbus/reply',
        ModbusReply, latch=True, queue_size=1)

    # request subscriber
    global modbus_request_sub
    modbus_request_sub = rospy.Subscriber(
        'agv05/peripheral/modbus/request',
        ModbusRequest, callbackModbusRequest)

    # setup diagnostic
    global diagnostic_updater_
    global diagnostic_request_modbus_, diagnostic_request_id_, \
        diagnostic_request_type_, diagnostic_request_address_, \
        diagnostic_request_data_, diagnostic_request_data_mask_, \
        diagnostic_reply_id_, diagnostic_reply_data_
    diagnostic_request_modbus_ = 0
    diagnostic_request_id_ = 0
    diagnostic_request_type_ = 0
    diagnostic_request_address_ = 0
    diagnostic_request_data_ = 0
    diagnostic_request_data_mask_ = 0
    diagnostic_reply_id_ = 0
    diagnostic_reply_data_ = 0
    diagnostic_updater_ = diagnostic_updater.Updater()
    diagnostic_updater_.setHardwareID("AGV05")
    diagnostic_updater_.add("Status", diagnosticStatus)


def diagnosticStatus(stat):
    # request
    stat.add("Request Modbus", diagnostic_request_modbus_)
    stat.add("Request Id", diagnostic_request_id_)
    if diagnostic_request_type_ == ModbusRequest.TYPE_READ_COIL:
        type = "Read Coil"
    elif diagnostic_request_type_ == ModbusRequest.TYPE_READ_DISCRETE_INPUT:
        type = "Read Discrete Input"
    elif diagnostic_request_type_ == ModbusRequest.TYPE_READ_HOLDING_REGISTER:
        type = "Read Holding Register"
    elif diagnostic_request_type_ == ModbusRequest.TYPE_READ_INPUT_REGISTER:
        type = "Read Input Register"
    elif diagnostic_request_type_ == ModbusRequest.TYPE_WRITE_COIL:
        type = "Write Coil"
    elif diagnostic_request_type_ == ModbusRequest.TYPE_MASK_WRITE_REGISTER:
        type = "Mask Write Register"
    else:
        type = "Unknown"
    stat.add("Request Request Type", type)
    stat.add("Request Address", diagnostic_request_address_)
    stat.add("Request Data", diagnostic_request_data_)
    stat.add("Request Data Mask", diagnostic_request_data_mask_)
    # reply
    stat.add("Reply Id", diagnostic_reply_id_)
    stat.add("Reply Data", diagnostic_reply_data_)

    stat.summary(diagnostic_updater.DiagnosticStatus.OK, "Status ok")


# main function
if __name__ == '__main__':
    # initial message
    rospy.init_node('agv05_modbus_client', anonymous=False)
    rospy.loginfo('agv05_modbus_client started')

    # initialization
    init()

    # ros spin
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        diagnostic_updater_.update()
        rate.sleep()

    # exit message
    rospy.loginfo('agv05_modbus_client exited')
