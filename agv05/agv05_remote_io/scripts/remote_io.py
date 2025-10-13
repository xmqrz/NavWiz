#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

from agv05_msgs.msg import RemoteIoReply
from agv05_msgs.msg import RemoteIoRequest
from agv05_remote_io.device import RemoteIoDevice
from dynamic_reconfigure.server import Server
from std_msgs.msg import Float32
from std_msgs.msg import Int8
from std_msgs.msg import UInt16MultiArray
import agv05_remote_io.cfg.RemoteIoConfig
import diagnostic_updater
import rospy
import threading

# configuration for the node
remote_io_count = 10

# callback function


def callbackConfig(config, level):
    rospy.loginfo("Config received")

    timeout = config.remote_io_communication_timeout
    communication_timeout_pub.publish(timeout)

    for i in range(1, remote_io_count + 1):
        enable = config['remote_io_%d_enable_' % i]
        ip_address = config['remote_io_%d_ip_address_' % i]
        port = config['remote_io_%d_port_' % i]
        model = config['remote_io_%d_model_' % i]
        if enable:
            devices[i] = RemoteIoDevice(ip_address, port, timeout, model)
        else:
            devices[i] = None
    return config


def callbackRemoteIoRequest(data):
    # get information from callback
    remote_io_ = data.remote_io
    request_id_ = data.request_id
    type_ = data.type
    data_ = data.data
    data_mask_ = data.data_mask

    # diagnostic
    global diagnostic_request_remote_io_, diagnostic_request_id_, \
        diagnostic_request_type_, diagnostic_request_data_, \
        diagnostic_request_data_mask_
    diagnostic_request_remote_io_ = remote_io_
    diagnostic_request_id_ = request_id_
    diagnostic_request_type_ = type_
    diagnostic_request_data_ = data_
    diagnostic_request_data_mask_ = data_mask_
    diagnostic_updater_.update()

    # create worker according to request type
    if type_ == RemoteIoRequest.TYPE_READ:
        threading.Thread(target=workerRead, args=(request_id_, remote_io_,)).start()

    elif type_ == RemoteIoRequest.TYPE_WRITE:
        threading.Thread(target=workerWrite, args=(request_id_, remote_io_, data_, data_mask_,)).start()


def workerRead(request_id_, remote_io_):
    # exit if remote io is not valid
    if remote_io_ > remote_io_count:
        rospy.logerr('Read request for invalid Remote IO %d' % remote_io_)
        return
    rio = devices[remote_io_]
    if rio is None:
        rospy.logerr('Read request for invalid Remote IO %d' % remote_io_)
        return
    elif rio.input_count() == 0:
        rospy.logerr('Read request for Remote IO %d without input, possible model configuration error' % remote_io_)
        return

    result = rio.read_inputs()
    if result == -1:
        rospy.logerr('Remote IO %d read input error: %s' % (remote_io_, rio.error()))
    else:
        publishRemoteIoReply(request_id_, result)


def workerWrite(request_id_, remote_io_, data_, data_mask_):
    # exit if remote io is not valid
    if remote_io_ > remote_io_count:
        rospy.logerr('Write request for invalid Remote IO %d' % remote_io_)
        return
    rio = devices[remote_io_]
    if rio is None:
        rospy.logerr('Write request for invalid Remote IO %d' % remote_io_)
        return
    elif rio.output_count() == 0:
        rospy.logerr('Write request for Remote IO %d without output, possible model configuration error' % remote_io_)
        return

    result = rio.mask_write_outputs(data_, data_mask_)
    if result == -1:
        rospy.logerr('Remote IO %d write output error: %s' % (remote_io_, rio.error()))
    else:
        publishRemoteIoReply(request_id_, 0)


def publishRemoteIoReply(request_id, data):
    # diagnostic
    global diagnostic_reply_id_, diagnostic_reply_data_
    diagnostic_reply_id = request_id
    diagnostic_reply_data_ = data
    diagnostic_updater_.update()

    # publish reply
    result = RemoteIoReply()
    result.request_id = request_id
    result.data = data
    remote_io_reply_pub.publish(result)


def callbackNumberOfSubscribers(data):
    global start_lv
    start_lv = data.data > 0


def readAllInputsOutputs():
    inputs = []
    outputs = []
    for rio in devices:
        if rio is None:
            inputs.append(0)
            outputs.append(0)
            continue

        result = rio.read_inputs()
        if result == -1:
            inputs.append(0)
        else:
            inputs.append(result)

        if result == -1 or rio.output_count() == 0:
            outputs.append(0)
            continue

        result = rio.read_outputs()
        if result == -1:
            outputs.append(0)
            inputs[-1] = 0
        else:
            outputs.append(result)

    input_msg = UInt16MultiArray()
    input_msg.data = inputs
    output_msg = UInt16MultiArray()
    output_msg.data = outputs
    remote_io_inputs_pub.publish(input_msg)
    remote_io_outputs_pub.publish(output_msg)


def init():
    # devices
    global devices
    devices = [None] * (remote_io_count + 1)

    # timeout publisher
    global communication_timeout_pub
    communication_timeout_pub = rospy.Publisher(
        'agv05/peripheral/remote_io/communication_timeout',
        Float32, latch=True, queue_size=1)

    # dynamic reconfigure
    global dyncfg_server
    dyncfg_server = Server(agv05_remote_io.cfg.RemoteIoConfig, callbackConfig)

    # reply publisher
    global remote_io_reply_pub
    remote_io_reply_pub = rospy.Publisher(
        'agv05/peripheral/remote_io/reply',
        RemoteIoReply, latch=True, queue_size=1)

    # request subscriber
    global remote_io_request_sub
    remote_io_request_sub = rospy.Subscriber(
        'agv05/peripheral/remote_io/request',
        RemoteIoRequest, callbackRemoteIoRequest)

    global start_lv
    start_lv = False

    # number of remote_io subscribers
    global remote_io_subscount_sub
    remote_io_subscount_sub = rospy.Subscriber(
        'agv05/remote_io/subscriber_count',
        Int8, callbackNumberOfSubscribers)

    # all_inputs publisher
    global remote_io_inputs_pub
    remote_io_inputs_pub = rospy.Publisher(
        'agv05/remote_io/inputs',
        UInt16MultiArray, latch=True, queue_size=1)

    # all_outputs publisher
    global remote_io_outputs_pub
    remote_io_outputs_pub = rospy.Publisher(
        'agv05/remote_io/outputs',
        UInt16MultiArray, latch=True, queue_size=1)

    # setup diagnostic
    global diagnostic_updater_
    global diagnostic_request_remote_io_, diagnostic_request_id_, \
        diagnostic_request_type_, diagnostic_request_data_, \
        diagnostic_request_data_mask_, diagnostic_reply_id_, \
        diagnostic_reply_data_
    diagnostic_request_remote_io_ = 0
    diagnostic_request_id_ = 0
    diagnostic_request_type_ = 0
    diagnostic_request_data_ = 0
    diagnostic_request_data_mask_ = 0
    diagnostic_reply_id_ = 0
    diagnostic_reply_data_ = 0
    diagnostic_updater_ = diagnostic_updater.Updater()
    diagnostic_updater_.setHardwareID("AGV05")
    diagnostic_updater_.add("Status", diagnosticStatus)


def diagnosticStatus(stat):
    # request
    stat.add("Request Remote Io", diagnostic_request_remote_io_)
    stat.add("Request Id", diagnostic_request_id_)
    if diagnostic_request_type_ == RemoteIoRequest.TYPE_READ:
        type = "Read"
    elif diagnostic_request_type_ == RemoteIoRequest.TYPE_WRITE:
        type = "Write"
    else:
        type = "Unknown"
    stat.add("Request Request Type", type)
    stat.add("Request Data", diagnostic_request_data_)
    stat.add("Request Data Mask", diagnostic_request_data_mask_)
    # reply
    stat.add("Reply Id", diagnostic_reply_id_)
    stat.add("Reply Data", diagnostic_reply_data_)

    stat.summary(diagnostic_updater.DiagnosticStatus.OK, "Status ok")


# main function
if __name__ == '__main__':
    # initial message
    rospy.init_node('agv05_remote_io', anonymous=False)
    rospy.loginfo('agv05_remote_io started')

    # initialization
    init()

    # ros spin
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        diagnostic_updater_.update()
        if start_lv:
            readAllInputsOutputs()
        rate.sleep()

    # exit message
    rospy.loginfo('agv05_remote_io exited')
