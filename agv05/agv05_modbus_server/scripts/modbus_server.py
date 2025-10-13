#!/usr/bin/env python
from __future__ import absolute_import
from __future__ import unicode_literals

import rospy
import subprocess
from pymodbus.server.sync import ModbusTcpServer
from pymodbus.datastore import ModbusSlaveContext, ModbusServerContext, ModbusSequentialDataBlock
from pymodbus.device import ModbusDeviceIdentification
from pymodbus.transaction import ModbusSocketFramer
from threading import Thread

from agv05_modbus_server.data_block import DiscreteInput, CoilOutput, HoldingRegister, InputRegister


redirect_cmd = 'sudo iptables -t nat -A PREROUTING -p tcp -m tcp --dport 502 -j REDIRECT --to-ports 1502 -w'
remove_redirect_cmd = redirect_cmd.replace(' -A ', ' -D ')


def modbus_server_worker(server):
    rospy.loginfo('agv05_modbus_server:Tcp server started.')
    server.serve_forever()
    rospy.loginfo('agv05_modbus_server:Tcp server stoped.')


# main function
if __name__ == '__main__':

    rospy.init_node('agv05_modbus_server', anonymous=False)
    rospy.loginfo('agv05_modbus_server started')

    try:
        subprocess.check_call(redirect_cmd.split())
    except Exception as ex:
        rospy.logerr('Failed to setup iptables redirection: %s', ex)

    # init data store
    discrete_input = DiscreteInput()
    holding_register = HoldingRegister()
    input_register = InputRegister()
    coil_output = CoilOutput()

    store = ModbusSlaveContext(di=discrete_input, co=coil_output, hr=holding_register, ir=input_register, zero_mode=False)
    context = ModbusServerContext(slaves=store, single=True)

    # init server information
    # TODO: dynamic identity info.
    identity = ModbusDeviceIdentification()
    identity.VendorName = 'DF Automation'
    identity.VendorUrl = 'http://www.dfautomation.com/'
    identity.ProductName = 'Zalpha'
    identity.ProductCode = 'Z'
    identity.ModelName = 'Zalpha'
    identity.MajorMinorRevision = '1.0'

    framer = ModbusSocketFramer

    ModbusTcpServer.allow_reuse_address = True
    ModbusTcpServer.daemon_threads = True

    rate = rospy.Rate(10)
    server = None
    while not rospy.is_shutdown():
        try:
            server = ModbusTcpServer(context, framer, identity, ('0.0.0.0', 1502))
            break
        except Exception:
            rospy.logwarn('agv05_modbus_server: Unable to start modbus server.')
        rate.sleep()

    if not server:
        rospy.loginfo('agv05_modbus_server: Unable to start modbus server.')

    if server:
        thread = Thread(target=modbus_server_worker, args=(server,))
        thread.start()
        rospy.spin()

        server.shutdown()
        thread.join()

        rospy.loginfo('agv05_modbus_server: Modbus tcp server stopped.')

    try:
        subprocess.check_call(remove_redirect_cmd.split())
    except Exception as ex:
        rospy.logerr('Failed to remove iptables redirection: %s', ex)
