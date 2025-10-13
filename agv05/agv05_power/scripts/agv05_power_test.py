#!/usr/bin/env python
import rospy

from agv05_msgs.msg import PowerSource

frequency = 5


def getch():
    import sys
    import tty
    import termios
    from select import select
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    timeout = 1.0 / frequency * 0.95
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    s = 0
    if rlist:
        s = sys.stdin.read(1)
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return s


def initPubSub():
    global power_source_pub_
    global batt1_current_pub_
    global auto_charger_pub_, manual_charger_pub_
    global manual_charger_connected_pub_
    power_source_pub_ = rospy.Publisher(
        "agv05/power/source",
        PowerSource, latch=True, queue_size=1)


def publishPowerSource():
    data = PowerSource()
    data.battery1_voltage = batt1_voltage
    data.battery2_voltage = 0
    data.battery_current = batt1_current
    data.auto_charger_voltage = auto_charger_voltage
    data.manual_charger_voltage = manual_charger_voltage
    data.manual_charger_connected = manual_charger_connected
    power_source_pub_.publish(data)


batt1_voltage = 0.0
auto_charger_voltage = 0.0
manual_charger_voltage = 0.0
batt1_current = 0.0
manual_charger_connected = False


def process():
    # variable
    global batt1_voltage, auto_charger_voltage, manual_charger_voltage, batt1_current
    global manual_charger_connected

    # read key
    key = getch()

    # print out information on screen
    print("\033[0;0H")  # jump to position 0,0
    print("agv05 power test")
    print("===================================")
    print("(use Key x to exit)")

    print("\n== Battery 1 voltage")
    print("Use Q to increase and A to decrease voltage, Z to reset to 26.0V")
    print("Output: Battery1=%f" % (batt1_voltage))

    print("\n== Charger and Current")
    print("Use E to toggle Auto Charger, D to toggle manual charger, C to toggle battery1 current, F to toggle manual charger connected")
    print("Output: AutoVoltage = %f ManualVoltage = %f Battery1Current = %f ManualConnected = %d" % (auto_charger_voltage, manual_charger_voltage, batt1_current, manual_charger_connected))

    # exit
    if (key == 'x'):
        return False
    # battery 1
    elif (key == 'q'):
        batt1_voltage += 0.01
    elif (key == 'a'):
        batt1_voltage -= 0.01
    elif (key == 'z'):
        batt1_voltage = 26.0
    # charger and current
    elif (key == 'e'):
        if (auto_charger_voltage == 0.0):
            auto_charger_voltage = 28.8
        else:
            auto_charger_voltage = 0.0
    elif (key == 'd'):
        if (manual_charger_voltage == 0.0):
            manual_charger_voltage = 28.8
        else:
            manual_charger_voltage = 0.0
    elif (key == 'c'):
        if (batt1_current == 2.0):
            batt1_current = -8.0
        else:
            batt1_current = 2.0
    elif (key == 'f'):
        if (manual_charger_connected):
            manual_charger_connected = False
        else:
            manual_charger_connected = True

    # output
    publishPowerSource()
    return True


# main function
if __name__ == '__main__':

    rospy.init_node('agv05_power_test', anonymous=False)
    rospy.loginfo('agv05_power_test: node started')

    # publisher
    initPubSub()

    # main loop
    rate = rospy.Rate(frequency)
    print("\033[2J")  # clear screen and jump to 0,0
    while not rospy.is_shutdown():
        if process() is False:
            break
        rate.sleep()

    # exit message
    print("\nExit...")
