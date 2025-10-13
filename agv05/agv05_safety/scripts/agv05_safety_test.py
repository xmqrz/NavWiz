#!/usr/bin/env python
import rospy

from std_msgs.msg import Float32
from std_msgs.msg import Bool

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
    global emergency_button_pub_, front_bumper_pub_, rear_bumper_pub_
    global safety_external1_pub_, safety_external2_pub_
    global safety_trigger_sub_
    emergency_button_pub_ = rospy.Publisher(
        "agv05/safety/button_emergency",
        Bool, latch=True, queue_size=1)
    front_bumper_pub_ = rospy.Publisher(
        "agv05/safety/bumper_front",
        Bool, latch=True, queue_size=1)
    rear_bumper_pub_ = rospy.Publisher(
        "agv05/safety/bumper_rear",
        Bool, latch=True, queue_size=1)
    safety_external1_pub_ = rospy.Publisher(
        "agv05/safety/safety_external1",
        Bool, latch=True, queue_size=1)
    safety_external2_pub_ = rospy.Publisher(
        "agv05/safety/safety_external2",
        Bool, latch=True, queue_size=1)


def publishEmergencyButton(trigger):
    global emergency_button_pub_
    data_bool = Bool()
    data_bool.data = trigger
    emergency_button_pub_.publish(data_bool)


def publishBumperFront(trigger):
    global front_bumper_pub_
    data_bool = Bool()
    data_bool.data = trigger
    front_bumper_pub_.publish(data_bool)


def publishBumperRear(trigger):
    global rear_bumper_pub_
    data_bool = Bool()
    data_bool.data = trigger
    rear_bumper_pub_.publish(data_bool)


def publishSafetyExternal1(trigger):
    global safety_external1_pub_
    data_bool = Bool()
    data_bool.data = trigger
    safety_external1_pub_.publish(data_bool)


def publishSafetyExternal2(trigger):
    global safety_external2_pub_
    data_bool = Bool()
    data_bool.data = trigger
    safety_external2_pub_.publish(data_bool)


emergency_button = False
bumper_front = False
bumper_rear = False
safety_external1 = False
safety_external2 = False


def process():
    # variable
    global emergency_button, bumper_front, bumper_rear
    global safety_external1, safety_external2

    # read key
    key = getch()

    # print out information on screen
    print("\033[0;0H")  # jump to position 0,0
    print("agv05 safety test")
    print("===================================")
    print("(use Key x to exit)")

    print("\n== Safety Trigger")
    print("Use 1, 2, 3, 4, 5 for all safety trigger")
    print("EmergencyButton: %d BumperFront: %d BumperRear: %d SafetyExternal1: %d SafetyExternal2: %d" % (
        emergency_button, bumper_front, bumper_rear, safety_external1, safety_external2))

    # exit
    if (key == 'x'):
        return False

    # safety trigger
    elif (key == '1'):
        emergency_button = not emergency_button
    elif (key == '2'):
        bumper_front = not bumper_front
    elif (key == '3'):
        bumper_rear = not bumper_rear
    elif (key == '4'):
        safety_external1 = not safety_external1
    elif (key == '5'):
        safety_external2 = not safety_external2

    # output
    publishEmergencyButton(emergency_button)
    publishBumperFront(bumper_front)
    publishBumperRear(bumper_rear)
    publishSafetyExternal1(safety_external1)
    publishSafetyExternal2(safety_external2)

    return True


# main function
if __name__ == '__main__':

    rospy.init_node('agv05_safety_test', anonymous=False)
    rospy.loginfo('agv05_safety_test: node started')

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
