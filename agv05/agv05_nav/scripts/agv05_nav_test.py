#! /usr/bin/env python
from __future__ import print_function

from agv05_msgs.msg import NavActionAction, NavActionGoal
import actionlib
import rospy

status = ""


def getch():
    import sys
    import tty
    import termios
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch


def feedback_cb(feedback):
    global status
    status = feedback


if __name__ == '__main__':
    rospy.init_node("agv05_nav_test")
    print("agv05_nav Testing")

    client = actionlib.SimpleActionClient('nav_action', NavActionAction)
    client.wait_for_server()
    goal = NavActionGoal()

    print(" i = forward, < = reverse, j = rotate left, l = rotate right, k = play/pause")
    print(" speed configuration: a = increase, s = decrease")
    print(" enable_sensor configuration: d = toggle")
    print(" press x to exit")
    speed = 0.1
    enable_sensor = True

    # main loop
    while True:

        # device status message
        global status
        print("speed = %.2f m/s, enable_sensor = %r, (%s)"
              % (speed, enable_sensor, status), end="\r")

        # wait for key pres
        key = getch()

        # key actions
        if (key == 'a'):
            if (speed < (1.0 - 0.1)):
                speed += 0.1
        elif (key == 's'):
            if (speed > (0.1 + 0.1)):
                speed -= 0.1
        elif (key == 'd'):
            enable_sensor = not enable_sensor
        elif (key == 'x'):
            print("")
            print("exiting..")
            break

        elif (key == 'i'):
            action_status = "Forward"
            goal.nav = goal.NAV_FORWARD
            goal.speed = speed
            goal.forward_flag = False
            goal.enable_sensor = enable_sensor
            client.send_goal(goal, feedback_cb=feedback_cb)
        elif (key == 'j'):
            action_status = "RotLeft"
            goal.nav = goal.NAV_ROTATE_LEFT
            goal.speed = speed
            goal.forward_flag = False
            goal.enable_sensor = enable_sensor
            client.send_goal(goal, feedback_cb=feedback_cb)
        elif (key == 'l'):
            action_status = "RotRigh"
            goal.nav = goal.NAV_ROTATE_RIGHT
            goal.speed = speed
            goal.forward_flag = False
            goal.enable_sensor = enable_sensor
            client.send_goal(goal, feedback_cb=feedback_cb)
        elif (key == ','):
            action_status = "Reverse"
            goal.nav = goal.NAV_REVERSE
            goal.speed = speed
            goal.forward_flag = False
            goal.enable_sensor = enable_sensor
            client.send_goal(goal, feedback_cb=feedback_cb)
        elif (key == 'u'):
            action_status = "UturnLf"
            goal.nav = goal.NAV_UTURN_LEFT
            goal.speed = speed
            goal.forward_flag = False
            goal.enable_sensor = enable_sensor
            client.send_goal(goal, feedback_cb=feedback_cb)
        elif (key == 'o'):
            action_status = "UturnRg"
            goal.nav = goal.NAV_UTURN_RIGHT
            goal.speed = speed
            goal.forward_flag = False
            goal.enable_sensor = enable_sensor
            client.send_goal(goal, feedback_cb=feedback_cb)
