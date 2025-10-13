#! /usr/bin/env python
from __future__ import print_function

import actionlib
import rospy
from agv05_msgs.msg import NavxActionAction, NavxActionGoal
from geometry_msgs.msg import Pose2D

frequency = 10


def getch():
    import sys
    import tty
    import termios
    from select import select
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    timeout = 1.0 / frequency
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    s = 0
    if rlist:
        s = sys.stdin.read(1)
    termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return s


def feedbackCallback(feedback):
    global status
    status = feedback


if __name__ == '__main__':

    rospy.init_node("agv05_navx_test")

    # variable
    global status
    status = ""
    nav_speed = 0.3
    nav_forward_flag = False
    nav_enable_sensor = True
    nav_path_start = Pose2D()
    nav_path_end = Pose2D()
    nav_path_cp1 = Pose2D()
    nav_path_cp2 = Pose2D()
    nav_heading_end = Pose2D()

    # actions server configuration
    print("\033[2J")  # clear screen and jump to 0,0
    print("Wait for action server...")
    client = actionlib.SimpleActionClient('navx_action', NavActionAction)
    client.wait_for_server()
    print("Action server detected.")

    while not rospy.is_shutdown():
        # print out information on screen
        print("\033[2J")  # clear screen and jump to 0,0
        # print("\033[0;0H")      #jump to position 0,0
        print("agv05_navx_test")
        print("======================================")
        print("\nNav Parameters:")
        print("Speed [Q/A] %0.2f" % nav_speed)
        print("Forward Flag [E] %d" % nav_forward_flag)
        print("Enable Sensor [R] %d" % nav_enable_sensor)
        print("Rotate Angle [W/S] %0.3f" % nav_heading_end.theta)
        print("Path Start x[D/C]:%0.2f y[F/V]:%0.2f" % (nav_path_start.x, nav_path_start.y))
        print("Path Stop x[G/B]:%0.2f y[H/N]:%0.2f" % (nav_path_end.x, nav_path_end.y))
        print("Path Control Point 1 x[J/M]:%0.2f y[K/,]:%0.2f" % (nav_path_cp1.x, nav_path_cp1.y))
        print("Path Control Point 2 x[L/.]:%0.2f y[;/?]:%0.2f" % (nav_path_cp2.x, nav_path_cp2.y))
        print("\nNav Status: %s" % status)
        print("\nNav Start Action:")
        print("Forward(1) Reverse(2) Bezier Forward(3) Bezier Reverse(4) Rotate Right(5) Rotate Left(6)")
        print("\nPress 'x' to exit")

        # read key
        key = getch()
        goal = NavActionGoal()

        if (key == 'x'):
            break
        elif (key == 'q'):
            if (nav_speed <= (1.00 - 0.049)):
                nav_speed += 0.05
        elif (key == 'a'):
            if (nav_speed >= (0.0 + 0.05)):
                nav_speed -= 0.05
        elif (key == 'w'):
            if (nav_heading_end.theta < 3.14159):
                nav_heading_end.theta += 0.01
        elif (key == 's'):
            if (nav_heading_end.theta > -3.14159):
                nav_heading_end.theta -= 0.01
        elif (key == 'e'):
            nav_forward_flag = not nav_forward_flag
        elif (key == 'r'):
            nav_enable_sensor = not nav_enable_sensor
        elif (key == 'd'):
            if (nav_path_start.x < 100.0):
                nav_path_start.x += 0.05
        elif (key == 'c'):
            if (nav_path_start.x > -100.0):
                nav_path_start.x -= 0.05
        elif (key == 'f'):
            if (nav_path_start.y < 100.0):
                nav_path_start.y += 0.05
        elif (key == 'v'):
            if (nav_path_start.y > -100.0):
                nav_path_start.y -= 0.05
        elif (key == 'g'):
            if (nav_path_end.x < 100.0):
                nav_path_end.x += 0.05
        elif (key == 'b'):
            if (nav_path_end.x > -100.0):
                nav_path_end.x -= 0.05
        elif (key == 'h'):
            if (nav_path_end.y < 100.0):
                nav_path_end.y += 0.05
        elif (key == 'n'):
            if (nav_path_end.y > -100.0):
                nav_path_end.y -= 0.05
        elif (key == 'j'):
            if (nav_path_cp1.x < 100.0):
                nav_path_cp1.x += 0.05
        elif (key == 'm'):
            if (nav_path_cp1.x > -100.0):
                nav_path_cp1.x -= 0.05
        elif (key == 'k'):
            if (nav_path_cp1.y < 100.0):
                nav_path_cp1.y += 0.05
        elif (key == ','):
            if (nav_path_cp1.y > -100.0):
                nav_path_cp1.y -= 0.05
        elif (key == 'l'):
            if (nav_path_cp2.x < 100.0):
                nav_path_cp2.x += 0.05
        elif (key == '.'):
            if (nav_path_cp2.x > -100.0):
                nav_path_cp2.x -= 0.05
        elif (key == ';'):
            if (nav_path_cp2.y < 100.0):
                nav_path_cp2.y += 0.05
        elif (key == '/'):
            if (nav_path_cp2.y > -100.0):
                nav_path_cp2.y -= 0.05
        elif (key == '0'):
            status = "Starting Nav Stop.."
            goal.nav = goal.NAV_IDLE
            goal.speed = 0.0
            client.send_goal(goal, feedback_cb=feedbackCallback)

        elif (key == '1'):
            status = "Starting Nav Forward.."
            goal.nav = goal.NAV_FORWARD
            goal.speed = nav_speed
            goal.forward_flag = nav_forward_flag
            goal.enable_sensor = nav_enable_sensor
            goal.path_start = nav_path_start
            goal.path_end = nav_path_end
            client.send_goal(goal, feedback_cb=feedbackCallback)
        elif (key == '2'):
            status = "Starting Nav Reverse.."
            goal.nav = goal.NAV_REVERSE
            goal.speed = nav_speed
            goal.forward_flag = nav_forward_flag
            goal.enable_sensor = nav_enable_sensor
            goal.path_start = nav_path_start
            goal.path_end = nav_path_end
            client.send_goal(goal, feedback_cb=feedbackCallback)
        elif (key == '3'):
            status = "Starting Nav Bezier Forward.."
            goal.nav = goal.NAV_BEZIER_FORWARD
            goal.speed = nav_speed
            goal.forward_flag = nav_forward_flag
            goal.enable_sensor = nav_enable_sensor
            goal.path_start = nav_path_start
            goal.path_end = nav_path_end
            goal.path_cp1 = nav_path_cp1
            goal.path_cp2 = nav_path_cp2
            client.send_goal(goal, feedback_cb=feedbackCallback)
        elif (key == '4'):
            status = "Starting Nav Bezier Reverse.."
            goal.nav = goal.NAV_BEZIER_REVERSE
            goal.speed = nav_speed
            goal.forward_flag = nav_forward_flag
            goal.enable_sensor = nav_enable_sensor
            goal.path_start = nav_path_start
            goal.path_end = nav_path_end
            goal.path_cp1 = nav_path_cp1
            goal.path_cp2 = nav_path_cp2
            client.send_goal(goal, feedback_cb=feedbackCallback)
        elif (key == '5'):
            status = "Starting Nav Rotate Right.."
            goal.nav = goal.NAV_ROTATE_RIGHT
            goal.enable_sensor = nav_enable_sensor
            goal.path_end = nav_heading_end
            client.send_goal(goal, feedback_cb=feedbackCallback)
        elif (key == '6'):
            status = "Starting Nav Rotate Left.."
            goal.nav = goal.NAV_ROTATE_LEFT
            goal.enable_sensor = nav_enable_sensor
            goal.path_end = nav_heading_end
            client.send_goal(goal, feedback_cb=feedbackCallback)

    # exit message
    print("\nExit...")
