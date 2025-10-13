#!/usr/bin/env python

import rospy
import struct
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy


# Hack: Use the smallest positive denormal to signal no obstacle sensing
denormal = struct.unpack('<d', struct.pack('<q', 1))[0]


class agv05_joy:

    def __init__(self):
        self.en = False
        self.en_noos = False  # no obstacle sensing
        self.lateral = False
        self.zero_data_pub_ = False
        self.speed = 0.2
        self.left_button = 0
        self.right_button = 0
        self.up_button = 0
        self.down_button = 0

    def joy_cb(self, data):
        try:
            number_of_button = len(data.buttons)
            if number_of_button == 15:
                self.en = data.buttons[0]
                self.en_noos = data.buttons[1]
                self.lateral = data.buttons[2] or data.buttons[3]
                self.left_button = data.buttons[11]
                self.right_button = data.buttons[12]
                self.up_button = data.buttons[13]
                self.down_button = data.buttons[14]
            elif number_of_button in (11, 14):
                self.en = data.buttons[0]
                self.en_noos = data.buttons[1]
                self.lateral = data.buttons[2] or data.buttons[3]
                if data.axes[6] > 0:
                    self.left_button = 1
                    self.right_button = 0
                elif data.axes[6] < 0:
                    self.right_button = 1
                    self.left_button = 0
                else:
                    self.left_button = 0
                    self.right_button = 0

                if data.axes[7] > 0:
                    self.up_button = 1
                    self.down_button = 0
                elif data.axes[7] < 0:
                    self.down_button = 1
                    self.up_button = 0
                else:
                    self.up_button = 0
                    self.down_button = 0
            elif number_of_button == 12:
                self.en = data.buttons[1]
                self.en_noos = data.buttons[2]
                self.lateral = data.buttons[0] or data.buttons[3]
                if data.axes[4] > 0:
                    self.left_button = 1
                    self.right_button = 0
                elif data.axes[4] < 0:
                    self.right_button = 1
                    self.left_button = 0
                else:
                    self.left_button = 0
                    self.right_button = 0

                if data.axes[5] > 0:
                    self.up_button = 1
                    self.down_button = 0
                elif data.axes[5] < 0:
                    self.down_button = 1
                    self.up_button = 0
                else:
                    self.up_button = 0
                    self.down_button = 0
            if data.buttons[4]:
                self.speed = self.speed - 0.2
                if self.speed < 0.2:
                    self.speed = 0.2
            if data.buttons[5]:
                self.speed = self.speed + 0.2
                if self.speed > 1.0:
                    self.speed = 1.0
            self.linear_x = data.axes[1]
            self.linear_y = data.axes[0] if self.lateral else 0
            self.angular_z = 0 if self.lateral else -data.axes[0] if self.linear_x < 0 else data.axes[0]
        except Exception:
            pass

    def publish(self, event):
        twist = Twist()
        linear_speed = 0.3
        angular_speed = 0.3
        if self.en or self.en_noos:
            twist.linear.x = self.linear_x * self.speed
            twist.linear.y = self.linear_y * self.speed
            twist.angular.z = self.angular_z * self.speed
            if self.up_button:
                twist.linear.x = linear_speed
            elif self.down_button:
                twist.linear.x = -linear_speed
            if self.left_button:
                twist.linear.y = linear_speed if self.lateral else 0
                twist.angular.z = 0 if self.lateral else -angular_speed if twist.linear.x < 0 else angular_speed
            elif self.right_button:
                twist.linear.y = -linear_speed if self.lateral else 0
                twist.angular.z = 0 if self.lateral else angular_speed if twist.linear.x < 0 else -angular_speed
            if not self.en:  # no obstacle sensing
                twist.linear.z = denormal
            pub.publish(twist)
            self.zero_data_pub_ = False
        elif not self.zero_data_pub_:
            twist.linear.x = 0
            twist.linear.y = 0
            twist.linear.z = 0
            twist.angular.z = 0
            pub.publish(twist)
            self.zero_data_pub_ = True
            self.speed = 0.2


def start():
    global pub
    pub = rospy.Publisher('agv05/nav/manual_cmd_vel', Twist, queue_size=1)
    joy = agv05_joy()
    rospy.Subscriber('joy', Joy, joy.joy_cb)
    rospy.init_node('agv05_joy')
    rospy.Timer(rospy.Duration(0.1), joy.publish)
    rospy.spin()


if __name__ == '__main__':
    start()
