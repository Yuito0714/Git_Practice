#!/usr/bin/env python

from __future__ import print_function # for print function in python2
# import sys, select, termios, tty

import numpy as np
import rospy
import math

from spinal.msg import ServoControlCmd
from spinal.msg import ServoStates
from spinal.msg import Imu
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

class Teleop():
    def __init__(self):

        self.joy_dead_zone = rospy.get_param('~joy_dead_zone', 0.1)
        self.vel_rate = rospy.get_param('~vel_rate', 250.0)
        self.max_val = rospy.get_param('~max_val', 250)
        self.max_turn_angle_val = rospy.get_param('~max_turn_angle', 1024)
        self.keep_vel = rospy.get_param('~keep_vel', [0.0, 0.0])
        self.servo_equ_angles = rospy.get_param('~servos_angle', [0, 0, 0])
        self.servo_cmd = rospy.get_param('~servo_cmd', [0, 0, 0])
        self.turning = rospy.get_param('~turning', False)
        self.imu_angle = rospy.get_param('imu_angle', 0.0)
        self.tar_angle = rospy.get_param('~tar_angle', 0.0)
        self.tar_angle_ref = rospy.get_param('~tar_angle_ref', 0.0)
        self.imu_kp = rospy.get_param('~imu_kp', 0.0)
        self.demo1 = rospy.get_param('~demo1', False)
        self.demo2 = rospy.get_param('~demo2', False)
        self.fast_turn = rospy.get_param('~fast_turn', False)
        # self.servo_arrived = rospy.get_param('~servo_arrived', [0, 0])
        self.fin_angle_rate = rospy.get_param('fin_angle_rate', 512.0)
        self.roll_angle_rate = rospy.get_param('~roll_angle_rate', 1024.0)   #胸ヒレ振れ幅変換
        self.caudal_oscillation_hz = rospy.get_param('~caudal_oscillation_hz', 1.5)
        self.caudal_axis = rospy.get_param('~caudal_axis', 5)
        self.roll_axis = rospy.get_param('~roll_axis', 2)
        self.right_stick_forward_negative = rospy.get_param('~right_stick_forward_negative', True)
        self.servo_center_angles = list(self.servo_equ_angles)
        
        self.cmd_pub = rospy.Publisher('/servo/target_states', ServoControlCmd, queue_size=1)

        self.joy_sub = rospy.Subscriber('/joy', Joy, self._joyCallback)
        self.twist_sub = rospy.Subscriber('/cmd_vel', Twist, self._twistCallback)
        self.servo_pos_sub = rospy.Subscriber('/servo/states', ServoStates, self._servoStateCallback)
        self.imu_sub = rospy.Subscriber('/imu', Imu, self._imuCallback)


    def _servoStateCallback(self, msg):
        self.servo_equ_angles = [msg.servos[0].angle % 4096, msg.servos[1].angle % 4096, msg.servos[2].angle]
        #rospy.loginfo("servo 0 at angle %s", msg.servos[0].angle)

    def _imuCallback(self, msg):
        self.imu_angle = msg.angles[2]

    def _joyCallback(self, msg):
             
        roll_input = 0.0
        if len(msg.axes) > self.roll_axis:
            roll_input = msg.axes[self.roll_axis]
        if math.fabs(roll_input) < self.joy_dead_zone:
            roll_input = 0.0

        right_vertical = 0.0
        if len(msg.axes) > self.caudal_axis:
            right_vertical = msg.axes[self.caudal_axis]

        if self.right_stick_forward_negative:
            caudal_enable = max(0.0, right_vertical)
        else:
            caudal_enable = max(0.0, -right_vertical)
        if caudal_enable < self.joy_dead_zone:
            caudal_enable = 0.0

        roll_offset = roll_input * self.roll_angle_rate
        self.servo_cmd[0] = self.servo_center_angles[0] + roll_offset
        self.servo_cmd[1] = self.servo_center_angles[1] - roll_offset

        if caudal_enable > 0.0:
            phase = 2.0 * math.pi * self.caudal_oscillation_hz * rospy.get_time()
            self.servo_cmd[2] = self.servo_center_angles[2] + self.fin_angle_rate * math.sin(phase)
        else:
            self.servo_cmd[2] = self.servo_center_angles[2]

        msg_0 = ServoControlCmd()
        msg_0.index = [2, 0, 1]
        msg_0.cmd.append(int(self.servo_cmd[2]))
        msg_0.cmd.append(int(self.servo_cmd[0]))
        msg_0.cmd.append(int(self.servo_cmd[1]))
        self.cmd_pub.publish(msg_0)


    def _twistCallback(self, msg):

        vel = msg.linear.x
        val = vel * self.vel_rate
        if vel > self.max_val:
            val = self.max_val
        if vel < -self.max_val:
            val = -self.max_val
        msg = ServoControlCmd()
        msg.index = [0]
        msg.cmd.append(int(val))

        self.cmd_pub.publish(msg)



if __name__=="__main__":

    rospy.init_node("teleop")

    teleop_node = Teleop()

    rospy.spin()
