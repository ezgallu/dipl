#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import Quaternion
from std_msgs.msg import UInt16
from pid import PID
from datetime import datetime


class DroneControl:

    def __init__(self):

        self.change = [0, 0, 0]

        # key pressed speed increment
        self.dx = 3
        self.dt = 0
        self.t_old = rospy.Time.now()

        # max speed
        self.max_thrust = 179
        self.max_side = 179
        self.min_speed = 0
        self.mid_speed = 92

        self.u_speed = Quaternion(0., 0., 0., 0.)

        self.pid_x = PID(1, 0, 0, self.max_side - self.mid_speed, self.mid_speed - self.min_speed)
        self.pid_y = PID(1, 0, 0, self.max_side - self.mid_speed, self.mid_speed - self.min_speed)
        self.pid_z = PID(1, 0, 0, self.max_thrust - self.mid_speed, self.mid_speed - self.min_speed)
        self.pid_yaw = PID(1, 0, 0, self.max_side - self.mid_speed, self.mid_speed - self.min_speed)

        self.ref_set = False
        self.speed = 0
        self.pose_ref = Quaternion(0., 0., 0., 0.)
        self.pose_meas = Vector3(0., 0., 0.)

        # initialize subscribers
        self.pose_subscriber = rospy.Subscriber(
            "drone/pose_ref",
            Quaternion,
            self.setpoint_cb)

        self.measurement_subscriber = rospy.Subscriber(
            "drone_position",
            Vector3,
            self.measurement_cb)

        self.drone_input = rospy.Publisher(
            'control/drone_input',
            Quaternion)

        # initialize publishers
        self.control_value = rospy.Publisher(
            'control_value',
            Quaternion,
            queue_size=10)

        self.toggle_speed = rospy.Publisher(
            'toggle_speed',
            UInt16)

        self.u = Quaternion(0., 0., 0., 0.)

        # Controller rate
        self.controller_rate = 50
        self.rate = rospy.Rate(self.controller_rate)

    def setpoint_cb(self, data):
        self.pose_ref = data
        self.speed = data.x
        self.ref_set = True

    def measurement_cb(self, data):
        self.pose_meas = data

    def compute_PID(self):
        self.u.x = self.pid_x.compute(self.pose_ref.x, self.pose_meas.x, dt) + self.mid_speed
        self.u.y = self.pid_y.compute(self.pose_ref.y, self.pose_meas.y, dt) + self.mid_speed
        self.u.z = self.pid_z.compute(self.pose_ref.z, self.pose_meas.z, dt) + self.mid_speed
        #self.u.z = self.pid_yaw.compute(self.pose_ref.w, self.pose_meas.w, dt) + self.mid_speed

    def controller_dron_comm(self):
        self.drone_input.publish(Quaternion(self.mid_speed, self.mid_speed, self.mid_speed, self.mid_speed))
        rospy.sleep(1)
        self.drone_input.publish(Quaternion(self.mid_speed, self.mid_speed, self.max_thrust, self.mid_speed))
        rospy.sleep(1)

    def run(self):
        """ Run ROS node - computes PID algorithms for z and vz control """

        while not self.ref_set:
            print("DroneControl.run() - Waiting for first reference.")
            rospy.sleep(1)

        print("DroneControl.run() - Starting position control")

        self.controller_dron_comm()
        while not rospy.is_shutdown():
            self.rate.sleep()

            t = rospy.Time.now()

            old = [self.u.x, self.u.y, self.u.z]
            for i in range(0, 2):
                if not self.change[i]:
                    old[i] = self.mid_speed
                else:
                    old[i] = old[i] + self.change[i]
                    if old[i] > self.max_side:
                        old[i] = self.max_side
                    elif old[i] < self.min_speed:
                        old[i] = self.min_speed
            old[2] = old[2] + self.change[2]
            if old[2] < self.min_speed:
                old[2] = self.min_speed
            elif old[2] > self.max_thrust:
                old[2] = self.max_thrust

            self.dt = (t - self.t_old).to_sec()
            self.t_old = t

            self.u_speed.x = old[0]
            self.u_speed.y = old[1]
            self.u_speed.z = old[2]
            self.u_speed.w = old[3]

            self.drone_input.publish(self.u_speed)
            self.toggle_speed.publish(self.speed)



if __name__ == "__main__":
    rospy.init_node("drone_control")
    try:
        control = DroneControl()
        control.run()
    except rospy.ROSInterruptException:
        pass