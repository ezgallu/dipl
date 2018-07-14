#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import math
import numpy as np
from geometry_msgs.msg import Quaternion, Vector3
from pid import PID
from std_msgs.msg import Float64


class DroneControl:

    def __init__(self):
        # flags
        self.stop = False
        self.drone_prepared = False
        self.ref_set = False
        self.goto = False

        # max speed
        self.max_thrust = 0.0
        self.min_thrust = 140.0
        self.max_side = 179.0
        self.min_speed = 0.0
        self.mid_speed = 92.0

        self.u = Quaternion(self.mid_speed, self.mid_speed, self.min_thrust, self.mid_speed)

self.pid_x = PID(3, 0, 0, self.max_side - self.mid_speed, self.min_speed - self.mid_speed)
self.pid_y = PID(3, 0, 0, self.max_side - self.mid_speed, self.min_speed - self.mid_speed)
self.pid_z = PID(1, 0.1, 0, self.min_thrust, self.max_thrust)
self.pid_yaw = PID(25, 0, 0, self.max_side - self.mid_speed, self.min_speed - self.mid_speed)

        self.pose_ref = Quaternion(0., 0., 0., -np.pi/2)
        self.pose_meas = Quaternion(-1., -1., -1., -1.)

        # SUBS AND PUBS
        # position reference
        self.pose_subscriber = rospy.Subscriber(
            "drone/pose_ref",
            Quaternion,
            self.setpoint_cb)

        # measured position
        self.measurement_subscriber = rospy.Subscriber(
            "drone_position",
            Quaternion,
            self.measurement_cb)

        # key listener
        self.key_subscriber = rospy.Subscriber(
            "keyboard",
            Vector3,
            self.keyboard_cb)

self.drone_input = rospy.Publisher(
    'control/drone_input',
    Quaternion)

        self.control_value = rospy.Publisher(
            'control_value',
            Quaternion,
            queue_size=10)

        self.error_pub = rospy.Publisher(
            'pose_error',
            Float64,
            queue_size=10)

        # Controller rate
        self.controller_rate = 10
        self.rate = rospy.Rate(self.controller_rate)
        self.controller_info = rospy.get_param("~verbose", False)

    def setpoint_cb(self, data):
        self.pose_ref = data
        self.ref_set = True

    def measurement_cb(self, data):
        self.pose_meas = data
        if data.z != -1.0 and not self.ref_set:
            self.ref_set = True
            self.pose_ref = data

    def keyboard_cb(self, data):
        # pressing down => 1. drone ready; 2. drone go to ref
        # pressing up => stop drone
        if data.x == 1.0:
            if self.drone_prepared:
                self.goto = True
            else:
                self.drone_prepared = True
        elif data.x == -1.0:
            if self.stop:
                self.stop = False
            else:
                self.stop = True
                self.goto = False

    def controller_dron_comm(self):
        self.drone_input.publish(Quaternion(self.mid_speed, self.mid_speed, self.max_thrust, self.mid_speed))
        rospy.sleep(1)
        self.drone_input.publish(Quaternion(self.mid_speed, self.mid_speed, self.min_thrust, self.mid_speed))
        rospy.sleep(1)

    def run(self):
""" First measurement for first reference. """
while not self.ref_set:
    print("DroneControl.run() - Waiting for first reference.")
    rospy.sleep(1)

self.controller_dron_comm()
""" Prepared drone """
while not self.drone_prepared:
    print("DroneControl.run() - Waiting for drone prepared.")
    rospy.sleep(1)

""" Run ROS node - computes PID algorithms for control """
print("DroneControl.run() - Starting position control")
self.t_old = rospy.Time.now()

while not rospy.is_shutdown():
    self.rate.sleep()

    t = rospy.Time.now()
    dt = (t - self.t_old).to_sec()
    self.t_old = t

    if dt < 0.99 / self.controller_rate:
        self.drone_input.publish(Quaternion(self.mid_speed, self.mid_speed, self.min_thrust, self.mid_speed))
        continue
    elif self.pose_meas.z == -1.0:
        self.drone_input.publish(Quaternion(self.mid_speed, self.mid_speed, self.min_thrust, self.mid_speed))
        continue

    if self.stop:   #flag for turn off PID control - turn off motors
        self.drone_input.publish(Quaternion(self.mid_speed, self.mid_speed, self.max_side, self.mid_speed))
        continue

    # HEIGHT CONTROL
    self.u.z = self.min_thrust - self.pid_z.compute(self.pose_ref.z, self.pose_meas.z, dt)

    # PITCH CONTROL OUTER LOOP
    # x - position control
    self.u.x = self.pid_x.compute(self.pose_ref.x, self.pose_meas.x, dt)

    # ROLL CONTROL OUTER LOOP
    # y position control
    self.u.y = self.pid_y.compute(self.pose_ref.y, self.pose_meas.y, dt)

    # PITCH AND ROLL YAW ADJUSTMENT
    roll_sp_2 = math.cos(self.pose_meas.w) * self.u.x + \
                math.sin(self.pose_meas.w) * self.u.y
    self.u.y = math.cos(self.pose_meas.w) * self.u.y - \
               math.sin(self.pose_meas.w) * self.u.x + self.mid_speed
    self.u.x = -roll_sp_2 + self.mid_speed

    # YAW CONTROL
    self.u.w = self.pid_yaw.compute(-np.pi/2, self.pose_meas.w, dt) + self.mid_speed

    # Calculate position error
    error = math.sqrt((self.pose_ref.x - self.pose_meas.x) ** 2 +
                      (self.pose_ref.y - self.pose_meas.y) ** 2 +
                      (self.pose_ref.z - self.pose_meas.z) ** 2)
    self.error_pub.publish(error)

    # Print out controller information
    if self.controller_info:
        print(dt)
        print("Comparison x:{}\nx_m:{}\ny:{}\ny_m:{}\nz:{}\nz_m{}\nyaw:{}\nyaw_m:{}".format(
            self.pose_ref.x,
            self.pose_meas.x,
            self.pose_ref.y,
            self.pose_meas.y,
            self.pose_ref.z,
            self.pose_meas.z,
            self.pose_ref.w,
            self.pose_meas.w))
        print("Current quadcopter height is: {}".format(self.pose_meas.z))
        print("Pitch PID output is:{}\n"
              "Roll PID output is:{}\n"
              "Yaw PID output is:{}\n"
              "Error: {}\n".format(self.u.x, self.u.y, self.u.w, error))

    self.drone_input.publish(self.u)


if __name__ == "__main__":
    rospy.init_node("drone_control")
    try:
        control = DroneControl()
        control.run()
    except rospy.ROSInterruptException:
        pass