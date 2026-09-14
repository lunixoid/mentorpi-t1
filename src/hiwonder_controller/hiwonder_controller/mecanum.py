#!/usr/bin/python3
# coding=utf8
# MentorPi T1 overlay: tank mapping (stock MentorPi_Tank signs), not JetRover mecanum.
import math

from ros_robot_controller_msgs.msg import MotorsState, MotorState

from hiwonder_controller.tank_kinematics import TANK_TRACK_WIDTH, TANK_WHEEL_DIAMETER, TANK_WHEELBASE, tank_wheel_rps


class MecanumChassis:
    def __init__(
        self,
        wheelbase=TANK_WHEELBASE,
        track_width=TANK_TRACK_WIDTH,
        wheel_diameter=TANK_WHEEL_DIAMETER,
    ):
        self.wheelbase = wheelbase
        self.track_width = track_width
        self.wheel_diameter = wheel_diameter

    def speed_covert(self, speed):
        """
        covert speed m/s to rps/s
        :param speed:
        :return:
        """
        # distance / circumference = rotations per second
        return speed / (math.pi * self.wheel_diameter)

    def set_velocity(self, linear_x, linear_y, angular_z):
        """
        Use polar coordinates to control moving
                    x
        v1 motor1|  ↑  |motor3 v3
          +  y - |     |
        v2 motor2|     |motor4 v4
        :param speed: m/s
        :param direction: Moving direction 0~2pi, 1/2pi<--- ↑ ---> 3/2pi
        :param angular_rate:  The speed at which the chassis rotates rad/sec
        :param fake:
        :return:
        """
        # vx = speed * math.sin(direction)
        # vy = speed * math.cos(direction)
        # vp = angular_rate * (self.wheelbase + self.track_width) / 2
        # v1 = vx - vy - vp
        # v2 = vx + vy - vp
        # v3 = vx + vy + vp
        # v4 = vx - vy + vp
        # v_s = [self.speed_covert(v) for v in [v1, v2, -v3, -v4]]
        # T1 tank: linear_y unused. Stock invert of motors 1/2; no extra linear flip.
        v_s = tank_wheel_rps(
            linear_x,
            angular_z,
            wheelbase=self.wheelbase,
            track_width=self.track_width,
            wheel_diameter=self.wheel_diameter,
        )
        data = []
        for i in range(len(v_s)):
            msg = MotorState()
            msg.id = i + 1
            msg.rps = float(v_s[i])
            data.append(msg)

        msg = MotorsState()
        msg.data = data
        return msg
