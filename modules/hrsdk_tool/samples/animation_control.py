##
## Copyright (C) 2017-2025 Hanson Robotics
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 3 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <https://www.gnu.org/licenses/>.
##

import time
import threading

import genpy
import rospy
from hr_msgs.msg import SetAnimation, SetExpression
from hr_msgs.srv import StringArray


class AnimationControl(object):
    """An example iterates all the arm animations, facial expressions,
    and head animations.
    """

    def __init__(self):
        # wait for the animation node up
        rospy.wait_for_service("/hr/animation/blender_api/get_loggers")

        self.available_expressions = rospy.ServiceProxy(
            "/hr/animation/available_expressions", StringArray
        )().data
        self.available_animations = rospy.ServiceProxy(
            "/hr/animation/available_animations", StringArray
        )().data
        self.available_arm_animations = rospy.ServiceProxy(
            "/hr/animation/available_arm_animations", StringArray
        )().data

        self.animation = rospy.Publisher(
            "/hr/animation/set_animation", SetAnimation, queue_size=10
        )
        self.expression = rospy.Publisher(
            "/hr/animation/set_expression", SetExpression, queue_size=10
        )
        self.arm_animation = rospy.Publisher(
            "/hr/animation/set_arm_animation", SetAnimation, queue_size=10
        )

        job = threading.Timer(0, self.iterate_animation)
        job.daemon = True
        job.start()

    def iterate_animation(self):
        for expression in self.available_expressions:
            command = SetExpression()
            command.name = expression
            command.magnitude = 1
            command.duration = genpy.Duration(2)
            self.expression.publish(command)
            time.sleep(2)

        for animation in self.available_animations:
            command = SetAnimation()
            command.name = animation
            command.magnitude = 1
            command.speed = 1
            self.animation.publish(command)
            time.sleep(2)

        for arm_animation in self.available_arm_animations:
            command = SetAnimation()
            command.name = arm_animation
            command.magnitude = 1
            command.speed = 1
            self.animation.publish(command)
            time.sleep(2)


if __name__ == "__main__":
    rospy.init_node("animation_control")
    animation_control = AnimationControl()
    rospy.spin()
