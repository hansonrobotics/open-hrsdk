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

import math
import threading
import time

import rospy
from hr_msgs.msg import TargetPosture
from hr_msgs.srv import SetActuatorsControl, SetActuatorsControlRequest


#int32 disable = 0
#int32 manual   = 1   # control motor directly
#int32 animation = 2 # control by animation

class ActuatorControl(object):
    """An example showing how to control indivisual actuators.
    """
    def __init__(self):
        rospy.wait_for_service("/hr/actuators/get_loggers")
        self.pose = rospy.Publisher("/hr/actuators/pose", TargetPosture, queue_size=10)

        self.actuator_names = ["RightShoulderRoll", "LeftShoulderRoll"]
        self.set_control = rospy.ServiceProxy("/hr/actuators/set_control", SetActuatorsControl)

        job = threading.Timer(0, self.move)
        job.daemon = True
        job.start()

    def set_manual_control(self):
        """
        Set the actuators to manual control, so them can be controlled
        directly.

        CONTROL_DISABLE: disable actuator
        CONTROL_MANUAL: control actuator by hand (code)
        CONTROL_ANIMATION: control actuator by animation
        """
        request = SetActuatorsControlRequest()
        request.control = SetActuatorsControlRequest.CONTROL_MANUAL
        request.actuators = self.actuator_names
        self.set_control(request)

    def move(self):
        """Move the actuators"""
        self.set_manual_control()
        x = 0
        while True:
            angle = math.pi * (1 + math.sin(x)) / 4
            pose = TargetPosture()
            pose.names = self.actuator_names
            pose.values = [-angle, angle]
            self.pose.publish(pose)
            x += 0.1
            time.sleep(0.1)


if __name__ == "__main__":
    rospy.init_node("actuator_control")
    ActuatorControl()
    rospy.spin()
