#
# Copyright (C) 2017-2024 Hanson Robotics
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#
import logging
import subprocess

import rospy
from std_msgs.msg import Bool
from std_srvs.srv import SetBool, SetBoolResponse

logger = logging.getLogger("hr.asr.status")


class Status(object):
    def __init__(self):
        self.service_running = False
        self.rate = rospy.Rate(0.5)
        self.status_pub = rospy.Publisher(
            "cloudspeech/status", Bool, queue_size=1, latch=True
        )
        self.status_srv = rospy.Service("~set_status", SetBool, self.handle_set_status)

    @staticmethod
    def check_online(url="google.com", port="80"):
        try:
            subprocess.check_output(
                ["ping", "-q", "-w", "1", "-c", "1", str(url)], stderr=subprocess.STDOUT
            )
        except Exception:
            return False
        return True

    def handle_set_status(self, msg):
        rospy.logwarn("handle_set_status: %s" % msg.data)
        self.service_running = msg.data

        return SetBoolResponse()

    def run(self):
        """Publish google speech service status. Should switch to offline speech recognition service when it's off."""

        while not rospy.is_shutdown():
            current_service_status = self.service_running
            self.status_pub.publish(self.service_running)

            online = Status.check_online("8.8.8.8")
            service_running = self.service_running and online
            if current_service_status != service_running:
                current_service_status = service_running
                logger.info(
                    "Speech recognition service status %s" % current_service_status
                )

            self.rate.sleep()
