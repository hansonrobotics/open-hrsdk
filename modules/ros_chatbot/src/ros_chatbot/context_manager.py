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

import logging
import os
import threading

from ros_chatbot.utils import get_ip, get_location

logger = logging.getLogger(__name__)


class ContextManager:
    def __init__(self):
        self._session_context = None

    @property
    def session_context(self):
        return self._session_context

    @session_context.setter
    def session_context(self, session_context):
        self._session_context = session_context
        self.preload_context()
        self.reset_context()

    def preload_context(self):
        """
        Preload necessary context data for the session.

        This method is responsible for initializing and loading any required
        context data for the session. It sets up geolocation information,
        client IP address, and other relevant parameters in the session context.
        """
        location = None
        location_thread = threading.Thread(target=lambda: get_location())
        location_thread.start()
        location_thread.join(timeout=2)
        if location_thread.is_alive():
            logger.warning("Geolocation retrieval timed out")
        else:
            location = get_location()
        if location:
            logger.info("Geolocation info %r", location)
            location_str = " ".join(
                filter(None, [location.get("neighborhood"), location.get("city")])
            )
            if location_str:
                logger.info("Set geolocation %s", location_str)
                self._session_context["geo_location"] = location_str
                self._session_context["location"] = location_str
                os.environ["LOCATION"] = location_str

        ip = None
        ip_thread = threading.Thread(target=lambda: get_ip())
        ip_thread.start()
        ip_thread.join(timeout=2)
        if ip_thread.is_alive():
            logger.warning("IP retrieval timed out")
        else:
            ip = get_ip()
        if ip:
            logger.info("Set client IP %s", ip)
            self._session_context["client_ip"] = ip
            os.environ["IP"] = ip

    def reset_context(self):
        self._session_context["turns"] = 0
        self._session_context["total_turns"] = 0
        self._session_context["done_steps"] = []
        self._session_context.proxy.delete_param("arf.events")
        self._session_context.proxy.delete_param("arf.scenes")
        self._session_context.proxy.delete_param("interlocutor")
        self._session_context.proxy.delete_param("block_chat")
        self._session_context["state"] = ""  # reset state
        self._session_context.user_context[
            "current_session"
        ] = self._session_context.sid
        self._session_context.user_context.proxy.expire("current_session", 72000)
