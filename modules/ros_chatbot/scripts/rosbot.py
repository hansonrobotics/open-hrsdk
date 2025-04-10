#!/usr/bin/env python

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

import asyncio
import logging
import os
import threading
import time
import uuid
from queue import Queue

import rospy
import socketio
from btree_client.robot import GenericRobot
from btree_client.schemas import ActionResult
from dotenv import load_dotenv
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from haipy.parameter_server_proxy import UserSessionContext
from hr_msgs.msg import TTS, ChatMessage, ChatResponse, ChatResponses
from std_srvs.srv import Trigger

from ros_chatbot.agents.model import AgentRequest
from ros_chatbot.cfg import ROSBotConfig
from ros_chatbot.db import write_request

logger = logging.getLogger("hr.ros_chatbot.rosbot")

load_dotenv()


class ROSBot(GenericRobot):
    def __init__(self, token):
        super(ROSBot, self).__init__("default", "default", namespace="/")
        self.token = token
        self.future = None
        self.current_speech_result = None
        self.events = Queue()
        self.loop = asyncio.new_event_loop()
        self.session_context = UserSessionContext("default", "default")
        self.trees = []
        self.running_arf = threading.Event()
        self.detect_speech_event = threading.Event()

        # Add this check for empty token
        self.btree_enabled = bool(token)
        if not self.btree_enabled:
            logger.warning("Behavior tree functionality is disabled due to missing token")

        rospy.Subscriber(
            "/hr/perception/hear/sentence", ChatMessage, self._user_speech_cb
        )
        rospy.Subscriber(
            "/hr/perception/hear/interim_speech",
            ChatMessage,
            self._user_interim_speech_cb,
        )

        self.wait_for_tts = rospy.ServiceProxy(
            "/hr/control/speech/wait_for_tts", Trigger
        )
        self.say_pub = rospy.Publisher("/hr/control/speech/say", TTS, queue_size=1)
        self._responses_publisher = rospy.Publisher(
            "/hr/interaction/chatbot_responses", ChatResponses, queue_size=1
        )

        Server(ROSBotConfig, self.reconfig)
        threading.Thread(target=self.monitor_tree, daemon=True).start()
        threading.Thread(target=self.start_background_loop, daemon=True).start()

    def start_background_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def _user_interim_speech_cb(self, msg):
        if msg.utterance:
            self.user_speaking.set()

    def _user_speech_cb(self, msg):
        if msg.utterance:
            speech_result = {}
            speech_result["transcript"] = msg.utterance
            speech_result["lang"] = msg.lang
            speech_result["audio_path"] = msg.audio_path
            speech_result["source"] = msg.source
            if self.detect_speech_event.is_set():
                self.current_speech_result = speech_result
            else:
                asyncio.run_coroutine_threadsafe(
                    self.sio.emit(
                        "event",
                        {"type": "chat", "text": msg.utterance, "lang": msg.lang},
                    ),
                    self.loop,
                )

    def is_hybrid(self):
        try:
            client = Client("/hr/interaction/chatbot", timeout=1)
            hybrid_mode = client.get_configuration(timeout=1)["hybrid_mode"]
        except Exception:
            hybrid_mode = False
        return hybrid_mode

    async def say(self, message):
        attachment = message.get("attachment")
        if attachment and attachment.get("agent_id"):
            agent_id = attachment.get("agent_id")
        else:
            agent_id = ""
        if self.is_hybrid():
            responses_msg = ChatResponses()
            response_msg = ChatResponse()
            response_msg.text = message["text"]
            response_msg.lang = message.get("lang") or "en-US"
            response_msg.agent_id = f"ARF-{agent_id}"
            response_msg.label = f"ARF-{agent_id}"
            responses_msg.responses.append(response_msg)
            if responses_msg.responses:
                self._responses_publisher.publish(responses_msg)
        else:
            msg = TTS()
            msg.text = message["text"]
            msg.lang = message.get("lang") or "en-US"
            msg.agent_id = f"ARF-{agent_id}"
            self.say_pub.publish(msg)
            logger.warning("Waiting for TTS to finish %r", message["text"])
            await asyncio.sleep(
                0.2
            )  # need to wait a bit for the message to be received by TTS node
            self.wait_for_tts()
            logger.warning("TTS has finished")
        return True

    def monitor_tree(self):
        if not self.btree_enabled:
            # If behavior trees are disabled, just wait indefinitely
            threading.Event().wait()
            return
        
        while True:
            while not self.trees:
                self.trees = self.session_context.get("btrees", [])
                time.sleep(1)
            while self.running_arf.is_set():
                logger.info("Btrees %s", self.trees)
                if self.future:
                    if self.future.done():
                        self.future = None
                if not self.future and self.trees:
                    next_tree = self.trees[0]
                    try:
                        logger.warning("Running next tree %r", next_tree)
                        self.future = asyncio.run_coroutine_threadsafe(
                            self.connect_socket(self.token, [next_tree]), self.loop
                        )
                        self.future.result()
                        self.future = None
                        logger.warning("Completed tree %r", next_tree)
                    except socketio.exceptions.ConnectionError as ex:
                        logger.error("ConnectionError %r", ex)
                        self.future.cancel()
                    except Exception as ex:
                        logger.error("Tree running error %r", ex)
                    finally:
                        self.trees.remove(next_tree)
                time.sleep(1)
            else:
                if self.future:
                    logger.warning("Cancelling ARF")
                    self.future.cancel()
                    try:
                        asyncio.run_coroutine_threadsafe(self.close(), self.loop)
                    except Exception as ex:
                        logger.error("Cancelling tree error %s", ex)
                    time.sleep(2)
                    self.future = None
                self.trees = []
            time.sleep(1)

    def set_running(self, running):
        if running:
            logger.warning("Disabling chatbot listening")
        else:
            logger.warning("Enabling chatbot listening")
        try:
            Client("/hr/interaction/chatbot").update_configuration(
                {"listen_speech": not running}
            )
            # Client("/hr/interaction/rosbot").update_configuration({"running": running})
        except Exception as ex:
            logger.error("Set running error %s", ex)
            rospy.set_param("/hr/interaction/chatbot/listen_speech", False)

    async def on_say(self, message):
        await self.sio.emit("ack", "say")
        try:
            success = await self.say(message)
        except Exception as ex:
            success = False
            logger.error("Say error %s", ex)
        result = ActionResult(success=success, event="say").dict()
        await self.sio.emit("ack", "say done")
        return result

    async def on_set(self, message):
        logger.info("set %s", message)
        return ActionResult(success=True, event="set").dict()

    async def wait_asr_result(self):
        while True:
            if self.current_speech_result:
                return self.current_speech_result
            else:
                await asyncio.sleep(0.1)

    def new_request(self, speech_result: dict):
        sid = rospy.get_param("/hr/interaction/chatbot/session", "")
        request = AgentRequest()
        request.sid = sid
        request.request_id = str(uuid.uuid4())
        request.question = speech_result["transcript"]
        request.lang = speech_result["lang"]
        request.audio = speech_result["audio_path"]
        request.source = speech_result["source"]
        request.session_context = self.session_context
        try:
            write_request(request)
        except Exception as ex:
            logger.error("Write request error: %s", ex)

    async def wait_for_speech(self, timeout, lang):
        """Waits for user's speech"""
        logger.warning("[Speech Start]")
        self.current_speech_result = None
        self.user_speaking.clear()
        try:
            logger.warning("Wait for speech event %s", timeout)
            await asyncio.wait_for(self.user_start_speaking(), timeout=timeout)
            logger.warning("Wait for speech result")
            result = await asyncio.wait_for(self.wait_asr_result(), timeout=30)
            if result:
                self.new_request(result)
                logger.warning("[Speech End]")
                logger.warning("Speech %r", result["transcript"])
                return result
            else:
                logger.info("No speech result")
        except asyncio.TimeoutError:
            logger.error("[Speech Timeout]")

    def enable_asr(self):
        try:
            dyn_client = Client("/hr/perception/speech_recognizer", timeout=1)
            dyn_client.update_configuration({"enable": True})
            logger.warning("Enable ASR")
            return True
        except Exception as ex:
            logger.exception(ex)
            return False

    async def on_detect_speech(self, message):
        logger.info("detect_speech %s", message)
        await self.sio.emit("ack", "detect_speech")
        self.enable_asr()
        self.detect_speech_event.set()

        try:
            result = await self.wait_for_speech(
                message["speech_timeout"],
                message["lang"],
            )
        except Exception as ex:
            logger.error(ex)
        finally:
            self.detect_speech_event.clear()
        self.empty_speech_queue()

        if result:
            return ActionResult(
                success=True, event="detect_speech", message=result
            ).dict()
        return ActionResult(success=False, event="detect_speech").dict()

    async def on_probe(self, message):
        return ActionResult(success=True, event="probe", message={}).dict()

    async def on_disconnect(self):
        if self.future:
            self.future.cancel()
            self.future = None
        self.set_running(False)
        logger.warning("rosbot is disconnected from server")

    async def on_connect(self):
        self.set_running(True)
        logger.warning("rosbot is connected to server")

    def reconfig(self, config, level):
        robot_mode = rospy.get_param("/hr/interaction/chatbot/robot_mode", None)
        if config.running:
            if robot_mode and robot_mode != "Stage":
                if not self.running_arf.is_set():
                    logger.warning("Running ARF")
                    self.running_arf.set()
            else:
                logger.warning("Can't run ARF in stage mode")
                config.running = False
        else:
            if self.running_arf.is_set():
                logger.warning("Stopping ARF")
                self.running_arf.clear()
                if self.future and not self.future.done():
                    future = asyncio.run_coroutine_threadsafe(self.close(), self.loop)
                    try:
                        future.result(timeout=10)
                    except Exception as ex:
                        logger.error(ex)
                        self.running_trees = []
                        self.future.cancel()
                        self.set_running(False)
        return config


if __name__ == "__main__":
    BTREE_SERVER_TOKEN = os.environ.get("BTREE_SERVER_TOKEN")
    if not BTREE_SERVER_TOKEN:
        logger.warning("BTREE_SERVER_TOKEN environment variable not set. Behavior tree functionality will be disabled.")
        # Initialize with a dummy token - the ROSBot class will handle this gracefully
        BTREE_SERVER_TOKEN = ""
    
    rospy.init_node("rosbot")
    bot = ROSBot(BTREE_SERVER_TOKEN)
    rospy.spin()
