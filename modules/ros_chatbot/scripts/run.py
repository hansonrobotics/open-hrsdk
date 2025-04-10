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

import copy
import json
import logging
import os
import re
import threading
import time
import uuid
import warnings
from datetime import datetime
from queue import Empty, Queue
from typing import List

import requests
import rospy
from dynamic_reconfigure.client import Client
from dynamic_reconfigure.server import Server
from haipy.nlp.translate import detect_language
from haipy.parameter_server_proxy import UserSessionContext
from haipy.scheduler.ims_drivers import BaseDriver
from haipy.scheduler.intention_manager import IntentionManager
from haipy.scheduler.schemas.documents import BaseDriverDocument
from haipy.utils import LANGUAGE_CODES_NAMES
from hr_msgs.msg import (
    TTS,
    ChatMessage,
    ChatResponse,
    ChatResponses,
    Event,
    EventMessage,
)
from hr_msgs.srv import (
    AgentRegister,
    AgentUnregister,
    RunByName,
    StringArray,
    StringTrigger,
    TTSTrigger,
)
from langchain_community.chat_message_histories import RedisChatMessageHistory
from pydantic import BaseModel
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

import ros_chatbot.interact.action_types as action_types
import ros_chatbot.interact.event_types as event_types
from ros_chatbot.activity_monitor import ActivityMonitor, EngagementLevel
from ros_chatbot.agents.model import AgentResponse, AgentStreamResponse, LLMAgent
from ros_chatbot.agents.rosagent import ROSGenericAgent
from ros_chatbot.cfg import ROSChatbotConfig
from ros_chatbot.chat_server import ChatServer
from ros_chatbot.context_manager import ContextManager
from ros_chatbot.data_loader import DataLoader
from ros_chatbot.db import write_conv_insight, write_responses
from ros_chatbot.interact.base import BasicEventGenerator
from ros_chatbot.interact.controller_manager import ControllerManager
from ros_chatbot.interact.state import State
from ros_chatbot.reconfiguration import (
    AgentReconfiguration,
    DriverReconfiguration,
    PromptTemplatesReconfiguration,
    SceneReconfiguration,
)
from ros_chatbot.scene_manager import SceneManager
from ros_chatbot.schemas import Scene
from ros_chatbot.state_machine import RobotState
from ros_chatbot.utils import (
    load_agent_config,
    load_modes_config,
    remove_duplicated_responses,
    strip_xmltag,
)
from ros_chatbot.visual_processing import VisualProcessingConfig

logger = logging.getLogger("hr.ros_chatbot.run")

# Suppress the specific ROS publisher warning
warnings.filterwarnings(
    "ignore",
    message=".*The publisher should be created with an explicit keyword argument 'queue_size'.*",
)

SOULTALK_HOT_UPLOAD_DIR = os.environ["SOULTALK_HOT_UPLOAD_DIR"]
CMS_DIR = os.environ["CMS_DIR"]  # eg. /hr/.hrsdk/cms_content/sophia-sophia
REDIS_SERVER_HOST = os.environ.get("REDIS_SERVER_HOST", "localhost")
REDIS_SERVER_PORT = os.environ.get("REDIS_SERVER_PORT", "6379")

BAR_PATTERN = re.compile(r"""\|[^\|]+\|""")  # e.g. |p|

HR_CHARACTER = os.environ["HR_CHARACTER"]
ROBOT_NAME = os.environ["ROBOT_NAME"]
ROBOT_BODY = os.environ["ROBOT_BODY"]
character = HR_CHARACTER.title()


class Chatbot:
    uid = "default"

    def __init__(self):
        self.cfg = None
        self.default_language = "en-US"
        self.server = ChatServer()
        self.arf_fire_timer = None
        self.scenes = {}

        self.state = State()
        self.user_speaking = False
        self.controller_manager = ControllerManager(self.state, self._handle_action)
        self.controller_manager.register_event_generator(
            "stage", BasicEventGenerator("stage")
        )
        self.activity_monitor = ActivityMonitor(
            window_size=300,
        )

        self.interrupted = threading.Event()
        # Below event is set then chatbot need to cancel thhe thinking
        self.chat_interupt_by_speech = threading.Event()
        # Below event will make sure chat will wait before publishing results for interupt_by_speech or timeout
        self.chat_interupt_by_activity = threading.Event()
        # This to append speech while user speaking
        self.chat_buffer = ""
        # time for last speech activity, needs to track in case its falsly interrupted. In case final result followed by interim result without it being ever finalized
        self.last_speech_activity = 0
        self.speech_to_silence_interval = 1.5  # In case speech is interupted it will wait for 1.5 seconds inactivity or final result. If no final result is received the responses will be published

        self.chat_requests = []  # for calculating the total dialogue turns
        self.last_chosen_response = None
        self.current_responses = {}  # response id -> response
        self.asr_dyn_client = None
        self.lock = threading.RLock()
        self.pre_lock = threading.RLock()
        self.start_silence_detection = threading.Event()
        self.performance_idle_flag = threading.Event()
        self.internet_available = True
        self.silence_event_timer = None
        self.robot_mode = ""
        self.agent_config = load_agent_config()
        self.modes_config = load_modes_config()

        self.session_context = UserSessionContext(self.uid, uuid.uuid1().hex)
        self.context_manager = ContextManager()
        self.chat_memory = RedisChatMessageHistory(
            f"{self.session_context.ns}.history",
            url=f"redis://{REDIS_SERVER_HOST}:{REDIS_SERVER_PORT}",
            key_prefix="",
            ttl=3600,
        )
        self.scene_manager = SceneManager(
            character,
            self.scenes,
            self.session_context,
            self.agent_config,
            self.server.document_manager,
        )
        self.agent_reconfiguration = AgentReconfiguration()
        self.scene_reconfiguration = SceneReconfiguration(
            character,
            self.session_context,
            self.on_enter_scene_callback,
            self.on_exit_scene_callback,
            self.on_scene_change_callback,
        )
        intention_manager = IntentionManager(self.session_context)
        self._drivers_pub = rospy.Publisher(
            "/hr/interaction/available_drivers", String, queue_size=10, latch=True
        )
        rospy.Subscriber(
            "/hr/interaction/update_driver_status",
            String,
            self._update_driver_status_callback,
        )
        intention_manager.add_tasks_update_callback(self._publish_drivers)
        logger.info("Registered callback to publish active drivers to ROS topic")
        self.driver_reconfiguration = DriverReconfiguration(
            self.session_context, intention_manager
        )
        self.activity_monitor.add_engagement_level_change_callback(
            self.driver_reconfiguration.on_engagement_level_change
        )
        self.driver_reconfiguration.add_driver_callback(self.driver_callback)
        self.prompt_templates_reconfiguration = PromptTemplatesReconfiguration(
            [agent.id for agent in self.server.agents.values()],
            self.session_context,
        )

        # Time to ignore speech until
        self.ignore_speech_until = 0
        # Keaps track of the thread for the chat request
        self.current_chat_thread = None

        # once configuration loaded it will start rest automatically
        Server(ROSChatbotConfig, self.reconfig)

        env_monitor = threading.Thread(
            name="env_monitor", target=self.env_monitoring, daemon=True
        )
        env_monitor.start()

        # conversation quality monitor
        monitor = threading.Thread(name="quality_monitor", target=self.monitoring)
        monitor.daemon = True
        monitor.start()

        state_monitor = threading.Thread(
            name="state_monitor", target=self.state_monitoring
        )
        state_monitor.daemon = True
        state_monitor.start()

        # silence event detection
        silence_event_detection = threading.Thread(target=self._silence_event_detection)
        silence_event_detection.daemon = True
        silence_event_detection.start()

        # Stream responses handling
        self.stream_responses = Queue()
        stream_handler = threading.Thread(target=self._stream_response_handler)
        stream_handler.daemon = True
        stream_handler.start()
        # visual processing
        self.visual_processing_config = VisualProcessingConfig()
        threading.Thread(target=self._visual_processing, daemon=True).start()

    def driver_callback(self, driver: BaseDriver, output_model: BaseModel):
        msg = None
        if hasattr(output_model, "metrics"):
            msg = JointState()
            msg.header.stamp = rospy.Time.now()
            for k, v in output_model.metrics.model_dump().items():
                msg.name.append(k)
                msg.position.append(v)
        if driver.type == "EmotionalDriver":
            if msg:
                self.emotional_metrics_pub.publish(msg)
                logger.info(f"Published emotional metrics: {msg}")
        elif driver.type == "PhysiologicalDriver":
            if msg:
                self.physical_metrics_pub.publish(msg)
                logger.info(f"Published physical metrics: {msg}")

    def _publish_drivers(self, drivers: List[BaseDriverDocument]):
        """Publish drivers from the intention manager to a ROS topic.

        Args:
            drivers: List of BaseDriverDocument objects
        """
        try:
            # Convert drivers to a simpler dict format for JSON serialization
            drivers_data = []
            for driver in drivers:
                driver_dict = {
                    "id": str(driver.id),
                    "created_at": driver.created_at.isoformat(),
                    "updated_at": driver.updated_at.isoformat(),
                    "name": driver.name,
                    "type": driver.type,
                    "level": driver.level,
                    "priority": driver.priority,
                    "context": {
                        "status": driver.context.status,
                        "reason": driver.context.reason
                        if hasattr(driver.context, "reason")
                        else "",
                        "description": driver.context.description
                        if hasattr(driver.context, "description")
                        else "",
                        "metrics": driver.context.metrics.to_dict(),
                    },
                    "valence": driver.context.valence,
                    "plans": [str(p) for p in driver.context.plans]
                    if driver.context.plans
                    else [],
                }
                drivers_data.append(driver_dict)
            self._drivers_pub.publish(json.dumps(drivers_data))
            logger.info("Published %d drivers to ROS topic", len(drivers))
        except Exception as e:
            logger.error("Error publishing drivers to ROS topic: %s", str(e))
            logger.exception("Full traceback:")

    def _update_driver_status_callback(self, msg):
        logger.info("Received update driver status message: %s", msg.data)
        data = json.loads(msg.data)
        driver_id = data.get("id")
        status = data.get("status")
        # find driver by driver_id
        driver = next(
            (
                d
                for d in self.driver_reconfiguration.intention_manager.drivers
                if str(d.id) == driver_id
            ),
            None,
        )
        if driver:
            logger.info("Updating driver status for %s to %s", driver_id, status)
            driver.context.status = status
            driver.save()
        else:
            logger.warning("Driver not found: %s", driver_id)

    def new_conversation(self):
        self.session_context.sid = uuid.uuid1().hex
        self.session_context["character"] = character
        self.session_context["robot_name"] = ROBOT_NAME
        self.session_context["robot_body"] = ROBOT_BODY
        self.session_context[
            "instant_situational_prompt"
        ] = self.scene_reconfiguration.instant_situational_prompt
        rospy.set_param("~session", self.session_context.sid)
        logger.warning("Started new conversation %s", self.session_context.sid)
        self.session_context["webui_language"] = LANGUAGE_CODES_NAMES.get(
            self.current_language, self.current_language
        )
        self.chat_memory.session_id = f"{self.session_context.ns}.history"
        self.driver_reconfiguration.on_namespace_change()
        self.prompt_templates_reconfiguration.on_namespace_change()
        self.context_manager.session_context = self.session_context
        self._load_chat_data()
        self.on_scene_change_callback(self.scene_reconfiguration.scenes)

    def set_asr_context(self, param):
        if param and isinstance(param, list):
            param = ", ".join(param)
        try:
            dyn_client = Client("/hr/perception/speech_recognizer", timeout=1)
            current_params = dyn_client.get_configuration()
            phrases = current_params["phrases"]
            if phrases:
                phrases = phrases + ", " + param
            else:
                phrases = param
            if phrases:
                # remove duplicates
                phrases = ", ".join(
                    list(set([phrase.strip() for phrase in phrases.split(",")]))
                )
            dyn_client.update_configuration({"phrases": phrases})
            logger.info("Updated asr context phrases to %s", phrases)
        except Exception as ex:
            logger.exception(ex)

    def set_tts_mapping(self, param):
        if param and isinstance(param, list):
            param = ", ".join(param)
        try:
            dyn_client = Client("/hr/control/speech/tts_talker", timeout=1)
            current_params = dyn_client.get_configuration()
            word_mappings = current_params["word_mappings"]
            if word_mappings:
                word_mappings = word_mappings + ", " + param
            else:
                word_mappings = param

            # remove duplicated mappings
            word_mappings = ",".join(
                list(
                    set(
                        [
                            mapping.strip()
                            for mapping in word_mappings.split(",")
                            if mapping.strip()
                        ]
                    )
                )
            )

            dyn_client.update_configuration({"word_mappings": word_mappings})
            logger.info("Updated tts word mappings to %s", word_mappings)
        except Exception as ex:
            logger.exception(ex)

    def on_enter_scene_callback(self, scene: Scene):
        logger.warning("Enter scene %s", scene.name)
        self.new_conversation()
        self.session_context["scene"] = scene.name

        self.scene_manager.update_scene(scene.name)

        for key, value in scene.variables.items():
            self.session_context[key] = value
            self.session_context.proxy.expire(key, 3600)

        if scene.asr_context:
            self.set_asr_context(scene.asr_context)
        if scene.tts_mapping:
            self.set_tts_mapping(scene.tts_mapping)

        threading.Timer(
            5, self.scene_manager.load_relevant_scenes, args=(scene.name,)
        ).start()

    def on_exit_scene_callback(self, scene: Scene):
        logger.warning("Exit scene %s", scene.name)
        self.session_context["ended_scene"] = scene.name

        threading.Thread(
            target=self.scene_manager.update_last_scene_document,
            daemon=True,
        ).start()

        for key in scene.variables.keys():
            del self.session_context[key]
        del self.session_context["scene"]
        del self.session_context["person_object"]
        del self.session_context["prompt_template"]

    def on_scene_change_callback(self, scenes: List[Scene]):
        # merge locally created scenes with the ones from CMS
        for scene in scenes:
            if scene.name:
                self.scenes[scene.name] = scene

    def _stream_response_handler(self):
        while True:
            response = self.stream_responses.get()
            # Process the response until queue is empty or finished is set top true
            while (
                response.stream_data.qsize() > 0
                or not response.stream_finished.is_set()
            ):
                try:
                    sentence = response.stream_data.get(
                        timeout=response.stream_response_timeout
                    )
                    response.stream_data.task_done()
                except Empty:
                    if response.stream_timeout():
                        response.stream_finished.set()
                        break
                    else:
                        if response.stream_finished.is_set():
                            break
                        continue
                if sentence:
                    self._say(
                        sentence,
                        response.lang,
                        response.agent_id,
                        response.request_id,
                        ignore_state=False,
                    )
                    response.answer += " " + sentence

            # update response document
            self.server.update_response_document(
                response_id=response.response_id, text=response.answer
            )
            self.stream_responses.task_done()

    def is_google_available(self):
        try:
            requests.get("http://www.google.com", timeout=5)
            return True
        except (requests.ConnectionError, requests.Timeout):
            return False

    def env_monitoring(self):
        while True:
            self.internet_available = self.is_google_available()
            time.sleep(1)

    def monitoring(self):
        while True:
            last_auto_response_time = self.state.getStateValue(
                "last_auto_response_time"
            )
            if last_auto_response_time:
                time_elapsed = time.time() - last_auto_response_time
                if time_elapsed > self.cfg.hybrid_when_idle:
                    # automatically turn to hybrid mode
                    if self.cfg.auto_automonous_free_chat:
                        logger.warning("Autonomous mode is deactivated from being idle")
                        self._set_hybrid_mode(True)
            self.activity_monitor.monitoring()

            try:
                metrics = self.activity_monitor.get_engagement_metrics()
                metrics = {k: v for k, v in metrics.items() if k != "engagement_level"}
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                for i, (k, v) in enumerate(metrics.items()):
                    msg.name.append(k)
                    msg.position.append(float(v))
                    if self.last_engagement_metrics_msg:
                        msg.velocity.append(
                            (float(v) - self.last_engagement_metrics_msg.position[i])
                            / (
                                msg.header.stamp.to_sec()
                                - self.last_engagement_metrics_msg.header.stamp.to_sec()
                            )
                        )
                    else:
                        msg.velocity.append(0)
                self.engagement_metrics_pub.publish(msg)
                self.last_engagement_metrics_msg = msg
            except Exception as e:
                logger.error("Failed to publish engagement metrics")
                logger.exception(e)

            time.sleep(1)

    def state_monitoring(self):
        while True:
            # arf idle
            tree_running = rospy.get_param("/hr/interaction/rosbot/running", None)
            current_state = self.session_context.get("state")
            if (
                tree_running is False
                and current_state is not None
                and current_state
                not in [
                    "idle",
                    "asleep",
                ]
                and self.cfg.listen_speech
            ):
                self.session_context["state"] = "idle"
                self._emit_event("event.idle")
            if current_state in ["asleep"]:
                # mute chatbot
                if self.cfg.auto_automonous_free_chat and not self.hybrid_mode:
                    logger.info("Hybrid mode is set by state asleep")
                    self._set_hybrid_mode(True)
            # repeat sleep animation
            while self.session_context.get("state") in ["asleep"]:
                logger.warning("Playing sleep animation")
                self.run_performance("shared/arf/sleep/sleep_2")
                time.sleep(0.2)

            scene = self.statemachine.scene
            if scene == "idle":
                self._emit_event("event.idle")

            time.sleep(1)

    def _visual_processing(self):
        while True:
            try:
                if self.is_asr_running() and self.visual_processing_config.enabled:
                    result = self.describe_view_service.call(
                        data=self.visual_processing_config.get_prompt()
                    )
                    if result.success:
                        insight = json.loads(result.message)
                        created_at = datetime.strptime(
                            insight["time"]["value"], insight["time"]["format"]
                        )
                        record = {
                            "created_at": created_at,
                            "conversation_id": self.session_context.sid,
                            "type": "vision",
                            "insight": insight,
                            "character": character,
                        }
                        self.visual_processing_config.update_results(
                            insight["content"], insight["utc_time"]
                        )
                        write_conv_insight(record)
                        # update visual clue
                        self.session_context["visual_clue"] = insight["content"]
                        rospy.set_param("~visual_clue", insight["content"])
                        logger.info("Write visual insight %s", insight["content"])
            except Exception as ex:
                logger.error(ex)
                # could be network or some o
            time.sleep(self.visual_processing_config.interval)

    def run_performance(self, performance_id):
        performance_id = performance_id.strip()
        response = self.performance_service(performance_id)
        if response.success:
            self.performance_idle_flag.clear()
            logger.info("Run performance #%s successfully", performance_id)
        else:
            logger.warning("Failed to run performance #%s", performance_id)
        self.performance_idle_flag.wait()

    def _performance_event_cb(self, msg):
        if msg.event in ["idle"]:
            self.performance_idle_flag.set()

    def _emit_silence_event(self):
        if self.is_asr_running() and self.cfg.listen_speech:
            # emit silence event only when ASR is running
            self._emit_event("event.silence")

    def _emit_event(self, event):
        logger.warning("Emitting event %s", event)
        event_request = self.server.new_request(
            self.session_context.sid,
            event,
            self.current_language,
            source="system",
            session_context=self.session_context,
        )
        self._chat(event_request)

    def _silence_event_detection(self):
        while True:
            self.start_silence_detection.wait()
            logger.info("Start silence detection")
            self.silence_event_timer = threading.Timer(5, self._emit_silence_event)
            self.silence_event_timer.start()
            self.silence_event_timer.join()
            logger.info("End silence detection")
            self.start_silence_detection.clear()
            time.sleep(0.5)

    @property
    def autonomous_mode(self):
        return not self.hybrid_mode

    @property
    def hybrid_mode(self):
        return self.cfg.hybrid_mode if self.cfg else True

    def start(self):
        self.last_language = rospy.get_param("/hr/lang", self.default_language)

        self.new_conversation()

        # for webui
        self.enable = self.cfg.enable

        self.node_name = rospy.get_name()

        chat_topic = rospy.get_param("~chat_topic", "/hr/interaction/chat")
        rospy.Subscriber(chat_topic, ChatMessage, self._chat_callback)

        chat_event_topic = rospy.get_param(
            "~chat_event_topic", "/hr/interaction/chat/event"
        )
        self._chat_event_pub = rospy.Publisher(
            chat_event_topic, String, queue_size=None
        )

        hear_topic = rospy.get_param("~hear_topic", "/hr/perception/hear/sentence")
        rospy.Subscriber(hear_topic, ChatMessage, self._speech_chat_callback)

        rospy.Subscriber("/hr/interaction/interlocutor", String, self._interlocutor_cb)

        self._chat_pub = rospy.Publisher(hear_topic, ChatMessage, queue_size=None)

        event_topic = rospy.get_param("~hear_event_topic", "/hr/perception/hear/event")
        rospy.Subscriber(event_topic, String, self._user_speech_event_callback)

        hybrid_response_topic = rospy.get_param(
            "~hybrid_response_topic", "/hr/interaction/chatbot_responses"
        )
        self._responses_publisher = rospy.Publisher(
            hybrid_response_topic, ChatResponses, queue_size=None
        )

        # receive user's choice
        response_chosen_topic = rospy.get_param(
            "~response_chosen_topic", "/hr/interaction/chatbot_response"
        )
        rospy.Subscriber(
            response_chosen_topic, ChatResponse, self._response_chosen_callback
        )

        say_topic = rospy.get_param("~say_topic", "/hr/control/speech/say")
        self._response_publisher = rospy.Publisher(say_topic, TTS, queue_size=None)
        rospy.Subscriber(say_topic, TTS, self._tts_cb)

        self.tts_service = rospy.ServiceProxy("/hr/control/speech/tts", TTSTrigger)
        self.is_performance_loaded = rospy.ServiceProxy(
            "/hr/control/is_performance_loaded", Trigger
        )

        speech_events_topic = rospy.get_param(
            "~speech_events_topic", "/hr/control/speech/event"
        )
        rospy.Subscriber(
            speech_events_topic, String, self._robot_speech_event_cb, queue_size=None
        )
        rospy.Subscriber(
            "/hr/perception/hear/interim_speech", ChatMessage, self._interim_speech_cb
        )

        rospy.Subscriber(
            "/hr/interaction/content_manager/update_event",
            String,
            self._content_update_cb,
            queue_size=10,
        )
        rospy.Subscriber("/hr/interaction/arf", String, self._arf)

        tts_ctrl_topic = rospy.get_param(
            "~tts_ctrl_topic", "/hr/control/speech/tts_control"
        )
        self.tts_ctrl_pub = rospy.Publisher(tts_ctrl_topic, String, queue_size=None)

        event_topic = rospy.get_param("~event_topic", "/hr/interaction/event")
        self.event_pub = rospy.Publisher(event_topic, EventMessage, queue_size=None)

        self.switch_character_pub = rospy.Publisher(
            "/hr/interaction/switch_character", String, queue_size=None
        )

        self.emotional_metrics_pub = rospy.Publisher(
            "/hr/interaction/emotional_metrics", JointState, latch=True, queue_size=None
        )
        self.physical_metrics_pub = rospy.Publisher(
            "/hr/interaction/physical_metrics", JointState, latch=True, queue_size=None
        )
        self.engagement_metrics_pub = rospy.Publisher(
            "/hr/interaction/engagement_metrics",
            JointState,
            latch=True,
            queue_size=None,
        )
        self.last_engagement_metrics_msg = None

        self.driver_reconfiguration.add_driver_callback(self.driver_callback)

        rospy.Subscriber(
            "/hr/control/performances/background/events",
            Event,
            self._performance_event_cb,
        )
        # Run performance by name
        self.performance_service = rospy.ServiceProxy(
            "/hr/control/performances/background/run_by_name", RunByName
        )

        self.describe_view_service = rospy.ServiceProxy(
            "/hr/interaction/describe_view", StringTrigger
        )

        rospy.Service("register", AgentRegister, self.ros_service_register)
        rospy.Service("unregister", AgentUnregister, self.ros_service_unregister)
        rospy.Service("available_agents", StringArray, self.list_all_installed_agents)
        rospy.Service("set_context", StringTrigger, self.ros_service_set_context)
        rospy.Service("get_context", StringTrigger, self.ros_service_get_context)
        rospy.Service("available_scenes", StringArray, self.list_all_scenes)

        # Set up agent configuration
        for agent in copy.copy(self.server.agents).values():
            self.agent_reconfiguration.set_up_agents_runtime_dynamic_reconfigure(
                agent,
                self.presets,
            )
        self.scene_reconfiguration.start_ddr()

    def run_reflection(self, text, lang):
        results = self.server.run_reflection(self.session_context.sid, text, lang)
        if results:
            persona = [r["text"] for r in results]
            logger.info("Run reflection persona %s", persona)
            existing_persona = self.session_context.get("persona", [])
            persona = existing_persona + persona
            self.session_context["persona"] = persona

    def _tts_cb(self, msg):
        if not msg.text:
            return
        self.activity_monitor.record_chat_activity()
        text = BAR_PATTERN.sub("", msg.text).strip()
        if not text:
            return

        self.server.add_record(text)

        # Inform on all TTS activity and let agents to collect data if needed
        text = strip_xmltag(text)
        for a in self.server.agents.values():
            a.character_said(text, msg.lang)

        # update redis memory
        self.chat_memory.add_ai_message(text)

        self.session_context["turns"] = len(self.chat_requests)
        self.session_context["last_active_time"] = datetime.utcnow()

        self.run_reflection(text, msg.lang)

        if not msg.request_id:
            self._record_other_agent_response(msg)

        current_state = self.session_context.get("state")
        if current_state in ["asleep"]:
            self.session_context["state"] = ""  # reset state

    def _record_other_agent_response(self, msg):
        response = AgentResponse()
        response.agent_id = msg.agent_id or "Human"
        response.response_id = str(uuid.uuid4())
        response.answer = msg.text
        response.attachment["published"] = True
        write_responses([response])
        self.server.publish(response.response_id)

    def _after_published_responses(self, request_id):
        request = self.server.requests.get(request_id)
        if request is None:
            # probably not responses from this dialogue system
            return

        if request_id not in self.chat_requests:
            logger.info("New request got responded %s", request)
            self.chat_requests.append(request_id)

    def _robot_speech_event_cb(self, msg):
        if msg.data:
            if msg.data.startswith("start"):
                self.state.update(robot_speaking=True)
                self.start_silence_detection.clear()
                if self.silence_event_timer and self.silence_event_timer.is_alive():
                    self.silence_event_timer.cancel()
            if msg.data.startswith("stop"):
                self.state.update(robot_speaking=False)
                self.start_silence_detection.set()

    def _interim_speech_cb(self, msg):
        s = String("speechcont")
        self._user_speech_event_callback(s)

    def _user_speech_event_callback(self, msg):
        if msg.data:
            self.activity_monitor.record_speech_activity()
            if msg.data.startswith("speechstart") or msg.data.startswith("speechcont"):
                self.state.update(user_speaking=True)
                self.user_speaking = True
                self.last_speech_activity = time.time()
                self.chat_interupt_by_activity.set()
            if msg.data.startswith("speechstop"):
                self.state.update(user_speaking=False)
                self.user_speaking = False

    def _response_chosen_callback(self, msg):
        """Handles the response chosen by user"""
        if msg.response_id in self.server.responses:
            self.server.publish(msg.response_id, label=msg.label, resolver="Human")
            response = self.server.responses[msg.response_id]
            self._publish_resolved_response(response, ignore_state=True)
        elif msg.response_id == "":
            # the response from external i.e. snet
            response = AgentResponse()
            response.request_id = msg.request_id
            response.response_id = msg.response_id
            response.agent_id = msg.agent_id
            response.lang = msg.lang
            response.answer = msg.text
            response.end()
            self._publish_resolved_response(response, ignore_state=True)
        else:
            logger.error("Response is lost %r", msg.response_id)

    def _arf(self, msg):
        if msg.data == "arf":
            logger.warning("Fire ARF!")
            self.fire_arf()
            time.sleep(2)
        else:
            logger.warning("Activate scene %r", msg.data)
            self.fire_arf(msg.data)

    def fire_arf(self, scene_name=None):
        self.session_context["last_active_time"] = datetime.utcnow()
        if self.statemachine.state == "initial":
            logger.warning("Set state machine: chat")
            self.statemachine.chat()
        if scene_name is not None:
            self.statemachine.scene = scene_name
        else:
            scene_name = self.statemachine.scene
        if not scene_name:
            logger.warning("Not in any scene")
            return

        scene = self.scenes.get(scene_name)
        if scene:
            if scene.type == "preset":
                for agent in copy.copy(self.server.agents).values():
                    if not isinstance(agent, LLMAgent):
                        continue
                    try:
                        if callable(agent.runtime_config_callback):
                            agent.runtime_config_callback({"prompt_preset": scene.name})
                    except Exception as ex:
                        logger.exception(ex)

        arf_events = self.session_context.get(f"arf.events.{scene_name}", [])
        fired = False
        for i, arf_event in enumerate(arf_events):
            _arf_event = json.loads(arf_event)
            if _arf_event["triggered"]:
                continue
            logger.warning("Trigger ARF %s", _arf_event)
            self._emit_event(_arf_event["arf_event"])
            _arf_event["triggered"] = True
            arf_events[i] = json.dumps(_arf_event)
            fired = True
            break
        if fired:
            # update arf events
            self.session_context.proxy.set_param(f"arf.events.{scene_name}", arf_events)
        else:
            logger.warning("No ARF to fire")

    def _load_chat_data(self):
        """Loads scene data, prompt templates and prompt preset"""
        self.data_loader = DataLoader(CMS_DIR, self.session_context)
        self.data_loader.load_all_data()
        self.scenes.update(self.data_loader.scenes)
        self.scene_manager.update_scenes(self.scenes)
        self.presets = self.data_loader.presets
        self._update_agent_presets()
        self._init_state_machine()

    def _update_agent_presets(self):
        agents = [
            agent
            for agent in self.server.agents.values()
            if isinstance(agent, LLMAgent) and agent.runtime_config_description
        ]
        for agent in agents:
            self.agent_reconfiguration.update_presets(agent, self.presets)

    def _init_state_machine(self):
        self.statemachine = RobotState(
            self.session_context,
            name=character,
            on_enter_scene_callback=self.on_enter_scene_callback,
            on_exit_scene_callback=self.on_exit_scene_callback,
        )
        self.statemachine.load_scenes(self.scenes.values())
        self.statemachine.manual()

    def _content_update_cb(self, msg):
        if msg.data == "updated":
            self._load_chat_data()
        if msg.data == "reload":
            self.reset_session()

    def _interlocutor_cb(self, msg):
        self.session_context["interlocutor"] = msg.data
        self.session_context.proxy.expire("interlocutor", 3600)

    def set_language(self, lang):
        try:
            rospy.set_param("/hr/lang", lang)
            dyn_client = Client("/hr/perception/speech_recognizer", timeout=1)
            dyn_client.update_configuration({"language": lang})
            logger.info("Updated language to %s", lang)
            return True
        except Exception as ex:
            logger.exception(ex)
            return False

    def _say(
        self, text, lang, agent_id, request_id="", audio_path="", ignore_state=False
    ):
        if not ignore_state and self.state.is_full_stopped():
            logger.warning("Robot is in full-stopped mode")
            return
        text = re.sub(r"""\[callback.*\]""", "", text)

        msg = TTS()
        msg.text = text
        # detect the language of the text
        if lang != "en-US" and "speak in" not in text.lower():
            detected_lang = detect_language(text) or self.current_language
            if detected_lang != lang:
                logger.warning(
                    "Detected language %s is different from the language %s",
                    detected_lang,
                    lang,
                )
                lang = detected_lang
        msg.lang = lang
        msg.agent_id = agent_id
        msg.request_id = request_id
        msg.audio_path = audio_path
        self._response_publisher.publish(msg)

        # Ignore speech in autonomous mode
        if self.cfg.ignore_speech_while_thinking and not self.cfg.hybrid_mode:
            # ignore final results for 2 seconds after TTS message published:
            # The 2 seconds should be enpough for most cases with good enough network connection, in case TTS is internet based
            self.ignore_speech_until = time.time() + 2.0

    def _set_hybrid_mode(self, hybrid_mode):
        dyn_client = Client("/hr/interaction/chatbot", timeout=1)
        dyn_client.update_configuration({"hybrid_mode": hybrid_mode})
        if hybrid_mode:
            self.state.update(last_auto_response_time=None)
        else:
            self.session_context["state"] = ""  # reset state

    def _handle_post_action(self, action):
        if not action:
            return
        action_type = action["type"]
        if action_type == action_types.SET_HYBRID_MODE:
            if self.cfg.auto_automonous_free_chat:
                self._set_hybrid_mode(True)

    def reset_session(self):
        self.server.session_manager.agent_sessions(True)
        rospy.set_param("~session", self.session_context.sid)
        if self.session_context and "block_chat" in self.session_context:
            del self.session_context["block_chat"]
        logger.warning("Reset. New session %s", self.session_context.sid)
        self.new_conversation()

    def reset(self):
        self.reset_session()
        self.controller_manager.reset()
        self.chat_requests = []
        self.session_context.clear()
        self.session_context["turns"] = 0
        self.server.ranker.records = []
        # Reset should not reset the current scene, so it will be the same as before
        scene = self.scene_reconfiguration.get_current_scene()
        if scene:
            self.on_enter_scene_callback(scene)

    def _handle_action(self, action):
        if not action:
            return
        action_type = action["type"]
        payload = action["payload"]
        if action_type == action_types.RESET:
            self.reset()
        if action_type == action_types.MONITOR:
            self._publish_event(payload)
        if action_type == action_types.PLACEHOLDER_UTTERANCE:
            self._say(payload["text"], self.current_language, payload["controller"])

    def _publish_event(self, event):
        type = event["type"]
        payload = event["payload"]
        msg = EventMessage()
        msg.type = type
        msg.payload = json.dumps(payload)
        msg.stamp = rospy.Time.now()
        self.event_pub.publish(msg)

    def _speech_chat_callback(self, msg):
        """If directly listening for speech"""
        if self.cfg.listen_speech:
            # Use seprate threads so it can be interrupted
            t = threading.Thread(target=self._chat_callback, args=(msg,))
            t.daemon = True
            t.start()
            self.session_context["last_active_time"] = datetime.utcnow()

    def is_asr_running(self):
        try:
            if self.asr_dyn_client is None:
                self.asr_dyn_client = Client(
                    "/hr/perception/speech_recognizer", timeout=1
                )
            asr_enabled = self.asr_dyn_client.get_configuration(timeout=1)["enable"]
        except Exception:
            asr_enabled = False
        return asr_enabled

    def _chat_callback(self, msg):
        """Responds to topic message"""
        with self.pre_lock:
            if self.cfg.concat_multiple_speech:
                # Only cancel request if there was speech activity
                if self.chat_interupt_by_activity.is_set():
                    if msg.utterance[0] not in [
                        ":",
                        "{",
                    ] and not msg.utterance.lower().startswith("event."):
                        self.chat_interupt_by_speech.set()
                        msg.utterance = self.chat_buffer + " " + msg.utterance

        # update webui language
        self.session_context["webui_language"] = LANGUAGE_CODES_NAMES.get(
            self.current_language, self.current_language
        )

        # Lock will wait for interuption to be handled so it will not execute two chats at the same time. in most cases the chat will be just waiting for chat to finish
        with self.lock:
            if not self.enable:
                return
            if not msg.utterance:
                return
            if msg.source == "rosbot":
                return
            if self.ignore_speech_until > time.time():
                return
            logger.info("Received chat message %r in %r", msg.utterance, msg.lang)
            self.start_silence_detection.clear()
            if self.silence_event_timer and self.silence_event_timer.is_alive():
                self.silence_event_timer.cancel()
            # is_prompt. brackets will define if the string is prompt[]
            is_prompt = msg.utterance[0] == "{" and msg.utterance[-1] == "}"

            if self.state.is_interruption_mode():
                logger.info("Ignore when it is in interruption resuming mode")
                return

            offline = msg.source == "fallback"
            if offline:
                logger.info("Offline speech input %r", msg.utterance)

            # ignore gibberish vosk inputs
            if offline and msg.utterance in [
                "by",
                "but",
                "both",
                "back",
            ]:
                logger.info("Ignore gibberish vosk inputs")
                return

            if not is_prompt and not offline:
                # Inform agents of the speech if they need that
                for a in self.server.agents.values():
                    a.speech_heard(msg.utterance, msg.lang)

                # update redis memory
                messages = self.chat_memory.messages
                last_message = messages[-1] if messages else None
                if not (
                    last_message
                    and last_message.type == "human"
                    and last_message.content == msg.utterance
                ):
                    # only add new message when the last message is from user and is different
                    # this is the case when you resend the question
                    self.chat_memory.add_user_message(msg.utterance)

            if self.cfg.enable_placeholder_utterance_controller:
                placeholder_contrller = self.controller_manager.get_controller(
                    "placeholder_utterance_controller"
                )
                if placeholder_contrller:
                    placeholder_contrller.enabled = True

            utterance = {}
            utterance["lang"] = msg.lang
            utterance["text"] = msg.utterance
            utterance["uuid"] = str(uuid.uuid4())
            if msg.utterance.startswith(":"):
                self.state.update(command=utterance)
                self.controller_manager.wait_for(event_types.USER_COMMAND, 1)
            else:
                self.state.update(utterance=utterance)
                self.controller_manager.wait_for(event_types.UTTERANCE, 1)

            # TODO: make sure the events arrive to controllers before calling act()
            # Or make chat as a controller
            self.controller_manager.wait_controller_finish()

            actions = self.controller_manager.act()
            if actions:
                for action in actions:
                    self._handle_action(action)
            else:
                logger.info("No actions")

            audio = os.path.basename(msg.audio_path) if msg.audio_path else ""

            if self.is_asr_running():
                if (
                    offline
                    and not self.cfg.offline_asr_free_chat
                    and self.internet_available
                ):
                    logger.info("Ignore offline asr when online asr is running")
                    return

            if self.autonomous_mode:
                if (
                    not msg.utterance.lower().startswith("event.")
                    and self.state.is_robot_speaking()
                    and not self.interrupted.is_set()
                ):
                    logger.warning(
                        "Ignore chat %r while robot is speaking",
                        msg.utterance,
                    )
                    return

            request = self.server.new_request(
                self.session_context.sid,
                msg.utterance,
                msg.lang or self.current_language,
                audio=audio,
                source=msg.source,
                session_context=self.session_context,
            )

            # if agents temperarily blocked
            if self.session_context.get("block_chat", False):
                logger.warning("Blocking other chat agents")
                request.context["agent"] = "AdhocBot"

            if (
                offline
                and not self.cfg.offline_asr_free_chat
                and self.internet_available
            ):
                request.context["agent"] = "AdhocBot"
                request.context["require_priority_content"] = True
                logger.info("Restrict the offline asr input to the priority content")

            if self.autonomous_mode:
                if (
                    not msg.utterance.lower().startswith("event.")
                    and self.state.is_robot_speaking()
                    and not self.interrupted.is_set()
                ):
                    logger.warning(
                        "Ignore chat request %r while robot is speaking",
                        request.question,
                    )
                    return
                # Allow streaming in autonomous mode
                request.allow_stream = True

            if not msg.utterance.lower().startswith(
                "event."
            ) and not msg.utterance.startswith(":"):
                self.session_context["input"] = msg.utterance

            self._chat(request)

            if actions:
                for action in actions:
                    self._handle_post_action(action)

    @property
    def current_language(self):
        current_language = rospy.get_param("/hr/lang", self.default_language)
        if self.last_language != current_language:
            logger.warning(
                "Switch language from %r to %r", self.last_language, current_language
            )
            self.server.on_switch_language(self.last_language, current_language)
            self.chat_memory.clear()
            self.last_language = current_language
        return current_language

    def _filter_responses_by_tag(self, responses, tag):
        if tag == "priority":
            return [
                r
                for r in responses
                if tag in r.attachment.get("tag", [])
                or "Skill" in r.attachment.get("topic_type", [])
            ]
        else:
            return [r for r in responses if tag in r.attachment.get("tag", [])]

    def resolve_responses(self, responses):
        if responses:
            response = self.server.resolver.resolve(responses)
            if response:
                response.attachment["published"] = True
                self.server.publish(
                    response.response_id, resolver_type=self.server.resolver.type
                )
                return [response] + [
                    r for r in responses if not r.attachment.get("published")
                ]

    def _chat(self, request):
        # Handles chat. Event is passed if the chat is interupted and results no need to be published or used.
        if not self.enable:
            return
        self.activity_monitor.record_chat_activity()
        # clear the interupt flag, it will be set(based ons ettings) if the speach is heard and chat do not need to wait.
        self.chat_interupt_by_speech.clear()
        self.chat_interupt_by_activity.clear()
        self._chat_event_pub.publish("start thinking")
        has_response = False
        self.current_responses = {}

        if self.cfg.enable_rag:
            self.scene_manager.create_rag(request.question)

        request.hybrid_mode = self.hybrid_mode

        if self.hybrid_mode:
            for responses in self.server.chat_with_ranking(request):
                if responses:
                    priority_responses = self._filter_responses_by_tag(
                        responses, "priority"
                    )
                    if request.context.get("require_priority_content"):
                        responses = priority_responses
                    if responses:
                        has_response = True

                    if self.cfg.auto_automonous_free_chat and request.source not in [
                        "web",
                        "webrepeat",
                    ]:
                        # check whether activate autonomous free chat
                        resolved_responses = self.resolve_responses(priority_responses)
                        if resolved_responses:
                            self._set_hybrid_mode(False)
                            logger.warning("Priority rule has been triggered")
                            self._publish_ros_responses(resolved_responses)
                            break
                    self._publish_ros_responses(responses)
                for response in responses:
                    self.current_responses[response.response_id] = response
        else:
            interupt_event = None
            if self.cfg.concat_multiple_speech:
                interupt_event = self.chat_interupt_by_speech
                self.chat_buffer = request.question

            responses = (
                self.server.chat_with_resolving(
                    request,
                    fast_score=self.cfg.fast_score,
                    interrupt_event=interupt_event,
                )
                or []
            )
            for response in responses:
                self.current_responses[response.response_id] = response

            # Responses returned however if there are new speech we need to wait for timeout to continue or speech to be interrupted
            cancelled = False
            if self.cfg.concat_multiple_speech:
                while not self.chat_interupt_by_speech.is_set():
                    if not self.chat_interupt_by_activity.is_set():
                        # Not interrupted so nothing to wait for
                        break
                    if (
                        self.last_speech_activity + self.speech_to_silence_interval
                        < time.time()
                    ):
                        # Timeout
                        break
                    time.sleep(0.02)
                if self.chat_interupt_by_speech.is_set():
                    cancelled = True
            if not cancelled:
                self.chat_buffer = ""
                if responses:
                    priority_responses = self._filter_responses_by_tag(
                        responses, "priority"
                    )
                    if request.context.get("require_priority_content"):
                        responses = priority_responses
                        if not responses:
                            logger.warning("No non-priority responses after removal")
                if responses:
                    has_response = True
                    self._publish_ros_responses(responses)
                    self.interrupted.clear()
        self._chat_event_pub.publish("stop thinking")
        if not has_response:
            logger.info("Chatbot has no response")
            self._chat_event_pub.publish("no response")
        return has_response

    def _publish_ros_responses(self, responses):
        """Publishes responses to ROS topic"""
        if not responses:
            return
        if not self.hybrid_mode:
            # publish the first published response
            resolved_response = responses[0]
            if resolved_response.attachment.get("published"):
                logger.info("Choose %s", resolved_response)
                self._publish_resolved_response(resolved_response)
                self.state.update(last_auto_response_time=time.time())
                # check whether deactivate autonomous free chat
                if "deactivate" in resolved_response.attachment.get("tag", []):
                    if self.cfg.auto_automonous_free_chat:
                        self._set_hybrid_mode(True)
                        logger.warning(
                            "Autonomous mode is deactivated by deactivation rules"
                        )
                if "activate" in resolved_response.attachment.get("tag", []):
                    if self.cfg.auto_automonous_free_chat:
                        self._set_hybrid_mode(False)
                        logger.warning("Activation rule has been triggered")
                responses = responses[1:]

        if not responses:
            return
        uniq_responses = remove_duplicated_responses(responses)
        uniq_responses = list(uniq_responses)

        responses_msg = ChatResponses()
        for response in uniq_responses:
            text = response.answer
            if text:
                # check whether activate autonomous free chat
                # this goes autonomous in any event
                if "activate" in response.attachment.get("tag", []):
                    if self.cfg.auto_automonous_free_chat:
                        self._set_hybrid_mode(False)
                        logger.warning("Activation rule has been triggered")
                    if not self.hybrid_mode:
                        self._publish_resolved_response(response)
                    else:
                        response_msg = ChatResponse()
                        response_msg.text = text
                        response_msg.lang = response.lang
                        response_msg.label = response.agent_id
                        response_msg.agent_id = response.agent_id
                        response_msg.request_id = response.request_id
                        response_msg.response_id = response.response_id
                        responses_msg.responses.append(response_msg)
                else:
                    response_msg = ChatResponse()
                    response_msg.text = text
                    response_msg.lang = response.lang
                    response_msg.label = response.agent_id
                    response_msg.agent_id = response.agent_id
                    response_msg.request_id = response.request_id
                    response_msg.response_id = response.response_id
                    responses_msg.responses.append(response_msg)

        if responses_msg.responses:
            self._responses_publisher.publish(responses_msg)

    def _publish_resolved_response(self, response: AgentResponse, ignore_state=False):
        """
        Publish the resolved response and handle any associated actions or contexts.
        """
        if response.answer:
            self.server.add_record(response.answer)
        self._after_published_responses(response.request_id)
        self.server.feedback(response, self.hybrid_mode)

        self.last_chosen_response = response
        self._say(
            response.answer,
            response.lang,
            response.agent_id,
            response.request_id,
            ignore_state=ignore_state,
        )
        # Handle the responses that are streaming, this will block until all the sentences are published for TTS
        if isinstance(response, AgentStreamResponse):
            self.stream_responses.put(response)
            # Long timeout for any unforeseen errors
            response.stream_finished.wait(timeout=response.stream_response_timeout)
            response.stream_finished.set()

        actions = response.attachment.get("actions", [])
        if actions:
            try:
                self._execute_action(actions)
            except Exception as ex:
                logger.error(ex)
        input_context = response.attachment.get("input_context", [])
        for c in input_context:
            # consume context
            key = f"context.output.{c}"
            logger.info("Consume context %r", key)
            del self.session_context[key]

    def _execute_action(self, actions):
        for action in actions:
            if action.get("name") == "switch-language":
                lang = action["properties"]["lang"]
                success = self.set_language(lang)
                if success:
                    logger.warning("Set target language %r successfully", lang)
            if action.get("name") == "play-audio-clip":
                text = action["properties"]["text"]
                audio = action["properties"]["audio-clip"]
                lang = action["properties"]["lang"]
                if not audio.startswith("/"):
                    # relative to the upload dir
                    audio = os.path.join(SOULTALK_HOT_UPLOAD_DIR, audio)
                self._say(text, lang, agent_id="Action", audio_path=audio)
            if action.get("name") == "NEXTTOPIC" and self.cfg.auto_fire_arf:
                logger.warning("Next topic")
                self.fire_arf()
            if action.get("name") == "set":
                for key, value in action["properties"].items():
                    self.session_context[key] = value
                    logger.warning("Set %s=%s", key, value)
                    if key == "arf_count_down" and self.cfg.auto_fire_arf:
                        if self.arf_fire_timer:
                            self.arf_fire_timer.cancel()
                        logger.warning("Auto ARF counting down %s", value)
                        self.arf_fire_timer = threading.Timer(value, self.auto_fire_arf)
                        self.arf_fire_timer.run()

    def auto_fire_arf(self):
        while self.cfg.auto_fire_arf:
            logger.warning("Checking auto arf")
            if self.user_speaking:
                logger.warning("User is speaking, do not fire new ARF")
                time.sleep(5)
                continue
            if self.session_context.get("block_chat"):
                logger.warning("Chat is blocked, do not fire new ARF")
                time.sleep(5)
                continue
            if (
                "topic_type" in self.last_chosen_response.attachment
                and self.last_chosen_response.attachment["topic_type"] == "ARF"
            ):
                logger.warning("The last ARF hasn't been answered, do not fire new ARF")
                time.sleep(5)
                continue
            if self.last_chosen_response and self.last_chosen_response.answer.endswith(
                "?"
            ):
                logger.warning("Wait for user answering the question")
                time.sleep(15)
                continue
            self.fire_arf()

    def ros_set_output_context(self, context: dict, output=False, finished=False):
        logger.info("ros set context: %r", context)
        if "output" in context:
            output = context.pop("output")
        if "finished" in context:
            finished = context.pop("finished")
        try:
            self.server.set_context(self.session_context.sid, context, output, finished)
            return True, ""
        except Exception as ex:
            logger.exception(ex)
            return False, str(ex)

    def ros_service_set_context(self, req):
        response = StringTrigger._response_class()
        try:
            context = json.loads(req.data)
        except Exception as ex:
            logger.exception(ex)
            response.success = False
            response.message = str(ex)
            return response
        if context:
            success, message = self.ros_set_output_context(context)
            if not success:
                response.success = success
                response.message = message
                return response
        response.success = True
        response.message = "ok"
        return response

    def ros_service_get_context(self, req):
        response = StringTrigger._response_class()
        try:
            context = self.server.get_context(self.session_context.sid)
            if req.data:
                context = context.get(req.data)
            if context:
                response.message = json.dumps(context)
            response.success = True
        except Exception as ex:
            logger.error(ex)
            response.success = False
            response.message = str(ex)
        return response

    def ros_service_register(self, req):
        response = AgentRegister._response_class()
        try:
            agent = ROSGenericAgent(
                req.node,
                req.languages,
                level=req.level,
                weight=req.weight,
                ttl=req.ttl,
            )
            self.server.agents[agent.id] = agent
            response.success = True
        except Exception as ex:
            logger.error(ex)
            response.success = False
            response.message = str(ex)
        return response

    def ros_service_unregister(self, req):
        response = AgentUnregister._response_class()
        if not req.node:
            response.success = False
            response.message = "Agent id was missing"
            return response
        if req.node in self.server.agents:
            del self.server.agents[req.node]
            response.success = True
        else:
            response.success = False
            response.message = (
                'Agent "%s" was not registered or has already unregistered' % req.node
            )
        return response

    def list_all_installed_agents(self, req):
        response = StringArray._response_class()
        response.data.extend(
            ["%s:%s" % (agent.type, agent.id) for agent in self.server.agents.values()]
        )
        return response

    def list_all_scenes(self, req):
        response = StringArray._response_class()
        response.data.extend(self.scenes.keys())
        return response

    def set_character(self, character):
        self.switch_character_pub.publish(character)

    def set_robot_mode(self, cfg):
        logger.warning("Set robot mode %s", cfg.robot_mode)
        config = self.modes_config.get(cfg.robot_mode)
        if config:
            for node, node_config in config.items():
                if node == "/hr/interaction/chatbot":
                    try:
                        for key, value in node_config.items():
                            setattr(cfg, key, value)
                    except Exception as ex:
                        logger.error(ex)
                else:
                    try:
                        client = Client(node, timeout=10)
                        client.update_configuration(node_config)
                        logger.info("Updated node %r config %s", node, node_config)
                    except Exception as ex:
                        logger.error(ex)
        return cfg

    def reconfig(self, config, level):
        if self.cfg is None:
            config.listen_speech = True
            self.cfg = config
            self.start()

        self.cfg = config

        if self.enable != config.enable:
            self.enable = config.enable
            if self.enable:
                logger.warning("Enabled chatbot")
            else:
                logger.warning("Disabled chatbot")
        if self.hybrid_mode:
            logger.info("Enabled hybrid mode")
            self.state.update(last_auto_response_time=None)
        else:
            logger.info("Disabled hybrid mode")

        self.server.set_timeout(config.timeout, config.min_wait_for)

        # set controllers
        self.controller_manager.setup_controllers(self.cfg)

        if config.enable_global_workspace_drivers:
            logger.warning("Enabling global workspace drivers")
            self.driver_reconfiguration.enable_global_workspace()
            # boost engagement level
            # so it can turn off GW when engagement level is dropped to NONE
            self.activity_monitor.record_chat_activity()
        else:
            logger.warning("Disabling global workspace drivers")
            self.driver_reconfiguration.disable_global_workspace()
        self.driver_reconfiguration.auto_global_workspace = config.auto_global_workspace

        # set character
        # if self.cfg.character:
        #    self.set_character(self.cfg.character)
        if self.robot_mode != config.robot_mode:
            self.cfg = self.set_robot_mode(self.cfg)
            self.robot_mode = config.robot_mode

        if self.cfg.enable_emotion_driven_response_primer:
            logger.warning("Emotion driven response primer is enabled")
            self.session_context["emotion_driven_response_primer"] = True
        else:
            logger.warning("Emotion driven response primer is disabled")
            self.session_context["emotion_driven_response_primer"] = False

        return self.cfg


if __name__ == "__main__":
    rospy.init_node("chatbot")
    bot = Chatbot()
    rospy.spin()
