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

import rospy
import websocket
import random
import json
import threading
import time
import logging
from std_msgs.msg import String
from hr_msgs.msg import ChatMessage, TTS, ChatResponse, Event, ChatResponses
from hr_msgs.srv import AgentChat, AgentFeedback,  RunByName, RunByNameRequest, RunByNameResponse
from dynamic_reconfigure.client import Client

# WebSocket configuration
WEBSOCKET_SERVER_URL = "wss://n8n-dev.hr-tools.io/wss"

# ROS topic to subscribe to
ROS_SUBSCRIBE_TOPIC = "/hr/interaction/n8n/robot_events"
ROS_PUBLISH_TOPIC = "/hr/interaction/n8n/received_events"
ROS_HEAR_EVENT_TOPIC = "/hr/perception/hear/sentence"
ROS_SAID_EVENT_TOPIC = "/hr/control/speech/say"
SESSION_ID_PARAM = "/hr/interaction/n8n/session_id"
SCENE_CONFIG_TOPIC = "/hr/interaction/prompts/scene"
CHAT_RESPONSES_TOPIC = "/hr/interaction/n8n/responses"
PERFORMANCES_SERVICE = "/hr/control/performances/background/run_by_name"
PERFORMANCES_STATUS_TOPIC = "/hr/control/performances/background/events"
PERFORMANCES_RUNNING_TOPIC =  "/hr/control/performances/background/running_performance"
CHATBOT_SUGGESTIONS = "/hr/interaction/chatbot_responses"
RPS_TOPIC="/hr/perception/rps_result"
ROS_AGENT_SETTINGS="/hr/interaction/agents/n8n"
VISUAL_PROMPTS="/hr/interaction/prompts/visual_processing"


# Configure logging
logger = logging.getLogger('hr.ros_chatbot.n8n')
logger.setLevel(logging.INFO)
handler = logging.StreamHandler()
formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
handler.setFormatter(formatter)
logger.addHandler(handler)

class WebSocketROSNode:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('websocket_ros_node', anonymous=True)
        # Get or create session_id parameter
        if not rospy.has_param(SESSION_ID_PARAM):
            rospy.set_param(SESSION_ID_PARAM, "session_{}".format(rospy.Time.now().to_nsec()))
        self.metaprompting_allowed = False
        self.current_scene = None
        self.last_scene_msg = None
        self.scenes = None
        self.session_id = rospy.get_param(SESSION_ID_PARAM)
        self.robot_name = rospy.get_param("/hr/robot_name")
        self.robot_body = rospy.get_param("/hr/robot_body")
        self.robot_character = rospy.get_param("/hr/character")
        # ROS subscriber
        self.subscriber = rospy.Subscriber(ROS_SUBSCRIBE_TOPIC, String, self.on_ros_message_received)
        
        # ROS hear event subscriber
        self.hear_event_subscriber = rospy.Subscriber(ROS_HEAR_EVENT_TOPIC, ChatMessage, self.on_hear_event_received)
        
        # ROS said event subscriber
        self.said_event_subscriber = rospy.Subscriber(ROS_SAID_EVENT_TOPIC, TTS, self.on_said_event_received)
        
        # ROS publisher
        self.publisher = rospy.Publisher(ROS_PUBLISH_TOPIC, String, queue_size=10)
        # Chat responses to chatbot
        self.chat_responses = rospy.Publisher(CHAT_RESPONSES_TOPIC, ChatResponse, queue_size=10)
        # Initialize WebSocket client
        self.ws = None
        self.ws_lock = threading.Lock()
        self.ws_connected = threading.Event()
        self.ws_thread = threading.Thread(target=self.connect_ws)
        self.ws_thread.daemon = True
        self.ws_thread.start()
        self.last_chatbot_session = None
        # Wait for WebSocket to be connected
        logger.info("Waiting for WebSocket connection to be established...")
        self.ws_connected.wait(timeout=2)
        if not self.ws_connected.is_set():
            logger.warning("WebSocket connection not established within the timeout period.")
        else:
            logger.info("WebSocket connection established.")
        self.enabled= None
        # Start ping thread
        self.ping_thread = threading.Thread(target=self.ping_ws)
        self.ping_thread.daemon = True
        self.ping_thread.start()
        self.n8n_agent_configs = Client(ROS_AGENT_SETTINGS, config_callback=self.n8n_cfg_callback)
        while self.enabled is None:
            # wait for callback, so make sure the scene update after
            time.sleep(0.1)
        # Dynamic reconfigure client for scene config
        self.robot_connected()
        self.scene_client = Client(SCENE_CONFIG_TOPIC, config_callback=self.on_scene_config_update)
        self.current_scene = None
        # Visual queues
        self.visual_prompts_configs = Client(VISUAL_PROMPTS, config_callback=self.on_visual_config_update)
        

        # ROS services
        self.agent_chat_service = rospy.Service('/hr/interaction/n8n/chat', AgentChat, self.handle_agent_chat)
        self.agent_feedback_service = rospy.Service('/hr/interaction/n8n/feedback', AgentFeedback, self.handle_agent_feedback)
        self.performance_runner = rospy.ServiceProxy(PERFORMANCES_SERVICE, RunByName)
        
        # Topics
        self.last_perfromance_event = None
        self.idle_performance_event = threading.Event()
        self.performance_events_sub = rospy.Subscriber(PERFORMANCES_STATUS_TOPIC, Event, self.performance_event_callback)
        # RPS_reselts
        self.rps_topic = rospy.Subscriber(RPS_TOPIC, String, self.rps_callback)
        
        # Suggested responses
        self.suggested_responses = rospy.Publisher(CHATBOT_SUGGESTIONS, ChatResponses)
        # TTS
        self.tts_pub = rospy.Publisher(ROS_SAID_EVENT_TOPIC, TTS)

        
    def n8n_cfg_callback(self, cfg, lvl=None):
        if cfg.enabled and self.enabled is False:
            self.change_session()
        try:
            self.metaprompting_allowed = cfg.allow_metaprompting
        except Exception:
            self.metaprompting_allowed = False
        self.enabled = cfg.enabled
        return cfg
    
    def robot_connected(self):
        self.send_to_ws({
             "event_type": "RobotConnected",
             "robot_name": self.robot_name,
             "robot_body": self.robot_body,
             "robot_character": self.robot_character,
             "session_id": self.session_id
         })

    def connect_ws(self):
        headers = {
            "X-API-Key": 'n8n-Top-Secret-code-to-be-never-used'
        }
        while not rospy.is_shutdown():
            try:
                logger.info("Connecting to WebSocket server...")
                self.ws = websocket.WebSocketApp(
                    WEBSOCKET_SERVER_URL,
                    header=[f"{key}: {value}" for key, value in headers.items()],
                    on_message=self.on_ws_message,
                    on_open=self.on_ws_open,
                    on_close=self.on_ws_close,
                    on_error=self.on_ws_error,
                    on_ping=self.on_ws_ping
                )
                self.ws.run_forever()
                time.sleep(2)
            except Exception as e:
                logger.error("WebSocket connection failed: {}. Retrying in 5 seconds...".format(e))
                
    def send_to_ws(self, message):
        if not self.enabled:
            return True
        try:
            with self.ws_lock:
                if self.ws and self.ws.sock and self.ws.sock.connected:
                    if "session_id" not in message:
                        message["session_id"] = self.session_id
                    self.ws.send(json.dumps(message))
                    return True
        except Exception as e:
            logger.warning("Failed to send message to WebSocket: {}".format(e))
        return False

    def on_ws_open(self, ws):
        logger.warn("Connected to WebSocket server")
        self.ws_connected.set()
        self.robot_connected()

    def on_ws_close(self, ws, close_status_code, close_msg):
        logger.warning("WebSocket connection closed. Reconnecting...")
        self.ws_connected.clear()

    def on_ws_error(self, ws, error):
        logger.error("WebSocket error: {}".format(error))
        self.ws_connected.clear()

    def on_ws_message(self, ws, message):
        if not self.enabled:
            return None
        logger.info("Received message from WebSocket: {}".format(message))
        msg = json.loads(message)
        if not 'event_type' in msg:
            pass
        if msg['event_type'] == 'ChatResults':
            self.publish_chat_response(msg)
        if msg['event_type'] == 'Performance':
            self.run_performance_from_message(msg)
        if msg['event_type'] == 'TTS':
            self.publish_tts(msg)
        if msg['event_type'] == 'UpdateScene':
            self.update_current_scene(msg)
        if msg['event_type'] == 'SetRobotSettings':
            self.set_robot_params(msg)
    
    
    def update_current_scene(self, msg):
        # Do not update anything unless there is a an issue
        if not self.metaprompting_allowed:
            return
        new_scene = msg.get('scene', {})
        changed = False
        if not self.current_scene or not self.scenes:
            return

        for scene in self.scenes:
            if scene.get('name') != self.current_scene:
                continue
            for key, value in new_scene.items():
                if key in scene:
                    scene[key] = value
                    changed = True
        if changed:
            self.scene_client.update_configuration({'scenes': json.dumps(self.scenes)})
            
            
    def on_ws_ping(self, ws, message):
        logger.info("Received ping from WebSocket server. Sending pong response.")
        try:
            ws.send('PONG')
        except Exception as e:
            pass

    def ping_ws(self):
        while not rospy.is_shutdown():
            if self.ws and self.ws.sock and self.ws.sock.connected:
                try:
                    logger.info("Sending ping to WebSocket server")
                    self.ws.sock.ping()
                except Exception as e:
                    logger.warning("Ping failed: {}".format(e))
            time.sleep(30)

    def on_ros_message_received(self, ros_message):
        logger.info("Received message from ROS topic: {}".format(ros_message.data))
        try:
            # Convert ROS message to JSON and add session_id, then send it via WebSocket
            event = json.loads(ros_message.data)
            event["session_id"] = self.session_id
            if not self.send_to_ws(event):
                logger.warning("WebSocket is not connected. Cannot send message.")
        except json.JSONDecodeError:
            logger.warning("Failed to parse ROS message as JSON")

    def on_hear_event_received(self, hear_event: ChatMessage):
        logger.info("Received hear event: utterance='{}', lang='{}', confidence={}, source='{}'".format(
            hear_event.utterance, hear_event.lang, hear_event.confidence, hear_event.source))
        try:
            # Convert hear event to JSON and add session_id, then send it via WebSocket
            utterance = hear_event.utterance.strip()
            if utterance == ':reset':
                self.change_session()
                return
            
            
            event = {
                "event_type": "RobotHears",
                "utterance": hear_event.utterance,
                "lang": hear_event.lang,
                "confidence": hear_event.confidence,
                "source": hear_event.source,
                "audio_path": hear_event.audio_path,
                "session_id": self.session_id
            }
            if not self.send_to_ws(event):
                logger.warning("WebSocket is not connected. Cannot send hear event.")
            else:
                logger.info("Sent hear event to WebSocket server: {}".format(event))
        except Exception as e:
            logger.warning("Failed to process hear event: {}".format(e))

    def on_said_event_received(self, said_event):
        logger.info("Received said event: text='{}', lang='{}', request_id='{}', agent_id='{}'".format(
            said_event.text, said_event.lang, said_event.request_id, said_event.agent_id))
        try:
            # Convert said event to JSON and add session_id, then send it via WebSocket
            event = {
                "event_type": "RobotSays",
                "text": said_event.text,
                "lang": said_event.lang,
                "request_id": said_event.request_id,
                "agent_id": said_event.agent_id,
                "audio_path": said_event.audio_path,
                "session_id": self.session_id
            }
            if not self.send_to_ws(event):
                logger.warning("WebSocket is not connected. Cannot send said event.")
            else:
                logger.info("Sent said event to WebSocket server: {}".format(event))
        except Exception as e:
            logger.warning("Failed to process said event: {}".format(e))
            
    
    def on_scene_config_update(self, config):
        try:
            current_scene_name = config.get('current', None)
            scenes_json = config.get('scenes', None)
            if current_scene_name and scenes_json:
                scenes = json.loads(scenes_json)
                self.current_scene = current_scene_name
                self.scenes = scenes
                found_scene = False
                for scene in scenes:
                    if scene.get('name') == current_scene_name:
                        self.current_scene = current_scene_name
                        found_scene = True
                        logger.info("Updated current scene: {}".format(self.current_scene))
                        # Send current scene to WebSocket
                        event = {
                            "event_type": "CurrentSceneUpdate",
                            "current_scene_name": self.current_scene,
                            "session_id": self.session_id,
                            "scene": scene
                        }
                        self.last_scene_msg = event
                        if not self.send_to_ws(event):
                            logger.warning("WebSocket is not connected. Cannot send current scene update.")
                        else:
                            logger.info("Sent current scene to WebSocket server: {}".format(event))
                        break
                if not found_scene:
                    logger.warning("Current scene '{}' not found in scenes list.".format(current_scene_name))
            else:
                logger.warning("Missing parameters in scene config update.")
        except Exception as e:
            logger.warning("Failed to update scene config: {}".format(e))
            
    def on_visual_config_update(self, cfg):
        if not cfg.results:
            return
        event = {
            "session_id": self.session_id,
            "event_type": "LLMVision",
            "visual_prompt": cfg.visual_prompt,
            "result_time": cfg.result_time,
            "results": cfg.results
        }
        if not self.send_to_ws(event):
            logger.warning("WebSocket is not connected. Cannot send vision update.")
        else:
            logger.info("Sent current vision data to WebSocket server: {}".format(event))
            
    def handle_agent_chat(self, req):
        logger.info("Handling AgentChat request: text='{}', lang='{}', session='{}'".format(req.text, req.lang, req.session))
        if self.last_chatbot_session is None:
            self.last_chatbot_session = req.session
        if self.last_chatbot_session != req.session:
            # changes session
            self.change_session()
            
        event = {
            "event_type": "ChatRequest",
            "text": req.text,
            "lang": req.lang,
            "request_id": req.request_id,
            "session_id": self.session_id,
            "response_prompt": rospy.get_param("/hr/interaction/prompts/response_prompt")
        }
        if not self.send_to_ws(event):
            logger.warning("WebSocket is not connected. Cannot send agent chat event.")
        # Example response generation (to be replaced with actual logic)
        response = AgentChat._response_class()
        response.state = 1  # Example state
        response.score = 0.95  # Example score
        return response

    def handle_agent_feedback(self, req):
        logger.info("Handling AgentFeedback request: request_id='{}', hybrid={}, chosen={}".format(req.request_id, req.hybrid, req.chosen))
        # Example response generation (to be replaced with actual logic)
        response = AgentFeedback._response_class()
        response.success = True  # Example success status
        return response
    
    def publish_chat_response(self, response):
        msg = ChatResponse()
        msg.text = response['text']
        msg.lang = response['lang']
        msg.label = response['label']
        msg.request_id = response['request_id']
        self.chat_responses.publish(msg)
    
    def performance_event_callback(self, event):
        if event.event == 'idle':
            self.idle_performance_event.set()
        
    def run_performance_from_message(self, message):
        hybrid = rospy.get_param("/hr/interaction/chatbot/hybrid_mode", False)
        performance = message['performance']
        if hybrid:
            suggestion = ChatResponse()
            suggestion.text = f"|t, {performance}|"
            suggestion.lang = 'en-US'
            suggestion.label = 'n8n'
            self.suggested_responses.publish(ChatResponses(responses=[suggestion]))
            # Probably not very accurate, but only thing we can do is push performance to operator.
            self.send_to_ws({
                "event_type": "PerformanceFinished",
                "performance": performance
            })
        else:
            t = threading.Thread(target=self.run_performance, daemon=True, args=(performance,))
            t.start()
        
    def run_performance(self, performance):
        # runs performance and blocks until it finished
        try:
            r = RunByNameRequest()
            r.id = performance
            result = self.performance_runner.call(r)
            if not result.success:
                return False
            self.idle_performance_event.clear()
            self.idle_performance_event.wait()
            self.send_to_ws({
                "event_type": "PerformanceFinished",
                "performance": performance
            })
        except Exception:
            pass
    
    def rps_callback(self, result):
        msg = {
            'event_type': 'RPSResult',
            'result': result.data
        }
        self.send_to_ws(msg)
        
    def publish_tts(self, msg):
        hybrid = rospy.get_param("/hr/interaction/chatbot/hybrid_mode", False)
        if not hybrid:
            ros_msg = TTS()
            ros_msg.text = msg['text']
            ros_msg.lang = msg['lang']
            ros_msg.agent_id = 'n8n'
            self.tts_pub.publish(ros_msg)
        else:
            suggestion = ChatResponse()
            suggestion.text = msg['text']
            suggestion.lang = msg['lang']
            suggestion.label = 'n8n'
            self.suggested_responses.publish(ChatResponses(responses=[suggestion]))
            
    def change_session(self):
        rospy.set_param(SESSION_ID_PARAM, "session_{}".format(rospy.Time.now().to_nsec()))
        self.session_id = rospy.get_param(SESSION_ID_PARAM)
        if self.last_scene_msg:
            self.last_scene_msg['session_id'] = self.session_id
            self.send_to_ws(self.last_scene_msg)
    
    def set_robot_params(self, msg):
        node = msg.get('node')
        params = msg.get('params', {})
        timeout = msg.get('timeout', 1)
        if not self.set_dyn_params(node, params, timeout):
            logger.warning("Failed to set dynamic parameters for node '{}'".format(node))
        
    def set_dyn_params(self, node: str, params: dict,timeout: int = 1):
        try:
            client = Client(node, timeout=timeout)
            client.update_configuration(params)
            return True
        except Exception as e:
            return False

if __name__ == '__main__':
    try:
        # Create and start WebSocket ROS node
        ws_ros_node = WebSocketROSNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        logger.warning("ROS node interrupted")
