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

import base64
import json
import logging
import os
import threading
import uuid
from datetime import datetime, timezone

import requests
import rospy
from haipy.nlp.intent_classifier import IntentDetector
from hr_msgs.srv import GetIntent, StringTrigger

# Subscribe to the compressed image type instead of the raw Image type
from sensor_msgs.msg import CompressedImage

logger = logging.getLogger("hr.ros_chatbot.services")

import cv2
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
api_key = os.environ.get("OPENAI_API_KEY")

IMAGE_FEED_HOME = os.environ.get("IMAGE_FEED_HOME", "/tmp/camera_feed")
openai_url = "https://api.openai.com"
alt_openai_url = os.environ.get("HR_OPENAI_PROXY", openai_url)
logger.warning(f"Using OpenAI URL: {alt_openai_url}")

class ChatbotServices(object):
    def __init__(self):
        self._intent_detector = IntentDetector("rasa")
        image_topic = rospy.get_param(
            "~image_topic", "/hr/perception/jetson/realsense/camera/color/image_raw/compressed"
        )
        # Subscribe using the CompressedImage message type
        rospy.Subscriber(image_topic, CompressedImage, self._fresh_image)
        rospy.Service("get_intent", GetIntent, self.get_intent)
        rospy.Service("describe_view", StringTrigger, self.describe_view)
        self.current_cv2_image = None
        self.lock = threading.RLock()
        # self.model = "gpt-4-vision-preview"
        self.model = "gpt-4o"
        self.current_openai_url = openai_url
        # GMT time
        self.last_active_time = datetime.now(timezone.utc)

    def _fresh_image(self, msg):
        try:
            with self.lock:
                # Convert the compressed image message to an OpenCV image
                self.current_cv2_image = bridge.compressed_imgmsg_to_cv2(msg, "bgr8")
                self.last_active_time = datetime.now(timezone.utc)
        except CvBridgeError as ex:
            logger.error(ex)

    def get_intent(self, req):
        response = GetIntent._response_class()
        try:
            result = self._intent_detector.detect_intent(req.text, req.lang)
            if result:
                response.intent = result["intent"]["name"]
                response.confidence = result["intent"]["confidence"]
        except Exception as ex:
            logger.error(ex)
        return response

    def describe_view(self, req):
        ret = StringTrigger._response_class()
        ret.success = False
        try:
            if not os.path.isdir(IMAGE_FEED_HOME):
                os.makedirs(IMAGE_FEED_HOME)
            datetime_str = datetime.strftime(datetime.now(), "%Y%m%d%H%M%S")
            hex = uuid.uuid4().hex
            image_file_name = f"{datetime_str}-{hex}.jpg"
            image_path = os.path.join(IMAGE_FEED_HOME, image_file_name)
            current_time = None
            with self.lock:
                if self.current_cv2_image is not None:
                    cv2.imwrite(image_path, self.current_cv2_image)
                    current_time = self.last_active_time   
                else:
                    ret.success = False
                    ret.message = "No image feed"
                    return ret

            def encode_image(image_path):
                with open(image_path, "rb") as image_file:
                    return base64.b64encode(image_file.read()).decode("utf-8")

            base64_image = encode_image(image_path)
            headers = {
                "Content-Type": "application/json",
                "Authorization": f"Bearer {api_key}",
            }
            payload = {
                "model": self.model,
                "messages": [
                    {
                        "role": "user",
                        "content": [
                            {"type": "text", "text": req.data},
                            {
                                "type": "image_url",
                                "image_url": {
                                    "url": f"data:image/jpeg;base64,{base64_image}"
                                },
                            },
                        ],
                    }
                ],
                "max_tokens": 800,
            }
            logger.warning("Sending image to GPT vision")
            response = requests.post(
                f"{self.current_openai_url}/v1/chat/completions", headers=headers, json=payload
            ).json()
            if "error" in response and response["error"]:
                ret.success = False
                ret.message = response["error"]["message"]
                self.current_openai_url = alt_openai_url if self.current_openai_url == openai_url else openai_url
            else:
                ret.success = True
                ret.message = json.dumps(
                    {
                        "content": response["choices"][0]["message"]["content"],
                        "image_file_name": image_file_name,
                        "time": {"value": datetime_str, "format": "%Y%m%d%H%M%S"},
                        "utc_time": current_time.isoformat(),
                    }
                )
            return ret
        except Exception as ex:
            logger.error(ex)
            ret.message = str(ex)
            return ret


if __name__ == "__main__":
    rospy.init_node("chatbot_services")
    ChatbotServices()
    rospy.spin()
