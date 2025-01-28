#
# Copyright (C) 2017-2025 Hanson Robotics
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
import json
import logging
import os
import uuid
from enum import Enum

import requests

from haipy.base import IntentClassifier

# from google.cloud import dialogflow


logger = logging.getLogger(__name__)

LANGUAGE_CODE_MAPPING = {"en-US": "en", "cmn-Hans-CN": "zh-CN"}


# class DialogFlowIntentClassifier(IntentClassifier):
#    def __init__(self, project_id=None, credential=None):
#        if not project_id:
#            project_id = os.environ.get("HR_DIALOGFLOW_PROJECT_ID")
#        if not project_id:
#            raise ValueError("Project ID was not specified")
#
#        self.project_id = project_id
#        if credential:
#            os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = os.path.join(
#                os.environ.get("HR_CONFIG_STORAGE", ""),
#                "scripts/interaction",
#                credential,
#            )
#
#        self.session_client = dialogflow.SessionsClient()
#        self.session_path = self.session_client.session_path(self.project_id, "default")
#
#    def ping(self):
#        return True
#
#    def session(self, sid=None):
#        sid = sid or str(uuid.uuid1())
#        self.sid = sid
#        self.session_path = self.session_client.session_path(self.project_id, self.sid)
#        return sid
#
#    def detect_intent(self, text, lang):
#        """Returns the result of detect intent with text as input.
#
#        Using the same session id between requests allows continuation
#        of the conversation."""
#        if lang in LANGUAGE_CODE_MAPPING:
#            lang = LANGUAGE_CODE_MAPPING[lang]
#
#        text_input = dialogflow.TextInput(text=text, language_code=lang)
#
#        query_input = dialogflow.QueryInput(text=text_input)
#
#        result = self.session_client.detect_intent(
#            session=self.session_path, query_input=query_input, timeout=1
#        )
#
#        entities = []
#        for output_context in result.query_result.output_contexts:
#            for key, value in output_context.parameters.items():
#                entities.append(
#                    {"entity": key, "value": value, "extrator": "dialogflow"}
#                )
#
#        response = {}
#        response["intent"] = {
#            "name": result.query_result.intent.display_name,
#            "confidence": result.query_result.intent_detection_confidence,
#        }
#        response["entities"] = entities
#
#        return response


class SoulTalkIntentClassifier(IntentClassifier):
    def __init__(self, urls=None, timeout=1):
        if not urls:
            host = os.environ.get("NLU_SERVER_HOST", "127.0.0.1")
            port = os.environ.get("SYSTEM_INTENT_SERVICE_PORT", "10200")
            urls = ["http://%s:%s" % (host, port)]
        self.urls = urls
        self.timeout = timeout
        self.max_text_length = 128
        self.confidence_threshold = 0.4

    def ping(self, url):
        try:
            response = requests.get(url, timeout=self.timeout)
        except Exception as ex:
            logger.error(ex)
            return False
        if response.status_code == 200:
            return True
        else:
            logger.error("Intent Classification Server %s is not available", url)
        return False

    def detect_intent(self, text, lang):
        if len(text) > self.max_text_length:
            logger.error("Input text exceeds %s characters" % self.max_text_length)
            return
        if lang in LANGUAGE_CODE_MAPPING:
            lang = LANGUAGE_CODE_MAPPING[lang]
        params = {"text": text, "language": lang}
        for url in self.urls:
            if not self.ping(url):
                logger.error("Intent classification service %s is not available", url)
                continue
            response = requests.get("%s/intent" % url, params, timeout=self.timeout)
            if response and response.status_code == 200:
                response = response.json()
                # TODO multiple intents matched
                if (
                    "intent" in response
                    and response["intent"]["confidence"] > self.confidence_threshold
                    and not response["intent"]["name"].startswith("ood/")
                ):
                    return response


class RasaIntentClassifier(IntentClassifier):
    def __init__(self, urls=None, timeout=1):
        self.languages = ["en"]
        if not urls:
            host = os.environ.get("NLU_SERVER_HOST", "127.0.0.1")
            port = os.environ.get("SYSTEM_INTENT_SERVICE_PORT", "10200")
            urls = ["http://%s:%s" % (host, port)]
        self.urls = urls
        self.timeout = timeout
        self.max_text_length = 128
        self.confidence_threshold = 0.4

    def ping(self, url):
        try:
            response = requests.get(url, timeout=self.timeout)
        except Exception as ex:
            logger.error(ex)
            return False
        if response.status_code == 200:
            return True
        else:
            logger.error("Intent Classification Server %s is not available", url)
        return False

    def detect_intent(self, text, lang):
        if lang in LANGUAGE_CODE_MAPPING:
            lang = LANGUAGE_CODE_MAPPING[lang]
        if lang not in self.languages:
            logger.error('Language "%s" is not supportd', lang)
            return
        if len(text) > self.max_text_length:
            logger.error("Input text exceeds %s characters" % self.max_text_length)
            return
        params = {"text": text, "language": lang}
        for url in self.urls:
            if not self.ping(url):
                logger.error("Intent classification service %s is not available", url)
                continue
            response = requests.post(f"{url}/model/parse", data=json.dumps(params))

            if response and response.status_code == 200:
                response = response.json()
                # TODO multiple intents matched
                if (
                    "intent" in response
                    and response["intent"]["confidence"] > self.confidence_threshold
                    and not response["intent"]["name"].startswith("ood/")
                ):
                    return response


class IntentDetector(object):
    class DetectorType(Enum):
        SOULTALK = "soultalk"
        RASA = "rasa"

    def __init__(self, service_type, **kwargs):
        self.intent_classifier = None
        self.service_type = service_type
        if self.service_type == self.DetectorType.SOULTALK.value:
            self.intent_classifier = SoulTalkIntentClassifier(**kwargs)
        elif self.service_type == self.DetectorType.RASA.value:
            self.intent_classifier = RasaIntentClassifier(**kwargs)

    def detect_intent(self, text, lang):
        if not self.intent_classifier:
            logger.error("No intent detector")
            return
        return self.intent_classifier.detect_intent(text, lang)

    def reset(self):
        pass


if __name__ == "__main__":
    host = "192.168.0.121"
    url = "http://%s:%s" % (host, 10200)

    # project_id = "asha-273607"
    # intent_classifier = DialogFlowIntentClassifier(project_id)
    # print((intent_classifier.detect_intent("can we talk in chinese", "en")))

    detector = IntentDetector("rasa", urls=["http://localhost:10200"])
    detector.detect_intent("can we talk in chinese", "en")
