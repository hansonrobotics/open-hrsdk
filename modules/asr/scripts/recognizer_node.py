#!/usr/bin/python
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



import logging
import threading
import time
import rospy
import json

from hr_msgs.msg import AudioStream, ChatMessage
from hr_msgs.srv import StringTrigger
from std_msgs.msg import Empty, Header, String

from speech_recognizer.phrase_engine import PhraseEngine
from speech_recognizer.recognizer import MultiRecognizer
from speech_recognizer.ddr_configs import DDRNode

logger = logging.getLogger("hr.asr.recognizer_node")


class RecognizerNode(object):
    """Speech recognition ROS class that communicates with other ROS nodes"""

    INIT_SPEECH_PHRASES = [
        "Sophia",
        "Hanson Robotics",
    ]

    def __init__(self):
        # Create an instance of DDRNode with the desired namespace
        ddr_configs = DDRNode(namespace='/hr/perception/speech_recognizer', callback=self.config_cb)
        # Supported languages
        self.languages_enum = {
            "English (American)": "en-US",
            "Amharic (Ethiopia)": "am-ET",
            "Arabic (Saudi Arabia)": "ar-SA",
            "Arabic (United Arab Emirates)": "ar-AE",
            "Brazilian Portuguese": "pt-BR",
            "British English": "en-GB",
            "Bulgarian (Bulgaria)": "bg-BG",
            "Chinese, Cantonese (Traditional Hong Kong)": "yue-Hant-HK",
            "Chinese, Mandarin (Simplified, China)": "cmn-Hans-CN",
            "Croatian (Croatia)": "hr-HR",
            "Czech (Czech Republic)": "cs-CZ",
            "Danish (Denmark)": "da-DK",
            "Dutch (Netherlands)": "nl-NL",
            "Estonian (Estonia)": "et-EE",
            "Finnish (Finland)": "fi-FI",
            "French (France)": "fr-FR",
            "Georgian (Georgia)": "ka-GE",
            "German (Germany)": "de-DE",
            "Greek (Greece)": "el-GR",
            "Hebrew (Israel)": "iw-IL",
            "Hindi (India)": "hi-IN",
            "Hungarian (Hungary)": "hu-HU",
            "English (India)": "en-IN",
            "Indonesian (Indonesia)": "id-ID",
            "Italian (Italy)": "it-IT",
            "Japanese (Japan)": "ja-JP",
            "Korean (South Korea)": "ko-KR",
            "Latvian (Latvia)": "lv-LV",
            "Lithuanian (Lithuania)": "lt-LT",
            "Norwegian Bokmål (Norway)": "no-NO",
            "Polish (Poland)": "pl-PL",
            "Romanian (Romania)": "ro-RO",
            "Russian (Russia)": "ru-RU",
            "Slovak (Slovakia)": "sk-SK",
            "Slovenian (Slovenia)": "sl-SI",
            "Spanish (Spain)": "es-ES",
            "Sundanese (Indonesia)": "su-ID",
            "Swedish (Sweden)": "sv-SE",
            "Turkish (Turkey)": "tr-TR",
            "Ukrainian (Ukraine)": "uk-UA",
            "Vietnamese (Vietnam)": "vi-VN",
            "Zulu (South Africa)": "zu-ZA",
        }
        self.reversed_languages_enum = {v: k for k, v in self.languages_enum.items()}
        # Create the enum edit_method for the 'language' parameter
        lang_enum = ddr_configs.enum(self.languages_enum)
        ddr_configs.new_param("enable", "Enable Speech Recognition", default=False)
        ddr_configs.new_param("language", "Speech Recognition Default Language", default="en-US", edit_method=lang_enum)
        ddr_configs.new_param("alt_language", "Alternative Language set by UI. Main Language always active", default="en-US", edit_method=lang_enum)
        ddr_configs.new_param("other_languages", "Other languages that are always on", default="[]")
        ddr_configs.new_param("continuous", "Enable Continuous Listening", default=False)
        ddr_configs.new_param("final_result_timeout", "Publish stable interim results as final after inactivity in seconds (Fix foreign languages delays). 0 - disabled. Only for non-English languages", default=1.2, min=0.0, max=5.0)
        ddr_configs.new_param("punctuation", "Enable Automatic Punctuation (if available)", default=True)
        ddr_configs.new_param("asr_activity_mon", "ASR Activity Monitoring (turns on/off ASR by activity)", default=True)
        ddr_configs.new_param("phrases", "Speech Recognition Context (separated by comma)", default="")
        ddr_configs.new_param("asr_inactivity_duration", "ASR Maximum Inactivity Duration (in minutes)", default=20, min=10, max=120)
        ddr_configs.new_param("skip_first_audio", "Skip audio in (s) after robot stops speaking (will round to 0.1 accuracy)", default=0.1, min=0.0, max=1.0)
        ddr_configs.new_param("decision_timeout", "Decision timeout (wait for little time for other language to return result)", default=0.3, min=0.1, max=2.0)
        ddr_configs.new_param("hybrid_all_results", "(DO NOT USE IF POSSIBLE) Publish all language results in hybrid mode", default=False)
        # Language selection parameters
        ddr_configs.new_param("decision_timeout", "Decision timeout (wait for little time for other language to return result)", default=0.3, min=0.1, max=2.0)
        ddr_configs.new_param("main_language_bias", "Bias for main langauge (adds to confidence) ", default=0.1, min=0.0, max=1.0)
        ddr_configs.new_param("previous_language_bias", "Bias to stick to previous tts or STT language", default=0.1, min=0.0, max=1.0)
        ddr_configs.new_param("min_confidence_for_main", "Ignore results with lower than this confidence for main language", default=0.3, min=0.0, max=1.0)
        ddr_configs.new_param("min_confidence_for_alternatives", "Ignore results with lower than this confidence for alternative languages", default=0.5, min=0.0, max=1.0)
        ddr_configs.new_param("min_length_to_switch", "Min length in characters to switch the langauge", default=3, min=0, max=30)

        
        ## node schema
        ddr_configs.new_param("node_schema", "Node Schema", default=self.config_schema())
        init_phrases = RecognizerNode.INIT_SPEECH_PHRASES
        self.phrase_engine = PhraseEngine(init_phrases=init_phrases)
        # Start configuration
        # Topic names
        sentence_topic = rospy.get_param("sentence_topic", "speech")
        chat_event_topic = rospy.get_param("chat_event_topic", "chat_events")
        interim_topic = rospy.get_param("interim_topic", "interim_speech")
        audio_stream_topic = rospy.get_param("audio_stream_topic", "audio")
        robot_speech_event_topic = rospy.get_param("robot_speech_event_topic", "/hr/control/speech/event")
        # ROS publishers
        self.speech_publisher = rospy.Publisher(
            sentence_topic, ChatMessage, queue_size=1
        )
        self.event_publisher = rospy.Publisher(chat_event_topic, String, queue_size=1)
        self.interim_publisher = rospy.Publisher(
            interim_topic, ChatMessage, queue_size=1
        )
        self.audio_publisher = rospy.Publisher(
            audio_stream_topic, AudioStream, queue_size=1
        )
        self.ddr_configs = ddr_configs
        self.last_asr_activity = time.time()
        self.recognizer = MultiRecognizer('en-US')
        self.recognizer.setup_cb(self.speech_cb, self.interim_cb, self.event_cb, self.audio_cb)
        self.recognizer.phrase_engine = self.phrase_engine
        ddr_configs.ddstart()
        self.recognizer.start()
        self.robot_speaking = True
        self.user_speaking = False
        self.last_interim_result = None

        # ROS subscribers
        rospy.Subscriber(
            robot_speech_event_topic, String, self.robot_speech_event_cb, queue_size=1
        )
        rospy.Subscriber(
            "/hr/control/webui/force_send_asr", Empty, self.force_send_asr_result
        )
        # Create speech recognizer and start recognizer

        # # context service
        # rospy.Service("~set_context", StringTrigger, self.ros_service_set_context)
        # asr activity monitor
        monitor_job = threading.Thread(target=self.monitor_asr_activity)
        monitor_job.daemon = True
        monitor_job.start()

    
    def config_schema(self, ):
        langs = ['--Please select--'] + list(self.languages_enum.keys())
        return json.dumps(
            {'other_languages': {
                'type': 'array', 
                'items':{'type': 'string', 'label':'language', 'enum': langs} 
            }})

    
    def config_cb(self, config, level):
        self.config = config
        self._update_phrase_engine(config)
        # Disable recognizer
        other_langs = []
        for l in json.loads(config.other_languages):
            try:
                other_langs.append(self.languages_enum[l])
            except KeyError:
                pass
        
        self.recognizer.update_config_from_node(config, other_langs)

        if config.enable:
            self.recognizer.enable()
            self.last_asr_activity = time.time()
        else:
            self.recognizer.disable()
        config.node_schema = self.config_schema()
        return config
    
    

    def monitor_asr_activity(self):
        """
        Monitors the activity of the speech recognizer.

        If the speech recognizer has been inactive for a duration longer than the configured
        inactivity duration, it will automatically disable the speech recognizer.
        """
        while True:
            if self.config.asr_activity_mon and self.last_asr_activity is not None:
                elapsed = time.time() - self.last_asr_activity
                if elapsed > self.config.asr_inactivity_duration * 60:
                    logger.info("Time since last asr activity %s", elapsed)
                    if self.enable_asr(False):
                        self.last_asr_activity = None
                        logger.warning(
                            "Automatically disabled ASR because of inactivity"
                        )
            time.sleep(1)

    def _update_phrase_engine(self, config):
        phrases = [p.strip() for p in config.phrases.split(",") if p.strip()]
        self.phrase_engine.init_phrases = phrases
        self.phrase_engine.clear_speech_phrases()
        self.phrase_engine.clear_response_phrases()
        self.phrase_engine.clear_topic_phrases()


    def enable_asr(self, enabled):
        if self.config.asr_activity_mon:
            self.ddr_configs.update_configuration({"enable": enabled})
        return True

    def audio_cb(self, data, rate=16000):
        """publishes the audio stream"""
        msg = AudioStream()
        msg.data = data
        msg.header = Header()
        msg.sample_rate = rate
        msg.channels = 1
        msg.header.stamp = rospy.Time.now()
        self.audio_publisher.publish(msg)


    def force_send_asr_result(self, msg):
        if self.last_interim_result:
            logger.info(
                "Force to send asr result %r", self.last_interim_result.utterance
            )
            self.recognizer.reset()
            self.speech_publisher.publish(self.last_interim_result)
            self.last_interim_result = None

    def robot_speech_event_cb(self, msg):
        """Used in continuous listening mode"""
        if msg.data:
            if msg.data.startswith("start"):
                # Reset speech recognition then TTS starts (restarting after TTS finish might be too late).
                self.robot_speaking = True
                self.recognizer.pause()
            # Wait for idle message so the STT wont restart until all sentences are spoken
            if msg.data.startswith("idle"):
                self.robot_speaking = False
                self.recognizer.resume(round(self.config.skip_first_audio / 0.1))

    def create_chat_message(self, message_type, text, stability, audio_path, language):
        msg = ChatMessage()
        msg.source = "cloudspeech-{}".format(language)
        msg.utterance = text
        msg.lang = language or self.config.get("language")
        msg.confidence = int(stability * 100)
        msg.audio_path = audio_path or ""
        return msg

    def speech_cb(self, text, stability, audio_path, language):
        if text:
            msg = self.create_chat_message(
                "speech", text, stability, audio_path, language
            )
            self.speech_publisher.publish(msg)
            logger.warning("Google speech: %s, %s", text, language)

    def word_cb(self, text, stability, language):
        if text:
            msg = self.create_chat_message("word", text, stability, None, language)
            self.words_publisher.publish(msg)

    def interim_cb(self, text, stability, language):
        if text:
            self.last_asr_activity = time.time()
            msg = self.create_chat_message("interim", text, stability, None, language)
            self.interim_publisher.publish(msg)
            self.last_interim_result = msg

    def event_cb(self, event):
        if event == "speechstart":
            self.user_speaking = True
            logger.info("User speaking")
        elif event == "speechstop":
            self.user_speaking = False
            logger.info("User speaking stopped")
        self.event_publisher.publish(event)

    def ros_service_set_context(self, req):
        response = StringTrigger._response_class()
        phrases = req.data.split(",")
        phrases = [p.strip() for p in phrases]

        self.phrase_engine.clear_speech_phrases()

        for phrase in phrases:
            if phrase:
                self.phrase_engine.add_speech_phrase(phrase)
        # update speech recogniser phrases
        self.recognizer.phrases = self.phrase_engine.get_phrases()

        self.recognizer.reset()

        response.success = True
        response.message = "ok"
        return response


if __name__ == "__main__":
    try:
        rospy.init_node("speech_recognizer")
        node = RecognizerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
