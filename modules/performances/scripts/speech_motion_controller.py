#!/usr/bin/env python
# -*- coding: utf-8 -*-

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


import os
import rospy
import yaml
import json
import threading
import time
import logging
import re
from performances.nodes import Node
from haipy.speech_motion.speech_motion import SpeechMotionUtils, SpeechMotionController
#from performances.speech_motion import SpeechMotionUtils, SpeechMotionController
from std_msgs.msg import String
from performances.cfg import RulesConfig, GeneratorConfig
from hr_msgs.srv import LoadPerformance, Run
from hr_msgs.srv import TTSData
from dynamic_reconfigure.server import Server
from copy import deepcopy
import random
logger = logging.getLogger('hr.performances.speech_motion_controller')

# RosNode for speech motion
# HEAD Specific ROS Controller
class SpeechMotionNode():

    def __init__(self):
        # Rules server
        self.config = None
        aa_library = rospy.get_param('/hr/control/animation_actions', {})
        nlu_server = rospy.get_param('/hr/control/nlu_server', "http://127.0.0.1:8210")
        lipsync_delay = rospy.get_param('/hr/control/speech/tts_talker/tts_delay', 0.2)
        self.controller = SpeechMotionController(aa_library, nlu_server=nlu_server, lipsync_delay=lipsync_delay)
        self.rules_server = Server(RulesConfig, self.rules_config_cb, '/hr/control/speech_motions')
        self.current_state = 'idle'
        rospy.Subscriber('/hr/behavior/current_state', String, self.update_state)
        rospy.Subscriber('/hr/control/speech/data', String, self.get_tts_data)
        self.loader = rospy.ServiceProxy('/hr/control/performances/speech_animation/load_performance', LoadPerformance)
        self.runner = rospy.ServiceProxy('/hr/control/performances/speech_animation/run', Run)
        self.tts_ready = rospy.Publisher('/hr/control/speech/tts_control', String, queue_size=2)
        self.id = 1

    def get_tts_data(self, msg):
        # no generation on interacting, also dont animate perfromances itself to avoid confusion
        if (self.config.interactive and not 'interacting' in self.current_state) or '|p|' in msg.data:
            self.tts_ready.publish('ready')
            return
        # Prefer JSON loads, if only json format is used
        try:
            data = json.loads(msg.data)
        except Exception as e   :
            data = yaml.safe_load(msg.data)

        nodes = SpeechMotionUtils.create_speech_data_from_tts_data(data)
        animated = self.controller.animate_motion(nodes, animate_arms=self.config.automated_arm_gestures)
        SpeechMotionController.clean_nodes(animated)
        if len(animated) > len(nodes):
            self.loader(json.dumps({'nodes': animated, 'id': "auto/{}".format(self.id)}, ensure_ascii=True))
            self.id += 1
            self.tts_ready.publish('ready')
            self.runner(0)
        else:
            self.tts_ready.publish('ready')

    def update_state(self, msg):
        self.current_state = msg.data


    def rules_config_cb(self, config, level):
        if self.config is None:
            self.controller.update_keyword_rules(yaml.safe_load(config.keyword_rules))
            # # Sorted Rules on first load, later on the order should remain same to prevent unexpected UI sorting
            config.keyword_rules = json.dumps(self.controller.get_keyword_rules())
            self.controller.update_da_rules(yaml.safe_load(config.da_rules))
            # # Sorted Rules on first load, later on the order should remain same to prevent unexpected UI sorting
            config.da_rules = json.dumps(self.controller.get_da_rules())
            self.config = config
        if self.config.keyword_rules != config.keyword_rules:
            self.controller.update_keyword_rules(yaml.safe_load(config.keyword_rules))
        if self.config.da_rules != config.da_rules:
            self.controller.update_da_rules(yaml.safe_load(config.da_rules))
        self.config = config
        self.controller.animated_shoulders = config.automated_shoulder_gestures
        self.controller.min_arms_hold_time = config.min_arms_hold_time
        self.controller.max_arms_hold_time = config.max_arms_hold_time
        self.controller.skip_keyword_rules = not config.enable
        self.controller.library.filter = config.negative_gesture_categories
        self.controller.library.reduce_robability = config.reduce_negativity
        self.controller.negative_gesture_categories = config.negative_gesture_categories
        self.controller.set_gesture_filters(
            config.rules_head_filter, config.rules_brow_filter, config.rules_eyelids_filter, config.rules_eyes_filter)
        self.controller.happy_gestures_probability = config.generic_gestures_happy_filter
        self.controller.set_generic_gestures_config(config.generic_gestures, config.generic_gestures_categories_excluded, \
                                                    config.generic_gestures_probability, config.generic_gestures_head_filter, \
                                                    config.generic_gestures_brow_filter, config.generic_gestures_eyelids_filter, \
                                                    config.generic_gestures_eyes_filter, config.generic_gestures_arms_filter)
        return config


# Performance generator for HEAD
class PerformanceGenerator():

    def __init__(self, speech_motion):
        self.speech_motion = speech_motion

        self.performances_dir = os.path.join(
            os.environ.get('PERFORMANCES_DIR', '/home/hr/workspace/hrsdk_configs/performances'))
        self.robot_name = rospy.get_param('/hr/robot_name')
        self.lipsync_delay = rospy.get_param('/hr/control/speech/tts_talker/tts_delay', 0.2)
        self.tts_data = rospy.ServiceProxy('/hr/control/speech/data', TTSData)
        self.in_progress = False
        self.generator = None
        self.cfg = None
        self.srv = Server(GeneratorConfig, self.update_cfg, namespace='/hr/control/performance_generation')

    def update_cfg(self, cfg, level):
        if self.cfg is None:
            self.cfg = cfg
            return cfg
        # Update progress
        if self.in_progress:
            self.cfg.progress = cfg.progress
            return self.cfg
        min_speed = min(cfg.min_speed, cfg.max_speed)
        max_speed = max(cfg.min_speed, cfg.max_speed)
        cfg.min_speed = min_speed
        cfg.max_speed = max_speed
        # Start Generation
        cfg.name = cfg.name.strip()
        cfg.name = cfg.name.replace(' ', '_')
        cfg.name = cfg.name.replace('.', '_')
        cfg.name = re.sub('[\W]&&[^/]+', '', cfg.name)
        cfg.name =cfg.name.lower()
        if cfg.start:
            cfg.start = False
            if cfg.name == "":
                cfg.status = "Performance name is required"
                self.cfg = cfg
                return cfg
            self.in_progress = True
            self.cfg = cfg
            self.generator = threading.Thread(target=self.genearate)
            self.generator.setDaemon(True)
            self.generator.start()
        else:
            self.cfg = cfg
        return self.cfg

    def add_speed_ssml(self, text, speed):
        try:
            if speed == 100:
                return text
            return '<prosody rate="{}%">{}</prosody>'.format(speed, text)
        except Exception:
            return text
            
    def genearate(self):
        current_loc = 'start'
        try:
            lines = self.cfg.lines.splitlines()
            for i, l in enumerate(lines):
                current_loc = str(i+1)
                self.srv.update_configuration({'progress': 'Working on  {} of {}'.format(i + 1, len(lines))})
                sentences = SpeechMotionUtils.split_text(l, split_sentences=self.cfg.split_sentences)
                nodes = []
                current_sentence_start = 0
                for s in sentences:
                    try:
                        s = s.lower()
                        try:
                            s = str(s)
                        except:
                            pass
                        # Try add speed
                        try:
                            speed = random.randint(self.cfg.min_speed, self.cfg.max_speed)
                        except Exception:
                            speed = 100
                        
                        s_with_speed = self.add_speed_ssml(s, speed)
                        tts_data = self.tts_data(s_with_speed, self.cfg.lang)
                        tts_data = yaml.safe_load(tts_data.data)
                        # matched with TTS words
                        text, text_words, words = SpeechMotionUtils.match_tts(tts_data, s)
                        if not text:
                            continue
                        # lipsync_delay needed to be added as sound file needs to be slightly delayed for lipsynch,
                        # therefore actual duration is longer
                        node = Node.create_empty_node_data(type='speech', duration=text['duration']+0.3, text=s,
                                                        start_time=current_sentence_start,
                                                        tts_data=tts_data, words=words, text_words=text_words, lang=self.cfg.lang, speed=round(speed/100.0, 2))
                        nodes.append(node)
                        current_sentence_start += text['duration'] + self.lipsync_delay + self.cfg.pause_between_sentences
                    except Exception as e:
                        logger.error(u"Exception {} while generating {}".format(e, s))
                        # Wait for TTS server to recover
                        time.sleep(10)
                        pass
                animated = self.speech_motion.controller.animate_motion(nodes)
                nodes = nodes + animated
                SpeechMotionController.clean_nodes(nodes)
                if self.cfg.name != '':
                    # Allow shared performances generations
                    robot_name = 'common' if self.cfg.name.startswith('shared') else self.robot_name
                    filename = os.path.join(self.performances_dir, robot_name, self.cfg.name, "{}.yaml".format(i + 1))
                    if not os.path.exists(os.path.dirname(filename)):
                        os.makedirs(os.path.dirname(filename))
                    with open(filename, 'w') as f:
                        yaml.dump({'nodes': deepcopy(nodes)}, f)

            self.srv.update_configuration({'progress': 'idle'})
            self.in_progress = False
        # Unhandled exceptions should show in UI, and not prevent to retry
        except Exception as e:
            logger.error(u"Exception {} while generating {}".format(e, current_loc))
            self.srv.update_configuration({'progress': f'Error while generating line {current_loc} exception:{e}'})
            self.in_progress = False
            

if __name__ == "__main__":
    rospy.init_node('SpeechMotionController')
    rosnode = SpeechMotionNode()
    generator = PerformanceGenerator(rosnode)
    rospy.spin()
