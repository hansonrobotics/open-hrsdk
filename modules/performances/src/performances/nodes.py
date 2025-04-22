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

# Nodes factory
import os
import pprint
import time
import logging
import random
import urllib
import re

from hr_msgs.msg import ChatMessage
from hr_msgs.msg import Event
from hr_msgs.msg import SetAnimation, SetExpression, Target, SomaState
from hr_msgs.msg import TTS
from performances.srv import RunByNameRequest
from std_msgs.msg import String, Int32, Float32
from threading import Timer
from topic_tools.srv import MuxSelect
import dynamic_reconfigure.client
import requests
import rospy
from six import string_types

logger = logging.getLogger('hr.performances.nodes')


class Node(object):
    # Create new Node from JSON
    @staticmethod
    def subClasses(cls):
        return cls.__subclasses__() + [g for s in cls.__subclasses__()
                                       for g in cls.subClasses(s)]

    @classmethod
    def createNode(cls, data, ros, start_time=0, id='', runner=None):
        for s_cls in cls.subClasses(cls):
            if data['name'] == s_cls.__name__:
                # Runner is still needed for Pause
                node = s_cls(data, ros, runner=runner)
                node.id = id
                if start_time > node.start_time:
                    # Start time should be before or on node starting
                    node.finished = True

                    if start_time < node.end_time():
                        node.started = True

                return node
        logger.error("Wrong node description: {0}".format(str(data)))

    @classmethod
    def create_empty_node_data(cls, type='speech', *args, **kwargs):
        data = {
            'name': type,
            'start_time': 0.0,
            'duration': 0.0,
        }
        for s_cls in cls.subClasses(cls):
            if data['name'] == s_cls.__name__:
                if hasattr(s_cls, 'data'):
                    data.update(s_cls.data)
        data.update(kwargs)
        return data


    def replace_variables_text(self, text):
        variables = re.findall("{(\w*?)}", text)
        for var in variables:
            val = self.ros.get_variable(self.id, var) or ''
            text = text.replace('{' + var + '}', val)
        return text

    def __init__(self, data, ros, **kwargs):
        self.data = data
        self.duration = max(0.1, float(data['duration']))
        self.start_time = data['start_time']
        self.started = False
        self.started_at = 0
        self.finished = False
        self.id = ''
        # Ros connections for accessing ROS topics and method
        # TODO make ROS topics and services singletons class for shared use.
        self.ros = ros

    # By default end time is started + duration for every node
    def end_time(self):
        return self.start_time + self.duration

    # Manages node states. Currently start, finish is implemented.
    # Returns True if its active, and False if its inactive.
    # TODO make sure to allow node publishing pause and stop
    def run(self, run_time):
        # ignore the finished nodes
        if self.finished:
            return False if not self.started else run_time < self.end_time()

        if self.started:
            # Time to finish:
            if run_time >= self.end_time():
                self.finished = True
                self.stop(run_time)
                return False
            # elif self.ros.paused:
            #     self.paused(run_time)
            else:
                self.cont(run_time)
        else:
            if run_time > self.start_time:
                try:
                    self.start(run_time)
                except Exception as ex:
                    logger.error(ex)
                self.started = True
                self.started_at = time.time()
        return True

    def __str__(self):
        return pprint.pformat(self.data)

    # Method to execute if node needs to start
    def start(self, run_time):
        pass

    # Method to execute while node is stopping
    def stop(self, run_time):
        pass

    # Method to call while node is running
    def cont(self, run_time):
        pass

    # # Method to call while runner is paused
    # def paused(self, run_time):
    #     pass

    # Method to get magnitude from either one number or range
    @staticmethod
    def _magnitude(magnitude):
        try:
            return float(magnitude)
        except TypeError:
            try:
                # Randomize magnitude
                return random.uniform(float(magnitude[0]), float(magnitude[1]))
            except:
                return 0.0


class speech(Node):

    data = {
        'lang': 'en-US',
        'voice': '',
    }

    def __init__(self, data, ros, **kwargs):
        Node.__init__(self, data, ros)
        if 'pitch' not in data:
            self.data['pitch'] = 1.0
        if 'speed' not in data:
            self.data['speed'] = 1.0
        if 'volume' not in data:
            self.data['volume'] = 1.0
        if 'voice' not in data:
            self.data['voice'] = ''
        # Backward compatibility
        if self.data['lang'] in ['en', 'zh']:
            self.data['lang'] = {'en': 'en-US', 'zh': 'cmn-Hans-CN'}[self.data['lang']]

    def start(self, run_time):
        self.say(self.data['text'], self.data['lang'], self.data['voice'])

    def say(self, text, lang, voice):
        # SSML tags for non-Cantonese
        text = "|p|"+text
        if 'HK' not in lang:
            text = self._add_ssml(text)
        if 'NONE' in lang:
            return
        text = self.replace_variables_text(text)
        tts = TTS()
        tts.text = text
        tts.lang = lang
        tts.voice = voice
        self.ros.topics['tts'].publish(tts)

    # adds SSML tags for whole text returns updated text.
    def _add_ssml(self, txt):
        # Ignore SSML if simplified syntax is used.
        if re.search(r"[\*\@]\w+", txt):
            return txt
        attrs = ""
        if self.data['speed'] != 1:
            attrs += " rate=\"{:.2f}\"".format(self.data['speed'])
        if self.data['pitch'] != 1:
            attrs += " pitch=\"{:+.2f}%\"".format((self.data['pitch']-1)*100)
        if self.data['volume'] != 1:
            attrs += " volume=\"{:+.0f}dB\"".format((self.data['volume']-1)*100)
        if len(attrs) > 0:
            txt = "<prosody" + attrs + ">" + txt + "</prosody>"
        return txt

class gesture(Node):
    def start(self, run_time):
            self.ros.topics['gesture'].publish(
                SetAnimation(self.data['gesture'], 1, float(self.data['speed']), self._magnitude(self.data['magnitude'])))

    def stop(self, run_time):
        if not self.finished and self.started:
            self.ros.topics['gesture'].publish(
                SetAnimation(self.data['gesture'], 1, 0, -0.5))

class arm_animation(Node):
    def start(self, run_time):
        self.ros.topics['arm_animation'].publish(
            SetAnimation(self.data['arm_animation'], 1, float(self.data['speed']), self._magnitude(self.data['magnitude'])))

    def stop(self, run_time):
        if not self.finished and self.started:
        # Remove arm animation by setting magnitude to 0
            self.ros.topics['arm_animation'].publish(
                SetAnimation(self.data['arm_animation'], 1, 0, 0))


class emotion(Node):
    def start(self, run_time):
        self.ros.topics['emotion'].publish(
            SetExpression(self.data['emotion'], self._magnitude(self.data['magnitude']),
                         rospy.Duration.from_sec(self.data['duration'])))


# Behavior tree
class interaction(Node):
    def start(self, run_time):
        self.ros.topics['bt_control'].publish(Int32(self.data['mode']))
        if self.data['chat'] == 'listening':
            self.ros.topics['speech_events'].publish(String('listen_start'))
        if self.data['chat'] == 'talking':
            self.ros.topics['speech_events'].publish(String('start'))
        time.sleep(0.02)
        self.ros.topics['interaction'].publish(String('btree_on'))

    def stop(self, run_time):
        # Disable all outputs
        self.ros.topics['bt_control'].publish(Int32(0))

        if self.data['chat'] == 'listening':
            self.ros.topics['speech_events'].publish(String('listen_stop'))
        if self.data['chat'] == 'talking':
            self.ros.topics['speech_events'].publish(String('stop'))
        time.sleep(0.02)
        self.ros.topics['interaction'].publish(String('btree_off'))


# Rotates head by given angle
class head_rotation(Node):
    def start(self, run_time):
        self.ros.topics['head_rotation'].publish(Float32(self.data['angle']))


class soma(Node):
    def start(self, run_time):
        s = SomaState()
        s.magnitude = 1
        s.ease_in.secs = 0
        s.ease_in.nsecs = 1000000 * 300
        s.name = self.data['soma']
        self.ros.topics['soma_state'].publish(s)

    def stop(self, run_time):
        s = SomaState()
        s.magnitude = 0
        s.ease_in.secs = 0
        s.ease_in.nsecs = 0
        s.name = self.data['soma']
        self.ros.topics['soma_state'].publish(s)


class expression(Node):
    pass
    # def __init__(self, data, ros, **kwargs):
    #     Node.__init__(self, data, ros)
    #     self.shown = False
    #
    # def start(self, run_time):
    #     try:
    #         self.ros.services['head_pau_mux']("/" + self.ros.robot_name + "/no_pau")
    #         logger.info("Call head_pau_mux topic {}".format("/" + self.ros.robot_name + "/no_pau"))
    #     except Exception as ex:
    #         logger.error(ex)
    #     self.shown = False
    #
    # def cont(self, run_time):
    #     # Publish expression message after some delay once node is started
    #     if (not self.shown) and (run_time > self.start_time + 0.05):
    #         self.shown = True
    #         self.ros.topics['expression'].publish(
    #             MakeFaceExpr(self.data['expression'], self._magnitude(self.data['magnitude'])))
    #         logger.info("Publish expression {}".format(self.data))
    #
    # def stop(self, run_time):
    #     try:
    #         self.ros.topics['expression'].publish(
    #             MakeFaceExpr('Neutral', self._magnitude(self.data['magnitude'])))
    #         time.sleep(min(1, self.duration))
    #         logger.info("Neutral expression")
    #         self.ros.services['head_pau_mux']("/blender_api/get_pau")
    #         logger.info("Call head_pau_mux topic {}".format("/blender_api/get_pau"))
    #     except Exception as ex:
    #         logger.error(ex)


class kfanimation(Node):
    pass
    # def __init__(self, data, ros, **kwargs):
    #     Node.__init__(self, data, ros)
    #     self.shown = False
    #     self.blender_disable = 'off'
    #     if 'blender_mode' in self.data.keys():
    #         self.blender_disable = self.data['blender_mode']
    #
    # def start(self, run_time):
    #     self.shown = False
    #     try:
    #         if self.blender_disable in ['face', 'all']:
    #             self.ros.services['head_pau_mux']("/" + self.ros.robot_name + "/no_pau")
    #         if self.blender_disable == 'all':
    #             self.ros.services['neck_pau_mux']("/" + self.ros.robot_name + "/cmd_neck_pau")
    #     except Exception as ex:
    #         # Dont start animation to prevent the conflicts
    #         self.shown = True
    #         logger.error(ex)
    #
    # def cont(self, run_time):
    #     # Publish expression message after some delay once node is started
    #     if (not self.shown) and (run_time > self.start_time + 0.05):
    #         self.shown = True
    #         self.ros.topics['kfanimation'].publish(
    #             PlayAnimation(self.data['animation'], int(self.data['fps'])))
    #
    # def stop(self, run_time):
    #     try:
    #         if self.blender_disable in ['face', 'all']:
    #             self.ros.services['head_pau_mux']("/blender_api/get_pau")
    #         if self.blender_disable == 'all':
    #             self.ros.services['neck_pau_mux']("/blender_api/get_pau")
    #     except Exception as ex:
    #         logger.error(ex)


class pause(Node):
    def __init__(self, data, ros, runner, **kwargs):
        Node.__init__(self, data, ros)
        self.runner = runner
        self.event_callback_ref = False
        self.timer = False

        if 'topic' not in self.data.keys():
            self.data['topic'] = False
        if 'on_event' not in self.data.keys():
            self.data['on_event'] = False
        if 'event_param' not in self.data.keys():
            self.data['event_param'] = False

    def start_performance(self):
        if self.timer:
            self.timer.cancel()

        if 'break' in self.data and not self.data['break']:
            self.runner.interrupt()
            self.runner.append_to_queue(self.data['on_event'])
        else:
            self.runner.run_full_performance(self.data['on_event'])

    # This function needs to be reused in wholeshow to make sure consistent matching
    @staticmethod
    def event_matched(param, msg):
        params = str(param).lower().split(',')
        matched = False
        for p in params:
            try:
                str(msg or '').lower().index(p.strip())
                matched = True
                continue
            except ValueError:
                matched = matched or False
        return matched

    def event_callback(self, msg=None):
        self.delete_callback_ref()

        if self.data['event_param']:
            # Check if any comma separated
            if not self.event_matched(self.data['event_param'], msg):
                return False

        if self.data['on_event']:
            self.start_performance()
        else:
            self.resume()

    def resume(self):
        if not self.finished:
            self.runner.resume()
        if self.timer:
            self.timer.cancel()

    def start(self, run_time):
        self.runner.pause()

        if 'topic' in self.data:
            topic = str(self.data['topic'] or '').strip()
            if topic != 'ROSPARAM':
                self.event_callback_ref = self.ros.register(topic, self.event_callback)
                # Paused SPEECH event should not be forwarded to chatbot if its enabled.
                # The filtering is in wholeshow node
                if self.data['event_param']:
                    # Currently only single PAUSE node can listen for keywords, so global param is fine.
                    rospy.set_param('/performances/keywords_listening', self.data['event_param'])
            else:
                if self.data['event_param']:
                    if rospy.get_param(self.data['event_param'], False):
                        # Resume current performance or play performance specified
                        self.timer = Timer(0.0, lambda: self.event_callback(self.data['event_param']))
                        self.timer.start()
                        return
        try:
            timeout = float(self.data['timeout'])
            if timeout > 0.1:
                self.timer = Timer(timeout, self.resume)
                self.timer.start()
        except (ValueError, KeyError) as e:
            logger.error(e)

    def delete_callback_ref(self):
        if self.event_callback_ref:
            self.ros.unregister(str(self.data['topic'] or '').strip(), self.event_callback_ref)
            self.event_callback_ref = None

    def stop(self, run_time):
        self.delete_callback_ref()
        if self.timer:
            self.timer.cancel()

    def end_time(self):
        return self.start_time + 0.1


class attention(Node):
    # Find current region at runtime
    def __init__(self, data, ros, **kwargs):
        Node.__init__(self, data, ros)
        self.topic = ['look_at', 'gaze_at']
        self.times_shown = 0

    @staticmethod
    def get_random_axis_position(regions, axis):
        """
        :param regions: list of dictionaries
        :param axis: string 'x' or 'y'
        :return: position and matched regions
        """

        position = 0
        matched = []

        if regions:
            regions = sorted(regions, key=lambda r: r[axis])
            prev_end = regions[0][axis]
            length = 0
            lengths = []

            for r in regions:
                begin = r[axis]
                end = begin + (r['width'] if axis == 'x' else r['height'])

                if prev_end > begin:
                    diff = prev_end - begin
                    lengths.append([length - diff, length - diff + end - begin])
                    begin = prev_end
                else:
                    lengths.append([length, length + end - begin])
                length += max(0, end - begin)
                prev_end = max(begin, end)

            rval = random.random() * length

            for i, length in enumerate(lengths):
                if length[0] <= rval <= length[1]:
                    matched.append(regions[i])
                    if not position:
                        position = regions[i][axis] + (regions[i]['width'] if axis == 'x' else regions[i]['height']) * (
                            (rval - length[0]) / (length[1] - length[0]))
        return position, matched

    @staticmethod
    # Gets x,y,z from given regions based on region type
    def get_point_from_regions(all_regions, region_type):
        regions = [{'x': r['x'], 'y': r['y'] - r['height'], 'width': r['width'], 'height': r['height']} for r in
                   all_regions
                   if r['type'] == region_type]
        if regions:
            y, matched = attention.get_random_axis_position(regions, 'x')
            z, matched = attention.get_random_axis_position(matched, 'y')
            # invert Y to match image in bg
            return {
                'x': 1,
                'y': -y,
                'z': z,
            }
        else:
            # Look forward
            return {'x': 1, 'y': 0, 'z': 0}

    # returns random coordinate from the region
    def get_point(self, region):
        #  regions = rospy.get_param(
        #     '/' + os.path.join("/hr/control/performances", os.path.dirname(self.id),
        #                        "properties/regions"), [])
        # if not(len(regions)):
        regions = rospy.get_param('/hr/control/regions', [])
        return self.get_point_from_regions(regions, region)

    def set_point(self, point):
        speed = 1 if 'speed' not in self.data else self.data['speed']
        for topic in self.topic:
            self.ros.topics[topic].publish(Target(point['x'], point['y'], point['z'], speed))

    def cont(self, run_time):
        if 'attention_region' in self.data and self.data['attention_region'] != 'custom':
            if 'interval' in self.data and run_time > self.times_shown * self.data['interval'] or not self.times_shown:
                self.set_point(self.get_point(self.data['attention_region']))
                self.times_shown += 1

        if not self.times_shown:
            self.set_point(self.data)
            self.times_shown += 1


class look_at(attention):
    # Find current region at runtime
    def __init__(self, data, ros, **kwargs):
        attention.__init__(self, data, ros)
        self.topic = ['look_at', 'gaze_at']


class gaze_at(attention):
    # Find current region at runtime
    def __init__(self, data, ros, **kwargs):
        attention.__init__(self, data, ros)
        self.topic = ['gaze_at']

class scene(Node):
    def start(self, run_time):
        self.ros.topics['arf'].publish(self.data['scene'])

    def stop(self, run_time):
        pass

class settings(Node):

    def setParameters(self, rosnode, params):
        try:
            cl = dynamic_reconfigure.client.Client(rosnode, timeout=0.1)
            params = self.set_variables(params)
            cl.update_configuration(params)
            cl.close()
        except:
            pass

    def set_variables(self, params):
        for k, v in params.items():
            if isinstance(v, string_types):
                params[k] = self.replace_variables_text(v)
            else:
                params[k] = v
        return params

    def start(self, run_time):
        if self.data['rosnode']:
            self.setParameters(self.data['rosnode'], self.data['values'])
