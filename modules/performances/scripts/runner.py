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

from threading import Thread, Lock, Condition
import logging
import json
import time
import yaml
import os
import fnmatch
import random
import copy

try:
    from pathlib import Path
except:
    # python2.7
    from pathlib2 import Path

import rospy
import hr_msgs.srv as srv
from dynamic_reconfigure.server import Server
from hr_msgs.msg import ChatMessage
from hr_msgs.msg import Event
from hr_msgs.msg import MakeFaceExpr, PlayAnimation
from hr_msgs.msg import SetAnimation, SetExpression, Target, SomaState
from hr_msgs.msg import TTS
from natsort import natsorted, ns
from performances.cfg import PerformancesConfig
from performances.nodes import Node
from performances.weak_method import WeakMethod
from std_msgs.msg import String, Int32, Float32
from std_srvs.srv import Trigger, TriggerResponse
from topic_tools.srv import MuxSelect

logger = logging.getLogger('hr.performances')


class Runner:
    def __init__(self, ros, namespace = '~', background = False):
        self.performances_dir = os.path.join(os.environ.get('PERFORMANCES_DIR', '/home/hr/workspace/hrsdk_configs/performances'))
        self.robot_name = rospy.get_param('/hr/robot_name')
        self.running = False
        self.paused = False
        self.autopause = False
        self.pause_time = 0
        self.start_time = 0
        self.start_timestamp = 0
        self.lock = Lock()
        self.run_condition = Condition()
        self.running_performance = None
        self.running_nodes = []
        self.unload_finished = False
        # References to event subscribing node callbacks
        self.interrupted_performances = []
        # Performances that already played as alternatives. Used to maximize different performance in single demo
        self.performances_played = {}
        self.worker = Thread(target=self.worker_loop)
        self.worker.setDaemon(True)
        self.queue = []
        logger.info('Starting performances node')
        self.ros = ros
        self.running_performance_pub = rospy.Publisher('{}running_performance'.format(namespace), String, queue_size=1)
        self.events_pub =  rospy.Publisher('{}events'.format(namespace), Event, queue_size=1)
        self.running_performance_pub = rospy.Publisher('{}running_performance'.format(namespace), String, queue_size=1)
        self.interrupt_tts = True
        self.load_properties()
        rospy.Service('{}reload_properties'.format(namespace), Trigger, self.reload_properties_callback)
        rospy.Service('{}set_properties'.format(namespace), srv.SetProperties, self.set_properties_callback)
        rospy.Service('{}load'.format(namespace), srv.Load, self.load_callback)
        rospy.Service('{}load_performance'.format(namespace), srv.LoadPerformance, self.load_performance_callback)
        rospy.Service('{}unload'.format(namespace), Trigger, self.unload_callback)
        rospy.Service('{}run'.format(namespace), srv.Run, self.run_callback)
        rospy.Service('{}run_by_name'.format(namespace), srv.RunByName, self.run_by_name_callback)
        rospy.Service('{}run_full_performance'.format(namespace).format(namespace), srv.RunByName, self.run_full_performance_callback)
        rospy.Service('{}resume'.format(namespace), srv.PerformanceCommand, self.resume_callback)
        rospy.Service('{}pause'.format(namespace), srv.PerformanceCommand, self.pause_callback)
        rospy.Service('{}stop'.format(namespace), srv.PerformanceCommand, self.stop_callback)
        rospy.Service('{}current'.format(namespace), srv.Current, self.current_callback)
        rospy.Service('{}is_performance_loaded'.format(namespace), Trigger, self.check_is_performance_loaded)
        # Shared subscribers for nodes
        rospy.Subscriber('{}events'.format(namespace), Event, self.runner_event_callback)
        # Shared subscribers for nodes
        if namespace != '~':
            # only default runner to interrupt performance
            self.interrupt_tts = False
            Server(PerformancesConfig, self.reconfig, namespace=namespace)
        else:
            if not background:
                Server(PerformancesConfig, self.reconfig)

        self.worker.start()

    def reconfig(self, config, level):
        with self.lock:
            self.autopause = config.autopause

        return config

    def reload_properties_callback(self, request):
        self.load_properties()
        return TriggerResponse(success=True)

    def unload_callback(self, request):
        self.unload()
        return TriggerResponse(success=True)

    def unload(self):
        self.stop()
        with self.lock:
            if self.running_performance:
                logger.info('unloading')
                self.running_performance = None
                self.unload_attention_regions()
                self.running_performance_pub.publish(String(json.dumps(None)))

    def set_properties_callback(self, request):
        self.ros.set_variable(request.id, json.loads(request.properties))
        return srv.SetPropertiesResponse(success=True)

    def load_callback(self, request):
        return srv.LoadResponse(success=True, performance=json.dumps(self.load(request.id)))

    def load_performance_callback(self, request):
        self.load_performance(json.loads(request.performance))
        return srv.LoadPerformanceResponse(True)

    def run_by_name_callback(self, request):
        self.stop()
        # Load random timeline if its folder of timelines
        if not self.load(request.id, random_timeline=True):
            return srv.RunByNameResponse(False)
        return srv.RunByNameResponse(self.run(0.0))

    def run_full_performance(self, id, start_time=0.0, unload_finished=False):
        self.stop()
        performances = self.load_folder(id) or self.load(id)
        if not performances:
            return False
        return self.run(start_time, unload_finished=unload_finished)

    def run_full_performance_callback(self, request):
        return self.run_full_performance(request.id, unload_finished=True)

    def load_folder(self, id):
        if id.startswith('shared'):
            robot_name = 'common'
        else:
            robot_name = rospy.get_param('/hr/robot_name')

        dir_path = os.path.join(self.get_path_by_robot_name(robot_name), id)
        if os.path.isdir(dir_path):
            root, dirs, files = next(os.walk(dir_path))

            files = fnmatch.filter(files, "*.yaml")
            if not files:
                # If no folder is picked one directory
                # Sub-directories are counted as sub-performances
                if not dirs:
                    return []
                if id in self.performances_played:
                    # All performances played. Pick any but last played
                    if set(self.performances_played[id]) == set(dirs):
                        dirs = self.performances_played[id][:-1]
                        self.performances_played[id] = []
                    else:
                        # Pick from not played performances
                        dirs = list(set(dirs) - set(self.performances_played[id]))
                else:
                    self.performances_played[id] = []
                # Pick random performance
                p = random.choice(dirs)
                self.performances_played[id].append(p)
                return self.load_folder(os.path.join(id, p))
            # make names in folder/file format
            return self.load(id)
        return []

    def load(self, id, random_timeline=False):
        robot_name = 'common' if id.startswith('shared') else rospy.get_param('/hr/robot_name')
        p = os.path.join(self.get_path_by_robot_name(robot_name), id)

        if os.path.isdir(p):
            root, dirs, files = next(os.walk(p))
            files = natsorted(fnmatch.filter(files, "*.yaml"), key=lambda f: f.lower())
            ids = ["{}/{}".format(id, f[:-5]) for f in files]
            timelines = [self.get_timeline(i) for i in ids]
            timelines = [t for t in timelines if t]
            if random_timeline:
                performance = random.choice(timelines)
            else:
                performance = {'id': id, 'name': os.path.basename(id), 'path': os.path.dirname(id),
                               'timelines': timelines, 'nodes': self.get_merged_timeline_nodes(timelines)}
        else:
            performance = self.get_timeline(id)

        if performance:
            self.load_performance(performance)
            return performance
        else:
            return None

    def get_path_by_robot_name(self, name):
        return os.path.join(self.performances_dir, name)

    def get_timeline(self, id):
        timeline = None
        robot_name = 'common' if id.startswith('shared') else rospy.get_param('/hr/robot_name')

        # search timeline, case insensitive
        performance_dir = self.get_path_by_robot_name(robot_name)
        target = '%s.yaml' % id.lower()
        p = None
        for f in Path(performance_dir).glob('**/*.yaml'):
            name = str(f.relative_to(performance_dir))
            if name.lower() == target:
                p = f
        if p:
            try:
                with p.open() as f:
                    timeline = yaml.safe_load(f.read())
                    timeline['id'] = id
                    timeline['name'] = os.path.basename(id)
                    timeline['path'] = os.path.dirname(id)
                    self.validate_timeline(timeline)
            except Exception as ex:
                logger.error(ex)
        else:
            logger.error("Timeline with id %s was not found", id)
        return timeline

    def get_timeline_duration(self, timeline):
        duration = 0

        if 'nodes' in timeline and isinstance(timeline['nodes'], list):
            for node in timeline['nodes']:
                duration = max(duration, (node['duration'] if 'duration' in node else 0) + node['start_time'])

        return duration

    def get_merged_timeline_nodes(self, timelines):
        merged = []
        offset = 0

        for timeline in timelines:
            if 'enabled' in timeline and not timeline['enabled']:
                continue

            duration = 0
            nodes = timeline.get('nodes', [])
            nodes = copy.deepcopy(nodes)

            for node in nodes:
                duration = max(duration, node['duration'] + node['start_time'])
                node['start_time'] += offset

            merged += nodes
            offset += duration

        return merged

    def validate_performance(self, performance):
        self.validate_timeline(performance)
        if 'timelines' in performance:
            for timeline in performance['timelines']:
                self.validate_timeline(timeline)
        return performance

    def validate_timeline(self, timeline):
        if 'nodes' not in timeline or not isinstance(timeline['nodes'], list):
            timeline['nodes'] = []

        for node in timeline['nodes']:
            if 'start_time' not in node:
                node['start_time'] = 0
            if node['name'] == 'pause':
                node['duration'] = 0.1
            if 'duration' not in node or not node['duration']:
                node['duration'] = 0

        return timeline

    def load_performance(self, performance):
        with self.lock:
            logger.info('load: {0}'.format(performance.get('id', 'NO ID')))
            self.validate_performance(performance)
            self.load_attention_regions(performance.get('id','invalid'))
            self.running_performance = performance
            self.running_performance_pub.publish(String(json.dumps(performance)))

    def run_callback(self, request):
        return srv.RunResponse(self.run(request.startTime))

    def load_attention_regions(self, id):
        regions = rospy.get_param(
            '/' + os.path.join('/hr', "control/performances", id,
                               "properties/regions"), [])
        rospy.set_param('/hr/control/performance_regions', regions)

    def unload_attention_regions(self):
        rospy.set_param('/hr/control/performance_regions', [])

    def run(self, start_time, unload_finished=False):
        start_time = float(start_time or 0)
        self.stop()
        # Wait for worker to stop performance and enter waiting before proceeding
        self.run_condition.acquire()
        with self.lock:
            success = self.running_performance and len(self.running_performance) > 0
            if success:
                self.unload_finished = unload_finished
                self.running = True
                self.start_time = start_time
                self.start_timestamp = time.time()
                log_data = {
                    'performance_report': True,
                    'performance_id': self.running_performance.get('id', ''),
                    'performance_time': start_time,
                    'performance_action': 'run'
                }
                logger.info('Running performance #{} at: {}'.format(log_data['performance_id'], start_time), extra={'data': log_data})
                # notify worker thread
                self.run_condition.notify()


            self.run_condition.release()
            return success

    def resume_callback(self, request):
        success = self.resume()
        with self.lock:
            run_time = self.get_run_time()

        return srv.PerformanceCommandResponse(success, run_time)

    def resume(self):
        success = False
        with self.lock:
            if self.running and self.paused:
                run_time = self.get_run_time()
                self.paused = False
                self.start_timestamp = time.time() - run_time
                self.start_time = 0
                self.events_pub.publish(Event('resume', run_time))
                log_data = {
                    'performance_report': True,
                    'performance_id': self.running_performance.get('id', ''),
                    'performance_time': run_time,
                    'performance_action': 'resume'
                }
                logger.info('Resume performance #{} at: {}'.format(log_data['performance_id'], run_time), extra={'data': log_data})
                success = True

        return success

    def stop(self):
        stop_time = 0.0
        with self.lock:
            if self.running:
                stop_time = self.get_run_time()
                for node in self.running_nodes:
                    node.stop(stop_time)
                self.running = False
                self.paused = False
                if self.interrupt_tts:
                    self.ros.topics['tts_control'].publish('shutup')
                log_data = {
                    'performance_report': True,
                    'performance_id': self.running_performance.get('id', '') if self.running_performance else '',
                    'performance_time': stop_time,
                    'performance_action': 'stop'
                }
                logger.info('Stopping performance #{} at: {}'.format(log_data['performance_id'], stop_time), extra={'data': log_data})
        return stop_time

    def stop_callback(self, request=None):
        return srv.PerformanceCommandResponse(True, self.stop())

    def pause_callback(self, request):
        if self.pause():
            with self.lock:
                return srv.PerformanceCommandResponse(True, self.get_run_time())
        else:
            return srv.PerformanceCommandResponse(False, 0)

    # Pauses current
    def pause(self):
        with self.lock:
            if self.running and not self.paused:
                self.pause_time = time.time()
                self.paused = True
                paused_time = self.get_run_time()
                self.events_pub.publish(Event('paused', paused_time))
                log_data = {
                    'performance_report': True,
                    'performance_id': self.running_performance.get('id', ''),
                    'performance_time': paused_time,
                    'performance_action': 'paused'
                }
                logger.info('Pause performance #{} at: {}'.format(log_data['performance_id'], paused_time), extra={'data': log_data})
                return True
            else:
                return False

    # Returns current performance
    def current_callback(self, request):
        with self.lock:
            current_time = self.get_run_time()
            running = self.running and not self.paused
            return srv.CurrentResponse(performance=json.dumps(self.running_performance),
                                       current_time=current_time,
                                       running=running)

    def check_is_performance_loaded(self, request):
        success = self.running_performance and len(self.running_performance) > 0
        return TriggerResponse(success=success)

    def interrupt(self):
        with self.lock:
            if self.running_performance:
                self.interrupted_performances.append({
                    'performance': self.running_performance,
                    'time': self.get_run_time(),
                    # need to store this for node objects to stay alive and for callbacks to work
                    'nodes': self.running_nodes
                })
        self.stop()

    def resume_interrupted(self):
        found = len(self.interrupted_performances)

        if found:
            data = self.interrupted_performances.pop()
            time = data['time']
            logger.info('Resuming interrupted #{0} at {1}. {2} performances left'.format(data['performance']
                                                                                       .get('id', ''), time, found - 1))
            self.load_performance(data['performance'])
            self.run(time)

        return found

    def append_to_queue(self, id, time=0):
        logger.info('Adding performance #{0} to the queue scheduled to run at {1}'.format(id, time))
        self.queue.append({'id': id, 'time': time})

    def load_scheduled(self):
        data = None
        with self.lock:
            not_empty = len(self.queue)
            if not_empty:
                data = self.queue.pop()

        if not_empty:
            logger.info('Loading performance #{0} at {1} from the queue'.format(data['id'], data['time']))
            self.run_full_performance(data['id'], start_time=data['time'])
        elif not self.resume_interrupted():
            return False

        return True

    def worker_loop(self):
        self.run_condition.acquire()
        while True:
            with self.lock:
                self.paused = False
                self.running = False

            self.events_pub.publish(Event('idle', 0))
            self.run_condition.wait()
            self.events_pub.publish(Event('running', self.start_time))

            with self.lock:
                if not self.running_performance:
                    continue

            behavior = True
            offset = 0
            timelines = self.running_performance['timelines'] if 'timelines' in self.running_performance else [
                self.running_performance]

            for i, timeline in enumerate(timelines):
                if 'enabled' in timeline and not timeline['enabled']:
                    continue

                # check if performance is finished without starting
                running = True
                self.running_nodes = [Node.createNode(node, ros, self.start_time - offset, timeline.get('id', ''),
                                                      runner=self) for node in timeline['nodes']]
                pid = timeline.get('id', '')
                finished = None
                run_time = 0
                pause = pid and self.get_property(os.path.dirname(pid), 'pause_behavior')
                # Pause must be either enabled or not set (by default all performances are
                # pausing behavior if its not set)

                with self.lock:
                    if not self.running:
                        break

                while running:
                    if not finished:
                        # Wait for a bit.
                        time.sleep(0.02)
                    with self.lock:
                        run_time = self.get_run_time()
                        if not self.running:
                            self.events_pub.publish(Event('finished', run_time))
                            break
                        if self.paused:
                            continue

                    running = False
                    # checks if any nodes still running
                    for k, node in enumerate(self.running_nodes):
                        running = node.run(run_time - offset) or running

                    if finished is None:
                        # true if all performance nodes are already finished
                        finished = not running


                log_data = {
                    'performance_report': True,
                    'performance_id': self.running_performance.get('id', '') if self.running_performance else '',
                    'performance_time': run_time,
                    'performance_action': 'finished'
                }
                if i == len(timelines) - 1:
                    logger.warning('Finished performance #{} at: {}'.format(log_data['performance_id'], run_time), extra={'data': log_data})

                offset += self.get_timeline_duration(timeline)

                with self.lock:
                    autopause = self.autopause and finished is False and i < len(timelines) - 1

                if autopause:
                    self.pause()

            if self.unload_finished:
                self.unload_finished = False
                self.unload()

    def get_run_time(self):
        """
        Must acquire self.lock in order to safely use this method
        :return:
        """
        run_time = 0

        if self.running:
            run_time = self.start_time
            if self.paused:
                run_time += self.pause_time - self.start_timestamp
            else:
                run_time += time.time() - self.start_timestamp

        return run_time



    def load_properties(self):
        robot_name = rospy.get_param('/hr/robot_name')
        robot_path = os.path.join(self.performances_dir, robot_name)
        common_path = os.path.join(self.performances_dir, 'common')
        for path in [common_path, robot_path]:
            for root, dirnames, filenames in os.walk(path):
                if '.properties' in filenames:
                    filename = os.path.join(root, '.properties')
                    if os.path.isfile(filename):
                        try:
                            with open(filename) as f:
                                properties = yaml.safe_load(f.read())
                                dir = os.path.relpath(root, path)
                                rospy.set_param(os.path.join('/hr/control/performances', dir).strip(
                                    "/.") + '/properties', properties)
                        except:
                            rospy.logerr("Cant load properties file for {}".format(dir))


    def get_property(self, path, name):
        param_name = os.path.join('/hr/control/performances', path, 'properties', name)
        return rospy.get_param(param_name, None)




    def runner_event_callback(self, msg):
        logger.info('Runner event: {0}'.format(msg.event))
        self.ros.notify('RUNNER', msg.event)
        if msg.event == 'idle':
            self.load_scheduled()

    @staticmethod
    def is_param(param):
        """ Checks if value is valid param.
        Has to start with slash
        """
        validator = rospy.names.global_name("param_name")
        try:
            validator(param, False)
            return True
        except rospy.names.ParameterInvalid:
            return False

class SharedROSConnector:
    # Class that provide ros interface for executing nodes, and can be shared by multiple runners
    def __init__(self):
        self.observers = {}
        self.robot_name = rospy.get_param('/hr/robot_name')
        self.variables = {}
        self.topics = {
            'look_at': rospy.Publisher('/hr/animation/set_face_target', Target, queue_size=1),
            'gaze_at': rospy.Publisher('/hr/animation/set_gaze_target', Target, queue_size=1),
            'head_rotation': rospy.Publisher('/hr/animation/set_head_tilt', Float32, queue_size=1),
            'emotion': rospy.Publisher('/hr/animation/set_expression', SetExpression, queue_size=3),
            'gesture': rospy.Publisher('/hr/animation/set_animation', SetAnimation, queue_size=3),
            'arm_animation': rospy.Publisher('/hr/animation/set_arm_animation', SetAnimation, queue_size=3),
            # 'interaction': rospy.Publisher('/behavior_switch', String, queue_size=1),
            # 'bt_control': rospy.Publisher('/behavior_control', Int32, queue_size=1),
            # 'chatbot': rospy.Publisher('/' + self.robot_name + '/speech', ChatMessage, queue_size=1),
            # 'speech_events': rospy.Publisher('/' + self.robot_name + '/speech_events', String, queue_size=1),
            'soma_state': rospy.Publisher("/hr/animation/set_soma_state", SomaState, queue_size=2),
            'tts': rospy.Publisher('/hr/control/speech/say', TTS, queue_size=1),
            'tts_control': rospy.Publisher('/hr/control/speech/tts_control', String, queue_size=1),
            'arf': rospy.Publisher('/hr/interaction/arf', String, queue_size=1)
        }
        # rospy.Subscriber('/' + self.robot_name + '/speech_events', String,
        #                  lambda msg: self.notify('speech_events', msg))
        # rospy.Subscriber('/' + self.robot_name + '/speech', ChatMessage, self.speech_callback)

    @staticmethod
    def is_param(param):
        """ Checks if value is valid param.
        Has to start with slash
        """
        validator = rospy.names.global_name("param_name")
        try:
            validator(param, False)
            return True
        except rospy.names.ParameterInvalid:
            return False

    def set_variable(self, id, properties):
        for key, val in properties.items():
            rospy.logerr("id {} key {} val {}".format(id, key, val))
            if id in self.variables:
                self.variables[id][key] = val
            else:
                self.variables[id] = {key: val}

    def get_variable(self, id, name):
        if os.path.dirname(id) in self.variables and name in self.variables[os.path.dirname(id)] \
                and self.variables[os.path.dirname(id)][name]:
            return self.variables[os.path.dirname(id)][name]
        else:
            val = None
            param_name = os.path.join('/hr/webui/performances', os.path.dirname(id),
                                      'properties/variables', name)
            if rospy.has_param(param_name):
                val = rospy.get_param(param_name)
                if self.is_param(val):
                    if rospy.has_param(val):
                        return str(rospy.get_param(val))
                    if rospy.has_param("/{}{}".format(self.robot_name, val)):
                        return str(rospy.get_param("/{}{}".format(self.robot_name, val)))

                    return None
            return val

    # Notifies register nodes on the events from ROS.
    def notify(self, event, msg):
        if event not in list(self.observers.keys()):
            return
        for i in range(len(self.observers[event]) - 1, -1, -1):
            try:
                self.observers[event][i](msg)
            except TypeError as e:
                # Remove dead methods
                del self.observers[event][i]

    # Registers callbacks for specific events. Uses weak reference to allow nodes cleanup after finish.
    def register(self, event, cb):
        if not event in self.observers:
            self.observers[event] = []
        m = WeakMethod(cb)
        self.observers[event].append(m)
        logger.info('Registering event for "{0}" which now has {1} handlers'.format(event, len(self.observers[event])))
        return m

    # Allows nodes to unsubscribe from events
    def unregister(self, event, ref):
        if event in self.observers:
            if ref in self.observers[event]:
                self.observers[event].remove(ref)
                logger.info('Unregistering event handler for "{0}" which now has {1} handlers'
                            .format(event, len(self.observers[event])))

    def speech_callback(self, msg):
        self.notify('SPEECH', msg.utterance)




if __name__ == '__main__':
    rospy.init_node('performances')
    ros = SharedROSConnector()
    # Runner for UI
    r1 = Runner(ros)
    # Runner for background performances
    r2 = Runner(ros, '/hr/control/performances/background/', background=True)
    # Runner for automated gestures
    r3 = Runner(ros, '/hr/control/performances/speech_animation/', background=True)

    rospy.spin()
