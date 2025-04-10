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

import logging
import os
import sys
import threading
import time
import unittest
import uuid

cwd = os.path.dirname(os.path.abspath(__file__))
os.environ["HR_CHATBOT_WORLD_DIR"] = os.path.join(cwd, "data")
sys.path.insert(1, os.path.join(cwd, "../src"))

logging.basicConfig(
    format="[%(name)s][%(levelname)s] %(asctime)s: %(message)s", level=logging.INFO
)

from haipy.interact import action_types
from haipy.interact import event_types as types
from haipy.interact.controller_manager import ControllerManager
from haipy.interact.controllers.base import SyncAgentController
from haipy.interact.event_generator import EventGenerator
from haipy.interact.state import State

logger = logging.getLogger(__name__)


class Robot(object):
    def __init__(self):
        self.actions = []

    def reset(self):
        self.actions = []

    def question(self, state, text, lang="en-US"):
        utterance = {
            "text": text,
            "lang": lang,
        }
        utterance["uuid"] = str(uuid.uuid4())
        state.update(utterance=utterance)

    def robot_speaking(self, state, flag):
        state.update(robot_speaking=flag)

    def user_speaking(self, state, flag):
        state.update(user_speaking=flag)

    def handle_action(self, action):
        if not action:
            return
        self.actions.append(action)


class ControllerTestCase(unittest.TestCase):
    def setUp(self):
        self.robot = Robot()

    def tearDown(self):
        pass

    def test_state(self):
        state = State()
        state.update(active=True, inactive=False, data={"a": 1})
        self.assertTrue(state.is_active())
        self.assertFalse(state.is_inactive())
        self.assertRaises(AttributeError, state.is_data)
        self.assertFalse(state.is_nonexist())

    def test_state_change(self):
        state = State()
        self.robot.question(state, "some text")
        data = state.getState()
        change = state.getStateChange()
        self.assertEqual(data["utterance"]["text"], "some text")
        self.assertEqual(change, ["utterance"])

    def test_state_wait_for_event(self):
        state = State()

        def run_events():
            self.robot.robot_speaking(state, True)

        threading.Timer(0.2, run_events).start()
        success = state.wait_for_update(0.1)
        self.assertTrue(not success)

        state = State()

        def run_events2():
            self.robot.robot_speaking(state, True)

        threading.Timer(0.1, run_events2).start()
        success = state.wait_for_update(0.2)
        self.assertTrue(success)

        state = State()

        def run_events3():
            self.robot.robot_speaking(state, True)

        threading.Timer(0.1, run_events3).start()
        success = state.wait_for_update()
        self.assertTrue(success)

    def test_event_generator(self):
        state = State()
        events = []
        EventGenerator(state, lambda event: events.append(event))
        self.robot.question(state, "hello")
        time.sleep(0.5)
        self.assertEqual(len(events), 1)
        self.assertEqual(events[0]["type"], types.UTTERANCE)
        self.assertEqual(events[0]["payload"]["text"], "hello")
        self.assertEqual(events[0]["payload"]["lang"], "en-US")

    @unittest.skip("obsolete")
    def test_event_generator2(self):
        """test wait for events"""
        state = State()
        event_generator = EventGenerator(state, lambda event: None)

        def run_events():
            self.robot.robot_speaking(state, True)
            self.robot.question(state, "hello")
            self.robot.robot_speaking(state, False)

        threading.Timer(0.5, run_events).start()
        success = event_generator.wait_for(types.UTTERANCE, 0.6)
        self.assertTrue(success)

        state = State()
        event_generator = EventGenerator(state, lambda event: None)

        def run_events2():
            self.robot.robot_speaking(state, True)
            self.robot.robot_speaking(state, False)

        threading.Timer(0, run_events2).start()
        success = event_generator.wait_for(types.UTTERANCE, 1)
        self.assertTrue(not success)

        state = State()
        event_generator = EventGenerator(state, lambda event: None)

        def run_events3():
            self.robot.robot_speaking(state, True)
            self.robot.robot_speaking(state, False)

        threading.Timer(0, run_events3).start()
        success = event_generator.wait_for([types.UTTERANCE, types.ROBOT_SPEAKING], 1)
        self.assertTrue(success)

    def test_placeholder_utterance_controller(self):
        state = State()
        controller_manager = ControllerManager(state, self.robot.handle_action)

        for controller in controller_manager.controllers.values():
            controller.enabled = False
        controller_manager.get_controller(
            "placeholder_utterance_controller"
        ).enabled = True

        self.robot.user_speaking(state, False)  # user speaking stopped
        self.robot.robot_speaking(state, False)  # robot speaking stopped

        def run_events():
            self.robot.robot_speaking(state, False)
            self.robot.question(state, "hello")

        threading.Timer(0, run_events).start()

        controller_manager.wait_for(types.UTTERANCE)
        time.sleep(1)
        self.assertEqual(len(self.robot.actions), 1)
        self.assertTrue(
            self.robot.actions[0]["type"], action_types.PLACEHOLDER_UTTERANCE
        )

    def test_interruption_controller(self):
        state = State()
        controller_manager = ControllerManager(state, self.robot.handle_action)

        for controller in controller_manager.controllers.values():
            controller.enabled = False
        interruption_controller = controller_manager.get_controller(
            "interruption_controller"
        )
        interruption_controller.enabled = True

        self.robot.user_speaking(state, False)  # user speaking stopped
        self.robot.robot_speaking(state, False)  # robot speaking stopped

        def short_pause_interrupt_events():
            self.robot.robot_speaking(state, True)  # robot speaking
            time.sleep(0.5)
            self.robot.user_speaking(state, True)  # user speaking
            self.robot.question(state, "some short keywords")  # input utterance
            self.robot.user_speaking(state, False)  # user speaking stopped

        def long_input_interrupt_events():
            self.robot.robot_speaking(state, True)  # robot speaking
            time.sleep(0.5)
            self.robot.user_speaking(state, True)  # user speaking
            self.robot.question(
                state, "some really long long long long text input"
            )  # input utterance
            self.robot.user_speaking(state, False)  # user speaking stopped

        ##############
        self.robot.reset()
        threading.Timer(0, short_pause_interrupt_events).start()
        controller_manager.wait_for(types.UTTERANCE)
        time.sleep(1)
        self.assertEqual(len(self.robot.actions), 1)
        self.assertEqual(self.robot.actions[0]["type"], action_types.INTERRUPTION)

        ##############
        self.robot.reset()
        threading.Timer(0, long_input_interrupt_events).start()
        controller_manager.wait_for(types.UTTERANCE)
        time.sleep(1)
        self.assertEqual(len(self.robot.actions), 1)
        self.assertEqual(self.robot.actions[0]["type"], action_types.INTERRUPTION)

    def test_responsivity_controller(self):
        state = State()
        controller_manager = ControllerManager(state, self.robot.handle_action)

        for controller in controller_manager.controllers.values():
            controller.enabled = False
        controller_manager.get_controller("responsivity_controller").enabled = True

        self.robot.user_speaking(state, False)  # user speaking stopped
        self.robot.robot_speaking(state, False)  # robot speaking stopped

        def full_stop_events():
            self.robot.robot_speaking(state, True)  # robot speaking
            time.sleep(0.5)
            self.robot.user_speaking(state, True)  # user speaking
            self.robot.question(state, "be quiet sophia")  # input utterance
            self.robot.user_speaking(state, False)  # user speaking stopped

        def wakeup_events():
            self.robot.robot_speaking(state, True)  # robot speaking
            time.sleep(0.5)
            self.robot.user_speaking(state, True)  # user speaking
            self.robot.question(state, "what do you think sophia")  # input utterance
            self.robot.user_speaking(state, False)  # user speaking stopped

        ##############
        self.robot.reset()
        threading.Timer(0, full_stop_events).start()
        controller_manager.wait_for(types.UTTERANCE)
        time.sleep(1)
        actions = controller_manager.act()
        self.assertEqual(len(actions), 1)
        self.assertEqual(actions[0]["type"], action_types.RESPONSIVITY)
        self.assertEqual(actions[0]["payload"]["responsivity"], 0)

        ##############
        self.robot.reset()
        threading.Timer(0, wakeup_events).start()
        controller_manager.wait_for(types.UTTERANCE)
        time.sleep(1)
        actions = controller_manager.act()
        self.assertEqual(len(actions), 1)
        self.assertEqual(actions[0]["type"], action_types.RESPONSIVITY)
        self.assertEqual(actions[0]["payload"]["responsivity"], 1)


if __name__ == "__main__":
    unittest.main()
    # python test/test_controllers.py ControllerTestCase.test_placeholder_utterance_controller
