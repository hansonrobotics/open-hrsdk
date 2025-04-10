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
import time
from datetime import datetime
from threading import Thread
from typing import List

from haipy.text_processing.template_renderer import Renderer
from transitions.extensions import HierarchicalMachine
from transitions.extensions.nesting import NestedState

from ros_chatbot.schemas import Scene

logger = logging.getLogger("hr.ros_chatbot.state_machine")
logging.getLogger("transitions").setLevel(logging.WARN)


NestedState.separator = "/"


class RobotState(HierarchicalMachine):
    INITIAL_STATE = "initial"
    CHATTING_STATE = "chatting"
    MANUAL_STATE = "manual"
    IDLE_STATE = "idle"

    def __init__(
        self,
        session_context,
        name,
        on_enter_scene_callback=None,
        on_exit_scene_callback=None,
        idle_timeout=0,
    ):
        HierarchicalMachine.__init__(
            self,
            states=[RobotState.INITIAL_STATE, RobotState.MANUAL_STATE],
            name=name,
            initial=RobotState.INITIAL_STATE,
            send_event=True,
        )
        self.on_enter_scene_callback = on_enter_scene_callback
        self.on_exit_scene_callback = on_exit_scene_callback
        self.idle_timeout = idle_timeout
        self.renderer = Renderer()
        self.session_context = session_context
        self.default_scene = None
        self.scenes = []

        Thread(target=self.update_state, daemon=True).start()

    def on_enter_scene(self, event):
        for scene in self.scenes:
            if scene.name == event.state.value:
                logger.info("Entering scene %s", scene)
                if self.on_enter_scene_callback:
                    self.on_enter_scene_callback(scene)
                return

    def on_exit_scene(self, event):
        for scene in self.scenes:
            if scene.name == event.state.value:
                logger.info("Exiting scene %s", scene)
                if self.on_exit_scene_callback:
                    self.on_exit_scene_callback(scene)
                return

    def load_scenes(self, scenes: List[Scene]):
        self.scenes = scenes

        for scene in self.scenes:
            # set default scene
            if scene.default:
                self.default_scene = scene
                break

            # render variable template in the order of variables
            # context = {}
            # context.update(self.session_context.items())
            # for key, value in scene.variables.items():
            #    if value is None:
            #        value = ""
            #    if value and "{" in value:
            #        value = self.renderer.render(value, context=context, compact=False)
            #        scene.variables[key] = value
            #    context[key] = value

        # set up scene states and transitions
        if self.scenes:
            children_states = [scene.name for scene in self.scenes]
            children_states.append(RobotState.IDLE_STATE)
            for root_state in [RobotState.CHATTING_STATE, RobotState.MANUAL_STATE]:
                self.add_states({"name": root_state, "children": children_states})
                for state in self.states[root_state].states.values():
                    state.on_enter.append(self.on_enter_scene)
                    state.on_exit.append(self.on_exit_scene)
            logger.info("states %s", self.get_nested_state_names())
            self.add_transition("chat", "*", RobotState.CHATTING_STATE)
            self.add_transition("manual", "*", RobotState.MANUAL_STATE)
            self.add_transition("reset", "*", RobotState.INITIAL_STATE)

    def update_chat_state(self):
        parent_state = self.state.split(NestedState.separator)[0]
        if parent_state in [RobotState.CHATTING_STATE]:
            last_active_time = self.session_context.get("last_active_time")
            if last_active_time and self.idle_timeout > 0:
                now = datetime.utcnow()
                elapse = (now - last_active_time).total_seconds()
                if elapse > self.idle_timeout:
                    next_state = f"{RobotState.CHATTING_STATE}{NestedState.separator}{RobotState.IDLE_STATE}"
                    if self.state != next_state:
                        self.to(next_state)
                    return

            for scene in self.scenes:
                if scene.conditions:
                    ret = self.evaluate_condition(scene.conditions)
                    if ret:
                        next_state = f"{RobotState.CHATTING_STATE}{NestedState.separator}{scene.name}"
                        if self.state != next_state:
                            self.to(next_state)
                        return

            if self.default_scene:
                next_state = f"{RobotState.CHATTING_STATE}{NestedState.separator}{self.default_scene.name}"
                if self.state != next_state:
                    self.to(next_state)

    def update_state(self):
        while True:
            self.update_chat_state()
            time.sleep(0.2)

    def evaluate_condition(self, condition):
        if isinstance(condition, list):
            return any([self._evaluate_condition(c) for c in condition])
        else:
            return self._evaluate_condition(condition)

    def _evaluate_condition(self, condition):
        """Evaluate a single condition"""
        condition = "{{%s}}" % condition
        context = {}
        context.update(self.session_context.items())
        ret = self.renderer.render(condition, context=context)
        if ret.lower() == "false" or ret.lower() == "none":
            ret = False
        elif ret.lower() == "true":
            ret = True
        return ret

    @property
    def scene(self):
        parent_state = self.state.split(NestedState.separator)[0]
        if parent_state in [RobotState.CHATTING_STATE, RobotState.MANUAL_STATE]:
            return self.get_state(self.state).value

    @scene.setter
    def scene(self, scene):
        parent_state = self.state.split(NestedState.separator)[0]
        if parent_state in [RobotState.CHATTING_STATE, RobotState.MANUAL_STATE]:
            self.to(f"{parent_state}{NestedState.separator}{scene}")


if __name__ == "__main__":
    import json

    from haipy.parameter_server_proxy import GlobalContext

    from ros_chatbot.schemas import Scene

    logging.basicConfig(level=logging.INFO)
    session_context = GlobalContext("default")
    statemachine = RobotState(session_context, name="Sophia")
    scenes = [Scene(**json.loads(scene)) for scene in session_context["arf.scenes"]]
    statemachine.load_scenes(scenes)
    statemachine.to(RobotState.CHATTING_STATE)
    print(statemachine.state)
    statemachine.scene = "ARF-Test-ARF-Example-Stranger"
    statemachine.to(RobotState.INITIAL_STATE)
    statemachine.reset()
    statemachine.chat()
    print(statemachine.state)

    while True:
        print(f"state: {statemachine.state}, current scene: {statemachine.scene}")
        time.sleep(1)
