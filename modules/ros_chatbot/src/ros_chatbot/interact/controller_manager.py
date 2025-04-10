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

import functools
import logging
import os
import threading
import time
from collections import defaultdict

import yaml
from benedict import benedict
from haipy.utils import envvar_yaml_loader, to_list

from . import event_types
from .controllers import registered_controllers
from .controllers.base import AsyncAgentController, SyncAgentController
from .event_generator import StateEvent

logger = logging.getLogger(__name__)


def synchronized(func):
    @functools.wraps(func)
    def wrapper(self, *args, **kwargs):
        lock = vars(self).get("_sync_lock", None)
        if lock is None:
            lock = vars(self).setdefault("_sync_lock", threading.RLock())
        with lock:
            return func(self, *args, **kwargs)

    return wrapper


class ControllerManager(object):
    def __init__(self, state, action_handler):
        self.state = state
        self.event_generator = StateEvent(self.state, self.on_event)
        self.action_handler = action_handler
        self._events = []
        self._listeners = defaultdict(list)
        self.added_new_event = threading.Event()

        envvar_yaml_loader()
        # load specs
        HR_CHATBOT_WORLD_DIR = os.environ.get("HR_CHATBOT_WORLD_DIR", "")
        spec_file = os.path.join(HR_CHATBOT_WORLD_DIR, "controllers.yaml")
        if spec_file and os.path.isfile(spec_file):
            with open(spec_file) as f:
                config = yaml.safe_load(f)
            self.specs = config["controllers"]
        else:
            self.specs = []
            logger.warning("Controller spec file was not found")

        # load config
        config_file = os.path.join(HR_CHATBOT_WORLD_DIR, "control.yaml")
        if config_file and os.path.isfile(config_file):
            with open(config_file) as f:
                config = yaml.safe_load(f)
                self.controller_configs = {
                    controller["id"]: benedict(controller.get("config", {}))
                    for controller in config["controllers"]
                }
        else:
            self.controller_configs = {}
            logger.warning("Controller config file was not found")

        self.install_controllers()

    def wait_for(self, event_types, timeout=None):
        """Waits for the coming of any of the events of a given type"""
        end_time = None
        if timeout is not None:
            end_time = time.time() + timeout
        pos = len(self._events)
        while True:
            self.added_new_event.clear()
            if end_time is not None:
                if end_time < time.time():
                    logger.info("event timeout")
                    return False
            for event in self._events[pos:]:
                if event["type"] in to_list(event_types):
                    return True
            self.added_new_event.wait(0.02)

    def get_controller(self, id):
        return self.controllers.get(id)

    def install_controllers(self):
        # install controllers
        self.controllers = {}
        for spec in self.specs:
            if spec["type"] not in registered_controllers:
                raise ValueError("Unknown controller type: %s" % spec["type"])
            if spec.get("disabled"):
                continue

            args = spec.get("args", {})
            args["state"] = self.state
            args["store"] = self
            cls = registered_controllers[spec["type"]]
            controller = cls(**args)
            for event in controller.subscribe_events:
                self.addEventListener(event, controller)
            self.controllers[controller.id] = controller
        if self.controllers:
            logger.info(
                "Added controllers %s", ", ".join(list(self.controllers.keys()))
            )

        # set controller config
        for id, controller in self.controllers.items():
            if id in self.controller_configs:
                config = self.controller_configs[id]
                controller.set_config(config)
            else:
                logger.info('controller "%s" has no config', id)

    def setup_controllers(self, cfg):
        for controller_id in [
            "placeholder_utterance_controller",
            "language_switch_controller",
            "interruption_controller",
            "emotion_controller",
            "monitor_controller",
            "command_controller",
            "responsivity_controller",
            "user_acquisition_controller",
        ]:
            cfg_name = "enable_%s" % controller_id
            if cfg_name in cfg:
                controller_enabled = getattr(cfg, cfg_name)
            else:
                continue
            controller = self.get_controller(controller_id)
            if controller:
                if controller.enabled != controller_enabled:
                    controller.enabled = controller_enabled
                    logger.info(
                        "Controller %s is %s"
                        % (
                            controller.id,
                            "enabled" if controller.enabled else "disabled",
                        )
                    )
            else:
                if controller_enabled:
                    setattr(cfg, cfg_name, False)
                    logger.warning("No controller %s configured", controller_id)

        placeholder_utterance_controller = self.get_controller(
            "placeholder_utterance_controller"
        )
        if placeholder_utterance_controller:
            if cfg.placeholder_utterances:
                utterances = cfg.placeholder_utterances.splitlines()
                utterances = [
                    utterance.strip() for utterance in utterances if utterance.strip()
                ]
                placeholder_utterance_controller.set_placeholder_utterances(utterances)
            else:
                placeholder_utterance_controller.set_placeholder_utterances([])
            placeholder_utterance_controller.set_prob_escalate_step(
                cfg.placeholder_prob_step
            )

    def dispatch(self, action):
        """Dispatches the actions from controllers"""
        self.action_handler(action)

    def wait_controller_finish(self):
        for controller in self.controllers.values():
            if isinstance(controller, AsyncAgentController):
                while True:
                    if controller.is_idle():
                        break
                    else:
                        logger.warning("wait for %s to finish", controller.id)
                        time.sleep(0.1)

    @synchronized
    def act(self):
        """Gets the synchronous actions"""
        actions = []
        for controller in self.controllers.values():
            if isinstance(controller, SyncAgentController):
                if not controller.events.empty():
                    try:
                        _actions = controller.act()
                        if _actions:
                            actions.extend(_actions)
                    except Exception as ex:
                        logger.exception(ex)
                    finally:
                        controller.done()
        return actions

    @synchronized
    def on_event(self, event):
        """Handles new events"""
        self._events.append(event)
        # event_id = len(self._events)
        # payload_str = str(event["payload"])[:40]  # TODO: stringify payload
        # logger.warning("Event %d type: %s, payload: %s...", event_id, event['type'], payload_str)
        self.added_new_event.set()

        event_listeners = []
        event_listeners += self._listeners.get(event["type"], [])
        event_listeners += self._listeners.get(event_types.ALL_EVENTS, [])
        for listener in event_listeners:
            if listener.enabled:
                logger.info("[%s] observe %s", listener.id, event)
                listener.observe(event)
            else:
                logger.debug("Event listener %r is disabled", listener.id)

    def addEventListener(self, type, listener):
        self._listeners[type].append(listener)

    def register_event_generator(self, state, generator):
        self.event_generator.register_event_generator(state, generator)

    def reset(self):
        # reset event generator
        self.event_generator.reset()

        # reset controllers
        for controller in self.controllers.values():
            if hasattr(controller, "reset"):
                try:
                    controller.reset()
                except Exception as ex:
                    logger.exception(ex)

        logger.warning("Reset controller manager")
