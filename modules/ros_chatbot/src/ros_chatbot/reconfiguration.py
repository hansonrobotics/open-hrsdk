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

import json
import logging
import threading
from functools import partial
from typing import Callable, List, cast

from apscheduler.job import Job
from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from haipy.parameter_server_proxy import UserSessionContext
from haipy.redis_event_listener import EventListener
from haipy.scheduler.ims_drivers import BaseDriver, Drivers
from haipy.scheduler.intention_manager import IntentionManager
from pydantic import BaseModel
from slugify import slugify

from ros_chatbot.activity_monitor import EngagementLevel
from ros_chatbot.agents.model import Agent, ConfigurableAgent, LLMAgent
from ros_chatbot.ddr_node import DDRNode, Enum
from ros_chatbot.schemas import Scene
from ros_chatbot.utils import (
    load_agent_config,
    load_prompt_templates_config,
    set_n8n_enabled,
)

logger = logging.getLogger("hr.ros_chatbot.reconfiguration")


class SceneReconfiguration:
    """
    This class handles the creation, storage, and updating of scene-related
    parameters using dynamic reconfiguration.
    """

    def __init__(
        self,
        character: str,
        session_context: UserSessionContext,
        on_scene_enter_callback: Callable,
        on_scene_exit_callback: Callable,
        on_scene_change_callback: Callable,
    ):
        self.current_scene = Scene(name="auto")
        self.character = character
        self.session_context = session_context
        self.on_scene_enter_callback = on_scene_enter_callback
        self.on_scene_exit_callback = on_scene_exit_callback
        self.on_scene_change_callback = on_scene_change_callback
        self.node = None
        self._instant_situational_prompt = ""

        config = load_agent_config()
        self.scenes = [
            self._create_scene_from_config(scene) for scene in config.get("scenes", [])
        ]

    def _create_scene_from_config(self, scene_config):
        variables = {}
        # Goes to REDIS
        variables_keys = [
            "general_prime",
            "situational_prime",
            "dynamic_situational_prime",
            "response_prime",
            "objective",
            "interlocutor",
        ]
        for attr in variables_keys:
            if attr in scene_config and scene_config[attr]:
                variables[attr] = scene_config[attr]
        conditions = scene_config.get("conditions", [])
        if isinstance(conditions, str):
            conditions = conditions.splitlines()
        return Scene(
            name=scene_config["name"],
            character=self.character,
            variables=variables,
            knowledge_base=scene_config.get("knowledge_base", ""),
            tts_mapping=scene_config.get("tts_mapping", ""),
            asr_context=scene_config.get("asr_context", ""),
            conditions=conditions,
        )

    def _format_scene_for_ddr(self, scene: Scene) -> dict:
        ordered_keys = [
            "name",
            "general_prime",
            "situational_prime",
            "dynamic_situational_prime",
            "response_prime",
            "interlocutor",
            "objective",
            "knowledge_base",
            "tts_mapping",
            "asr_context",
            "conditions",
        ]
        formatted_scene = {}
        for key in ordered_keys:
            if key == "name":
                formatted_scene[key] = scene.name
            elif key in ["knowledge_base", "tts_mapping", "asr_context"]:
                formatted_scene[key] = getattr(scene, key)
            elif key == "conditions":
                formatted_scene[key] = "\n".join(scene.conditions)
            else:
                formatted_scene[key] = scene.variables.get(key, "")
        return formatted_scene

    def start_ddr(self):
        logger.info("Starting scene reconfiguration")
        self.node = DDRNode(
            namespace="/hr/interaction/prompts/scene",
            callback=lambda config, level: self.reconfig(config, level),
        )
        self.node.new_param("current", "Current scene", default="auto")
        self.node.new_param(
            "instant_situational_prompt",
            "Instant Situational Prompt",
            default="",
        )
        self.node.new_param(
            "scenes",
            "Scenes",
            default=json.dumps(
                [self._format_scene_for_ddr(scene) for scene in self.scenes]
            ),
        )
        self.node.new_param(
            "node_schema",
            "node_schema",
            default=self.node_schema(["auto"] + [scene.name for scene in self.scenes]),
        )
        self.node.ddstart()

    def get_current_scene(self) -> Scene:
        if self.node:
            return next(
                (
                    scene
                    for scene in self.scenes
                    if scene.name == self.current_scene.name
                ),
                Scene(name="auto"),
            )
        else:
            logger.warning("Scene reconfiguration node not started")

    @property
    def instant_situational_prompt(self):
        return self._instant_situational_prompt

    def reconfig(self, config, level):
        try:
            self._instant_situational_prompt = config.get(
                "instant_situational_prompt", ""
            )
            self.session_context[
                "instant_situational_prompt"
            ] = self._instant_situational_prompt
            self.scenes = [
                self._create_scene_from_config(scene)
                for scene in json.loads(config["scenes"])
            ]
            self.on_scene_change_callback(self.scenes)
            logger.warning(
                f"Reconfigured scenes: {[scene.name for scene in self.scenes]}"
            )

            scene_names = ["auto"] + [scene.name for scene in self.scenes]
            config["node_schema"] = self.node_schema(scene_names)

            # Find requested scene or default to auto
            current_scene_obj = next(
                (scene for scene in self.scenes if scene.name == config["current"]),
                Scene(name="auto"),
            )

            # Handle scene transitions if scene has changed
            if self.current_scene.name != current_scene_obj.name:
                self._handle_scene_transition(current_scene_obj)
                self.current_scene = current_scene_obj

            return config

        except Exception as e:
            logger.exception(f"Failed to reconfigure scene: {str(e)}")
            return config

    def _handle_scene_transition(self, new_scene):
        """Helper method to handle scene exit/enter callbacks"""
        if self.current_scene.name != "auto":
            logger.info(f"Exiting scene: {self.current_scene.name}")
            self.on_scene_exit_callback(self.current_scene)

        if new_scene.name != "auto":
            logger.info(f"Entering scene: {new_scene.name}")
            self.on_scene_enter_callback(new_scene)

    def node_schema(self, scene_names: List[str]):
        node_schema = {
            "current": {
                "type": "string",
                "default": "auto",
                "enum": scene_names,
                "title": "Current Scene",
            },
            "instant_situational_prompt": {
                "type": "string",
                "default": "",
                "title": "Instant Situational Prompt",
                "format": "textarea",
                "expand_height": True,
            },
            "scenes": {
                "type": "array",
                "format": "tabs",
                "items": {
                    "type": "object",
                    "headerTemplate": "{{self.name}}",
                    "properties": {
                        "name": {
                            "type": "string",
                            "default": "",
                            "title": "Scene Name",
                        },
                        "general_prime": {
                            "type": "string",
                            "default": "",
                            "format": "textarea",
                            "expand_height": True,
                            "title": "General Prime",
                        },
                        "situational_prime": {
                            "type": "string",
                            "default": "",
                            "format": "textarea",
                            "expand_height": True,
                            "title": "Situational Prime",
                        },
                        "dynamic_situational_prime": {
                            "type": "string",
                            "default": "",
                            "format": "textarea",
                            "expand_height": True,
                            "title": "Dynamic Situational Prime",
                        },
                        "response_prime": {
                            "type": "string",
                            "default": "",
                            "format": "textarea",
                            "expand_height": True,
                            "title": "Response Prime (Magic response prompt will be concatenated)",
                        },
                        "objective": {
                            "type": "string",
                            "default": "",
                            "format": "textarea",
                            "expand_height": True,
                            "title": "Objective (REQUIRES GLOBAL WORKSPACE DRIVERS TO BE ENABLED)",
                        },
                        "interlocutor": {
                            "type": "string",
                            "default": "",
                            "title": "Interlocutor (The user name the robot is talking to)",
                        },
                        "knowledge_base": {
                            "type": "string",
                            "default": "",
                            "format": "textarea",
                            "expand_height": True,
                            "title": "Knowledge Base (The knowledge base of the scene used for RAG)",
                        },
                        "tts_mapping": {
                            "type": "string",
                            "default": "",
                            "title": "TTS Mapping",
                        },
                        "asr_context": {
                            "type": "string",
                            "default": "",
                            "title": "ASR Context",
                        },
                    },
                },
            },
        }
        return json.dumps(node_schema)


class AgentReconfiguration(object):
    def __init__(self):
        self._dd_servers = {}
        self.lock = threading.RLock()

    def set_up_agents_runtime_dynamic_reconfigure(self, agent: Agent, presets: dict):
        """
        Sets up the runtime dynamic reconfiguration for the given chat agent.

        Args:
            agent (Agent): The agent for which to set up the dynamic reconfigure server.
            presets (dict): The presets for the agent.
        """
        callback = partial(self.agent_config_callback, agent, presets)

        def build_schema(cfg):
            schema = {
                k: {"type": "string", "format": "textarea"}
                if v.get("type") == "text"  # text type is a textarea in the UI
                else {"type": "string"}
                for k, v in cfg.items()
                if isinstance(v["default"], str)
            }
            return json.dumps(schema)

        def build_enum(dd: DDynamicReconfigure, enum: list):
            # allow int and string anums for ddynrec. enum is array of name,value, description
            if not enum:
                return ""
            result = [
                dd.const(e[0], "str" if isinstance(e[1], str) else "int", e[1], e[2])
                for e in enum
                if len(e) == 3
            ]
            return dd.enum(result, "enum")

        def get_type(val):
            return {int: "int", float: "double", bool: "bool", str: "str"}.get(
                type(val), "str"
            )

        if agent.runtime_config_description is not None:
            with self.lock:
                try:
                    cfg = agent.runtime_config_description
                    dd = DDynamicReconfigure(
                        f"/hr/interaction/agents/{slugify(agent.id, separator='_')}"
                    )
                    for k, v in cfg.items():
                        dd.add(
                            k,
                            get_type(v.get("default")),
                            level=0,
                            description=v.get("description"),
                            default=v.get("default"),
                            min=v.get("min"),
                            max=v.get("max"),
                            edit_method=build_enum(dd, v.get("enum", [])),
                        )
                    dd.add_variable("node_schema", "UI Schema", build_schema(cfg))
                    if agent.id in self._dd_servers:
                        logger.info(
                            "Shutting down existing dynamic reconfigure server for agent %s",
                            agent.id,
                        )
                        self._dd_servers[agent.id].dyn_rec_srv.set_service.shutdown()
                    logger.info(
                        "Starting dynamic reconfigure server for agent %s", agent.id
                    )
                    dd.start(callback)
                    agent.runtime_config_callback = dd.dyn_rec_srv.update_configuration
                    self._dd_servers[agent.id] = dd
                except Exception as e:
                    logger.exception(
                        "Failed to register agent configuration %s: %s", agent.id, e
                    )

    def agent_config_callback(
        self, agent: Agent, presets: dict, config: dict, level: int
    ) -> dict:
        agent.enabled = config.get("enabled")

        if agent.id == "n8n" and config.get("enable_db_keeper") is not None:
            logger.warning("Setting n8n enabled to %s", config.get("enable_db_keeper"))
            set_n8n_enabled(config.get("enable_db_keeper"))

        if isinstance(agent, LLMAgent):
            prompt_preset = config.get("prompt_preset")
            current_preset = agent.config.get("prompt_preset")

            if prompt_preset and prompt_preset != current_preset:
                if prompt_preset == "default":
                    keys_to_reset = [
                        "system_prime",
                        "general_prime",
                        "situational_prime",
                        "dynamic_situational_prime",
                        "topical_prime",
                        "response_prime",
                        "prompt_prime",
                        "prompt_preset",
                    ]
                    agent.reset_config(keys_to_reset)
                    for key in keys_to_reset:
                        if key in agent.runtime_config_description:
                            config[key] = agent.config[key]
                    logger.info("Reset agent %s preset %r", agent.id, prompt_preset)
                elif prompt_preset in presets:
                    new_config = {"prompt_preset": prompt_preset}
                    new_config.update(presets[prompt_preset]["prompts"])
                    logger.info("Update agent %s preset %r", agent.id, prompt_preset)
                    config.update(new_config)
                    agent.set_config(new_config)
                else:
                    logger.error(
                        "Unrecognized preset %r in %s", prompt_preset, presets.keys()
                    )
            else:
                config_updates = {
                    k: v
                    for k, v in config.items()
                    if k in agent.runtime_config_description
                }
                agent.set_config(config_updates, base=False, update_server=False)
        elif isinstance(agent, ConfigurableAgent):
            config_updates = {
                k: v for k, v in config.items() if k in agent.runtime_config_description
            }
            agent.set_config(config_updates, base=False, update_server=False)
        return config

    def update_presets(self, agent: LLMAgent, presets: dict):
        prompt_preset_desc = agent.runtime_config_description.get("prompt_preset")
        if not prompt_preset_desc:
            return

        enum_list = prompt_preset_desc["enum"]
        enum_list.clear()
        enum_list.append(["default", "default", "Default"])  # Add default preset

        new_presets = [
            [preset, preset, preset_data["title"]]
            for preset, preset_data in presets.items()
            if [preset, preset, preset_data["title"]] not in enum_list
        ]

        enum_list.extend(new_presets)

        for preset in new_presets:
            logger.info("Added new preset %r to agent %s", preset[2], agent.id)

        self.set_up_agents_runtime_dynamic_reconfigure(
            agent,
            presets,
        )


class DriverReconfiguration:
    def __init__(
        self,
        session_context,
        intention_manager: IntentionManager,
    ):
        self.session_context = session_context
        self.intention_manager = intention_manager
        self.event_listener = EventListener(active=False)
        self._dd_servers = {}
        self.lock = threading.RLock()
        self.agent_config = load_agent_config()
        self._driver_callbacks: List[Callable[[BaseDriver, BaseModel], None]] = []
        self.drivers = {}
        self._create_drivers()
        self._driver_callbacks: List[Callable[[BaseDriver, BaseModel], None]] = []
        self.auto_global_workspace = False

    def add_driver_callback(self, callback: Callable[[BaseDriver, BaseModel], None]):
        self._driver_callbacks.append(callback)

    def on_engagement_level_change(self, new_level):
        if not self.auto_global_workspace:
            return
        enabled = new_level > EngagementLevel.NONE
        logger.warning("Set global workspace %s", enabled)
        self.enable_global_workspace() if enabled else self.disable_global_workspace()

        # the higher the engagement level, the more frequent the drivers should run
        # TODO: adjust driver timers
        # if new_level == EngagementLevel.NONE:
        #    for driver in self.drivers.values():
        #        driver.set_timer(0)
        # elif new_level == EngagementLevel.MINIMAL:
        #    for driver in self.drivers.values():
        #        driver.set_timer(30)
        # elif new_level == EngagementLevel.MODERATE:
        #    for driver in self.drivers.values():
        #        driver.set_timer(60)
        # elif new_level == EngagementLevel.HIGH:
        #    for driver in self.drivers.values():
        #        driver.set_timer(300)
        # elif new_level == EngagementLevel.INTENSIVE:
        #    for driver in self.drivers.values():
        #        driver.set_timer(900)

    def enable_global_workspace(self):
        self.event_listener.active = True
        self.event_listener.resume_scheduler()
        self.session_context["global_workspace_enabled"] = True
        self.intention_manager.enabled = True
        self.intention_manager.resume_scheduler()
        for driver in self.drivers.values():
            if driver.enabled:
                driver.connect()

    def disable_global_workspace(self):
        self.event_listener.active = False
        self.event_listener.pause_scheduler()
        self.session_context["global_workspace_enabled"] = False
        self.intention_manager.enabled = False
        self.intention_manager.pause_scheduler()
        for driver in self.drivers.values():
            if driver.enabled:
                driver.disconnect()

    def _driver_callback(self, driver: BaseDriver, output_model: BaseModel):
        for callback in self._driver_callbacks:
            callback(driver, output_model)

    def _create_drivers(self):
        driver_cfgs = self.agent_config.get("drivers", [])
        all_drivers = Drivers.get_all_drivers()

        for driver_cfg in driver_cfgs:
            if driver_cfg["type"] in all_drivers:
                driver_class = all_drivers[driver_cfg["type"]]
                driver = driver_class(
                    driver_cfg["name"],
                    self.session_context,
                    self.intention_manager,
                    self.event_listener,
                    self._driver_callback,
                )
                driver.type = driver_cfg["type"]
                driver.enabled = driver_cfg["enabled"]
                driver.timer_interval = driver_cfg.get("args", {}).get(
                    "timer_interval", None
                )
                self.set_up_driver_ddr(driver)
                self.drivers[driver.name] = driver
            else:
                logger.error("Driver %s not found", driver_cfg["type"])
        logger.info("drivers %s", self.drivers)

    def set_up_driver_ddr(self, driver: BaseDriver):
        callback = partial(self.driver_config_callback, driver)
        logger.info("Starting driver reconfiguration for %s", driver.type)
        ddr_node = DDRNode(
            namespace=f"/hr/interaction/drivers/{slugify(driver.name, separator='_')}",
            callback=callback,
        )
        schema = {
            "prompt_template": {
                "type": "string",
                "default": "",
                "format": "textarea",
                "expand_height": True,
                "title": "Prompt Template",
            },
        }
        ddr_node.ddr.add_variable("node_schema", "UI Schema", json.dumps(schema))
        ddr_node.new_param("enabled", "Enabled", default=driver.enabled)
        ddr_node.new_param("run_once", "Run Once", default=False)
        ddr_node.new_param(
            "timer_interval",
            "Timer Interval",
            default=driver.timer_interval or 0,
            edit_method=ddr_node.add_enums(
                [
                    Enum(
                        name="No timer",
                        value=0,
                        description="No timer",
                        type="int",
                    ),
                    Enum(
                        name="30 seconds",
                        value=30,
                        description="30 seconds",
                        type="int",
                    ),
                    Enum(
                        name="1 minute",
                        value=60,
                        description="1 minute",
                        type="int",
                    ),
                    Enum(
                        name="5 minutes",
                        value=300,
                        description="5 minutes",
                        type="int",
                    ),
                    Enum(
                        name="15 minutes",
                        value=900,
                        description="15 minutes",
                        type="int",
                    ),
                    Enum(
                        name="30 minutes",
                        value=1800,
                        description="30 minutes",
                        type="int",
                    ),
                    Enum(
                        name="1 hour",
                        value=3600,
                        description="1 hour",
                        type="int",
                    ),
                    Enum(
                        name="2 hours",
                        value=7200,
                        description="2 hours",
                        type="int",
                    ),
                    Enum(
                        name="4 hours",
                        value=14400,
                        description="4 hours",
                        type="int",
                    ),
                    Enum(
                        name="8 hours",
                        value=28800,
                        description="8 hours",
                        type="int",
                    ),
                    Enum(
                        name="12 hours",
                        value=43200,
                        description="12 hours",
                        type="int",
                    ),
                    Enum(
                        name="24 hours",
                        value=86400,
                        description="24 hours",
                        type="int",
                    ),
                ]
            ),
        )
        ddr_node.new_param(
            "trigger_keys",
            "Trigger Keys (separated by commas)",
            default=",".join(driver.trigger_keys),
        )
        ddr_node.new_param(
            "output_key",
            "Output Key",
            default=driver.output_key,
        )
        ddr_node.new_param(
            "prompt_template",
            "Prompt Template",
            default=driver.prompt_template,
        )
        ddr_node.ddstart()
        self._dd_servers[driver.type] = ddr_node

    def driver_config_callback(self, driver: BaseDriver, config: dict, level: int):
        driver.enabled = config.get("enabled")

        driver.connect() if driver.enabled else driver.disconnect()

        if config.get("prompt_template") != driver.prompt_template:
            driver.set_prompt_template(config.get("prompt_template"))

        driver.output_key = config.get("output_key")

        trigger_keys = {
            key.strip() for key in config.get("trigger_keys").split(",") if key.strip()
        }
        if trigger_keys and trigger_keys != driver.trigger_keys:
            driver.set_trigger_keys(trigger_keys)

        timer_interval = config.get("timer_interval")
        if timer_interval != driver.timer_interval:
            driver.set_timer(timer_interval)

        if config.get("run_once"):
            # run the driver once
            try:
                logger.warning("Running driver %r once", driver.name)
                driver.handle(None)
            except Exception as e:
                logger.exception("Error running driver %r: %s", driver.name, e)
            config["run_once"] = False

        # report the timer interval for drivers from the event listener scheduler
        summary = ""
        for job in self.event_listener._scheduler.get_jobs():
            job = cast(Job, job)
            if job.id.startswith("timer_"):
                driver_name = job.id.split("_")[1]
                timer_interval = job.trigger.interval.seconds
                summary += f'Driver "{driver_name}" has timer interval {timer_interval} seconds\n'
        if summary:
            logger.info(summary.strip())

        return config

    def on_namespace_change(self):
        self.event_listener.unsubscribe_all()
        self.event_listener.change_namespace(self.session_context.ns)
        self.intention_manager.reset()  # maybe not a good idea to reset
        self.session_context["global_workspace_enabled"] = self.event_listener.active

        for driver in self.drivers.values():
            driver.connect() if driver.enabled else driver.disconnect()


class PromptTemplatesReconfiguration:
    def __init__(self, agent_ids: List[str], session_context: UserSessionContext):
        config = load_prompt_templates_config()
        self.templates = config.get("prompt_templates", [])
        self.session_context = session_context
        self.agent_ids = agent_ids
        self.start_ddr()

    def start_ddr(self):
        logger.info("Starting prompt templates reconfiguration")
        self.node = DDRNode(
            namespace="/hr/interaction/prompts/templates",
            callback=lambda config, level: self.reconfig(config, level),
        )
        self.node.new_param(
            "templates",
            "Prompt Templates",
            default=json.dumps(self.templates[:]),
        )
        self.node.new_param(
            "node_schema",
            "node_schema",
            default=self.node_schema(),
        )
        self.node.ddstart()

    def node_schema(self):
        return json.dumps(
            {
                "templates": {
                    "type": "array",
                    "format": "tabs",
                    "items": {
                        "type": "object",
                        "headerTemplate": "{{self.name}}",
                        "properties": {
                            "name": {
                                "type": "string",
                                "default": "Unnamed template",
                                "title": "Template Name",
                            },
                            "template": {
                                "type": "string",
                                "default": "Unnamed template",
                                "format": "textarea",
                                "expand_height": True,
                                "title": "Prompt Template",
                            },
                            "conditions": {
                                "type": "array",
                                "title": "Conditions (Select one or more conditions for the template to be used)",
                                "uniqueItems": True,
                                "format": "checkbox",
                                "items": {
                                    "type": "string",
                                    "enum": [
                                        "global_workspace",
                                        "any_agent",
                                        *sorted(self.agent_ids),
                                    ],
                                    "enum_titles": [
                                        "Global Workspace",
                                        "Any Agent",
                                        *[
                                            f"Agent: {agent_id}"
                                            for agent_id in sorted(self.agent_ids)
                                        ],
                                    ],
                                    "options": {
                                        "enum": [
                                            {
                                                "title": "Global Workspace",
                                                "infoText": "The template will be used if the global workspace is enabled",
                                            },
                                            {
                                                "title": "Any Agent",
                                                "infoText": "The template will be used if the agent ID matches any of the conditions",
                                            },
                                        ]
                                        + [
                                            {
                                                "title": f"Agent: {agent_id}",
                                                "infoText": f"The template will be used if the agent ID matches {agent_id}",
                                            }
                                            for agent_id in sorted(self.agent_ids)
                                        ],
                                    },
                                },
                            },
                        },
                    },
                },
            }
        )

    def export_templates(self):
        json_templates = [json.dumps(template) for template in self.templates]
        self.session_context["prompt_templates"] = json_templates

    def on_namespace_change(self):
        self.export_templates()

    def reconfig(self, config, level):
        self.templates = json.loads(config.get("templates"))
        self.export_templates()
        return config
