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
from pathlib import Path
from typing import Dict, List

import yaml
from haipy.schemas.airtable_schemas import OperationSceneRecord
from haipy.utils import to_list

from ros_chatbot.schemas import Scene, SceneEvent
from ros_chatbot.utils import load_sheet_meta

logger = logging.getLogger(__name__)


class DataLoader:
    def __init__(self, cms_dir: str, session_context):
        self.cms_dir = cms_dir
        self.session_context = session_context
        self.presets = {}
        self.scenes = {}
        self.scenes["default"] = Scene(name="default", type="preset", default=True)

    def load_operation_scenes(self) -> List[dict]:
        scene_file = Path(self.cms_dir) / "airtable-operation-scenes.yaml"
        scenes = []
        if scene_file.exists():
            with open(scene_file) as f:
                scenes = yaml.safe_load(f)
                scenes = [OperationSceneRecord(**scene) for scene in scenes]
                logger.info("Loaded %d scenes from %s", len(scenes), scene_file)
        else:
            logger.warning("Scene file %s does not exist", scene_file)

        loaded_scenes = []
        for scene in scenes:
            fields = scene.fields
            variables = {
                "general_prime": fields.GeneralPrimer,
                "situational_prime": fields.SituationalPrimer,
                "response_prime": fields.ResponsePrimer,
                "objective": fields.Objective,
                "location": fields.Location,
                "prompt_template": fields.PromptTemplate,
            }
            # Filter prompts to include only specific keys and non-empty values
            prompts = {
                k: v
                for k, v in variables.items()
                if k in ["general_prime", "situational_prime", "response_prime"] and v
            }
            # Filter variables to include only non-empty values
            variables = {k: v for k, v in variables.items() if v}

            loaded_scene = {
                "name": fields.Name,
                "type": "preset",
                "variables": variables,
                "tts_mapping": fields.TTSMapping or "",
                "asr_context": fields.ASRContext or "",
                "knowledge_base": fields.KnowledgeBase or "",
                "prompts": prompts,
            }
            loaded_scenes.append(loaded_scene)
            logger.info(
                "Loaded scene: %s with variables: %s", loaded_scene["name"], variables
            )
        return loaded_scenes

    def load_prompt_presets(self) -> Dict[str, dict]:
        preset_file = Path(self.cms_dir) / "prompt_presets.yaml"
        presets = {}
        if preset_file.is_file():
            with open(preset_file) as f:
                presets_data = yaml.safe_load(f)
                if "presets" in presets_data:
                    presets.update(presets_data["presets"])
                    logger.info(
                        "Loaded %d prompt presets from %s", len(presets), preset_file
                    )
        else:
            logger.info("Prompt preset file %s is not found", preset_file)
        return presets

    def load_arf_scene_sheets(self) -> Dict[str, Scene]:
        sheets = load_sheet_meta(Path(self.cms_dir) / "arf_sheets")

        scenes = {}
        for sheet in sheets:
            events = [
                SceneEvent(**{"scene": sheet.scene, "arf_event": arf_event})
                for arf_event in sheet.arf_events
            ]
            scene = Scene(
                **{
                    "name": sheet.scene,
                    "default": sheet.header.get("DefaultScene", False),
                    "conditions": to_list(sheet.header.get("SceneCondition", [])),
                    "variables": sheet.header.get("Variables", {}),
                    "asr_context": sheet.header.get("ASRContext", ""),
                    "tts_mapping": sheet.header.get("TTSMapping", ""),
                }
            )
            scenes[scene.name] = {
                "scene": scene,
                "events": events,
            }
        logger.info("Loaded ARF scenes")
        return scenes

    def load_all_data(self):
        """Loads all necessary data including prompt presets, ARF scene sheets, and operation scenes"""

        # load prompt presets
        presets = self.load_prompt_presets()
        self.presets.update(presets)
        self.scenes.update(
            {
                key: Scene(
                    name=key,
                    type="preset",
                    variables=value.get("context", {}),
                    knowledge_base=value.get("knowledge_base", ""),
                )
                for key, value in self.presets.items()
            }
        )
        logger.info("Scenes %s", self.scenes)

        # load arf scenes
        scenes_data = self.load_arf_scene_sheets()

        self.session_context.proxy.delete_param("arf.events")
        self.session_context.proxy.delete_param("arf.scenes")
        self.session_context.proxy.delete_param("interlocutor")

        for scene_name, data in scenes_data.items():
            events = data["events"]
            scene = data["scene"]
            if events:
                self.session_context.proxy.set_param(
                    f"arf.events.{scene_name}",
                    [event.model_dump_json() for event in events],
                )
            self.scenes[scene.name] = scene

        self.session_context.proxy.set_param(
            "arf.scenes", [scene.model_dump_json() for scene in self.scenes.values()]
        )

        # load operation scenes
        operation_scenes = self.load_operation_scenes()
        for scene in operation_scenes:
            key = scene["name"]
            self.presets[key] = {}
            self.presets[key]["title"] = scene["name"]
            self.presets[key]["prompts"] = scene["prompts"]
            self.scenes[scene["name"]] = Scene(**scene)
        logger.info("All data loaded successfully")
