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

import json
import random

import rospy
from std_srvs.srv import Trigger, TriggerResponse

from ros_chatbot.ddr_node import DDRNode


class TriggerServiceNode:
    def __init__(self):
        rospy.init_node("priming_service")
        rospy.Service(
            "/hr/interaction/prompts/response_prompt",
            Trigger,
            self.trigger_service_callback,
        )
        self.response_primers = [
            {
                "enabled": True,
                "name": "funny",
                "prompt": "Respond to below question in very funny way",
                "probability": 0.5,
            },
            {
                "enabled": True,
                "name": "serious",
                "prompt": "Respond to asnwer as a serious robot.",
                "probability": 0.5,
            },
        ]
        self.length_primers = [
            {
                "enabled": True,
                "name": "Medium",
                "prompt": "Respond in 2-3 sentences",
                "probability": 0.5,
            },
            {
                "enabled": True,
                "name": "Smart",
                "prompt": "If question is short respond in 1 sentence. For more elaborate questions respond in multiple sentences",
                "probability": 0.7,
            },
            {
                "enabled": True,
                "name": "Long",
                "prompt": "Respond in long question",
                "probability": 0.2,
            },
        ]
        self.current_reponse_primer = "auto"
        self.current_length_primer = "auto"
        self.start_ddrs()

    def start_ddr(self, label="response"):
        node = DDRNode(
            namespace=f"/hr/interaction/prompts/{label}",
            callback=lambda config, level: self.update_prompts(
                config, label == "response"
            ),
        )
        node.new_param("current", f"Current {label} primer", default="auto")
        primers = self.response_primers if label == "response" else self.length_primers
        node.new_param("prompts", "Current prompts", default=json.dumps(primers))
        node.new_param("node_schema", "node_schema", default=self.node_schema(primers))
        node.ddstart()
        return node

    def start_ddrs(self):
        self.response_ddr = self.start_ddr("response")
        self.length_ddr = self.start_ddr("length")

    def update_prompts(self, config, is_response):
        prompts = json.loads(config["prompts"])
        if is_response:
            self.current_reponse_primer = config["current"]
            self.response_primers = prompts
        else:
            self.current_length_primer = config["current"]
            self.length_primers = prompts
        # config.prompts = json.dumps(primers)
        config.node_schema = self.node_schema(prompts)
        return config

    def node_schema(self, prompts):
        names = [p["name"] for p in prompts if p["name"]]
        node_schema = {
            "current": {"type": "string", "default": "auto", "enum": ["auto"] + names},
            "prompts": {
                "type": "array",
                "format": "tabs",
                "items": {
                    "type": "object",
                    "headerTemplate": "{{self.name}} - {{self.probability}}",
                    "properties": {
                        "enabled": {
                            "type": "boolean",
                            "default": True,
                            "format": "checkbox",
                        },
                        "name": {"type": "string", "default": "", "maxLength": 10},
                        "probability": {"type": "number", "default": 0.0},
                        "prompt": {
                            "type": "string",
                            "default": "",
                            "format": "textarea",
                            "expand_height": True,
                        },
                    },
                },
            },
        }
        return json.dumps(node_schema)

    def get_by_name(self, primers, name):
        for primer in primers:
            if primer["name"] == name:
                return primer
        return primers[0]

    def pick_random_primer(self, primers):
        total_probability = sum(
            primer["probability"]
            for primer in primers
            if primer["enabled"] and primer["prompt"]
        )
        random_number = random.uniform(0, total_probability)
        cumulative_probability = 0
        for primer in primers:
            if primer["enabled"] is False or primer["prompt"] == "":
                continue
            cumulative_probability += primer["probability"]
            if random_number <= cumulative_probability:
                return primer

    def format_response_prompt(self):
        response_primer = (
            self.pick_random_primer(self.response_primers)
            if self.current_reponse_primer == "auto"
            else self.get_by_name(self.response_primers, self.current_reponse_primer)
        )
        length_primer = (
            self.pick_random_primer(self.length_primers)
            if self.current_length_primer == "auto"
            else self.get_by_name(self.length_primers, self.current_length_primer)
        )
        prompt = response_primer["prompt"]
        if prompt[-1] not in [".", "?", "!"]:
            prompt += "."
        prompt += " " + length_primer["prompt"]
        return prompt

    def trigger_service_callback(self, request):
        response = TriggerResponse()
        response.success = True
        response.message = self.format_response_prompt()
        return response

    def run(self):
        r = rospy.Rate(1)
        while not rospy.is_shutdown():
            r.sleep()
            rospy.set_param(
                "/hr/interaction/prompts/response_prompt", self.format_response_prompt()
            )
        rospy.spin()


if __name__ == "__main__":
    node = TriggerServiceNode()
    node.run()
