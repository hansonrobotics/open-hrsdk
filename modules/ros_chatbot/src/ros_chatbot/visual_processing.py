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

from .ddr_node import DDRNode
import json 
class VisualProcessingConfig():
    def __init__(self):
        self.default_prompt = (
            "Just provide a straightforward description of the contents of the picture I have taken. "
            "If there are people in the picture, please focus on them, "
            'beginning with the phrase "You can see"'
        )
        self.cfg = None
        self.enabled = False
        self.interval = 5
        self.ddr = DDRNode('/hr/interaction/prompts/visual_processing', self.cfg_callback)
        self.ddr.new_param("enabled", "Enable GPT4-V descriptions", False)
        self.ddr.new_param("visual_prompt", "Visual prompt", self.default_prompt)
        self.ddr.new_param("interval", "Interval after results to new reuest",5,0,120)
        self.ddr.new_param("result_time", "Last result time (utc)","")
        self.ddr.new_param("results", "Last results","")
        
        self.ddr.new_param("node_schema", "node_schema", json.dumps({
            'visual_prompt': {'type': 'string','format': 'textarea'},
            'results': {'type': 'string','format': 'textarea'},           
        }))
        self.ddr.ddstart()
    
    def cfg_callback(self, config, level):
        if self.cfg is None:
            # Keep empty until its set.
            config.results = ""
            config.result_time = ""
        self.cfg = config
        self.enabled = self.cfg.enabled
        self.interval = self.cfg.interval
        return config
    
    def get_prompt(self):
        try:
            return self.cfg.visual_prompt
        except Exception:
            return self.default_prompt

    def update_results(self, results, utc_time):
        self.ddr.update_configuration({"results": results, 'result_time': utc_time})
        
        
        
            
    