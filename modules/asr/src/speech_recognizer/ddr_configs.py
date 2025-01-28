#
# Copyright (C) 2017-2024 Hanson Robotics
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.
#

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
import re
# Dynamic reconfigure node, allow to make parameters in the children constructors similar to ROS params
    

class DDRNode:

    def __init__(self, namespace=None, callback=None):
        self.__callback = callback
        self._dd_cfg = {}
        self.__ddr = DDynamicReconfigure(namespace)
    
    def ddstart(self):
        self.__ddr.start(self.__dd_callback)
    

    def __dd_callback(self, config, level=None):
        if self.__callback:
            config = self.__callback(config, level)
        self._dd_cfg = config
        return config
    
    def _get_cfg_entry(self, name, default):
        return self._dd_cfg.get(name, default)

    
    def new_param(self, name, description, default=None, min=None, max=None, edit_method=""):
        type = self.__get_type(default)
        self.__ddr.add(name, type, 0, description, default, min, max, edit_method)
        setattr(__class__,name, property(lambda self=self, x=name, d=default: self._get_cfg_entry(x, d)))

    # allolws update configuration
    def update_configuration(self, params):
        self.__ddr.dyn_rec_srv.update_configuration(params)        
    
    def enum(self, options):
        enum = []
        if type(options) == list:
            options = {o:o for o in options}
        if type(options) != dict:
            raise TypeError("Enum options must be a dict or list")
        
        for k, v in options.items():
            # value name pairs
            c = self.__ddr.const(self.__name_from_str(k), self.__get_type(v), v, k)
            enum.append(c)
        return self.__ddr.enum(enum, 'enum')
    
    @staticmethod
    def __get_type(value):
        if type(value) == int:
            return "int"
        elif type(value) == float:
            return "double"
        elif type(value) == str:
            return "str"
        elif type(value) == bool:
            return "bool"
        else:
            raise TypeError("Unknown type for value: " + str(value))
        
    @staticmethod
    def __name_from_str(input_str):
        # Replace any non-letter and non-number character with a single underscore
        output_str = re.sub('[^a-zA-Z0-9]+', '_', input_str).lower()[:30]
        
        return output_str
    