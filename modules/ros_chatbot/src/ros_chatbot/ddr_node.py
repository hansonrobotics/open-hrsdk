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

import re
from typing import Any, List

from ddynamic_reconfigure_python.ddynamic_reconfigure import DDynamicReconfigure
from pydantic import BaseModel


class Enum(BaseModel):
    name: str
    type: str
    value: Any
    description: str


class DDRNode:
    """
    Dynamic reconfigure node, allow to make parameters in the children constructors similar to ROS params
    """

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

    def new_param(
        self, name, description, default=None, min=None, max=None, edit_method=""
    ):
        type = self.__get_type(default)
        self.__ddr.add(name, type, 0, description, default, min, max, edit_method)
        setattr(
            __class__,
            name,
            property(lambda self=self, x=name, d=default: self._get_cfg_entry(x, d)),
        )

    # allolws update configuration
    def update_configuration(self, params):
        self.__ddr.dyn_rec_srv.update_configuration(params)

    def enum(self, options):
        enum = []
        if isinstance(options, list):
            options = {o: o for o in options}
        if not isinstance(options, dict):
            raise TypeError("Enum options must be a dict or list")

        for k, v in options.items():
            # value name pairs
            c = self.__ddr.const(self.__name_from_str(k), self.__get_type(v), v, k)
            enum.append(c)
        return self.__ddr.enum(enum, "enum")

    def add_enums(self, enums: List[Enum]) -> str:
        """
        Add multiple enum constants and create an enum edit_method.
        """
        enum_list = [
            self.__ddr.const(enum.name, enum.type, enum.value, enum.description)
            for enum in enums
        ]
        return self.__ddr.enum(enum_list, "enum")

    def build_enum(self, enum: list):
        """
        allow int and string anums for ddynrec. enum is array of name,value, description
        """
        if not enum:
            return ""
        result = [
            self.__ddr.const(
                e[0], "str" if isinstance(e[1], str) else "int", e[1], e[2]
            )
            for e in enum
            if len(e) == 3
        ]
        return self.__ddr.enum(result, "enum")

    @staticmethod
    def __get_type(value):
        type_mapping = {int: "int", float: "double", str: "str", bool: "bool"}
        value_type = type(value)
        if value_type in type_mapping:
            return type_mapping[value_type]
        else:
            raise TypeError(f"Unsupported type for value: {value_type.__name__}")

    @staticmethod
    def __name_from_str(input_str):
        # Replace any non-letter and non-number character with a single underscore
        output_str = re.sub("[^a-zA-Z0-9]+", "_", input_str).lower()[:30]

        return output_str

    @property
    def ddr(self):
        return self.__ddr
