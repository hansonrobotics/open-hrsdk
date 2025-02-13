#
# Copyright (C) 2017-2025 Hanson Robotics
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
from .basic import (
    ARFBitBTreeGenerator,
    BasicARFGenerator,
    GambitBTreeGenerator,
    GeneralChatGenerator,
    ReflectionGenerator,
    SimpleARFGenerator,
)
from .copyme import CopyMe
from .instructive import InstructiveARFGenerator
from .macro import MacroGenerator
from .master_btree import MasterTreeGenerator
from .prompt import PromptPresetGenerator, PromptTemplateGenerator
from .timeline import TimelineGenerator

_generator_classes = [
    ARFBitBTreeGenerator,
    BasicARFGenerator,
    CopyMe,
    GambitBTreeGenerator,
    GeneralChatGenerator,
    InstructiveARFGenerator,
    MacroGenerator,
    MasterTreeGenerator,
    PromptPresetGenerator,
    PromptTemplateGenerator,
    ReflectionGenerator,
    SimpleARFGenerator,
    TimelineGenerator,
]

registered_generators = {cls.__name__.lower(): cls for cls in _generator_classes}
