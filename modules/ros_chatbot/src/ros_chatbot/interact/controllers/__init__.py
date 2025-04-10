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

from .responsivity_controller import ResponsivityController
from .interruption_controller import InterruptionController
from .interruption_resume_controller import InterruptionResumeController
from .placeholder_utterance_controller import PlaceholderUtteranceController
from .language_switch_controller import LanguageSwitchController
from .command_controller import CommandController
from .emotion_controller import EmotionController
from .monitor_controller import MonitorController
from .user_acquisition_controller import UserAcquisitionController

_controller_classes = [
    ResponsivityController,
    InterruptionController,
    InterruptionResumeController,
    PlaceholderUtteranceController,
    LanguageSwitchController,
    CommandController,
    EmotionController,
    MonitorController,
    UserAcquisitionController,
]

registered_controllers = {cls.type: cls for cls in _controller_classes}
