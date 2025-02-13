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
from setuptools import setup

setup(
    name="ttsserver",
    version="0.6.0",
    packages=["ttsserver"],
    description=("Hanson Robotics TTS Server"),
    url="https://github.com/hansonrobotics/open-hrsdk/ttsserver",
    author="Wenwei Huang",
    author_email="wenwei@hansonrobotics.com",
    entry_points={"console_scripts": ["run_tts_server=ttsserver.server:main"]},
)
