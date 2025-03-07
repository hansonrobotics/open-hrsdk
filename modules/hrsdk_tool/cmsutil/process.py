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

import glob
import logging
import os
import shutil
from enum import Enum

import coloredlogs
from haipy.arf.airtables import ARFContentBase, RobotOperationContentBase
from haipy.utils import clean_dir
from haipy.parameter_server_proxy import ParameterServerProxy

coloredlogs.install(level="WARN")

logger = logging.getLogger(__name__)


class ContentType(str, Enum):
    robot = "robot"
    character = "character"
    character_robot = "character_robot"


class ContentManager(object):
    def __init__(self, content_type: ContentType):
        self.content_type = content_type

    def pull(self):
        if self.content_type in ("robot", "robot+character"):
            logger.warning("Pulling robot content")
            RobotOperationContentBase(ARF_DOWNLOAD_DIR).download(
                ROBOT_OPERATION_BASE_ID, API_KEY, ROBOT_NAME
            )
        if self.content_type in ("character", "robot+character"):
            logger.warning("Pulling character content")
            ARFContentBase(ARF_DOWNLOAD_DIR).download(
                CHARACTER_DEVELOPMENT_BASE_ID, API_KEY, HR_CHARACTER
            )
        logger.info("Pulling content finished")

    def build(self, include_test):
        clean_dir(PROCESSED_DIR)

        if self.content_type in ("robot", "robot+character"):
            logger.warning("Building robot content")
            RobotOperationContentBase(ARF_DOWNLOAD_DIR).process_script(
                ROBOT_NAME, include_test=include_test, output_dir=PROCESSED_DIR
            )
        if self.content_type in ("character", "robot+character"):
            logger.warning("Building character content")
            ARFContentBase(ARF_DOWNLOAD_DIR).process_script(
                HR_CHARACTER, include_test=include_test, output_dir=PROCESSED_DIR
            )
        logger.info("Building content finished")

    def deploy(self):
        """Copy the processed files to destination directories"""
        logger.warning("Deploying content")

        BEHAVIOR_PROJECT_DIR = os.environ["BEHAVIOR_PROJECT_DIR"]
        ADHOC_BEHAVIOR_DIR = os.path.join(BEHAVIOR_PROJECT_DIR, "adhoc")
        PERFORMANCES_DIR = os.environ["PERFORMANCES_DIR"]
        SOULTALK_HOT_UPLOAD_DIR = os.environ["SOULTALK_HOT_UPLOAD_DIR"]

        # behaviors
        behavior_tree_dir = os.path.join(PROCESSED_DIR, "behavior_trees")
        if os.path.isdir(behavior_tree_dir) and os.path.isdir(ADHOC_BEHAVIOR_DIR):
            try:
                shutil.copytree(
                    behavior_tree_dir, ADHOC_BEHAVIOR_DIR, dirs_exist_ok=True
                )
            except Exception as ex:
                logger.error(ex)

        # timelines
        timeline_dir = os.path.join(PROCESSED_DIR, "timelines")
        if os.path.isdir(timeline_dir) and os.path.isdir(PERFORMANCES_DIR):
            try:
                shutil.copytree(timeline_dir, PERFORMANCES_DIR, dirs_exist_ok=True)
            except Exception as ex:
                logger.error(ex)

        # soultalk
        soultalk_dir = os.path.join(PROCESSED_DIR, "soultalk")
        if os.path.isdir(soultalk_dir) and os.path.isdir(SOULTALK_HOT_UPLOAD_DIR):
            try:
                clean_dir(SOULTALK_HOT_UPLOAD_DIR)
                shutil.copytree(
                    soultalk_dir, SOULTALK_HOT_UPLOAD_DIR, dirs_exist_ok=True
                )
            except Exception as ex:
                logger.error(ex)


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser()
    parser.add_argument("-p", "--pull", action="store_true", help="pull ARF content")
    parser.add_argument("-b", "--build", action="store_true", help="build ARF content")
    parser.add_argument(
        "--content-type",
        choices=ContentType._member_names_,
        required=True,
        default="character_robot",
        help="build ARF content for chracter",
    )
    parser.add_argument(
        "-d", "--deploy", action="store_true", help="deploy ARF content"
    )
    parser.add_argument(
        "--test", action="store_true", default=False, help="Inlucde test content"
    )
    args = parser.parse_args()

    HR_CHARACTER = os.environ.get("HR_CHARACTER")
    ROBOT_NAME = os.environ.get("ROBOT_NAME")
    API_KEY = os.environ["AIRTABLE_API_KEY"]
    ARF_DOWNLOAD_DIR = os.environ["ARF_DOWNLOAD_DIR"]
    PROCESSED_DIR = os.environ["PROCESSED_DIR"]
    ROBOT_OPERATION_BASE_ID = os.environ["AIRTABLE_ROBOT_OPERATION_BASE_ID"]
    CHARACTER_DEVELOPMENT_BASE_ID = os.environ["AIRTABLE_CHARACTER_DEVELOPMENT_BASE_ID"]

    manager = ContentManager(args.content_type)
    if args.pull:
        manager.pull()
    if args.build:
        manager.build(args.test)
    if args.deploy:
        manager.deploy()
