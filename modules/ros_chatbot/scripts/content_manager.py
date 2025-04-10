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

import logging
import os
import threading
import time

import requests
import rospy
from dynamic_reconfigure.server import Server
from haipy.arf.airtables import OperationSceneTable, RobotOperationContentBase
from haipy.memory_manager import PromptTemplate, copy_collection
from haipy.s3sync import s3sync
from haipy.utils import clean_dir, dump_yaml
from std_msgs.msg import String
from std_srvs.srv import Trigger

from ros_chatbot.cfg import ContentManagerConfig

logger = logging.getLogger("hr.ros_chatbot.content_manager")

HR_CHARACTER = os.environ["HR_CHARACTER"]
ROBOT_NAME = os.environ["ROBOT_NAME"]
CMS_BUCKET = os.environ.get("CMS_BUCKET", "s3://dl.cms.hansonrobotics.com")
CMS_DIR = os.environ["CMS_DIR"]  # eg. /hr/.hrsdk/cms_content/sophia-sophia
SUCCESS_FILE = os.path.join(CMS_DIR, ".success")


class ContentManager(object):
    def __init__(self):
        self.include_test = None
        self.content_update_pub = rospy.Publisher(
            "~update_event", String, queue_size=10, latch=True
        )

        self.cfg = None
        self._lock = threading.RLock()
        self._pulling = threading.Event()

        RobotOperationContentBase("/tmp").monitor_scenes_update(
            HR_CHARACTER, self._pull_operation_scenes
        )

        threading.Thread(target=self._automatic_pull, daemon=True).start()

    def _pull_operation_scenes(self, scenes_table: OperationSceneTable):
        scenes = []
        for record in scenes_table.records:
            logger.warning("Scene Name: %s", record.fields.Name)
            scenes.append(record.model_dump())
        operation_scenes_file = os.path.join(CMS_DIR, "airtable-operation-scenes.yaml")
        dump_yaml(scenes, operation_scenes_file)
        logger.warning("Dumped operation scenes to %s", operation_scenes_file)
        self.content_update_pub.publish("updated")

    def _automatic_pull(self):
        while True:
            if not os.path.isfile(SUCCESS_FILE) and not self._pulling.is_set():
                # pull content if it is empty
                logger.warning("Automatic content pull")
                self._pull()

            if self.cfg is not None and self.cfg.automatic_pull:
                self._pull()
                time.sleep(self.cfg.pull_interval * 60)
            time.sleep(1)

    def _pull_helper(self):
        logger.warning(
            "Updating content, including test content? %s",
            "Yes" if self.include_test else "No",
        )
        if not os.path.isdir(CMS_DIR):
            os.makedirs(CMS_DIR)

        # sync character content & robot content
        success = True
        clean_dir(CMS_DIR, keep_file=["airtable-operation-scenes.yaml"])
        for content_type, name in zip(
            ["characters", "robots"], [HR_CHARACTER, ROBOT_NAME]
        ):
            if self.include_test:
                s3_source = os.path.join(CMS_BUCKET, "dist", "test", content_type, name)
            else:
                s3_source = os.path.join(CMS_BUCKET, "dist", "prod", content_type, name)
            ret = s3sync(s3_source, CMS_DIR, delete=False)
            if ret != 0:
                logger.warning("Content synchronization failed")
                success = False

        # copy prompt templates collection
        copy_collection(
            os.environ["CLOUD_MONGO_DATABASE_URL"],
            os.environ["LOCAL_MONGO_DATABASE_URL"],
            model=PromptTemplate,
            drop=True,
        )
        logger.info("Copied prompt templates")

        if success:
            with open(SUCCESS_FILE, "w") as f:
                f.write(str(time.time()))
            self._reload()
            self.content_update_pub.publish("updated")

    def _pull(self):
        with self._lock:
            self._pulling.set()
            try:
                self._pull_helper()
            finally:
                self._pulling.clear()

    def _reload(self):
        self.content_update_pub.publish("reloading")
        try:
            # self._reload_behavior_tree()
            self._reload_soultalk()
        except Exception as ex:
            logger.error("Error during reloading: %s", ex)
        else:
            logger.info("Reloading content finished successfully")
        finally:
            self.content_update_pub.publish("reloaded")

    def _reload_behavior_tree(self):
        try:
            reload_trees = rospy.ServiceProxy(
                "/hr/behavior/interactive_fiction/reload_trees", Trigger
            )
            reload_trees.wait_for_service(timeout=2)
            reload_trees()
            logger.warning("Behavior tree has been reloaded")
        except Exception as ex:
            logger.error("Error reloading behavior tree: %s", ex)
            raise

    def _reload_soultalk(self):
        try:
            soultalk_url = os.environ.get("SOULTALK_SERVER_HOST", "127.0.0.1")
            soultalk_port = os.environ.get("SOULTALK_SERVER_PORT", "8801")
            response = requests.post(
                f"http://{soultalk_url}:{soultalk_port}/reset",
                json={"uid": "default", "sid": "default", "reload": True},
            )
            logger.info("Reset soultalk %s", response)
        except Exception as ex:
            logger.error("Error reloading soultalk: %s", ex)
            raise

    def reconfig(self, config, level):
        self.cfg = config
        try:
            if self.include_test is None or self.include_test != config.test:
                self.include_test = config.test
                self._pull()
            elif config.pull:
                self._pull()
        except Exception as ex:
            logger.error("Error during reconfiguration: %s", ex)
        finally:
            config.pull = False
            self.include_test = config.test
        return config


if __name__ == "__main__":
    rospy.init_node("content_manager")
    node = ContentManager()
    Server(ContentManagerConfig, node.reconfig)
    while not rospy.is_shutdown():
        rospy.spin()
