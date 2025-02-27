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

import argparse
import logging
import os
from haipy.s3sync import s3sync
from haipy.utils import clean_dir

logger = logging.getLogger(__name__)

CMS_DOWNLOAD_DIR = os.path.expanduser("~/.cms_content/deploy")
CMS_BUCKET = "s3://dl.cms.hansonrobotics.com"


class ContentManager(object):
    def __init__(self, character, robot, include_test):
        self.character = character
        self.robot = robot
        self.include_test = include_test

    def download(self):
        logger.warning(
            "Downloading content - character %r, robot %r, including test content? %s",
            self.character,
            self.robot,
            "Yes" if self.include_test else "No",
        )
        if not os.path.isdir(CMS_DOWNLOAD_DIR):
            os.makedirs(CMS_DOWNLOAD_DIR)

        # sync character content & robot content
        success = True
        dest = os.path.join(CMS_DOWNLOAD_DIR, f"{self.character}-{self.robot}")
        clean_dir(dest)
        for content_type, name in zip(
            ["characters", "robots"], [self.character, self.robot]
        ):
            if self.include_test:
                s3_source = os.path.join(CMS_BUCKET, "dist", "test", content_type, name)
            else:
                s3_source = os.path.join(CMS_BUCKET, "dist", "prod", content_type, name)
            ret = s3sync(s3_source, dest, delete=False)
            if ret != 0:
                logger.warning("Content synchronization failed")
                success = False


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", default="sophia", help="the robot name")
    parser.add_argument("--character", default="sophia", help="the character name")
    parser.add_argument(
        "--test",
        default=True,
        action="store_true",
        help="whether it should include test",
    )
    args = parser.parse_args()
    ContentManager(args.character, args.robot, args.test).download()
