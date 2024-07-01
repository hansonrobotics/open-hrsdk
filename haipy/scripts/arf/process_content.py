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
import argparse
import logging
import os
import sys

import coloredlogs

from haipy.arf.airtables import ARFContentBase, RobotOperationContentBase

if __name__ == "__main__":
    formatter_str = "%(asctime)s %(levelname)-7s %(name)s: %(message)s"
    coloredlogs.install(logging.INFO, fmt=formatter_str, stream=sys.stdout)

    parser = argparse.ArgumentParser()
    parser.add_argument("-d", "--base-dir", default=".", help="the content home dir")
    parser.add_argument(
        "-t",
        "--base-type",
        required=True,
        choices=["arf", "event"],
        help="the base type",
    )
    parser.add_argument(
        "-n", "--name", required=True, help="the character or robot name"
    )
    parser.add_argument(
        "--test", action="store_true", default=False, help="include test content"
    )
    parser.add_argument("--output-dir", default="output", help="output directory")
    args = parser.parse_args()

    if os.path.isdir(args.base_dir):
        if args.base_type == "arf":
            ARFContentBase(args.base_dir).process_script(args.name, args.test)
        elif args.base_type == "event":
            records = RobotOperationContentBase(args.base_dir).process_script(
                args.name, args.test
            )
