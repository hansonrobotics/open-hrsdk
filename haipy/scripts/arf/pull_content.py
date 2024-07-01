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
import dotenv

from haipy.arf.airtables import ARFContentBase, RobotOperationContentBase

if __name__ == "__main__":
    formatter_str = "%(asctime)s %(levelname)-7s %(name)s: %(message)s"
    coloredlogs.install(logging.INFO, fmt=formatter_str, stream=sys.stdout)

    parser = argparse.ArgumentParser()
    parser.add_argument("--robot", help="robot name")
    parser.add_argument("--character", help="character name")
    parser.add_argument("--output-dir", default=".", help="output directory")
    args = parser.parse_args()

    if not (args.robot or args.character):
        print(
            "At least one of the arguments are required: --robot or --character",
            file=sys.stderr,
        )

    dotenv.load_dotenv()
    api_key = os.environ["AIRTABLE_API_KEY"]

    if args.robot:
        base_id = os.environ["AIRTABLE_ROBOT_OPERATION_BASE_ID"]
        RobotOperationContentBase(args.output_dir).download(
            base_id, api_key, args.robot
        )
    if args.character:
        base_id = os.environ["AIRTABLE_CHARACTER_DEVELOPMENT_BASE_ID"]
        ARFContentBase(args.output_dir).download(base_id, api_key, args.character)
