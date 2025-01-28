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
import logging
import os
import subprocess

logger = logging.getLogger(__name__)

AWS_DEFAULT_REGION = os.environ.get("AWS_DEFAULT_REGION", "ap-east-1")


def s3sync(source, dest, delete=True):
    command = [
        "aws",
        "s3",
        "sync",
        f"--region={AWS_DEFAULT_REGION}",
        "--delete",
        "--no-progress",
        source,
        dest,
    ]
    if not delete:
        command.remove("--delete")
    process = subprocess.Popen(command, stdout=subprocess.PIPE)

    output, error = process.communicate()
    if output:
        logger.info(output)
    if error:
        logger.error(error)
    return process.poll()
