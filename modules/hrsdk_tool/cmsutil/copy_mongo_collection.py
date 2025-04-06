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

import os
import haipy.memory_manager as mm
import logging

logger = logging.getLogger(__name__)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    cloud_uri = os.environ.get('MONGO_DATABASE_URL')
    local_uri = os.environ.get('LOCAL_MONGO_DATABASE_URL')
    templates = mm.copy_collection(cloud_uri, local_uri, mm.PromptTemplate)
    print(templates)
    persons = mm.copy_collection(cloud_uri, local_uri, mm.Person)
    print(persons)
    performances = mm.copy_collection(cloud_uri, local_uri, mm.Performance)
    print(performances)
