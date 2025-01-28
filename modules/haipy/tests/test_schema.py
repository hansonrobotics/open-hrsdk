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
import json
import os
import time

import pytest

from haipy.schemas import airtable_schemas

cwd = os.path.dirname(os.path.abspath(__file__))


@pytest.fixture
def event_data_path():
    return os.path.join(cwd, "test_data/event-db")


@pytest.fixture
def arf_data_path():
    return os.path.join(cwd, "test_data/character-db")


class TestSchema:
    def testAirtableSchema(self, arf_data_path, event_data_path):
        with open(os.path.join(arf_data_path, "tables/Scripts.json")) as f:
            x = json.load(f)
            table = airtable_schemas.ScriptTable(records=x)
            assert len(table.records) > 0
            # table2 = airtable_schemas.ScriptTable(records=table.dict()['records'])
            # assert len(table.records) == len(table2.records)
        with open(os.path.join(arf_data_path, "tables/Scenes.json")) as f:
            x = json.load(f)
            table = airtable_schemas.SceneTable(records=x)
            assert len(table.records) > 0
            # table2 = airtable_schemas.SceneTable(records=table.dict()['records'])
            # assert len(table.records) == len(table2.records)
        with open(os.path.join(event_data_path, "tables/Events.json")) as f:
            x = json.load(f)
            table = airtable_schemas.EventTable(records=x)
            assert len(table.records) > 0
            # table2 = airtable_schemas.EventTable(records=table.dict()['records'])
            # assert len(table.records) == len(table2.records)
        with open(os.path.join(event_data_path, "tables/Scripts.json")) as f:
            x = json.load(f)
            table = airtable_schemas.ScriptTable(records=x)
            assert len(table.records) > 0
            # table2 = airtable_schemas.ScriptTable(records=table.dict()['records'])
            # assert len(table.records) == len(table2.records)
