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
from typing import List, Type, TypeVar, Union

from bunnet import Document, PydanticObjectId, init_bunnet
from pymongo import MongoClient

from .models.chatlog import ChatRequest, ChatResponse, Scene
from .models.performance import Performance
from .models.person import Person
from .models.prompt_template import PromptTemplate

models = [Performance, Person, PromptTemplate, ChatRequest, ChatResponse, Scene]
database = None

import threading
from concurrent.futures import ThreadPoolExecutor
from queue import Queue

import haipy.memory_manager as mm

T = TypeVar("T", bound=Document)


class DocumentManager(object):
    class BackgroundTask:
        def __init__(self, executor, func, *args, **kwargs):
            self.func = func
            self.args = args
            self.kwargs = kwargs
            self.executor = executor

        def __call__(self):
            future = self.executor.submit(self.func, *self.args, **self.kwargs)
            result = future.result()

    def __init__(self, mongo_uri: str, db_name: str = "memory"):
        init(mongo_uri, db_name)
        self.tasks = Queue()
        threading.Thread(target=self._run, daemon=True).start()
        self.executor = ThreadPoolExecutor(max_workers=4)

    def add_task(self, func, *args, **kwargs):
        self.tasks.put(self.BackgroundTask(self.executor, func, *args, **kwargs))

    def add_document(self, document: Type[T]):
        self.add_task(document.save)

    def _run(self):
        while True:
            task = self.tasks.get()
            task()


def init(mongo_uri: str, db_name: str = "memory"):
    global database
    database = getattr(MongoClient(mongo_uri), db_name)
    init_bunnet(database=database, document_models=models)


def all(model: Type[T]) -> List[T]:
    instances = model.all().to_list()
    return instances


def add(model: Type[T]) -> T:
    instance = model.create()
    return instance


def get(model: Type[T], id: PydanticObjectId) -> T:
    instance = model.get(id)
    if instance:
        return instance


def delete(model: Type[T], id: PydanticObjectId) -> bool:
    instance = model.get(id)
    if instance:
        instance.delete()
        return True
    else:
        return False


def update(model: Type[T], id: PydanticObjectId, data: dict) -> Union[bool, T]:
    des_body = {k: v for k, v in data.items() if v is not None}
    update_query = {"$set": {field: value for field, value in des_body.items()}}
    instance = model.get(id)
    if instance:
        instance.update(update_query)
        return instance
    return False


def copy_collection(cloud_uri: str, local_uri: str, model: Type[T], drop=False):
    """Copy collection data from cloud database to local database"""
    init(cloud_uri)
    instances = all(model)
    init(local_uri)
    if drop:
        model.get_motor_collection().drop()
    for instance in instances:
        instance.save()
    return instances
