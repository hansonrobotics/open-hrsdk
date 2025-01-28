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
import logging
import os
import re
from datetime import date, datetime
from typing import Union

import aioredis
import dateutil.parser
import yaml
from benedict import benedict

from haipy.utils import ExpiryValue

REDIS_SERVER_HOST = os.environ.get("REDIS_SERVER_HOST", "localhost")
REDIS_SERVER_PORT = os.environ.get("REDIS_SERVER_PORT", "6379")
REDIS_PASSWORD = os.environ.get("REDIS_PASSWORD")

logger = logging.getLogger(__name__)

FIELD_SEP = "."

redis_conn = aioredis.Redis(
    host=REDIS_SERVER_HOST,
    port=int(REDIS_SERVER_PORT),
    password=REDIS_PASSWORD,
    decode_responses=True,
)


class AsyncParameterServerProxy(object):
    def __init__(self, ns="default"):
        """ns: the namespace"""
        self.r = redis_conn
        if ns is None:
            raise ValueError("Namespace missing")
        self.ns = ns

    async def ping(self):
        try:
            success = await self.r.ping()
            return success
        except aioredis.exceptions.ConnectionError:
            return False

    async def incr(self, key: str, amount: int = 1):
        """
        Increments the value of ``key`` by ``amount``.  If no key exists,
        the value will be initialized as ``amount``
        """
        fullkey = FIELD_SEP.join([self.ns, key])
        return await self.r.incr(fullkey, amount)

    async def expire(self, key: str, time: int):
        keys = await self.list_param(key)
        for key in keys:
            ttl = await self.r.ttl(key)
            if ttl == -1:  # the key has no expiry time
                await self.r.expire(key, time)
            if ttl > 0 and time > ttl:  # extend the ttl
                await self.r.expire(key, time)

    def encode(self, value):
        """encode the data to redis data"""
        if isinstance(value, str):
            return value
        elif isinstance(value, (int, float, bool)):
            return "%s(%s)" % (type(value).__name__, value)
        elif isinstance(value, (datetime, date)):
            return "%s(%s)" % (type(value).__name__, value.isoformat())
        elif isinstance(value, (list, tuple)):  # only store basic types in the list
            return [self.encode(i) for i in value]

    async def push(self, key: str, *items, left=False):
        """Set parameter

        Setting empty value will just delete the key

        If the value is a non-string then serialize it in the form: "type(value)"
        """
        if not key:
            raise ValueError("Invalid key")

        fullkey = FIELD_SEP.join([self.ns, key])
        params = benedict(keypath_separator=FIELD_SEP)
        params[fullkey.split(FIELD_SEP)] = items

        async with self.r.pipeline() as pipe:
            for _key in params.keypaths():
                value = params[_key]
                value = self.encode(value)
                if value and len(value) > 0:
                    if left:
                        pipe.lpush(_key, *value)
                    else:
                        pipe.rpush(_key, *value)
            await pipe.execute()

    async def set_dict(self, data: dict):
        for key, value in data.items():
            await self.set_param(key, value)

    async def set_param(self, key: str, data, append=False, ttl: int = -1):
        """Set parameter

        Setting empty value will just delete the key

        If the value is a non-string then serialize it in the form: "type(value)"
        """
        if not key:
            raise ValueError("Invalid key")

        # remove all the sub-params
        if not append:
            await self.delete_param(key)

        fullkey = FIELD_SEP.join([self.ns, key])
        params = benedict(keypath_separator=FIELD_SEP)
        params[fullkey.split(FIELD_SEP)] = data

        async with self.r.pipeline() as pipe:
            for _key in params.keypaths():
                value = params[_key]
                expiry = None
                if isinstance(value, ExpiryValue):
                    value, expiry = value.value, value.expiry
                value = self.encode(value)
                if isinstance(value, str):
                    if value != "":
                        pipe.set(_key, value)
                        if expiry is not None:
                            pipe.expire(_key, expiry)
                        elif ttl != -1:
                            pipe.expire(_key, ttl)
                elif isinstance(value, list):  # only store basic types in the list
                    if len(value) > 0:
                        pipe.rpush(_key, *value)
                        if ttl != -1:
                            pipe.expire(_key, ttl)
            await pipe.execute()

    async def get_param(self, key: str = "", default=None):
        """get parameter"""
        keys = await self.list_param(key)
        if len(keys) == 0:
            return default
        else:
            fullkey = FIELD_SEP.join([self.ns, key])
            params = benedict(keypath_separator=FIELD_SEP)
            for _key in keys:
                _type = await self.r.type(_key)
                if _type == "string":
                    value = await self.r.get(_key)
                elif _type == "list":
                    value = await self.r.lrange(_key, 0, -1)
                elif _type == "hash":
                    value = await self.r.hgetall(_key)
                elif _type == "set":
                    value = await self.r.smembers(_key)
                elif _type == "none":
                    continue
                else:
                    logger.error("The key %r has unsupported type %r", _key, _type)
                    continue
                if _key == fullkey:
                    value = self.decode(value)
                    return value
                else:
                    trimmed_key = _key[len(fullkey) :]
                    trimmed_key = trimmed_key.lstrip(FIELD_SEP)
                    if not trimmed_key:
                        return value
                    params[trimmed_key] = value

            data = self.decode(params)
            if isinstance(data, benedict):
                data = data.unflatten(FIELD_SEP)
                data = json.loads(data.to_json())  # data = dict(data)
            return data

    def decode(self, data: Union[str, list, benedict]):
        """Format the node values to their original form

        datetime, int, float, int
        """

        def cast_object(value):
            pattern = re.compile(r"(\w+)\((.+)\)")
            matchobj = pattern.match(value)
            if matchobj:
                _type, arg = matchobj.groups()
                _type = _type.strip()
                arg = arg.strip()
                if _type == "datetime":
                    return dateutil.parser.isoparse(arg)
                elif _type == "int":
                    return int(arg)
                elif _type == "float":
                    return float(arg)
                elif _type == "bool":
                    if arg.lower() == "true":
                        return True
                    elif arg.lower() == "false":
                        return False
                    else:
                        raise TypeError("Illegal boolean value %r" % arg)
                else:
                    logger.warning("Unknown object type %r", _type)

        cast_pipe = [cast_object]

        def cast(value):
            for cast_func in cast_pipe:
                _value = cast_func(value)
                if _value is not None:
                    return _value
            return value

        # type casting
        if isinstance(data, str):
            return cast(data)
        elif isinstance(data, list):
            return [cast(i) for i in data]
        elif isinstance(data, benedict):
            for _key in data.keypaths()[:]:
                value = data[_key]
                if isinstance(value, str):
                    data[_key] = cast(value)
                elif isinstance(value, list):
                    value = [cast(i) for i in value]
                    data[_key] = value
            return data

    async def list_param(self, key: str = ""):
        """list parameter names"""
        fullkey = FIELD_SEP.join([self.ns, key])
        if key:
            this_key = await self.r.keys(pattern=f"{fullkey}")
            subkeys = await self.r.keys(pattern=f"{fullkey}.*")
            keys = this_key + subkeys
        else:
            keys = await self.r.keys(pattern=f"{self.ns}.*")
        return keys

    async def delete_param(self, key: str = ""):
        """delete parameter"""
        keys = await self.list_param(key)
        if keys:
            await self.r.delete(*keys)

    async def load_param(self, fname: str):
        """load parameters from file"""
        with open(fname) as f:
            data = yaml.safe_load(f)
            for key in data:
                await self.set_param(key, data[key])

    async def dump_param(self, fname: str):
        """dump parameters to file"""
        data = await self.get_param("")
        with open(fname, "w") as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)


if __name__ == "__main__":
    import asyncio

    proxy = AsyncParameterServerProxy()

    async def test():
        variables = await proxy.list_param()
        print(variables)
        await proxy.set_param("a", 1)
        await proxy.set_param("b", [1, 2, 3])
        value = await proxy.get_param("a")
        value = await proxy.get_param("b")
        print(value)

    asyncio.run(test())
