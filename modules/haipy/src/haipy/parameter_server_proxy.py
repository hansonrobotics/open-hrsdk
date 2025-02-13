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
import uuid
from collections import namedtuple
from datetime import date, datetime
from typing import Callable, Dict, Union

import dateutil.parser
import redis
import yaml
from benedict import benedict
from pydantic import BaseModel

from haipy.utils import ExpiryValue

REDIS_SERVER_HOST = os.environ.get("REDIS_SERVER_HOST", "localhost")
REDIS_SERVER_PORT = os.environ.get("REDIS_SERVER_PORT", "6379")
REDIS_PASSWORD = os.environ.get("REDIS_PASSWORD")

logger = logging.getLogger(__name__)

FIELD_SEP = "."

redis_conn = redis.StrictRedis(
    host=REDIS_SERVER_HOST,
    port=int(REDIS_SERVER_PORT),
    password=REDIS_PASSWORD,
    decode_responses=True,
)

SESSION_PATTERN = re.compile(r"""^[A-Za-z0-9]{20,32}$""")
RESERVED_CONTEXT_KEYS = ["system", "last_active_time", "scene"]
SessionMeta = namedtuple("SessionMeta", ["uid", "sid", "age"])
DEFAULT_SESSION = "default"


class ParameterServerProxy(object):
    def __init__(self, ns="default"):
        """ns: the namespace"""
        self.r = redis_conn
        if ns is None:
            raise ValueError("Namespace missing")
        self.ns = ns

    def ping(self):
        try:
            success = self.r.ping()
            return success
        except redis.exceptions.ConnectionError:
            return False

    def incr(self, key: str, amount: int = 1):
        """
        Increments the value of ``key`` by ``amount``.  If no key exists,
        the value will be initialized as ``amount``
        """
        fullkey = FIELD_SEP.join([self.ns, key])
        return self.r.incr(fullkey, amount)

    def expire(self, key: str, time: int):
        keys = self.list_param(key)
        for key in keys:
            ttl = self.r.ttl(key)
            self.r.expire(key, time)

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

    def push(self, key: str, *items, left=False):
        """Set parameter

        Setting empty value will just delete the key

        If the value is a non-string then serialize it in the form: "type(value)"
        """
        if not key:
            raise ValueError("Invalid key")

        fullkey = FIELD_SEP.join([self.ns, key])
        params = benedict(keypath_separator=FIELD_SEP)
        params[fullkey.split(FIELD_SEP)] = items

        with self.r.pipeline() as pipe:
            for _key in params.keypaths():
                value = params[_key]
                value = self.encode(value)
                if value and len(value) > 0:
                    if left:
                        pipe.lpush(_key, *value)
                    else:
                        pipe.rpush(_key, *value)
            pipe.execute()

    def set_dict(self, data: dict, ttl: int = -1):
        for key, value in data.items():
            self.set_param(key, value, ttl=ttl)

    def set_param(self, key: str, data, append=False, ttl: int = -1):
        """Set parameter

        Setting empty value will just delete the key

        If the value is a non-string then serialize it in the form: "type(value)"
        """
        if not key:
            raise ValueError("Invalid key")

        # remove all the sub-params
        if not append:
            self.delete_param(key)

        fullkey = FIELD_SEP.join([self.ns, key])
        params = benedict(keypath_separator=FIELD_SEP)
        params[fullkey.split(FIELD_SEP)] = data

        with self.r.pipeline() as pipe:
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
            pipe.execute()

    def get_param(self, key: str = "", default=None):
        """get parameter"""
        keys = self.list_param(key)
        if len(keys) == 0:
            return default
        else:
            fullkey = FIELD_SEP.join([self.ns, key])
            params = benedict(keypath_separator=FIELD_SEP)
            for _key in keys:
                _type = self.r.type(_key)
                if _type == "string":
                    value = self.r.get(_key)
                elif _type == "list":
                    value = self.r.lrange(_key, 0, -1)
                elif _type == "hash":
                    value = self.r.hgetall(_key)
                elif _type == "set":
                    value = self.r.smembers(_key)
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
                data = dict(data)  # convert to standard dict, is it needed?
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
            return [self.decode(i) for i in data]
        elif isinstance(data, benedict):
            for _key, value in data.flatten("#").items():
                if isinstance(value, str):
                    data[_key.split("#")] = self.decode(value)
                elif isinstance(value, list):
                    data[_key.split("#")] = [self.decode(i) for i in value]
            return data

    def list_param(self, key: str = ""):
        """list parameter names"""
        fullkey = FIELD_SEP.join([self.ns, key])
        if key:
            this_key = self.r.keys(pattern=f"{fullkey}")
            subkeys = self.r.keys(pattern=f"{fullkey}.*")
            keys = this_key + subkeys
        else:
            keys = self.r.keys(pattern=f"{self.ns}.*")
        return keys

    def delete_param(self, key: str = ""):
        """delete parameter"""
        keys = self.list_param(key)
        if keys:
            self.r.delete(*keys)

    def load_param(self, fname: str):
        """load parameters from file"""
        with open(fname) as f:
            data = yaml.safe_load(f)
            for key in data:
                self.set_param(key, data[key])

    def dump_param(self, fname: str):
        """dump parameters to file"""
        data = self.get_param("")
        with open(fname, "w") as f:
            yaml.dump(data, f, default_flow_style=False, allow_unicode=True)


class GlobalContext(dict):
    def __init__(self, ns="default"):
        self.ns = ns
        self.proxy = ParameterServerProxy(ns)

    def expire(self, key: str, ttl: int):
        """Sets the expiry for the key with TTL (in seconds)"""
        return self.proxy.expire(key, ttl)

    def expireat(self, key: str, timestamp: int):
        """Sets the expiry for the key at a certain absolute UNIX timestamp"""
        return self.proxy.expireat(key, timestamp)

    def sub(self, ns):
        """Returns a global context with sub-namespace"""
        return GlobalContext(FIELD_SEP.join([self.ns, ns]))

    def alive(self):
        return self.proxy.ping()

    def update(self, items):
        if isinstance(items, dict):
            items = items.items()
        for k, v in items:
            self[k] = v

    def get(self, key, default=None):
        try:
            return self.__getitem__(key)
        except KeyError:
            return default

    def __setitem__(self, key, item):
        self.proxy.set_param(key, item)

    def __getitem__(self, key):
        if self.__contains__(key):
            return self.proxy.get_param(key)
        else:
            raise KeyError(key)

    def __len__(self):
        return len(self.keys())

    def __delitem__(self, key):
        return self.proxy.delete_param(key)

    def clear(self):
        return self.proxy.delete_param()

    def keys(self):
        """Returns top level unique keys"""
        fullkeys = self.proxy.list_param()
        keys = [key[len(f"{self.proxy.ns}.") :] for key in fullkeys]
        keys = list(set([key.split(FIELD_SEP)[0] for key in keys]))
        return keys

    def values(self):
        data = self.proxy.get_param() or {}
        return data.values()

    def items(self):
        data = self.proxy.get_param() or {}
        return data.items()

    def __cmp__(self, dict_):
        return self.__cmp__(self.__dict__, dict_)

    def __contains__(self, item):
        fullkeys = self.proxy.list_param()
        keys = [key[len(f"{self.proxy.ns}.") :] for key in fullkeys]
        return item in keys or any([key.startswith(f"{item}.") for key in keys])

    has_key = __contains__

    def __iter__(self):
        data = self.proxy.get_param() or {}
        return iter(data)

    def __str__(self):
        return str(self.items())


class UserSessionContext(GlobalContext):
    LAST_ACTIVE_TIME = "last_active_time"

    def __init__(self, uid, sid=None, session_duration=1200):
        self.uid = uid
        self.sid = sid
        self.session_duration = session_duration
        self.user_context = GlobalContext(self.uid)
        if self.uid == "default" and self.sid is None:
            self.sid = DEFAULT_SESSION  # default session for default user
        else:
            if self.sid is None:
                sessions = self.find_latest_session()
                active_sessions = [
                    session
                    for session in sessions
                    if session.age < self.session_duration
                ]
                staled_sessions = [
                    session
                    for session in sessions
                    if session.age >= self.session_duration
                ]
                for staled_session in staled_sessions:
                    ns = "%s.%s" % (self.uid, staled_session.sid)
                    stale_session_context = GlobalContext(ns)
                    stale_session_context.clear()
                    logger.info(
                        "Removed staled session %s, age %s", ns, staled_session.age
                    )
                if active_sessions:
                    active_session = active_sessions[0]
                    self.sid = active_session.sid
                    logger.warning(
                        "Resume lastest session %s, age %s",
                        active_session.sid,
                        active_session.age,
                    )
                if not self.sid:
                    self.sid = uuid.uuid1().hex
        super(UserSessionContext, self).__init__("%s.%s" % (self.uid, self.sid))
        self[self.LAST_ACTIVE_TIME] = datetime.utcnow()

    def find_latest_session(self):
        """Find all the sessions in the user name space and sort by their age."""
        sessions = []
        now = datetime.utcnow()
        for sid in self.user_context:
            if SESSION_PATTERN.match(sid):
                key = "%s.%s" % (sid, self.LAST_ACTIVE_TIME)
                if key in self.user_context:
                    session_time = self.user_context[key]
                    age = (now - session_time).total_seconds()
                    sessions.append(SessionMeta(self.uid, sid, age))

        return sorted(sessions, key=lambda x: x[2])

    def session_expired(self):
        if self.get(self.LAST_ACTIVE_TIME):
            now = datetime.utcnow()
            session_time = self[self.LAST_ACTIVE_TIME]
            age = (now - session_time).total_seconds()
            return age > self.session_duration
        else:
            return False

    def load_context(self):
        """Loads session context. The session ID must be 20-32 of length."""
        if self.sid is None:
            return
        session_ids = [DEFAULT_SESSION]  # ignore all session namespaces
        for k, v in self.user_context.items():
            if k not in session_ids and SESSION_PATTERN.match(k):
                session_ids.append(k)
            if k not in RESERVED_CONTEXT_KEYS and k not in session_ids:
                self[k] = v

    def save_context(self):
        """Persists session context to user context"""
        if self.sid is None:
            return
        session_ids = [DEFAULT_SESSION]  # ignore all session namespaces
        for k, v in self.items():
            if k not in session_ids and SESSION_PATTERN.match(k):
                session_ids.append(k)
            if k not in RESERVED_CONTEXT_KEYS and k not in session_ids:
                self.user_context[k] = v


class EventListener(object):

    class PMessage(BaseModel):
        type: str
        pattern: str
        channel: str
        data: str

    def __init__(self):
        redis_conn.config_set("notify-keyspace-events", "AKE")
        self.listeners = []
        self.pubsub = redis_conn.pubsub()
        self.pubsub.psubscribe(**{"__keyevent@0__:*": self.event_dispatch})
        self.pubsub.run_in_thread(sleep_time=0.01)

    def on_key_change(self, pattern: str, handler: Callable[[Dict], None]):
        self.listeners.append((pattern, handler))

    def event_dispatch(self, event: Dict):
        event = self.PMessage(**event)
        for pattern, handler in self.listeners:
            if re.match(pattern, event.data):
                handler(event)


if __name__ == "__main__":
    proxy = ParameterServerProxy()
    print(type(proxy.get_param("system.now")))
    context = UserSessionContext("default")
    print(context.session_expired())
    context.load_context()
    print(context)
    context["name"] = "test"
    context.save_context()

    # import time
    # context = GlobalContext()
    # context['a'] = 1
    # context.expire('a', 1)
    # while True:
    #    print(context['a'])
    #    time.sleep(0.3)
    def handler(event):
        print(f"event {event}")

    l = EventListener()
    l.on_key_change("a", handler)
