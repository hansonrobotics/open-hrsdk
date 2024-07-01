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
import os
import time
from datetime import date, datetime

import pytest

from haipy.parameter_server_proxy import GlobalContext, ParameterServerProxy
from haipy.utils import ExpiryValue


@pytest.fixture
def data():
    return {
        "chat": {
            "name": "john",
            "turn": 1,
            "context": {
                "location": "hk",
                "lang": "en",
            },
        },
        "agent": {
            "name": "aiml",
            "config": {"role": "main", "level": 1, "weight": 0.5},
        },
        "today": datetime.now(),
        "success": True,
        "fail": False,
    }


@pytest.fixture
def namespace():
    return "testcase"


@pytest.fixture
def proxy(namespace):
    proxy = ParameterServerProxy(namespace)
    if not proxy.ping():
        pytest.skip("Redis server is not available")
    yield proxy
    proxy.delete_param()


@pytest.fixture
def context(namespace):
    context = GlobalContext(namespace)
    if not context.alive():
        pytest.skip("Redis server is not available")
    yield context
    context.clear()


class TestParameterServerProxy:
    def testGlobalContext(self, data, context):
        context.clear()
        context["abc"] = [1, 2, 3]
        assert context["abc"] == [1, 2, 3]

        context["string"] = ["1", "2", "3"]
        assert context["string"] == ["1", "2", "3"]

        context["dict"] = {"a": "b"}
        assert context["dict"] == {"a": "b"}
        assert context["dict.a"] == "b"
        with pytest.raises(KeyError):
            context["dict.b"]

        assert set(context.keys()) == set(["string", "abc", "dict"])

        assert len(context) == 3
        del context["dict"]
        assert len(context) == 2
        assert set(context.keys()) == set(["string", "abc"])

        subcontext = context.sub("output")
        subcontext["test"] = 2
        assert context["output.test"] == 2
        assert "test" in subcontext
        assert "test" not in context

        context.clear()
        values = context.values()
        items = context.items()
        assert len(values) == 0
        assert len(items) == 0

        context["abc"] = [1, 2, 3]
        assert context.get("abc") == [1, 2, 3]
        assert context.get("abc2") is None
        assert context.get("abc2", 1) == 1

        context.expire("abc", 1)
        time.sleep(1.5)
        with pytest.raises(KeyError):
            context["abc"]

        context["p.q.1"] = "1"
        context["p.q.2"] = "1"
        assert type(context["p"]) == dict
        assert type(context["p.q"]) == dict

    def testGlobalContextDatetime(self, data, proxy, context):
        proxy.delete_param()
        proxy.set_param("test", data)

        assert isinstance(context["test.today"], datetime)

    def testBooleanValue(self, data, proxy, context):
        proxy.set_param("test", data)
        assert context["test.success"]
        assert not context["test.fail"]

    def testParameterServerProxy(self, data, tmpdir, proxy):
        proxy.delete_param("test")
        proxy.set_param("test", data)
        value = proxy.get_param("test.agent")
        assert value["config"]["level"] == 1
        assert value["name"] == "aiml"

        proxy.set_param("test.agent", {"a": 1, "b": 2})
        value = proxy.get_param("test.agent")
        assert value["a"] == 1
        assert "config" not in value
        assert value["b"] == 2
        assert "name" not in value

        proxy.delete_param("test.agent")
        value = proxy.get_param("test.agent")
        assert value is None

        value = proxy.get_param("test.chat")
        assert value["context"]["location"] == "hk"
        assert value["name"] == "john"

        proxy.set_param("test.names", ["john", "bob"])
        value = proxy.get_param("test.names")
        assert value == ["john", "bob"]

        cwd = os.path.dirname(os.path.abspath(__file__))
        filename = os.path.join(cwd, "test_data", "params.yaml")
        proxy.load_param(filename)
        value = proxy.get_param("test.fruits")
        assert value == ["apple", "banana"]

        ofile = os.path.join(tmpdir, "test.yaml")
        proxy.dump_param(ofile)

    def testEmptyValue(self, proxy):
        proxy.set_param("test.a", "")
        proxy.set_param("test.a.b", 1)

        value = proxy.get_param("test.a")
        assert value == {"b": 1}

    def testPush(self, proxy):
        proxy.delete_param("aa")
        proxy.push("aa", 1)
        proxy.push("aa", 2)
        proxy.push("aa", 3)
        params = proxy.get_param("aa")
        assert params == [1, 2, 3]
        proxy.delete_param("aa")

    def testExpiryValue(self, proxy):
        # set single value
        value = ExpiryValue(1, 1)
        proxy.set_param("expiryvalue", value)
        value = proxy.get_param("expiryvalue")
        assert value == 1
        time.sleep(1.5)
        value = proxy.get_param("expiryvalue")
        assert value is None

        # set boolean value
        value = ExpiryValue(True, 1)
        proxy.set_param("expiryvalue", value)
        value = proxy.get_param("expiryvalue")
        assert value == True
        time.sleep(1.5)
        value = proxy.get_param("expiryvalue")
        assert value is None

        # set datetime value
        now = datetime.now()
        value = ExpiryValue(now, 1)
        proxy.set_param("expiryvalue", value)
        value = proxy.get_param("expiryvalue")
        assert value == now
        time.sleep(1.5)
        value = proxy.get_param("expiryvalue")
        assert value is None

        # set dict
        value = ExpiryValue(1, 1)
        proxy.set_param("dict.expiryvalue", value)
        value = proxy.get_param("dict")
        assert value["expiryvalue"] == 1
        time.sleep(1.5)
        value = proxy.get_param("dict")
        assert value is None

    def testDictParam(self, proxy, data):
        proxy.delete_param()
        proxy.set_param("agent", {"a": 1, "b": 2})
        proxy.set_dict(data)

        assert proxy.get_param("agent.name") == "aiml"
