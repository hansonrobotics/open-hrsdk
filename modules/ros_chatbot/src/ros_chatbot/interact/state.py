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

import copy
import threading
import logging

logger = logging.getLogger(__name__)


class State(object):
    def __init__(self):
        self._state = {}
        self._state_diffs = []
        self.lock = threading.RLock()
        self.updated = threading.Event()

    def reset(self):
        with self.lock:
            self._state = {}
            self._state_diffs = []
            self.updated.set()
        logger.warning("Reset state")

    def update(self, **kwargs):
        with self.lock:
            state_updated = False
            for key, value in kwargs.items():
                if key in self._state:
                    if value != self._state[key]:
                        self._state[key] = value
                        self._state_diffs.append(key)
                        state_updated = True
                else:
                    self._state[key] = value
                    self._state_diffs.append(key)
                    state_updated = True
            if state_updated:
                self.updated.set()

    def wait_for_update(self, timeout=None):
        flag = self.updated.wait(timeout)
        self.updated.clear()
        return flag

    def getState(self):
        return copy.copy(self._state)

    def getStateValue(self, key):
        return copy.copy(self._state.get(key))

    def getStateChange(self):
        return self._state_diffs[:]

    def __getattr__(self, attr):
        """
        Automatically get the attribute value from the internal state.

        e.g. is_robot_speaking will return the value of "robot_speaking"
        in the state.
        """

        def f():
            if attr.startswith("is_"):
                key = attr[3:]
                if key in self._state:
                    value = self._state[key]
                    if isinstance(value, bool):
                        return value
                    else:
                        raise AttributeError("State %r is not boolean" % key)
                else:
                    return False  # if the state is not set return False
            else:
                raise AttributeError("No such attribute %r" % attr)

        return f


if __name__ == "__main__":
    state = State()
    state.update(abc=False, abcd=True)
    print(state.is_abc())
    print(state.is_abcd())
    print(state.is_abcde())
