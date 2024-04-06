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
import pickle
import os


class Cacheable(object):
    """Cacheable class"""

    def save(self, cache_id, obj):
        """Saves the object with the cache id"""
        raise NotImplementedError

    def load(self, cache_id):
        """Loads the object with the cache id"""
        raise NotImplementedError


class FileCacheable(Cacheable):

    def __init__(self, cache_dir, suffix=""):
        if not os.path.isdir(cache_dir):
            os.makedirs(cache_dir)
        self.cache_dir = cache_dir
        self.suffix = suffix

    def get_cache_filename(self, cache_id):
        cache_file = os.path.join(self.cache_dir, "%s%s" % (cache_id, self.suffix))
        return cache_file

    def save(self, cache_id, obj):
        cache_file = self.get_cache_filename(cache_id)
        with open(cache_file, "wb") as f:
            pickle.dump(obj, f)

    def load(self, cache_id):
        cache_file = self.get_cache_filename(cache_id)
        with open(cache_file, "rb") as f:
            obj = pickle.load(f)
            return obj


if __name__ == "__main__":
    cachable = FileCacheable("/tmp/test", ".timing")
    cachable.save("asdfaf", b"asdfadsf")
    o = cachable.load("asdfaf")
    print(o)
