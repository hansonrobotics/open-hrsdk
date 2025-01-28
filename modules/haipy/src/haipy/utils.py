# -*- coding: utf-8 -*-
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
import re
import shutil
from collections import OrderedDict, defaultdict, deque, namedtuple
from collections.abc import Iterable
from pathlib import Path

import yaml
from yaml import SafeDumper as OrderedDumper
from yaml.representer import SafeRepresenter

logger = logging.getLogger(__name__)

ExpiryValue = namedtuple("ExpiryValue", ["value", "expiry"])


def dict_representer(dumper, data):
    return dumper.represent_dict(iter(data.items()))


OrderedDumper.add_representer(OrderedDict, dict_representer)
OrderedDumper.add_representer(defaultdict, dict_representer)
OrderedDumper.add_representer(deque, SafeRepresenter.represent_list)
OrderedDumper.add_representer(str, SafeRepresenter.represent_str)

Language = namedtuple("Language", ["name", "bcp47_code", "iso639_code"])

LANGUAGES = [
    Language(name="Arabic", bcp47_code="ar-SA", iso639_code="ar"),
    Language(name="Chinese", bcp47_code="cmn-Hans-CN", iso639_code=""),
    Language(name="Czech", bcp47_code="cs-CZ", iso639_code=""),
    Language(name="German", bcp47_code="de-DE", iso639_code=""),
    Language(name="Greek", bcp47_code="el-GR", iso639_code=""),
    Language(name="British English", bcp47_code="en-GB", iso639_code=""),
    Language(name="American English", bcp47_code="en-US", iso639_code=""),
    Language(name="Spanish", bcp47_code="es-ES", iso639_code=""),
]

LANGUAGE_CODES_NAMES = {
    "am-ET": "Amharic",
    "ar-SA": "Arabic",
    "bg-BG": "Bulgarian",
    "cmn-Hans-CN": "Chinese",
    "cs-CZ": "Czech",
    "de-DE": "German",
    "el-GR": "Greek",
    "en-GB": "English",
    "en-US": "English",
    "es-ES": "Spanish",
    "fr-FR": "French",
    "hi-IN": "Hindi",
    "hu-HU": "Hungarian",
    "it-IT": "Italian",
    "ja-JP": "Japanese",
    "ko-KR": "Korean",
    "nl-NL": "Dutch",
    "no-NO": "Norwegian",  # no or nb?
    "nb-NO": "Norwegian",
    "pl-PL": "Polish",
    "pt-BR": "Portuguese",
    "ru-RU": "Russian",
    "tr-TR": "Turkish",
    "yue-Hant-HK": "Cantonese",
}

# https://docs.microsoft.com/azure/cognitive-services/speech-service/language-support#text-to-speech
LANGUAGE_BCP47_CODES = {
    "Amharic": "am-ET",
    "Arabic": "ar-SA",
    "Cantonese": "yue-Hant-HK",
    "Chinese": "cmn-Hans-CN",
    "Czech": "cs-CZ",
    "Dutch": "nl-NL",
    "English": "en-US",
    "French": "fr-FR",
    "German": "de-DE",
    "Greek": "el-GR",
    "Hindi": "hi-IN",
    "Hungarian": "hu-HU",
    "Italian": "it-IT",
    "Japanese": "ja-JP",
    "Korean": "ko-KR",
    "Mandarin Chinese": "cmn-Hans-CN",
    "Mandarin": "cmn-Hans-CN",
    "Norwegian": "no-NO",
    "Polish": "pl-PL",
    "Portuguese": "pt-BR",
    "Russian": "ru-RU",
    "Spanish": "es-ES",
    "Turkish": "tr-TR",
    "am": "am-ET",
    "ar": "ar-SA",
    "cs": "cs-CZ",
    "de": "de-DE",
    "el": "el-GR",
    "en": "en-US",
    "es": "es-ES",
    "fr": "fr-FR",
    "hi": "hi-IN",
    "hk": "yue-Hant-HK",
    "hu": "hu-HU",
    "it": "it-IT",
    "ja": "ja-JP",
    "ko": "ko-KR",
    "nl": "nl-NL",
    "no": "no-NO",
    "pl": "pl-PL",
    "pt": "pt-BR",
    "ru": "ru-RU",
    "tr": "tr-TR",
    "zh": "cmn-Hans-CN",
}

# https://cloud.google.com/translate/docs/languages
LANGUAGE_ISO639_CODES = {
    "Amharic": "am",
    "Arabic": "ar",
    "Cantonese": "hk",
    "Chinese": "zh",
    "Czech": "cs",
    "Dutch": "nl",
    "English": "en",
    "French": "fr",
    "German": "de",
    "Greek": "el",
    "Hindi": "hi",
    "Hungarian": "hu",
    "Italian": "it",
    "Japanese": "ja",
    "Korean": "ko",
    "Mandarin Chinese": "zh",
    "Mandarin": "zh",
    "Norwegian": "no",
    "Polish": "pl",
    "Portuguese": "pt",
    "Russian": "ru",
    "Spanish": "es",
    "Turkish": "tr",
    "am-ET": "am",
    "ar-SA": "ar",
    "cmn-Hans-CN": "zh",
    "cs-CZ": "cs",
    "de-DE": "de",
    "el-GR": "el",
    "en-US": "en",
    "es-ES": "es",
    "fr-FR": "fr",
    "hi-IN": "hi",
    "hu-HU": "hu",
    "it-IT": "it",
    "ja-JP": "ja",
    "ko-KR": "ko",
    "nl-NL": "nl",
    "no-NO": "no",
    "pl-PL": "pl",
    "pt-BR": "pt",
    "ru-RU": "ru",
    "tr-TR": "tr",
    "yue-Hant-HK": "hk",
}


def abs_path(root_dir, p):
    """Returns absolute path"""
    if p.startswith("/"):
        return p
    if p.startswith("~"):
        return os.path.expanduser(p)
    return os.path.join(root_dir, p)


def envvar_yaml_loader():
    envvar_matcher = re.compile(r"\${([^}^{]+)\}")

    def env_var_single_replace(match):
        return (
            os.environ[match.group(1)]
            if match.group(1) in os.environ
            else match.group()
        )

    def constructor(loader, node):
        value = loader.construct_scalar(node)
        newvalue = re.sub(envvar_matcher, env_var_single_replace, value)
        if value == newvalue:
            raise ValueError('The environment variable was not set: "%s"' % value)
        return newvalue

    yaml.SafeLoader.add_implicit_resolver("!envvar", envvar_matcher, None)
    yaml.SafeLoader.add_constructor("!envvar", constructor)


def iterable(obj):
    return isinstance(obj, Iterable) and not isinstance(obj, str)


def to_list(obj):
    if obj is None:
        return []
    if iterable(obj):
        return obj
    else:
        return [obj]


def reduce(array):
    if len(array) == 1:
        return array[0]
    else:
        return array


def dump_yaml(data, fname):
    with open(fname, "w") as f:
        yaml.dump(
            data,
            f,
            Dumper=OrderedDumper,
            default_flow_style=False,
            allow_unicode=True,
            sort_keys=False,
        )


def mkdir(dir):
    if not os.path.isdir(dir):
        os.makedirs(dir)


def create_soultalk_package_meta(version=1.0):
    return {
        "description": "Soultalk package",
        "version": version,
        "files": {"topic": ["topics/*.yaml"]},
    }


def to_number(i):
    if isinstance(i, int) or isinstance(i, float):
        return i
    try:
        if "." in i:
            return float(i)
        else:
            return int(i)
    except (TypeError, ValueError):
        return float(i)


def parse_yaml_str(text: str):
    try:
        params = yaml.safe_load(text)
        if isinstance(params, dict):
            return params
        else:
            return {"__root__": params}
    except Exception as ex:
        logger.error(ex)
        return {}


def to_bool(value):
    if isinstance(value, str):
        if value.lower() in ["t", "true", "yes"]:
            return True
        elif value.lower() in ["f", "false", "no"]:
            return False
        else:
            return False
    else:
        return bool(value)


def clean_dir(dirname):
    for path in Path(dirname).glob("**/*"):
        if path.is_file():
            path.unlink()
        elif path.is_dir():
            shutil.rmtree(path)
