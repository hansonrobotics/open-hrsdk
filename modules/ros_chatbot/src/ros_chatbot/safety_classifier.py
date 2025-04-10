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

import logging
import os
import re

import yaml

logger = logging.getLogger(__name__)


class TextSafetyClassifier(object):
    """Text based safety classifier"""

    def __init__(self):
        self.inappropriate_filters = {}
        HR_CHATBOT_WORDS_FILTER_FIlE = os.environ.get("HR_CHATBOT_WORDS_FILTER_FIlE")
        if HR_CHATBOT_WORDS_FILTER_FIlE:
            with open(HR_CHATBOT_WORDS_FILTER_FIlE) as f:
                safety_config = yaml.safe_load(f)
                inappropriate_words = safety_config.get("inappropriate_words")
                for lang in inappropriate_words:
                    if lang in ["cmn-Hans-CN", "yue-Hant-HK"]:
                        filter = re.compile(
                            r"(%s)" % "|".join(inappropriate_words[lang]),
                            flags=re.IGNORECASE,
                        )
                    else:
                        filter = re.compile(
                            r"\b(%s)\b" % "|".join(inappropriate_words[lang]),
                            flags=re.IGNORECASE,
                        )
                    self.inappropriate_filters[lang] = filter
                    logger.info("Built %s filter", lang)

    def classify(self, text, lang):
        """Returns True when the text is safe and False otherwise"""
        if lang in self.inappropriate_filters:
            filter = self.inappropriate_filters[lang]
            m = filter.search(text)
            if m:
                logger.warning("unsafe match string %r", m.string)
                return False
        return True
