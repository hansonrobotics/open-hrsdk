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

import six
from google.cloud import translate

logger = logging.getLogger("hr.asr.translator")


class Translator(object):
    def __init__(self):
        self.client = translate.Client()

    def translate(self, text, target_language, source_language):
        """Translate foreign text English"""

        try:
            if isinstance(text, six.binary_type):
                text = text.decode("utf-8")

            result = self.client.translate(text)
            translated_text = result["translatedText"]
            logger.info("translation: %s", translated_text)
            return translated_text
        except Exception as ex:
            logger.exception(ex)

        return ""
