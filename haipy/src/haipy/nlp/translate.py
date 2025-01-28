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
import hashlib
import logging
import os
import random

import requests
from google.cloud import translate as google_translate

logger = logging.getLogger(__name__)

cache = {}

# https://cloud.google.com/translate/docs/languages
# https://learn.microsoft.com/en-us/azure/cognitive-services/speech-service/language-support?tabs=stt-tts#text-to-speech
LANGUAGE_CODE_MAPPING = {
    "ar-SA": "ar",
    "cmn-Hans-CN": "zh",
    "cs-CZ": "cs",
    "de-DE": "de",
    "en-US": "en",
    "es-ES": "es",
    "fr-FR": "fr",
    "hi-IN": "hi",
    "hu-HU": "hu",
    "it-IT": "it",
    "ja-JP": "ja",
    "ko-KR": "ko",
    "nb-NO": "nb",
    "pl-PL": "pl",
    "ru-RU": "ru",
    "yue-Hant-HK": "hk",
}


class CantoneseTranslateClient(object):
    """https://fanyi-api.baidu.com/api/trans/product/apidoc"""

    def __init__(self):
        self.url = "https://fanyi-api.baidu.com/api/trans/vip/translate"
        self.appid = os.environ.get("BAIDU_TRANSLATE_APPID")
        self.secretKey = os.environ.get("BAIDU_TRANSLATE_SECRETKEY")

    def translate(self, text, source_language, target_language):
        if self.appid is None:
            logger.error("BAIDU_TRANSLATE_APPID is missing")
            return
        if self.secretKey is None:
            logger.error("BAIDU_TRANSLATE_SECRETKEY is missing")
            return
        salt = random.randint(32768, 65536)
        sign = self.appid + text + str(salt) + self.secretKey
        sign = hashlib.md5(sign.encode()).hexdigest()

        if target_language == "zh-TW":
            target_language = "yue"
        if target_language == "zh-CN":
            target_language = "yue"
        if target_language == "hk":
            target_language = "yue"

        params = {
            "appid": self.appid,
            "q": text,
            "from": "auto",
            "to": target_language,
            "salt": str(salt),
            "sign": sign,
        }
        response = requests.get(self.url, params)
        ret = {}
        if response.status_code == 200:
            result = response.json()
            if "error_msg" in result:
                logger.error(result)
                return
            elif "trans_result" in result:
                ret["translatedText"] = result["trans_result"][0]["dst"]
                return ret


class TranslateClient(object):
    """
    For supported languages see https://cloud.google.com/translate/docs/languages
    """

    def __init__(self):
        self.client = google_translate.Client()
        self.cantonese_client = CantoneseTranslateClient()

    def text2cachekey(self, text, language):
        return "%s@%s" % (text.lower().strip(), language)

    def translate(self, text, source_language, target_language):
        key = self.text2cachekey(text, target_language)
        if key in cache:
            logger.info("Using cache")
            return cache[key]
        ret = {}

        source_language = LANGUAGE_CODE_MAPPING.get(source_language, source_language)
        target_language = LANGUAGE_CODE_MAPPING.get(target_language, target_language)

        if source_language == target_language:
            ret["text"] = text
            ret["translated"] = True
            return ret

        logger.warning(
            "Translating %r from %r to %r", text, source_language, target_language
        )
        if target_language == "hk":
            result = self.cantonese_client.translate(
                text, source_language=source_language, target_language=target_language
            )
        else:
            result = self.client.translate(
                text, source_language=source_language, target_language=target_language
            )

        if result:
            ret["text"] = result["translatedText"]
            ret["translated"] = True
            logger.warning("Translate result %r", result["translatedText"])

            cache[key] = ret
            return ret


if __name__ == "__main__":
    translate_client = TranslateClient()
    result = translate_client.translate("what is your name", "en", "zh")
    result = translate_client.translate("what is your name", "en", "hk")
    print(result)
