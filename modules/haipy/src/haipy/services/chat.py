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

import aiohttp
import requests

from haipy.utils import LANGUAGE_BCP47_CODES

logger = logging.getLogger(__name__)


class ChatClient(object):
    def __init__(self):
        self.url = os.environ.get("CHAT_BASE_URL", "http://localhost:9100")

    def chat(self, user_id, conversation_id, text, lang, app_id, **kwargs):
        lang = LANGUAGE_BCP47_CODES.get(lang, lang)
        context = kwargs.get("context") or {}
        payload = json.dumps(
            {
                "user_id": user_id,
                "sid": conversation_id,
                "text": text,
                "lang": lang,
                "context": context,
                "app_id": app_id,
            }
        )
        headers = {"Accept": "application/json", "Content-Type": "application/json"}
        if "CHAT_TOKEN" in os.environ:
            headers["Authorization"] = "Bearer %s" % os.environ.get("CHAT_TOKEN")
        try:
            resp = requests.post(f"{self.url}/chat", headers=headers, data=payload)
            if resp.ok:
                response = resp.json()
                return response
            else:
                logger.error("Error %s(%r)", resp.status_code, resp.reason)
        except Exception as ex:
            logger.error("Chat client error %s", ex)

    async def async_chat(
        self, user_id, conversation_id, text, lang, client_ip, app_id, **kwargs
    ):
        lang = LANGUAGE_BCP47_CODES.get(lang, lang)
        context = kwargs.get("context") or {}
        if client_ip:
            conn = aiohttp.TCPConnector(local_addr=(client_ip, 0))
        else:
            conn = aiohttp.TCPConnector()

        payload = json.dumps(
            {
                "user_id": user_id,
                "conversation_id": conversation_id,
                "text": text,
                "lang": lang,
                "context": context,
                "app_id": app_id,
            }
        )
        headers = {"Accept": "application/json", "Content-Type": "application/json"}
        if "CHAT_TOKEN" in os.environ:
            headers["Authorization"] = "Bearer %s" % os.environ.get("CHAT_TOKEN")
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{self.url}/chat", headers=headers, data=payload
                ) as resp:
                    if resp.ok:
                        response = await resp.json()
                        return response
                    else:
                        logger.error("Error %s(%r)", resp.status, resp.reason)
        except Exception as ex:
            logger.exception("Chat client error %s", ex)

    async def async_reset(self, user_id):
        payload = json.dumps({"user_id": user_id})
        headers = {"Accept": "application/json", "Content-Type": "application/json"}
        if "CHAT_TOKEN" in os.environ:
            headers["Authorization"] = "Bearer %s" % os.environ.get("CHAT_TOKEN")
        try:
            async with aiohttp.ClientSession() as session:
                async with session.post(
                    f"{self.url}/reset", headers=headers, data=payload
                ) as resp:
                    if resp.ok:
                        response = await resp.json()
                        return response
                    else:
                        logger.error("Error %s(%r)", resp.status, resp.reason)
        except Exception as ex:
            logger.exception("Chat client error %s", ex)


if __name__ == "__main__":
    import asyncio

    asyncio.run(ChatClient().async_chat("default", "hello", "en-US"))
