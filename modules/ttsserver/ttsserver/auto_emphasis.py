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

import numpy as np
import requests

NLU_SERVER_URL = "http://localhost:8102/v1.0/nlu/da"
logger = logging.getLogger("hr.ttsserver.auto_emphasis")
disabled = True


def auto_emphasis(text):
    """Automatically emphasize the text"""
    global disabled

    if disabled:
        return text
    params = {"text": text, "language": "en-US"}

    try:
        logger.info("Auto emphasis: %s", text)
        response = requests.get(NLU_SERVER_URL, params=params)
    except requests.adapters.ConnectionError:
        disabled = True
        logger.warn("Automatic emphasis is diabled due to connection error")
        return text

    def emphasize(token):
        token = token.lower()
        for tok in tokens_to_emphasize:
            if token.startswith(tok):
                return "*{}*".format(token)
        for tok in tokens_to_strongly_emphasize:
            if token.startswith(tok):
                return "**{}**".format(token)
        return token

    if response.status_code == 200:
        result = response.json()
        logger.info("NLU result %s", result)
        tokens = result["tokens"]
        if len(tokens) < 5:
            return text
        whitespace_tokens = text.split()
        tokens_to_strongly_emphasize = []
        tokens_to_emphasize = []
        for token in tokens:
            if token["attention"] > 0.9:  # strong emphasis threashold
                tokens_to_strongly_emphasize.append(token["token"])
            elif token["attention"] > 0.5:  # emphasis threashold
                tokens_to_emphasize.append(token["token"])
        text = " ".join([emphasize(token) for token in whitespace_tokens])
    else:
        logger.error("Get NLU result error")
    return text


if __name__ == "__main__":
    print(auto_emphasis("Good afternoon"))
