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
import logging
import os

import requests

logger = logging.getLogger(__name__)

SENTENCE_EMBEDDING_HOST = os.environ.get("SENTENCE_EMBEDDING_HOST", "127.0.0.1")
SENTENCE_EMBEDDING_PORT = os.environ.get("SENTENCE_EMBEDDING_PORT", 10100)


def get_sentence_embedding(sentences, lang, timeout=1):
    url = f"http://{SENTENCE_EMBEDDING_HOST}:{SENTENCE_EMBEDDING_PORT}/sembedding"

    params = {"articles": [{"text": sentence} for sentence in sentences], "lang": lang}
    try:
        response = requests.post(
            url,
            json=params,
            timeout=timeout,
        )
    except requests.exceptions.ReadTimeout as ex:
        logger.error(ex)
        return

    if response.status_code == 200:
        json = response.json()
        if "embeddings" in json:
            return json["embeddings"]
