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
import uuid

import pandas as pd
from tqdm import tqdm

from haipy.vectordb import ChatVectorDB

logger = logging.getLogger(__name__)

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)
    df = pd.read_csv("finetuning_train.csv")
    index = ChatVectorDB("sophia")
    index_stats_response = index.index.describe_index_stats()
    print(index_stats_response)
    conversation_id = str(uuid.uuid4())
    pbar = tqdm(total=df.shape[0])
    for i, row in df.iterrows():
        question = row["text"]
        answer = row["labels"]
        episode_done = row["episode_done"]
        logger.info("Question %r, answer %r", question, answer)
        try:
            index.write_document(
                {
                    "question": question,
                    "answer": answer,
                    "label": "GPT3-finetune",
                    "resolver": "auto",
                    "conversation_id": conversation_id,
                },
            )
        except Exception as ex:
            logger.exception("row %s, error %s", i, ex)
        if episode_done is True:
            logger.info("New conversation")
            conversation_id = str(uuid.uuid4())
        pbar.update(1)
    pbar.close()
