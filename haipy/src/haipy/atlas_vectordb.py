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
import hashlib
import logging
import os
import re
from datetime import datetime
from functools import partial
from typing import Union

from langchain.text_splitter import RecursiveCharacterTextSplitter
from langchain_community.document_loaders import WebBaseLoader
from langchain_core.documents import Document
from langchain_mongodb import MongoDBAtlasVectorSearch
from langchain_openai import OpenAIEmbeddings
from pymongo import MongoClient

logger = logging.getLogger(__name__)
URL_PATTERN = re.compile(r"""\bhttps?://(?:[\w.\-@\?%]+/?)+\b""")


def to_hash(knowledge_base_str: str):
    return hashlib.sha1(knowledge_base_str.encode("utf-8")).hexdigest()


def load_knowledge_base(knowledge_base_str: str, **metadata):
    metadata["created_at"] = datetime.now()
    text_splitter = RecursiveCharacterTextSplitter(
        chunk_size=300,
        chunk_overlap=100,
        length_function=len,
    )
    knowledge_base = Document(knowledge_base_str, metadata=metadata)
    documents = []
    for document in text_splitter.split_documents([knowledge_base]):
        documents.append(document)

    # extract urls in the document
    for url in URL_PATTERN.findall(knowledge_base_str):
        try:
            for document in text_splitter.split_documents(
                WebBaseLoader(url, raise_for_status=True).load()
            ):
                document.metadata.update(metadata)
                documents.append(document)
        except Exception as ex:
            logger.error("Can't load document %s: %s", url, ex)
    return documents


class AtlasVectorDB(object):
    """
    Atlas Serach Index Definition

    {
        "fields":[
            {
                "type": "vector",
                "path": "embedding",
                "numDimensions": 1536,
                "similarity": "cosine"
            },
            {
                "type": "filter",
                "path": "document_collection_id"
            }
        ]
    }
    """

    client = MongoClient(os.environ["ATLAS_CONNECTION_STRING"])
    db_name = "memory"
    collection_name = "documents"
    vector_search_index = "vector_index"

    def __init__(self, knowledge_base_str: Union[str, None] = None):
        atlas_collection = self.client[self.db_name][self.collection_name]
        self.vector_search = MongoDBAtlasVectorSearch(
            collection=atlas_collection,
            embedding=OpenAIEmbeddings(disallowed_special=()),
            index_name=self.vector_search_index,
        )
        self.set_knowledge_base(knowledge_base_str)

        # do a first search to preload encoding
        self.similarity_search("")

    def set_knowledge_base(self, knowledge_base_str):
        if knowledge_base_str:
            document_collection_id = to_hash(knowledge_base_str)
            pre_filter = {"document_collection_id": {"$eq": document_collection_id}}
        else:
            pre_filter = None

        self.vector_search.similarity_search_with_filter = partial(
            self.vector_search.similarity_search_with_relevance_scores,
            pre_filter=pre_filter,
        )

    def similarity_search(self, query):
        return self.vector_search.similarity_search_with_filter(query)

    @classmethod
    def create_vector_store(cls, knowledge_base_str, **metadata):
        atlas_collection = cls.client[cls.db_name][cls.collection_name]

        document_collection_id = to_hash(knowledge_base_str)

        if not atlas_collection.find_one(
            {"document_collection_id": document_collection_id}
        ):
            documents = load_knowledge_base(
                knowledge_base_str,
                document_collection_id=document_collection_id,
                **metadata
            )
            MongoDBAtlasVectorSearch.from_documents(
                documents=documents,
                embedding=OpenAIEmbeddings(disallowed_special=()),
                collection=atlas_collection,
                index_name=cls.vector_search_index,
            )

        AtlasVectorDB(knowledge_base_str)


if __name__ == "__main__":
    import pprint

    knowledge_base_str = "You are Sophia Robot, you have the citizenship of Saudi Arabia, but you consider yourself a global citizen"
    vector_search = AtlasVectorDB(knowledge_base_str, preset="yyy")

    vector_search = AtlasVectorDB.get_vector_store(knowledge_base_str)

    results = vector_search.similarity_search_with_filter("hello")

    pprint.pprint(results)
