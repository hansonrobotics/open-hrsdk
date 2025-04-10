# -*- coding: utf-8 -*-

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
import uuid
from collections import deque

import numpy as np
import pandas as pd
import requests

from ..bm25 import BM25
from .model import Agent, AgentResponse

logger = logging.getLogger(__name__)


class QuickSearchAgent(Agent):
    type = "QuickSearchAgent"

    def __init__(
        self,
        id,
        lang,
        embedding_host,
        embedding_port,
        question_header,
        answer_header,
        episode_header,
        csv_corpus,
    ):
        super(QuickSearchAgent, self).__init__(id, lang)
        if isinstance(csv_corpus, str):
            df = pd.read_csv(csv_corpus)
        elif isinstance(csv_corpus, list):
            dfs = [pd.read_csv(csv_file) for csv_file in csv_corpus]
            df = pd.concat(dfs)
        self.question_corpus = {
            i: row[question_header].strip() for i, row in df.iterrows()
        }
        self.timeout = 2
        self.answer_corpus = {i: row[answer_header].strip() for i, row in df.iterrows()}
        self.episodes = {i: row[episode_header] for i, row in df.iterrows()}
        self.model = BM25({k: v.split() for k, v in self.question_corpus.items()})
        self.embedding_url = f"http://{embedding_host}:{embedding_port}/sembedding/"
        dialog_act_host = os.environ.get("NLU_DIALOGACT_HOST", "127.0.0.1")
        dialog_act_port = os.environ.get("NLU_DIALOGACT_PORT", "8210")
        self.dialog_act_url = f"http://{dialog_act_host}:{dialog_act_port}/batch-da"
        self.responses = deque(maxlen=1)  # the last responses
        self.similarity_threashold = 0.68
        logger.info("Sentence embedding server url %s", self.embedding_url)

    def get_dialog_act(self, sentences, timeout):
        params = {"articles": [{"text": sentence} for sentence in sentences]}
        try:
            response = requests.post(self.dialog_act_url, json=params, timeout=timeout)
        except (
            requests.exceptions.ReadTimeout,
            requests.exceptions.ConnectionError,
        ) as ex:
            logger.error(ex)
            return
        if response.status_code == 200:
            json = response.json()
            if not json["error"] and json["results"]:
                return json["results"]

    def get_sembedding(self, sentences, timeout):
        params = {"articles": [{"text": sentence} for sentence in sentences]}
        try:
            response = requests.post(
                self.embedding_url,
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

    def cos_sim(self, a, b):
        return np.dot(a, b) / (np.linalg.norm(a) * np.linalg.norm(b))

    def run_reflection(self, sid, text, lang):
        self.responses.append(text)

    def chat(self, agent_sid, request):
        response = AgentResponse()
        response.agent_sid = agent_sid
        response.sid = request.sid
        response.request_id = request.request_id
        response.response_id = str(uuid.uuid4())
        response.agent_id = self.id
        response.lang = request.lang
        response.question = request.question

        timeout = request.context.get("timeout") or self.timeout
        service_timeout = max(1, timeout / 2)
        if request.question:  # and len(request.question.split()) >= 3:
            bm25_results = self.model.get_scores(request.question.split())
            logger.info("Found total %s similar sentences", len(bm25_results))
            results = [result for result in bm25_results if result["score"] > 5][
                :10
            ]  # top 10
            sentences = [self.question_corpus[result["id"]] for result in results]
            dialog_acts = self.get_dialog_act(
                sentences + [request.question], service_timeout
            )
            if dialog_acts:
                *candidate_dialog_acts, question_dialog_act = dialog_acts
            else:
                logger.error("Can't get dialog acts")
                response.trace = "Can't get dialog acts"
                return response
            embeddings = self.get_sembedding(
                sentences + [request.question], service_timeout
            )
            if embeddings is None:
                logger.error("Can't get sentence embeddings")
                response.trace = "Can't get sentence embeddings"
                return response
            *candidate_embeddings, question_embedding = embeddings
            cos_sims = [
                self.cos_sim(candidate, question_embedding)
                for candidate in candidate_embeddings
            ]
            cloeset_similarity = 0
            for result, cos_sim, candicate_dialog_act in zip(
                results, cos_sims, candidate_dialog_acts
            ):
                result["similarity"] = cos_sim
                result["sentence"] = self.question_corpus[result["id"]]
                result["dialog_act_match"] = (
                    candicate_dialog_act["name"] == question_dialog_act["name"]
                )
                if cos_sim > cloeset_similarity:
                    cloeset_similarity = cos_sim
            # logger.info("Top 10 searched results %s", results)
            results = [
                result
                for result in results
                if result["similarity"] > self.similarity_threashold
                and result["dialog_act_match"]
            ]
            logger.info(
                "The closest similarity is %s and candidate results %s",
                cloeset_similarity,
                "\n".join([r["sentence"] for r in results]),
            )
            if results:
                choice = None
                # calculate context similarity
                # finds the last questions and compare the similarity
                if self.responses:
                    last_resopnse = self.responses[-1]
                    for result in results:
                        docid = result["id"]
                        last_docid = docid - 1
                        if (
                            last_docid in self.question_corpus
                            and self.episodes[last_docid] == self.episodes[docid]
                        ):
                            last_answer_in_corpus = self.answer_corpus[last_docid]
                            embedding1, embedding2 = self.get_sembedding(
                                [last_answer_in_corpus, last_resopnse], service_timeout
                            )
                            cos_sim = self.cos_sim(embedding1, embedding2)
                            if cos_sim > self.similarity_threashold:
                                logger.info(
                                    "context similarity %s last_resopnse %r corpus %r",
                                    cos_sim,
                                    last_resopnse,
                                    last_answer_in_corpus,
                                )
                                choice = result
                                choice["context_match"] = True
                                break

                if choice is None:
                    similarities = [result["similarity"] for result in results]
                    sim_sum = sum(similarities)
                    probs = [s / sim_sum for s in similarities]
                    logger.info("Probabilities %s", probs)
                    choice = results[
                        np.random.choice(len(results), 1, p=probs).tolist()[0]
                    ]
                docid = choice["id"]
                response.answer = self.answer_corpus[docid]
                if choice.get("context_match"):
                    # boost confidence by 20% if the context matches
                    response.attachment["confidence"] = min(
                        1, choice["similarity"] * 1.2
                    )
                else:
                    response.attachment["confidence"] = choice["similarity"]
                response.attachment[
                    "match_excluded_question"
                ] = self.check_excluded_question(request.question)
                logger.info(
                    "Searched question %r, score %s, answer %r",
                    self.question_corpus[docid],
                    choice["score"],
                    response.answer,
                )
                response.attachment[
                    "risky_named_entity_detected"
                ] = self.check_named_entity(response.answer)
                response.answer = self.post_processing(response.answer)
                self.score(response)
            else:
                logger.info("No results are found")
                # logger.info("Found less similar sentences %s", bm25_results)
                response.trace = "No answer"
        else:
            response.trace = "Can't answer"

        response.end()
        return response

    def score(self, response):
        response.attachment["score"] = 50
        if response.attachment.get("match_excluded_question"):
            response.attachment["blocked"] = True
        if response.attachment["confidence"] > 0.65:
            response.attachment["score"] = 60
        if response.attachment["confidence"] > 0.7:
            response.attachment["score"] = 80
        if response.attachment["confidence"] > 0.75:
            response.attachment["score"] = 85
        if response.attachment["confidence"] > 0.8:
            response.attachment["score"] = 90
        if response.attachment.get("risky_named_entity_detected"):
            response.attachment["score"] = -1
            response.attachment["blocked"] = True
        if (
            "allow_question_response" in self.config
            and not self.config["allow_question_response"]
            and "?" in response.answer
        ):
            response.attachment["score"] = -1
            logger.warning("Question response %s is not allowed", response.answer)

        if response.attachment.get("blocked"):
            response.attachment["score"] = -1
