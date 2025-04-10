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
import re

logger = logging.getLogger(__name__)

TIMELINE_PATTERN = re.compile(r"""\|\s*t\s*,.*\|""", re.IGNORECASE)


class ResponseRanker(object):
    """Rank agent responses based on agent order"""

    def __init__(self, agents):
        self.agents = agents
        self.records = []

    def norm(self, s):
        if s is None:
            return s
        s = re.sub(r"\[.*\]", "", s)  # remote [xxx] mark
        s = " ".join(s.split())  # remove consecutive spaces
        s = s.strip()
        return s

    def add_record(self, record):
        # TODO: session aware
        if record:
            self.records.append(record)

    def check_repeat(self, response):
        # TODO: check repeat for Chinese
        text = response.answer
        text_length = len(text.split())
        if text_length < 5:
            # it's okay to repeat short answer
            logger.info(
                "Repeation pass for short answer from %s (length: %s)",
                response.agent_id,
                text_length,
            )
            return False
        repeat = any(
            [record for record in self.records if self.norm(record) == self.norm(text)]
        )
        if repeat:
            logger.warning("Repeation detected from agent %s", response.agent_id)
        return repeat

    def score(self, agent, response):
        if response is None:
            return -1
        if not response.valid():
            return -1

        if agent.allow_repeat or response.attachment.get("allow_repeat"):
            is_repeat = False
        else:
            is_repeat = self.check_repeat(response)
        response.attachment["repeat"] = is_repeat
        if TIMELINE_PATTERN.search(response.answer):
            score = 100
        else:
            score = response.attachment.get("score") or 50
            if is_repeat:
                score = -1
            logger.debug("agent %s attachment: %r", agent.id, response.attachment)

        return score

    def rank(self, responses):
        """Ranks respnoses by level, and then by score * weight"""
        for response in responses:
            agent = self.agents[response.agent_id]
            score = self.score(agent, response)
            ranking_measures = {}
            ranking_measures["level"] = agent.level
            ranking_measures["weight"] = agent.weight
            ranking_measures["score"] = score
            response.attachment["ranking_measures"] = ranking_measures
            response.confidence = agent.weight * score / 100

        ranked_responses = []
        try:
            ranked_responses = sorted(
                responses,
                key=lambda response: response.confidence,
                reverse=False,
            )
            for response in ranked_responses:
                ranking_measures = response.attachment["ranking_measures"]
                logger.info(
                    "Agent: %r, response: %r, level: %r, weight: %r, score: %r, weighted_score: %r",
                    response.agent_id,
                    response.answer,
                    ranking_measures["level"],
                    ranking_measures["weight"],
                    ranking_measures["score"],
                    ranking_measures["score"] * ranking_measures["weight"],
                )
        except Exception as ex:
            logger.exception(ex)
        return ranked_responses
