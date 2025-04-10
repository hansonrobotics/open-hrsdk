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
from typing import List, Optional
import random

from .agents.model import AgentResponse
from .utils import softmax

logger = logging.getLogger(__name__)



class AbstractResolver(object):

    def __init__(self, ranker):
        self.ranker = ranker


class SimpleResponseResolver(AbstractResolver):
    """Pick the first response by ranking score"""

    type = "SimpleResponseResolver"

    def resolve(self, responses: List[AgentResponse]) -> Optional[AgentResponse]:
        responses = self.ranker.rank(responses)
        logger.info("Ranked responses %s", responses)
        responses = [
            response
            for response in reversed(responses)
            if not (
                response.attachment.get("score", 50) == -1
                or response.attachment.get("repeat", False)
            )
        ]
        if responses:
            # if responses[0].attachment.get("non-verbal"):
            #    # if it is non-verbal answer, merge it with the next response
            #    if len(responses) > 1:
            #        logger.warning("Merged non-verbal answer with the next response")
            #        responses[1].answer = (
            #            responses[0].answer + " " + responses[1].answer
            #        )
            #        return responses[1]
            return responses[0]
        logger.error("No response")


class PreferentialResponseResolver(AbstractResolver):
    """Preferential response resolver that chooses responses based on preference scores."""

    type = "PreferentialResponseResolver"

    def resolve(self, responses: List[AgentResponse]) -> Optional['AgentResponse']:
        responses = self.ranker.rank(responses)
        logger.info("Ranked responses: %s", responses)

        filtered_responses = [
            response
            for response in reversed(responses)
            if not (
                response.attachment.get("score", 50) == -1
                or response.attachment.get("repeat", False)
            )
        ]

        if not filtered_responses:
            logger.error("No valid response found")
            return None

        top_preference = filtered_responses[0].preference

        # If top response preference is below 0, return it immediately, for backward compatibility
        if top_preference < 0:
            return filtered_responses[0]

        high_preference = [r for r in filtered_responses if r.preference > 10]
        mid_preference = [r for r in filtered_responses if 1 <= r.preference <= 10]
        low_preference = [r for r in filtered_responses if r.preference == 0]


        # If there is one or more responses with preference > 10, choose randomly from them
        if high_preference:
            chosen_response = random.choice(high_preference)
            return chosen_response

        # For preferences between 1-10, choose randomly based on weighted probability
        if mid_preference:
            chosen = random.choices(mid_preference, weights=[r.preference for r in mid_preference], k=1)
            return chosen[0]

        # If the max preference is 0, return that response
        if low_preference:
            chosen_response = random.choice(low_preference)
            return chosen_response

        logger.error("No suitable response found")
        return None
    
    @staticmethod
    def preferences(agents):
        # Returns -1 if there is a preference of 11 otherwise return sum of preferences for all called agents
        prefs = [a.config.get("preference", -1) for a in agents]
        if max(prefs) > 10:
            return -1 # Wait for 11
        return sum([p for p in prefs if p > 0])
    

class ProbabilisticResponseResolver(AbstractResolver):

    """Pick the response by probability"""

    type = "ProbabilisticResponseResolver"

    def get_max_level_responses(self, responses):
        try:
            max_level = max(
                [
                    response.attachment["ranking_measures"]["level"]
                    for response in responses
                ]
            )
            logger.info("Max level %r", max_level)
            responses = [
                response
                for response in responses
                if response.attachment["ranking_measures"]["level"] == max_level
            ]
            return responses
        except Exception as ex:
            logger.error(ex)
        return []

    def resolve(self, responses):
        from numpy.random import choice

        responses = self.ranker.rank(responses)
        responses = [
            response
            for response in responses
            if not response.attachment.get("repeat", False)
        ]
        responses = self.get_max_level_responses(responses)

        scores = [
            response.attachment["ranking_measures"]["score"] for response in responses
        ]

        if scores:
            scores = [score if score > 30 else 0 for score in scores]  # threshold
            prob = softmax(scores)
            logger.info("scores %r softmax prob %r", scores, prob)
            response = choice(responses, p=prob)
            return response
        else:
            logger.error("No response")