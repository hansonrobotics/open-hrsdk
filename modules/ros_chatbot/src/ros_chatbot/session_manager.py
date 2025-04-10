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

from ros_chatbot.agents.model import SessionizedAgent

logger = logging.getLogger(__name__)


class SessionManager(object):
    def __init__(self, agents):
        self._agents = agents
        self._sessions = {}  # user session -> agent sessions

    def agent_sessions(self, reset=False):
        """Retrieve or create agent session"""
        for agent in list(self._agents.values()):
            if isinstance(agent, SessionizedAgent) and agent.enabled:
                if reset:
                    # reset agent session
                    if hasattr(agent, "reset_session"):
                        agent.reset_session()
                if self._sessions.get(agent.id) is None:
                    try:
                        agent_session = agent.new_session()
                        logger.info(
                            "New session for agent %r %r", agent.id, agent_session
                        )
                        self._sessions[agent.id] = agent_session
                    except Exception as ex:
                        logger.exception(ex)
            else:
                self._sessions[agent.id] = None
        return self._sessions
