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

from haipy.chat_history import ChatHistory
from haipy.parameter_server_proxy import UserSessionContext

from .base import PromptTemplateEval

logger = logging.getLogger(__name__)


class RedisChat(PromptTemplateEval):
    def __init__(self, uid, sid):
        self.session_context = UserSessionContext(uid, sid)
        self.prompt_templates = self.session_context["prompt_templates"]
        self.chat_history = ChatHistory(ns=f"{uid}.{sid}.history")

    def evaluate_template(self, template_name, context):
        _context = {}
        if self.session_context:
            _context.update(self.session_context.items())
        _context.update({"history": self.chat_history.format_history_text(input)})
        prompt_template = self.prompt_templates.get(template_name)

        return super(RedisChat, self).eval(prompt_template, _context)


if __name__ == "__main__":
    a = RedisChat("default", "default")
    print(a.prompt_templates)
    ans = a.evaluate_template("default")
    print(ans)
