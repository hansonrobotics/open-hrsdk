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
import re

from langchain.prompts import ChatPromptTemplate
from langchain_core.output_parsers import StrOutputParser
from langchain_openai import ChatOpenAI

from haipy.text_processing.template_renderer import Renderer

logger = logging.getLogger(__name__)


class PromptTemplateEval(object):
    def __init__(self):
        self.renderer = Renderer()
        self.openai_model = ChatOpenAI()
        self.output_parser = StrOutputParser()

    def eval(self, prompt_template, context):
        prompt_str = self.renderer.render(prompt_template, context, compact=False)
        prompt_str = re.sub("\n{2,}", "\n\n", prompt_str)
        prompt = ChatPromptTemplate.from_template(prompt_str, template_format="jinja2")
        chain = prompt | self.openai_model | self.output_parser
        logger.warning("Prompt: \n%s", prompt_str)
        ret = chain.invoke({})
        return ret
