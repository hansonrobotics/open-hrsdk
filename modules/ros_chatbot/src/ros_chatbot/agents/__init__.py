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

from .ai21 import AI21Agent
from .aiml import AIMLAgent
from .baidu_unit import BaiduUnitAgent
from .blenderbot import BlenderBotAgent
from .chatgpt import ChatGPTAgent, GPT4Agent
from .chatgpt_web import ChatGPTWebAgent
from .chatscript import ChatScriptAgent
from .ddg import DDGAgent
from .dummy import DummyAgent
from .gpt2 import GPT2Agent
from .gpt3 import GPT3Agent
from .legend_chat import LegendChatAgent
from .llama import LlamaAgent
from .llm_chat import (
    ClaudeChatAgent,
    LlamaChatAgent,
    LLMChatAgent,
    OpenAIChatAgent,
    ToolCallingLLMChatAgent,
)
from .qa import QAAgent
from .quickchat import QuickChatAgent
from .quicksearch import QuickSearchAgent
from .rosagent import ROSGenericAgent
from .snet import SNetAgent
from .soultalk import SoulTalkAgent
from .tg_agent import TGAgent
from .translator import TranslatorAgent
from .vector_chat import VectorChatAgent
from .xiaoi import XiaoIAgent
from .xiaoice import XiaoIceAgent
from .youchat import YouChatAgent

_agent_classes = [
    AI21Agent,
    AIMLAgent,
    BaiduUnitAgent,
    BlenderBotAgent,
    ChatGPTAgent,
    ChatGPTWebAgent,
    ChatScriptAgent,
    ClaudeChatAgent,
    DDGAgent,
    DummyAgent,
    GPT2Agent,
    GPT3Agent,
    GPT4Agent,
    LLMChatAgent,
    LegendChatAgent,
    LlamaAgent,
    LlamaChatAgent,
    OpenAIChatAgent,
    QAAgent,
    QuickChatAgent,
    QuickSearchAgent,
    ROSGenericAgent,
    SNetAgent,
    SoulTalkAgent,
    TGAgent,
    ToolCallingLLMChatAgent,
    TranslatorAgent,
    VectorChatAgent,
    XiaoIAgent,
    XiaoIceAgent,
    YouChatAgent,
]

registered_agents = {cls.type: cls for cls in _agent_classes}
