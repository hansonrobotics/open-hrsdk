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
import os
from itertools import groupby

from langchain_community.chat_message_histories import RedisChatMessageHistory

REDIS_SERVER_HOST = os.environ.get("REDIS_SERVER_HOST", "localhost")
REDIS_SERVER_PORT = os.environ.get("REDIS_SERVER_PORT", "6379")


class ChatHistory(object):
    def __init__(
        self, ns, ai_message_prompt="AI: ", user_message_prompt="Human: ", ttl=3600
    ):
        self.history = RedisChatMessageHistory(
            ns,
            url=f"redis://{REDIS_SERVER_HOST}:{REDIS_SERVER_PORT}",
            key_prefix="",
            ttl=ttl,
        )
        self.ai_message_prompt = ai_message_prompt
        self.user_message_prompt = user_message_prompt

    def set_sid(self, sid):
        self.history.session_id = f"default.{sid}.history"

    def filtered_messages(self, current_input=None):
        messages = self.history.messages
        if current_input:
            # remove the last human message that happens to be the current input
            # message
            last_message = messages[-1] if messages else None
            if (
                last_message
                and last_message.type == "human"
                and last_message.content == current_input
            ):
                del messages[-1]

        # remove event messages
        messages = [
            message
            for message in messages
            if not (message.type == "human" and message.content.startswith("event."))
        ]

        # merge consecutive AI messages
        merged_messages = []
        for key, groups in groupby(messages, key=lambda message: message.type):
            groups = list(groups)
            message = groups[0]
            content = [group.content for group in groups]
            message.content = " ".join(content)
            merged_messages.append(message)

        return merged_messages

    def format_history_text(self, current_input=None, format=None):
        """Gets chat history"""
        if format is None:
            messages = self.filtered_messages(current_input)
            messages = messages[-20:]  # the last N messages
            history = [
                (
                    f"{self.user_message_prompt}{message.content}"
                    if message.type == "human"
                    else f"{self.ai_message_prompt}{message.content}"
                )
                for message in messages
            ]
            return "\n".join(history) or "[No history]"
        elif format == "llama3":
            return self.format_llama3_history_text(current_input)

    def format_llama3_history_text(self, current_input=None):
        """Special Tokens used with Meta Llama 3
        <|begin_of_text|>: This is equivalent to the BOS token
        <|eot_id|>: This signifies the end of the message in a turn.
        <|start_header_id|>{role}<|end_header_id|>: These tokens enclose the role for a particular message. The possible roles can be: system, user, assistant.
        <|end_of_text|>: This is equivalent to the EOS token. On generating this token, Llama 3 will cease to generate more tokens.
        A prompt should contain a single system message, can contain multiple alternating user and assistant messages, and always ends with the last user message followed by the assistant header.

        See also
        https://docs.aws.amazon.com/bedrock/latest/userguide/bedrock-runtime_example_bedrock-runtime_Llama3_InvokeLlama_section.html
        """
        messages = self.filtered_messages(current_input)
        history = [
            (
                f"<|start_header_id|>user<|end_header_id|>{message.content}<|eot_id|>"
                if message.type == "human"
                else f"<|start_header_id|>assistant<|end_header_id|>{message.content}<|eot_id|>"
            )
            for message in messages
        ]
        return "".join(history) or "<|start_header_id|>user<|end_header_id|><|eot_id|>"

    def add_user_message(self, message: str):
        self.history.add_user_message(message)

    def add_ai_message(self, message: str):
        self.history.add_ai_message(message)

    def clear_message(self):
        self.history.clear()


if __name__ == "__main__":
    history = ChatHistory("default.default.history")
    # history.clear_message()
    text = history.format_history_text()
    print(text)
