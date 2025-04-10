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

import asyncio
import argparse
import queue
import logging
from functools import partial

from telethon import TelegramClient, events
from telethon.network import ConnectionTcpAbridged
from telethon.tl import types
from telethon.utils import get_display_name

logger = logging.getLogger(__name__)


def get_user_id(entity):
    if isinstance(entity, types.User):
        return str(entity.id)


def generate_session(session, api_id, api_hash):
    """Generates session file"""
    with TelegramClient(session, api_id, api_hash) as client:
        pass


async def display_users(session, api_id, api_hash):
    async with TelegramClient(session, api_id, api_hash) as client:
        dialogs = await client.get_dialogs(limit=10)
        for dialog in dialogs:
            print(
                "Name: %s, id: %s"
                % (get_display_name(dialog.entity), get_user_id(dialog.entity))
            )


class MyTelegramClient(TelegramClient):
    def __init__(self, session_user_id, api_id, api_hash, id):
        super().__init__(
            session_user_id, api_id, api_hash, connection=ConnectionTcpAbridged
        )
        loop.run_until_complete(self.connect())
        if not loop.run_until_complete(self.is_user_authorized()):
            raise RuntimeError("User is not authorized")
        self.answer = queue.Queue(maxsize=10)
        self.id = id

    def get_dialog_entity(self, dialogs):
        for dialog in dialogs:
            if get_user_id(dialog.entity) == self.id:
                return dialog.entity

    async def chat(self, msg, timeout=None):
        if timeout == -1:
            timeout = None
        self.add_event_handler(self.message_handler, events.NewMessage)

        dialogs = await self.get_dialogs(limit=10)
        self.entity = self.get_dialog_entity(dialogs)

        if msg:
            await self.send_message(self.entity, msg, link_preview=False)

        try:
            answer = await loop.run_in_executor(
                None, partial(self.answer.get, timeout=timeout)
            )
        except queue.Empty:
            answer = None
        return answer

    async def message_handler(self, event):
        chat = await event.get_chat()
        if get_user_id(chat) == self.id:
            if chat.is_self:
                # self chat
                self.answer.put(event.text)
            else:
                if not event.is_group and not event.out:
                    self.answer.put(event.text)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    subparser = parser.add_subparsers()

    session_parser = subparser.add_parser("session", help="Generate session file")
    session_parser.add_argument("--api-id", required=True, help="the telegram api_id")
    session_parser.add_argument(
        "--api-hash", required=True, help="the telegram api_hash"
    )
    session_parser.add_argument(
        "--session", required=True, help="the session file to be created"
    )
    session_parser.set_defaults(run="session")

    chat_parser = subparser.add_parser("chat")
    chat_parser.add_argument("--api-id", required=True, help="the telegram api_id")
    chat_parser.add_argument("--api-hash", required=True, help="the telegram api_hash")
    chat_parser.add_argument("--session", required=True, help="the session file")
    chat_parser.add_argument("--id", required=True, help="the user id")
    chat_parser.add_argument(
        "--question", required=True, help="the question to send to telegram chatbot"
    )
    chat_parser.add_argument("--timeout", type=float, help="timeout")
    chat_parser.set_defaults(run="chat")

    users_parser = subparser.add_parser("users")
    users_parser.add_argument("--api-id", required=True, help="the telegram api_id")
    users_parser.add_argument("--api-hash", required=True, help="the telegram api_hash")
    users_parser.add_argument(
        "--session", required=True, help="the session file to be created"
    )
    users_parser.set_defaults(run="users")

    args = parser.parse_args()

    if hasattr(args, "run"):
        if args.run == "chat":
            loop = asyncio.get_event_loop()
            client = MyTelegramClient(args.session, args.api_id, args.api_hash, args.id)
            answer = loop.run_until_complete(client.chat(args.question, args.timeout))
            if answer:
                print(answer)
        elif args.run == "session":
            generate_session(args.session, args.api_id, args.api_hash)
        elif args.run == "users":
            loop = asyncio.get_event_loop()
            loop.run_until_complete(
                display_users(args.session, args.api_id, args.api_hash)
            )
    else:
        parser.print_usage()
