#!/usr/bin/env python

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

import os
import sys
import logging

import coloredlogs
from gevent.pywsgi import WSGIServer
from flask import Flask
from flask import jsonify
from flask import request

from ros_chatbot.chat_server import ChatServer


class ChatbotServerRestAPIWrapper(object):
    def __init__(self):
        self.server = ChatServer()
        self.default_language = "en-US"

    def session(self):
        data = request.args
        sid = data.get("sid")
        sid = self.server.session(sid)
        json_response = {}
        json_response["err_code"] = 0
        json_response["err_msg"] = ""
        json_response["response"] = {"sid": sid}
        return jsonify(json_response)

    def chat(self):
        """
        parameters:
        -----------
            sid: session id
            text: input question
            lang: language code
            context: chat context
            mode: resolving/ranking
        """
        data = request.args
        sid = data.get("sid")
        text = data.get("text")
        lang = data.get("lang", self.default_language)
        context = data.get("context")
        mode = data.get("mode", "resolving")

        json_response = {}
        json_response["err_code"] = 0
        json_response["err_msg"] = ""

        response = None
        try:
            chat_request = self.server.new_request(sid, text, lang, context=context)
            response = self.server.chat(chat_request, mode)
        except Exception as ex:
            json_response["err_code"] = 1
            json_response["err_msg"] = ex.message

        if isinstance(response, list):
            json_responses = []
            for _response in response:
                json_responses.append(_response.to_dict())
            json_response["response"] = json_responses
        elif response:
            json_response["response"] = response.to_dict()
        return jsonify(json_response)

    def publish(self):
        data = request.args
        agent_id = data.get("agent_id")
        request_id = data.get("request_id")
        lang = data.get("lang", self.default_language)
        answer = data.get("answer")
        label = data.get("label")

        self.server.publish(agent_id, request_id, lang, answer, label)

        json_response = {}
        json_response["err_code"] = 0
        json_response["err_msg"] = ""
        return jsonify(json_response)

    def status(self):
        json_response = {}
        json_response["err_code"] = 0
        json_response["err_msg"] = ""
        return jsonify(json_response)


def create_server(args):
    chatbot = ChatbotServerRestAPIWrapper()

    app = Flask(__name__)
    app.add_url_rule("/session", "session", chatbot.session)
    app.add_url_rule("/chat", "chat", chatbot.chat)
    app.add_url_rule("/publish", "publish", chatbot.publish)
    app.add_url_rule("/status", "status", chatbot.status)

    server = WSGIServer((args.host, args.port), app)
    return server


def main():
    import argparse

    parser = argparse.ArgumentParser("Chatbot Server")

    parser.add_argument(
        "--port", dest="port", type=int, default=9100, help="Server port"
    )
    parser.add_argument("--host", dest="host", default="localhost", help="Server host")

    if "coloredlogs" in sys.modules and os.isatty(2):
        formatter_str = "%(asctime)s %(levelname)-7s %(name)s: %(message)s"
        coloredlogs.install(logging.INFO, fmt=formatter_str)

    args = parser.parse_args()
    server = create_server(args)
    server.serve_forever()


if __name__ == "__main__":
    main()
