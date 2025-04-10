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

import datetime
import hashlib
import logging
import os
import socket
import uuid
from contextlib import contextmanager

import six
from haipy.db_models import ChatRequest, ChatResponse, ConvInsight
from sqlalchemy import create_engine, text
from sqlalchemy.orm import sessionmaker

logger = logging.getLogger("hr.ros_chatbot.db")


def hash_string(string):
    if isinstance(string, six.text_type):  # convert utf-8 to ascii
        string = string.encode("utf-8")
    m = hashlib.md5(string)
    return m.hexdigest()


def get_hostid():
    return hash_string(str(uuid.UUID(int=uuid.getnode())))


def get_hostname():
    return socket.gethostname()


hostname = get_hostname()
hostid = get_hostid()


@contextmanager
def session_scope():
    """Provide a transactional scope around a series of operations."""
    session = Session()
    try:
        yield session
        session.commit()
    except Exception as ex:
        logger.exception(ex)
        session.rollback()
        raise
    finally:
        session.close()


@contextmanager
def remote_session_scope():
    """Provide a transactional scope around a series of operations."""
    session = RemoteSession()
    try:
        yield session
        session.commit()
    except Exception as ex:
        logger.exception(ex)
        session.rollback()
        raise
    finally:
        session.close()


def _get_request_record(request):
    attachment = {}
    if request.source:
        attachment["source"] = request.source
    if request.audio:
        attachment["audio"] = request.audio
    if request.tag:
        attachment["tag"] = request.tag
    if request.scene:
        attachment["scene"] = request.scene
    if request.context:
        attachment["context"] = request.context
    attachment["location"] = os.environ.get("LOCATION", "")
    attachment["ip"] = os.environ.get("IP", "")
    attachment["hostname"] = hostname
    attachment["hostid"] = hostid
    return ChatRequest(
        app_id=request.app_id,
        request_id=request.request_id,
        created_at=request.time,
        user_id=request.user_id,
        conversation_id=request.sid,
        text=request.question,
        lang=request.lang,
        **attachment,
    )


def _get_conv_insight(record):
    return ConvInsight(
        conversation_id=record["conversation_id"],
        type=record["type"],
        insight=record["insight"],
        created_at=record["created_at"],
    )


def write_request(request):
    """Writes chatbot request to database"""
    if not request:
        return
    with session_scope() as session:
        session.add(_get_request_record(request))
    if write_to_remote:
        with remote_session_scope() as session:
            session.add(_get_request_record(request))


def _get_response_records(responses):
    records = []
    for response in responses:
        attachment = response.attachment if response.attachment else {}
        attachment["robot"] = os.environ.get("ROBOT_NAME", "")
        attachment["body"] = os.environ.get("ROBOT_BODY", "")
        record = ChatResponse(
            request_id=response.request_id,
            response_id=response.response_id,
            conversation_id=response.sid,
            created_at=response.end_dt,
            agent_id=response.agent_id,
            text=response.answer,
            lang=response.lang,
            trace=response.trace,
            **attachment,
        )
        records.append(record)
    return records


def write_responses(responses):
    """Writes chatbot responses to database"""
    if not responses:
        return
    with session_scope() as session:
        for record in _get_response_records(responses):
            session.add(record)
    if write_to_remote:
        with remote_session_scope() as session:
            for record in _get_response_records(responses):
                session.add(record)


def write_conv_insight(record):
    """Writes conversation insight to database"""
    if not record:
        return
    with session_scope() as session:
        session.add(_get_conv_insight(record))
    if write_to_remote:
        with remote_session_scope() as session:
            session.add(_get_conv_insight(record))


def update_published_response(response_id: str, attachment: dict):
    """Updates the response with published state"""
    with session_scope() as session:
        response = (
            session.query(ChatResponse)
            .filter(ChatResponse.response_id == response_id)
            .one_or_none()
        )
        if response:
            response.published_at = datetime.datetime.utcnow()
            response.attachment.update(attachment)
            session.add(response)
        else:
            logger.warning(
                "Failed to look up published responses. response_id %s", response_id
            )
    if write_to_remote:
        with remote_session_scope() as session:
            response = (
                session.query(ChatResponse)
                .filter(ChatResponse.response_id == response_id)
                .one_or_none()
            )
            if response:
                response.published_at = datetime.datetime.utcnow()
                response.attachment.update(attachment)
                session.add(response)
            else:
                logger.warning(
                    "Failed to look up published responses. response_id %s", response_id
                )


def get_chat_stream(conversation_id: str):
    with session_scope() as session:
        query = text(
            f"SELECT * FROM chat_stream where ConversationID='{conversation_id}'"
        )
        logger.warning("Query %s", query)
        results = session.execute(query).all()
        return [r._asdict() for r in results]


root_password = os.environ.get("MYSQL_ROOT_PASSWORD")
database = os.environ.get("MYSQL_DATABASE")
remote_password = os.environ.get("MYSQL_REMOTE_PASSWORD")
remote_user = os.environ.get("MYSQL_REMOTE_USER")
remote_host = os.environ.get("MYSQL_REMOTE_HOST")
remote_port = os.environ.get("MYSQL_REMOTE_PORT")
remote_db_schema_version = os.environ.get("MYSQL_SCHEMA_VERSION")

db_url = f"mysql+mysqldb://root:{root_password}@127.0.0.1:3306/{database}?charset=utf8"
remote_db_url = f"mysql+mysqldb://{remote_user}:{remote_password}@{remote_host}:{remote_port}/{database}?charset=utf8"

# https://docs.sqlalchemy.org/en/13/faq/connections.html#mysql-server-has-gone-away
# https://docs.sqlalchemy.org/en/13/core/pooling.html#dealing-with-disconnects
connect_timeout = 3
engine = create_engine(
    db_url,
    echo=False,
    pool_pre_ping=True,
    connect_args={"connect_timeout": connect_timeout},
)
remote_engine = create_engine(
    remote_db_url,
    echo=False,
    pool_pre_ping=True,
    connect_args={"connect_timeout": connect_timeout},
)

RemoteSession = sessionmaker(bind=remote_engine)
Session = sessionmaker(bind=engine)
write_to_remote = False

if os.environ.get("WRITE_TO_REMOTE_DB") == "1":
    db_version = "unknown"
    try:
        with remote_engine.connect() as con:
            rs = con.execute("SELECT version_num FROM alembic_version")
            for row in rs:
                db_version = row[0]
                break
    except Exception as ex:
        logger.error(ex)
    if db_version == remote_db_schema_version:
        write_to_remote = True
        logger.error("Remote DB schema verified")
    else:
        logger.error(
            "Remote DB schema version %r, expect %r",
            db_version,
            remote_db_schema_version,
        )
else:
    logger.warning("Writing to remote DB is disabled")
    write_to_remote = False
