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
import datetime
import hashlib
import logging
import os
import uuid
from contextlib import contextmanager
from typing import List

from sqlalchemy import create_engine
from sqlalchemy.orm import sessionmaker

import haipy.db_models as models
import haipy.schemas.chat as schemas

logger = logging.getLogger(__name__)


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


def _get_request_record(request: schemas.ChatRequest):
    attachment = {}
    attachment["ip"] = request.client_ip
    attachment["shortcut"] = request.shortcut
    attachment["audio"] = request.audio
    attachment["context"] = request.context
    return models.ChatRequest(
        app_id=request.app_id,
        request_id=request.request_id,
        created_at=request.created_at,
        user_id=request.user_id,
        conversation_id=request.conversation_id,
        text=request.text,
        lang=request.lang,
        **attachment,
    )


def write_chat_request(request: schemas.ChatRequest):
    """Writes chatbot request to database"""
    with session_scope() as session:
        session.add(_get_request_record(request))
    if write_to_remote:
        with remote_session_scope() as session:
            session.add(_get_request_record(request))


def delete_chat_request(request_id):
    with session_scope() as db:
        db_object = (
            db.query(models.ChatRequest).filter(
                models.ChatRequest.request_id == request_id
            )
        ).one_or_none()
        if db_object:
            db.delete(db_object)


def _get_response_records(responses: List[schemas.ChatResponse]):
    records = []
    for response in responses:
        attachment = response.attachment if response.attachment else {}
        record = models.ChatResponse(
            request_id=response.request_id,
            response_id=response.response_id,
            conversation_id=response.conversation_id,
            created_at=response.created_at,
            agent_id=response.agent_id,
            text=response.text,
            lang=response.lang,
            trace=response.trace,
            **attachment,
        )
        records.append(record)
    return records


def write_chat_responses(responses: List[schemas.ChatResponse]):
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


def update_published_response(response_id: str) -> schemas.ChatResponseBase:
    """Updates the pubilshed response"""
    with session_scope() as session:
        response = (
            session.query(models.ChatResponse)
            .filter(models.ChatResponse.response_id == response_id)
            .one_or_none()
        )
        if response:
            response.published_at = datetime.datetime.utcnow()
            session.add(response)
        else:
            logger.warning(
                "Failed to look up published responses. response_id %s", response_id
            )
    if write_to_remote:
        with remote_session_scope() as session:
            response = (
                session.query(models.ChatResponse)
                .filter(models.ChatResponse.response_id == response_id)
                .one_or_none()
            )
            if response:
                response.published_at = datetime.datetime.utcnow()
                session.add(response)
            else:
                logger.warning(
                    "Failed to look up published responses. response_id %s", response_id
                )


def get_chat_responses(conversation_id: str) -> schemas.ChatResponseBase:
    responses = []
    with session_scope() as db:
        responses = (
            (
                db.query(models.ChatResponse)
                .filter(models.ChatResponse.conversation_id == conversation_id)
                .filter(models.ChatResponse.published_at.isnot(None))
                .order_by(models.ChatResponse.created_at.desc())
            )
            .limit(100)
            .all()
        )
        responses = [schemas.ChatResponse.from_orm(response) for response in responses]
    return responses


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
engine = create_engine(db_url, echo=False, pool_pre_ping=True)
remote_engine = create_engine(remote_db_url, echo=False, pool_pre_ping=True)

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
