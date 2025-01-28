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
import json
import logging
import os
import re
import shutil
import tarfile
import uuid
from abc import ABCMeta, abstractmethod
from collections import defaultdict
from pathlib import Path

import airtable
import boto3
import requests
from dateutil.parser import parse
from slugify import slugify

from haipy.arf.spreadsheet_converter import SpreadsheetConverter
from haipy.schemas.airtable_schemas import (
    EventStatus,
    EventTable,
    SceneStatus,
    SceneTable,
    ScriptRecord,
    ScriptStatus,
    ScriptTable,
    ScriptType,
    Stage,
)
from haipy.utils import create_soultalk_package_meta, dump_yaml, mkdir

logger = logging.getLogger(__name__)


def get_script_stage(record: ScriptRecord, buf):
    name = record.fields.Name
    if not record.fields.Start:
        raise ValueError("Script %r has no start time" % name)
    if not record.fields.End:
        raise ValueError("Script %r has no end time" % name)
    start_time = record.fields.Start
    end_time = record.fields.End
    if buf:
        logger.info("Add time buffer %s seconds", buf)
        start_time -= datetime.timedelta(seconds=buf)
        end_time += datetime.timedelta(seconds=buf)
    now = datetime.datetime.now(tz=datetime.timezone.utc)

    if now >= start_time and now <= end_time:
        return Stage.CurrentStage
    else:
        if now > end_time:
            logger.warning("Script %r is in the past. diff %s", name, end_time - now)
            return Stage.PostStage
        elif now < start_time:
            logger.warning(
                "Script %r is in the future. diff %s", name, start_time - now
            )
            return Stage.PreStage
        else:
            logger.error(
                "Unknown script stage %r start %s end %s now %s",
                name,
                start_time,
                end_time,
                now,
            )


def _prepare_spreadsheet(meta, record, event_stage, scripts_dir, output_dir):
    script_folder = os.path.join(scripts_dir, record.id)
    template_result = None
    for filename in Path(script_folder).glob("*.xlsx"):
        logger.info("Found spreadsheet %r", os.path.basename(filename))
        try:
            converter = SpreadsheetConverter(meta, record, filename, output_dir)
            template_result = converter.generate_from_template()
            converter.to_chat_rules(event_stage)
            converter.to_intents(event_stage)
            converter.to_timelines(event_stage=event_stage)
            converter.to_behavior_tree()
            converter.to_sheet_meta()
            converter.to_prompt_templates(template_result)
        except Exception as ex:
            logger.exception("Error %s", ex)
    return template_result


def _prepare_soultalk(record, scripts_dir, output_dir):
    script_folder = os.path.join(scripts_dir, record.id)
    dest = os.path.join(output_dir, f"soultalk")

    if not os.path.isdir(dest):
        os.makedirs(dest)
    package_file = os.path.join(dest, "package.yaml")
    if not os.path.isfile(package_file):
        dump_yaml(create_soultalk_package_meta(), package_file)

    topics = os.path.join(dest, "topics")
    for filename in Path(script_folder).glob("**/*.yaml"):
        shutil.copy(filename, topics)
        logger.info("Copied soultalk script %r", os.path.basename(filename))

    # decompress tar.gz files to the package home directory
    for filename in Path(script_folder).glob("**/*.tar.gz"):
        with tarfile.open(filename) as f:
            f.extractall(dest)
        logger.info("Uncompressed %r", os.path.basename(filename))


def _prepare_behavior_tree(record, scripts_dir, output_dir):
    script_folder = os.path.join(scripts_dir, record.id)
    for filename in Path(script_folder).glob("**/*.json"):
        logger.info("Found behavior tree script %r", os.path.basename(filename))


def _prepare_timeline(name, record, scripts_dir, output_dir):
    """copy the timeline scripts to the robot performance folder"""
    script_folder = os.path.join(scripts_dir, record.id)
    if not output_dir or not os.path.isdir(output_dir):
        logger.error("Performance dir does not exist")
        return
    if not name:
        logger.error("robot name is not determined")
        return

    name = record.fields.Name
    dest = os.path.join(output_dir, name, name)
    mkdir(dest)
    for filename in Path(script_folder).glob("**/*.yaml"):
        shutil.copy(filename, dest)
        logger.info("Copied timeline %r", os.path.basename(filename))


def _prepare_timeline_package(name, record, scripts_dir, output_dir, remove=False):
    """copy the timeline package to the robot performance folder"""
    output_dir = os.path.join(output_dir, "timelines")
    script_folder = os.path.join(scripts_dir, record.id)
    mkdir(output_dir)

    dest = Path(output_dir, name)
    if not dest.is_dir():
        dest.mkdir(parents=True)

    if remove:
        # remove all the the .cmscopy folders
        for path in dest.glob("**/*"):
            if path.is_dir() and path.joinpath(".cmscopy").is_file():
                logger.info("Removing %s", path)
                shutil.rmtree(path)

    for filename in Path(script_folder).glob("**/*.tar.gz"):
        with tarfile.open(filename) as f:
            filenames = f.getnames()
            topfolders = [name.split("/")[0] for name in filenames]
            topfolders = list(set(topfolders))
            if len(topfolders) != 1:
                logger.error("Can't find timeline top name")
                continue
            timeline_dir = dest.joinpath(topfolders[0])
            # the folder without .cmscopy is the master folder, ignore the folder
            if timeline_dir.is_dir():
                if not timeline_dir.joinpath(".cmscopy").is_file():
                    logger.warning("Ignore the timeline %r", timeline_dir)
                    continue
                shutil.rmtree(timeline_dir)
            f.extractall(dest)
            with open(timeline_dir.joinpath(".cmscopy"), "w") as f:
                now = datetime.datetime.now(tz=datetime.timezone.utc)
                f.write(now.strftime("%c"))
            logger.info("Inflated timeline package %r", os.path.basename(filename))


class ContentBase(metaclass=ABCMeta):
    S3_URL_PATTERN = re.compile(r"s3://([^/]*)/(.*)")
    s3 = boto3.client("s3")
    MIME_TYPE_TO_EXTENTIONS = {
        "application/vnd.google-apps.spreadsheet": ".xlsx",
        "application/vnd.google-apps.document": ".docx",
        "application/vnd.openxmlformats-officedocument.spreadsheetml.sheet": ".xlsx",
        "application/x-yaml": ".yaml",
        "text/csv": ".csv",
    }

    def __init__(self, home_dir):
        self.home_dir = home_dir

    @staticmethod
    def _download_file_by_link(url, dest):
        r = requests.get(url, allow_redirects=True)
        if r.status_code == 200:

            def get_filename():
                if "Content-Disposition" in r.headers:
                    filename_pattern = re.compile(r'^.*filename="([\w,\s.-]+)".*$')
                    matchobj = filename_pattern.match(r.headers["Content-Disposition"])
                    if matchobj:
                        filename = matchobj.group(1)
                        return filename
                return uuid.uuid4().hex

            filename = get_filename()

            # add extension
            if "." not in filename:
                mimeType = r.headers["Content-Type"]
                ext = ContentBase.MIME_TYPE_TO_EXTENTIONS.get(mimeType, ".bin")
                filename += ext

            asfile = os.path.join(dest, filename)

            directory = os.path.dirname(asfile)
            if not os.path.isdir(directory):
                os.makedirs(directory)

            with open(asfile, "wb") as f:
                f.write(r.content)
            return asfile

    @staticmethod
    def _download_attachment(url, asfile):
        r = requests.get(url, allow_redirects=True)
        if r.status_code == 200:
            directory = os.path.dirname(asfile)
            if not os.path.isdir(directory):
                os.makedirs(directory)

            with open(asfile, "wb") as f:
                f.write(r.content)
            return asfile
        else:
            logger.error(
                "Can't download attachment %s status code: %r", asfile, r.status_code
            )

    @staticmethod
    def time_diff(timestr: str):
        """Compute the total seconds of difference between the given time and
        the system current time.
        """
        now = datetime.datetime.now(tz=datetime.timezone.utc)
        timediff = parse(timestr) - now
        return timediff.total_seconds()

    @abstractmethod
    def get_meta(self, name, include_test, **conditions):
        """Returns a list of script records"""
        pass

    def download(self, base_id, api_key, name, **conditions):
        self.download_tables(base_id, api_key)
        try:
            # download all the script files including test files
            meta = self.get_meta(name, include_test=True, **conditions)
        except FileNotFoundError as ex:
            logger.error(ex)
            return
        scripts = meta["scripts"]
        scripts_dir = os.path.join(self.download_folder, "scripts")
        mkdir(scripts_dir)
        if scripts:
            use_cache = conditions.get("use_cache", False)
            self.download_scripts(scripts, scripts_dir, use_cache)
        else:
            logger.info("No scripts to download")

    def download_scripts(
        self, script_records: dict, download_folder: str, use_cache: bool
    ):
        """
        download_folder: the root download folder for scripts
        script_records: list of script records
        """

        for record in script_records:
            logger.warning(
                "Pulling script %r type %s", record.fields.Name, record.fields.Type
            )
            # check the destination directory to decide whether to cache or not
            cached = False
            dest = os.path.join(download_folder, record.id)
            if use_cache and os.path.isdir(dest):
                st = os.stat(dest)
                last_modified_time = record.fields.Last_Modified
                cloud_mtime = parse(last_modified_time)
                local_mtime = datetime.datetime.fromtimestamp(
                    st.st_mtime, tz=datetime.timezone.utc
                )
                logger.info("cloud mtime %s", cloud_mtime)
                logger.info("local mtime %s", local_mtime)
                if local_mtime >= cloud_mtime:
                    cached = True
                    logger.warning("Using cached files")
            if not cached and os.path.isdir(dest):
                shutil.rmtree(dest, ignore_errors=True)

            if record.fields.Type == ScriptType.TimelinePackage:
                # use the script name to generate the s3 link
                link = (
                    "s3://dl.cms.hansonrobotics.com/performances/{name}.tar.gz".format(
                        name=record.fields.Name.lower()
                    )
                )
            else:
                link = record.fields.Published_Link
            if link:
                if not cached:
                    matchobj = self.S3_URL_PATTERN.match(link)
                    if matchobj:
                        # download s3 object
                        bucket, s3_object = matchobj.groups()
                        file = os.path.join(dest, os.path.basename(s3_object))
                        if not os.path.isdir(dest):
                            os.makedirs(dest)
                        self.s3.download_file(bucket, s3_object, file)
                        logger.info("Downloaded s3 file %s", file)
                    else:
                        # download google drive spreadsheet files
                        file = self._download_file_by_link(link, dest)
                        logger.info("Downloaded spreadsheet %s", file)

            # download attachments
            attachments = record.fields.Attachments
            if attachments:
                if not cached:
                    for attachment in attachments:
                        dest = os.path.join(download_folder, record.id, "attachments")
                        asfile = os.path.join(dest, attachment.filename)
                        self._download_attachment(attachment.url, asfile)
                        logger.info("Downloaded attachment %s", attachment)

    def get_current_meta_context(self, name, include_test):
        """Get the current event context such as location"""
        context = {}
        try:
            meta = self.get_meta(name, include_test=include_test)
        except FileNotFoundError as ex:
            logger.error(ex)
            return {}
        now = datetime.datetime.now(tz=datetime.timezone.utc)
        for event_record in meta.get("events", []):
            if now >= event_record.fields.Start and now <= event_record.fields.End:
                if event_record.fields.Location:
                    context["location"] = event_record.fields.Location
        return context

    def process_script(self, name, include_test, output_dir=None):
        """
        name: robot name or character name
        include_test: whether include "Test" scripts
        output_dir: the output directory for the processed files

        returns the processed records
        """
        if output_dir is None:
            output_dir = os.path.join(self.home_dir, "processed")
        mkdir(output_dir)
        try:
            meta = self.get_meta(name, include_test=include_test)
        except FileNotFoundError as ex:
            logger.error(ex)
            return
        scripts_dir = os.path.join(self.download_folder, "scripts")
        base = meta.get("base")  # arf or event

        # save meta file for debugging
        meta_file = os.path.join(output_dir, f"{name}-{base}.json")
        with open(meta_file, "w") as f:
            json.dump(meta, f, indent=4, default=str)

        script_event_mapping = {}
        event_stage = ""
        event_records = meta.get("events", [])
        for event_record in event_records:
            for script_record_id in event_record.fields.Scripts:
                if script_record_id in script_event_mapping:
                    existing_event_record = script_event_mapping[script_record_id]
                    # Do not allow one script to be included in multiple events
                    logger.error(
                        "Script %r is included in multiple events %r and %r",
                        script_record_id,
                        event_record.fields.Event,
                        existing_event_record.fields.Event,
                    )
                script_event_mapping[script_record_id] = event_record

        script_records = []
        processed_result = defaultdict(list)
        for record in meta.get("scripts", []):
            event_record = None
            if record.id in script_event_mapping:
                event_record = script_event_mapping[record.id]
            meta["event_record"] = event_record
            type = record.fields.Type
            logger.warning(
                "Processing script %r type %s", record.fields.Name, record.fields.Type
            )
            mkdir(output_dir)
            name = meta["name"]
            if type == ScriptType.GoogleSheet:
                spreadsheet_result = _prepare_spreadsheet(
                    meta, record, event_stage, scripts_dir, output_dir
                )
                if spreadsheet_result and event_record:
                    processed_result[event_record.id].extend(spreadsheet_result)
            if type == ScriptType.Soultalk:
                _prepare_soultalk(record, scripts_dir, output_dir)
            if type == ScriptType.BehaviorTree:
                _prepare_behavior_tree(record, scripts_dir, output_dir)
            if type == ScriptType.Timeline:
                _prepare_timeline(name, record, scripts_dir, output_dir)
            if type == ScriptType.TimelinePackage:
                _prepare_timeline_package(name, record, scripts_dir, output_dir)
            script_records.append(record)

        btrees = {}
        for event_id, results in processed_result.items():
            btrees[event_id] = sum(
                [
                    [result["tree"]] * result["repeat"]
                    for result in results
                    if result["type"] == "behavior"
                ],
                [],
            )
        return {
            "script_event_mapping": script_event_mapping,
            "events": event_records,
            "scripts": script_records,
            "btrees": btrees,
        }


class RobotOperationContentBase(ContentBase):
    BASE_NAME = "Robot Operation"

    def __init__(self, home_dir):
        super(RobotOperationContentBase, self).__init__(home_dir)
        self.download_folder = os.path.join(
            home_dir, slugify(self.BASE_NAME, lowercase=False)
        )
        self.tables_folder = os.path.join(self.download_folder, "tables")
        self.scripts_meta_file = os.path.join(self.tables_folder, "Scripts.json")
        self.events_meta_file = os.path.join(self.tables_folder, "Events.json")

        self.meta = {"base": "event", "basename": self.BASE_NAME}

    def download_tables(self, base_id, api_key):
        events_tbl = airtable.Airtable(base_id, "Events", api_key)
        scripts_tbl = airtable.Airtable(base_id, "Scripts", api_key)
        mkdir(self.tables_folder)
        scripts = scripts_tbl.get_all()
        with open(self.scripts_meta_file, "w") as f:
            json.dump(scripts, f, indent=4, default=str)
        events = events_tbl.get_all()
        with open(self.events_meta_file, "w") as f:
            json.dump(events, f, indent=4, default=str)

    def get_meta(self, name, include_test, **conditions):
        logger.info("Getting event scripts")
        with open(self.scripts_meta_file) as f:
            records = json.load(f)
            # filter out empty records
            records = [record for record in records if "Name" in record["fields"]]
            scripts_table = ScriptTable(records=records)
            scripts_lookup = {script.id: script for script in scripts_table.records}
        now = datetime.datetime.now(tz=datetime.timezone.utc)
        with open(self.events_meta_file) as f:
            records = json.load(f)
            # filter out empty records
            records = [record for record in records if "Event" in record["fields"]]
            events_table = EventTable(records=records)
            robot_events = []
            # select the confirmed events that are in the future
            robot_events += [
                record
                for record in events_table.records
                if name in record.fields.RobotNames
                and record.fields.Status in [EventStatus.Confirmed]
                and record.fields.End >= now
            ]
            # select all test and permanent events
            robot_events += [
                record
                for record in events_table.records
                if name in record.fields.RobotNames
                and record.fields.Status in [EventStatus.Test, EventStatus.Permanent]
            ]

        active_scripts = []
        for event in robot_events:
            # check event status flag
            if not include_test and event.fields.Status == EventStatus.Test:
                logger.info('Ignore "Test" event %r', event.fields.Event)
                continue
            for script_id in event.fields.Scripts:
                script = scripts_lookup[script_id]
                # check script status flag
                if not include_test and script.fields.Status == ScriptStatus.Test:
                    logger.info("Ignore Test script %r", script.fields.Name)
                    continue
                if script.fields.Status in [ScriptStatus.Done, ScriptStatus.Test]:
                    active_scripts.append(script)
        meta = {}
        meta.update(self.meta)
        meta["name"] = name
        meta["scripts"] = active_scripts
        meta["events"] = robot_events
        return meta


class ARFContentBase(ContentBase):
    BASE_NAME = "ARF Content Development"

    def __init__(self, home_dir):
        super(ARFContentBase, self).__init__(home_dir)
        self.download_folder = os.path.join(
            home_dir, slugify(self.BASE_NAME, lowercase=False)
        )
        self.tables_folder = os.path.join(self.download_folder, "tables")
        self.scripts_meta_file = os.path.join(self.tables_folder, "Scripts.json")
        self.scenes_meta_file = os.path.join(self.tables_folder, "Scenes.json")

        self.meta = {"base": "arf", "basename": self.BASE_NAME}

    def download_tables(self, base_id, api_key):
        scenes_tbl = airtable.Airtable(base_id, "Scenes", api_key)
        scripts_tbl = airtable.Airtable(base_id, "Scripts", api_key)
        mkdir(self.tables_folder)
        scripts = scripts_tbl.get_all()
        with open(self.scripts_meta_file, "w") as f:
            json.dump(scripts, f, indent=4, default=str)
        scenes = scenes_tbl.get_all()
        with open(self.scenes_meta_file, "w") as f:
            json.dump(scenes, f, indent=4, default=str)

    def get_meta(self, name, include_test, **conditions):
        logger.info("Getting ARF scripts")
        with open(self.scripts_meta_file) as f:
            records = json.load(f)
            # filter out empty records
            records = [record for record in records if "Name" in record["fields"]]
            scripts_table = ScriptTable(records=records)
            scripts_lookup = {script.id: script for script in scripts_table.records}
        with open(self.scenes_meta_file) as f:
            records = json.load(f)
            # filter out empty records
            records = [record for record in records if "Name" in record["fields"]]
            scenes_table = SceneTable(records=records)
            character_scenes = [
                record
                for record in scenes_table.records
                if name in record.fields.CharacterNames
                and record.fields.Status in [SceneStatus.Done, SceneStatus.Test]
            ]

        active_scripts = []
        for scene in character_scenes:
            if not include_test and scene.fields.Status == SceneStatus.Test:
                logger.info("Ignore Test scene %r", scene.fields.Event)
                continue
            for script_id in scene.fields.Scripts:
                script = scripts_lookup[script_id]
                # check script status flag
                if not include_test and script.fields.Status == ScriptStatus.Test:
                    logger.info("Ignore Test script %r", script.fields.Name)
                    continue
                if script.fields.Status in [ScriptStatus.Done, ScriptStatus.Test]:
                    active_scripts.append(script)

        meta = {}
        meta.update(self.meta)
        meta["name"] = name
        meta["scripts"] = active_scripts
        meta["scenes"] = character_scenes
        return meta
