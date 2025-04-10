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

import datetime as dt
import difflib
import logging
import math
import os
import re
import subprocess
from collections import Counter, OrderedDict
from collections.abc import Sequence
from pathlib import Path

import boto3
import requests
import yaml
from botocore.config import Config
from bs4 import BeautifulSoup
from haipy.schemas.airtable_schemas import Sheet
from langchain_anthropic import ChatAnthropic
from langchain_aws import ChatBedrock
from langchain_core.language_models import BaseChatModel
from langchain_openai import ChatOpenAI
from pytz import timezone

logger = logging.getLogger("hr.ros_chatbot.utils")

PUNCTUATORS = re.compile(r"""([.?!]+)""")

UNICODE_PUNCTUATION_MARKS = re.compile(r"""[.?!。？！]""", flags=re.UNICODE)

# https://cloud.google.com/translate/docs/languages
# https://learn.microsoft.com/en-us/azure/cognitive-services/speech-service/language-support?tabs=stt-tts#text-to-speech
LANGUAGE_CODE_MAPPING = {
    "ar-SA": "ar",
    "cmn-Hans-CN": "zh",
    "cs-CZ": "cs",
    "de-DE": "de",
    "en-US": "en",
    "es-ES": "es",
    "fr-FR": "fr",
    "hi-IN": "hi",
    "hu-HU": "hu",
    "it-IT": "it",
    "ja-JP": "ja",
    "ko-KR": "ko",
    "nb-NO": "nb",
    "pl-PL": "pl",
    "ru-RU": "ru",
    "yue-Hant-HK": "hk",
}
LANGUAGE_CODE_INV_MAPPING = {v: k for k, v in LANGUAGE_CODE_MAPPING.items()}

english_stopwords = {
    "i",
    "me",
    "my",
    "myself",
    "we",
    "our",
    "ours",
    "ourselves",
    "you",
    "your",
    "yours",
    "yourself",
    "yourselves",
    "he",
    "him",
    "his",
    "himself",
    "she",
    "her",
    "hers",
    "herself",
    "it",
    "its",
    "itself",
    "they",
    "them",
    "their",
    "theirs",
    "themselves",
    "what",
    "which",
    "who",
    "whom",
    "this",
    "that",
    "these",
    "those",
    "am",
    "is",
    "are",
    "was",
    "were",
    "be",
    "been",
    "being",
    "have",
    "has",
    "had",
    "having",
    "do",
    "does",
    "did",
    "doing",
    "a",
    "an",
    "the",
    "and",
    "but",
    "if",
    "or",
    "because",
    "as",
    "until",
    "while",
    "of",
    "at",
    "by",
    "for",
    "with",
    "about",
    "against",
    "between",
    "into",
    "through",
    "during",
    "before",
    "after",
    "above",
    "below",
    "to",
    "from",
    "up",
    "down",
    "in",
    "out",
    "on",
    "off",
    "over",
    "under",
    "again",
    "further",
    "then",
    "once",
    "here",
    "there",
    "when",
    "where",
    "why",
    "how",
    "all",
    "any",
    "both",
    "each",
    "few",
    "more",
    "most",
    "other",
    "some",
    "such",
    "no",
    "nor",
    "not",
    "only",
    "own",
    "same",
    "so",
    "than",
    "too",
    "very",
    "s",
    "t",
    "can",
    "will",
    "just",
    "don",
    "should",
    "now",
}

DEFAULT_PROMPT_TEMPLATE = {
    "gpt": """## Rules
{%- if system_prime %}
{{system_prime}}
{%- endif %}
{%- if tune %}
- Speak in a {{tune}} tune.
{%- endif %}
{%- if language %}
- Speak in {{language}}.
{%- endif %}
{%- if current_date and next_week %}
- Current Date: {{current_date}}
- Next week: {{next_week}}
{%- endif %}
{%- if current_time %}
- Current Time: {{current_time}}
{%- endif %}

{% if robot_persona %}
## Your Persona
{{ robot_persona }}
{% endif %}

{% if person_object %}
## Information about user
{{ person_object }}
{% endif %}

{% if rag %}
## Retrieved Information
{{ rag }}
{% endif %}

{% if past_conversation_summaries %}
## Past Conversation Summaries
{{past_conversation_summaries}}
{% endif %}

## Situation
{{general_prime}}
{{situational_prime}}
{{response_prime}}
{{episodic_memory}}

{% if visual_clue %}
## Visual Clue
Here is what you can see through your camera.
{{visual_clue}}
{% endif %}

{% if reply_instruction %}
## Important Reply Instruction
{{reply_instruction}}
{% endif %}

{% if done_steps %}
## Completed Plans
- {{ done_steps|join('\n- ') }}
{% endif %}

{% if emotion_state %}
## Emotional State
{{emotion_state}}
{% endif %}

{% if physiological_state %}
## Physiological State
{{physiological_state}}
{% endif %}

{% if tasks %}
## Issues to address or tasks to work on
- {{ tasks|join('\n- ') }}

NOTE: You must follow the tasks and guide the conversation
NOTE: Get right into the topic, do not talk anything else unless being asked, and then circle back to the task
NOTE: The tasks are sorted by priority, so you are encouraged to start with the highest priority task but the other tasks are also important
{% endif %}

{% if history %}
## Conversation History
{{history}}
{% endif %}

{% if input %}
Answer the user's following questions.

Human: {{input}}
AI:{% endif %}
""",
    "claude": """<Rules>
{% if system_prime %}
{{system_prime}}
{% endif %}
{% if tune %}
- Speak in a {{tune}} tune.
{% endif %}
{% if language %}
- Speak in {{language}}.
{% endif %}
{% if current_date and next_week %}
- Current Date: {{current_date}}
- Next week: {{next_week}}
{% endif %}
{% if current_time %}
- Current Time: {{current_time}}
{% endif %}
</Rules>

{% if robot_persona %}
<YourPersona>
{{ robot_persona }}
</Persona>
{% endif %}

{% if person_object %}
<UserInformation>
{{ person_object }}
</PersonObject>
{% endif %}

{% if rag %}
<RetrievedInformation>
{{ rag }}
</RetrievedInformation>
{% endif %}

{% if past_conversation_summaries %}
<PastConversationSummaries>
{{past_conversation_summaries}}
</PastConversationSummaries>
{% endif %}

<Situation>
{{general_prime}}
{{situational_prime}}
{{response_prime}}
{{episodic_memory}}
</Situation>

{% if visual_clue %}
<VisualClue>
Here is what you can see through your camera.
{{visual_clue}}
</VisualClue>
{% endif %}

{% if reply_instruction %}
<ReplyInstruction>
{{reply_instruction}}
</ReplyInstruction>
{% endif %}

{% if history %}
<History>
{{history}}
</History>
{% endif %}

{% if input %}
Answer the user's following questions.

Human: {{input}}
AI:{% endif %}
""",
    "llama": """<|begin_of_text|>## Rules
{% if system_prime %}
{{system_prime}}
{% endif %}
{% if tune %}
- Speak in a {{tune}} tune.
{% endif %}
{% if language %}
- Speak in {{language}}.
{% endif %}
{% if current_date and next_week %}
- Current Date: {{current_date}}
- Next week: {{next_week}}
{% endif %}
{% if current_time %}
- Current Time: {{current_time}}
{% endif %}

{% if robot_persona %}
## Your Persona
{{ robot_persona }}
{% endif %}

{% if person_object %}
## Information about user
{{ person_object }}
{% endif %}

{% if rag %}
## Retrieved Information
{{ rag }}
{% endif %}

{% if past_conversation_summaries %}
## Past Conversation Summaries
{{past_conversation_summaries}}
{% endif %}

## Situation
{{general_prime}}
{{situational_prime}}
{{response_prime}}
{{episodic_memory}}

{% if visual_clue %}
## Visual Clue
Here is what you can see through your camera.
{{visual_clue}}
{% endif %}

{% if reply_instruction %}
## Important Reply Instruction
{{reply_instruction}}
{% endif %}

{% if history %}
## Conversation History
{{history}}
{% endif %}

{% if input %}
<|start_header_id|>user<|end_header_id|>Human: {{input}}<|eot_id|>
<|start_header_id|>assistant<|end_header_id|>{% endif %}
    """,
}

try:
    import googlemaps

    gmaps = googlemaps.Client(key=os.environ.get("GCLOUD_API_KEY"))
except Exception as ex:
    logger.error("Couldn't initialize googlemaps client: %s", ex)
    gmaps = None


def str_cleanup(text):
    if text:
        text = text.strip()
        text = " ".join(text.split())
        if text and text[0] == ".":
            text = text[1:]
    return text


def shorten(text, max_len=100):
    """soft truncate the text"""
    if not text or len(text.split()) < max_len:
        # No need to cut off
        return text, ""
    sens = PUNCTUATORS.split(text)
    ret = ""
    idx = 0
    while idx < len(sens):
        chunk = ""
        if sens[idx]:
            if idx + 1 < len(sens):
                punctuator = sens[idx + 1]
                chunk = sens[idx] + punctuator
            else:
                chunk = sens[idx]
            next_text = ret + chunk
            if len(next_text.split()) > max_len:
                if len(ret.split()) > 3:  # if truncated text is long enough, stop
                    break
            ret = next_text
            idx += 1
        idx += 1

    res = "".join(sens[idx:])

    # If rest part is too short, then don't cut
    if len(res.split()) < 4:
        ret = text
        res = ""

    ret = str_cleanup(ret)
    res = str_cleanup(res)
    if text != ret:
        logger.info(
            "Truncate text: %s, length: %s, max length: %s",
            ret,
            len(ret.split()),
            max_len,
        )
    else:
        logger.info("Doesn't truncate text")
    return ret, res


def check_online(url="8.8.8.8", port="80", timeout=1):
    try:
        subprocess.check_output(
            ["ping", "-q", "-w", str(timeout), "-c", "1", str(url)],
            stderr=subprocess.STDOUT,
        )
    except Exception:
        return False
    return True


def get_current_time_str():
    format = "%Y-%m-%d %H:%M:%S"
    return dt.datetime.strftime(dt.datetime.utcnow(), format)


def abs_path(root_dir, p):
    """Returns absolute path"""
    if p.startswith("/"):
        return p
    if p.startswith("~"):
        return os.path.expanduser(p)
    return os.path.join(root_dir, p)


def remove_puncuation_marks(text):
    if text:
        text = re.sub(UNICODE_PUNCTUATION_MARKS, "", text)
        text = text.strip()
    return text


def get_location():
    if gmaps is None:
        return
    location = {}
    try:
        response = gmaps.geolocate()
        reverse_geocode_results = gmaps.reverse_geocode(
            (response["location"]["lat"], response["location"]["lng"])
        )
        for result in reverse_geocode_results:
            for address_component in result.get("address_components"):
                name = address_component.get("long_name")
                if not name:
                    continue
                types = address_component.get("types")
                if "political" in types:
                    if "neighborhood" in types:
                        location["neighborhood"] = name
                    if "locality" in types:
                        location["city"] = name
                    if "country" in types:
                        location["country"] = name
                    if "administrative_area_level_1" in types:
                        location["administrative_area_level_1"] = name
                    if "administrative_area_level_2" in types:
                        location["administrative_area_level_2"] = name
                if "route" in types:
                    location["route"] = name
                if "street_number" in types:
                    location["street_number"] = name
        return location
    except Exception as ex:
        logger.error(ex)


def get_ip():
    logger.info("Getting public IP address")
    try:
        ip = subprocess.check_output(
            ["wget", "--timeout", "3", "-qO-", "ipinfo.io/ip"]
        ).strip()
        ip = ip.decode("utf-8")
        logger.info("Got IP %s", ip)
    except subprocess.CalledProcessError as ex:
        ip = None
        logger.error("Can't find public IP address")
        logger.error(ex)
    if not ip:
        logger.error("Public IP is invalid")
    return ip


def get_ip_location(ip):
    host = os.environ.get("GEOLOCATION_SERVER_HOST", "localhost")
    port = os.environ.get("GEOLOCATION_SERVER_PORT", 8105)
    try:
        response = requests.get(f"http://{host}:{port}/{ip}", timeout=1)
        if response.status_code == 200:
            result = response.json()
            if result["country"] == "Macau":
                result["city"] = "Macau"
            if result["country"] == "HK":
                result["city"] = "Hong Kong"
            return result
    except Exception as ex:
        logger.error(ex)


def get_local_time(timezone_str):
    return dt.datetime.now(timezone("UTC")).astimezone(timezone(timezone_str))


def check_repeating_words(text):
    """Checks if some words repeating too much"""
    if text:
        count = Counter(text.split())
        for word, freq in count.most_common(10):
            word = word.lower()
            if freq >= 3 and word.isalpha() and word not in english_stopwords:
                logger.warning("Frequent repeating word detected: %s", word)
                return True
    return False


def token_sub(pattern, text):
    """Substitude the tokens in the text"""

    def replace(mobj):
        return "'" + mobj.groups()[1]

    if text:
        return pattern.sub(replace, text)


def softmax(x):
    """A simple implemention that works for 1D vector"""
    exp = [math.exp(i) for i in x]
    _sum = sum(exp)
    exp = [i / _sum for i in exp]
    return exp


def argmax(x):
    return x.index(max(x))


def envvar_yaml_loader():
    envvar_matcher = re.compile(r"\${([^}^{]+)\}")

    def env_var_single_replace(match):
        return (
            os.environ[match.group(1)]
            if match.group(1) in os.environ
            else match.group()
        )

    def constructor(loader, node):
        value = loader.construct_scalar(node)
        newvalue = re.sub(envvar_matcher, env_var_single_replace, value)
        if value == newvalue:
            raise ValueError('The environment variable was not set: "%s"' % value)
        return newvalue

    yaml.SafeLoader.add_implicit_resolver("!envvar", envvar_matcher, None)
    yaml.SafeLoader.add_constructor("!envvar", constructor)


def iterable(obj):
    return isinstance(obj, Sequence) and not isinstance(obj, str)


def to_list(obj):
    if obj is None:
        return []
    if iterable(obj):
        return obj
    else:
        return [obj]


def remove_duplicated_responses(responses):
    """removes duplicated responses but keeps the order"""
    responses = to_list(responses)
    ordered_responses = OrderedDict()
    for response in responses:
        ordered_responses[response.answer] = response
    uniq_responses = ordered_responses.values()
    if len(uniq_responses) != len(responses):
        logger.info("Removed duplicated responses")
    return uniq_responses


def search_answer_post_processing(text):
    # replace (year) with "in year"
    text = re.sub(r"\((\d\d\d\d)\)", r" in \1", text)

    # remove (blah blah)
    text = re.sub(r"\(.*\)", "", text)

    text = " ".join(text.split())

    return text


def text_similarity(sentence1, sentence2):
    """
    >>> s = similarity(
        "meeting those conditions ~hello teach", "meeting those conditions hello world!"
    )
    >>> print(s)
    >>> s = similarity("~hello   world", "hello world!")
    >>> print(s)
    """

    def norm(text):
        text = re.sub(r"\W", "", text, re.UNICODE)
        norm = " ".join(text.split())
        norm = norm.lower()
        return norm

    norm1 = norm(sentence1)
    norm2 = norm(sentence2)
    m = difflib.SequenceMatcher(None, norm1, norm2)
    return m.ratio()


def get_named_entities(text, model="en_core_web_trf", timeout=1):
    """Gets named entities in the text"""
    host = os.environ.get("SPACY_NLP_SERVER_HOST", "127.0.0.1")
    port = os.environ.get("SPACY_NLP_SERVER_PORT", "10100")
    url = f"http://{host}:{port}/process"

    params = {"articles": [{"text": text}], "model": model}
    try:
        response = requests.post(
            url,
            json=params,
            timeout=timeout,
        )
    except requests.exceptions.ReadTimeout as ex:
        logger.error(ex)
        return
    if response.status_code == 200:
        response = response.json()
        if "result" in response and response["result"]:
            return response["result"][0].get("ents", [])


def strip_xmltag(text):
    soup = BeautifulSoup(text, "html.parser")
    strings = list(soup.strings)
    notags = "".join(strings)
    notags = notags.strip()
    return notags


def get_attention(landmarks):
    """
    Calculates the attention angle based on facial landmarks.

    The attention angle is determined using the formula:
    angle = (|a.b| - |a.c|) / |a.a|
    where 'a' is the vector between the left and right eyes,
    'b' is the vector between the left eye and the nose,
    and 'c' is the vector between the right eye and the nose.

    Attention means the face is close and frontal.

    Example of landmarks:
    landmarks = {
        name: landmark
        for name, landmark in zip(
            person.body.landmarks_names, person.body.landmarks
        )
    }
    """
    NOSE = "nose"
    if "leftEye" in landmarks:
        LeftEye = "leftEye"
    elif "left_eye" in landmarks:
        LeftEye = "left_eye"
    else:
        return False
    if "rightEye" in landmarks:
        RightEye = "rightEye"
    elif "right_eye" in landmarks:
        RightEye = "right_eye"
    else:
        return False
    a = (
        landmarks[LeftEye].x - landmarks[RightEye].x,
        landmarks[LeftEye].y - landmarks[RightEye].y,
    )
    b = (
        landmarks[LeftEye].x - landmarks[NOSE].x,
        landmarks[LeftEye].y - landmarks[NOSE].y,
    )
    c = (
        landmarks[RightEye].x - landmarks[NOSE].x,
        landmarks[RightEye].y - landmarks[NOSE].y,
    )
    angle = (abs(a[0] * b[0] + a[1] * b[1]) - abs(a[0] * c[0] + a[1] * c[1])) / (
        a[0] * a[0] + a[1] * a[1]
    )
    size = a[0] * a[0] + a[1] * a[1]
    return abs(angle) < 0.3 and size > 800


def load_sheet_meta(arf_sheet_dir: str):
    """Loads event sheet meta data"""
    sheets = []
    arf_sheet_dir = Path(arf_sheet_dir)
    if arf_sheet_dir.is_dir():
        for sheet_file in arf_sheet_dir.glob("*.yaml"):
            with open(sheet_file) as f:
                sheet_metas = yaml.safe_load(f)
            for sheet_meta in sheet_metas:
                sheet = Sheet(**sheet_meta)
                sheets.append(sheet)
    return sheets


def get_llm(model_id: str, model_kwargs: dict) -> BaseChatModel:
    """
    Retrieve the language model based on the provided model ID and region name.

    Args:
        model_id (str): The identifier of the model to be retrieved.
        region_name (str): The AWS region name where the model is hosted.
        model_kwargs (dict): Additional keyword arguments to configure the model.

    Returns:
        ChatBedrock: An instance of the ChatBedrock class configured with the specified model.
    """
    default_region_name = "us-west-2"
    tokens_in_word = 1.4

    if model_id.startswith("aws-meta."):
        region_name = model_kwargs.pop("region_name", default_region_name)
        return ChatBedrock(
            model_id=model_id.split(".", 1)[1],
            model_kwargs=model_kwargs,
            region_name=region_name,
        )
    elif model_id.startswith("aws-anthropic."):
        region_name = model_kwargs.pop("region_name", default_region_name)
        retry_config = Config(
            region_name=region_name, retries={"max_attempts": 10, "mode": "standard"}
        )
        session = boto3.session.Session()
        boto3_bedrock_runtime = session.client("bedrock-runtime", config=retry_config)
        return ChatBedrock(
            client=boto3_bedrock_runtime,
            model_id=model_id.split(".", 1)[1],
            model_kwargs=model_kwargs,
            region_name=region_name,
        )
    elif model_id.startswith("openai."):
        return ChatOpenAI(
            model=model_id.split(".", 1)[1],
            temperature=model_kwargs.get("temperature", 0.6),
            max_tokens=int(model_kwargs.get("max_words", 100) * tokens_in_word),
            top_p=model_kwargs.get("top_p", 1),
            frequency_penalty=model_kwargs.get("frequency_penalty", 0),
            presence_penalty=model_kwargs.get("presence_penalty", 0),
            base_url=model_kwargs.pop("base_url", None),
        )
    elif model_id.startswith("anthropic."):
        return ChatAnthropic(
            model=model_id.split(".", 1)[1],
            temperature=model_kwargs.get("temperature", 0.6),
            max_tokens=int(model_kwargs.get("max_words", 100) * tokens_in_word),
            top_p=model_kwargs.get("top_p", 1),
        )
    else:
        raise ValueError(f"Unsupported model ID: {model_id}")


def load_agent_config() -> dict:
    """
    Load the agent configuration

    Returns:
        dict: The configuration data loaded from the agent config file.
    """
    envvar_yaml_loader()
    hr_chatbot_world_dir = os.environ.get("HR_CHATBOT_WORLD_DIR", "")
    agent_spec_file = os.path.join(hr_chatbot_world_dir, "agents.yaml")

    if not os.path.exists(agent_spec_file):
        raise FileNotFoundError(
            f"Agent specification file not found: {agent_spec_file}"
        )

    with open(agent_spec_file, "r") as f:
        config = yaml.safe_load(f)

    return config


def load_prompt_templates_config() -> dict:
    """
    Load the prompt templates configuration from the specified file.
    """
    prompt_templates_file = os.environ.get("HR_PROMPT_TEMPLATES_FILE")
    if not prompt_templates_file or not os.path.isfile(prompt_templates_file):
        raise FileNotFoundError(
            f"Prompt templates configuration file not found: {prompt_templates_file}"
        )

    with open(prompt_templates_file, "r") as f:
        config = yaml.safe_load(f)

    return config


def load_modes_config() -> dict:
    """
    Load the modes configuration from the specified file.

    Returns:
        dict: The configuration data for modes.
    """
    modes_file = os.environ.get("HR_MODES_FILE")
    if not modes_file or not os.path.isfile(modes_file):
        raise FileNotFoundError(f"Modes configuration file not found: {modes_file}")

    with open(modes_file, "r") as f:
        config = yaml.safe_load(f)

    version = config.get("version")
    if not isinstance(version, int) or not (1 <= version < 2):
        raise ValueError(f"Unsupported configuration version: {version}")

    return config.get("modes", {})


def set_n8n_enabled(enabled: bool):
    """Enable or disable n8n workflow.

    Args:
        enabled (bool): Whether to enable or disable the workflow
    """
    n8n_url = os.environ.get("N8N_URL")
    n8n_api_key = os.environ.get("N8N_API_KEY")
    if not n8n_url or not n8n_api_key:
        logger.error("N8N_URL or N8N_API_KEY is not set")
        return
    headers = {"X-N8N-API-KEY": n8n_api_key}
    endpoint = "activate" if enabled else "deactivate"
    try:
        response = requests.post(
            os.path.join(n8n_url, endpoint), headers=headers, timeout=5
        )
        if response.status_code != 200:
            logger.error("Failed to %s n8n workflow: %s", endpoint, response.json())
        else:
            logger.warning("n8n workflow %s", endpoint)
    except requests.exceptions.RequestException as e:
        logger.error("Failed to %s n8n workflow: %s", endpoint, str(e))


if __name__ == "__main__":
    text = r"""<prosody rate="0.94">Hola señor Gobernador.</prosody>"""
    print(strip_xmltag(text))
