import hashlib
import json
import logging
import os
import re
import shutil
import subprocess
import uuid
import xml.etree.ElementTree as ET

import sox
from bs4 import BeautifulSoup
from slugify import slugify

from ttsserver.auto_emphasis import auto_emphasis

from .espp.emotivespeech import emotive_speech

logger = logging.getLogger("hr.ttsserver.ttsbase")

ILLEGAL_CHARS = re.compile(r"""[/]""")


def get_duration(audio_file):
    try:
        return sox.file_info.duration(audio_file)
    except Exception as ex:
        logger.error(ex)
    return 0.0


def wave_to_mp3(audio_file, output_file=None):
    """Converts wave audio file to mp3 audio file"""
    logger.info("Convert to mp3")
    if output_file is None:
        output_file = "%s.mp3" % os.path.splitext(audio_file)[0]
    tfm = sox.Transformer()
    tfm.build(audio_file, output_file)


def is_xml(text):
    if re.search(r"<.+>", text, re.UNICODE) is None:
        return False
    else:
        root = '<_root_ xmlns:amazon="www.amazon.com">{}</_root_>'.format(text)
        try:
            ET.fromstring(root.encode("utf-8"))
        except Exception:
            return False
        return True


def is_ssml(text):
    try:
        el = ET.fromstring(text)
        if el.tag == "speak" or "speak" in el.tag:  # with namespace
            return True
        else:
            return False
    except Exception:
        return False


def has_html_tag(text):
    return bool(re.search(r"<(\w+).*>.*</\1>", text))


def strip_xmltag(text):
    soup = BeautifulSoup(text, "html.parser")
    strings = list(soup.strings)
    notags = "".join(strings)
    notags = notags.strip()
    return notags


def wrap_speak_tag(text):
    """Wrap text in <speak></speak> to be legal SSML"""
    if text.startswith("<speak>"):
        return text
    try:
        el = ET.fromstring(text)
        if el.tag == "speak":
            return text
    except Exception:
        pass
    text = text.replace("<speak>", "")
    text = text.replace("</speak>", "")
    return "<speak>{}</speak>".format(text)


# User data class to store information
class TTSData:
    def __init__(self, text=None, wavout=None, format="wav"):
        self.text = text
        self.wavout = wavout
        self.format = format
        self.phonemes = []
        self.markers = []
        self.words = []
        self.id = ""
        self.raw_nodes = []

    def get_duration(self):
        return get_duration(self.wavout)

    def get_nodes(self):
        typeorder = {"marker": 1, "word": 2, "phoneme": 3}
        items = self.markers + self.words + self.phonemes
        items = sorted(items, key=lambda x: (x["start"], typeorder[x["type"]]))
        return items

    def __repr__(self):
        return "<TTSData wavout {}, text {}>".format(self.wavout, self.text)


class TTSException(Exception):
    def __init__(self, msg):
        self.msg = msg

    def __str__(self):
        return self.msg


class TTSBase(object):
    def __init__(self):
        self.output_dir = "."
        self.cache_dir = os.path.expanduser("{}/cache".format(self.output_dir))
        self.emo_cache_dir = os.path.expanduser("{}/emo_cache".format(self.output_dir))
        self.tts_params = {}
        self.voice_id = ""

    def set_output_dir(self, output_dir):
        self.output_dir = os.path.expanduser(output_dir)
        self.cache_dir = os.path.expanduser("{}/cache".format(self.output_dir))
        self.emo_cache_dir = os.path.expanduser("{}/emo_cache".format(self.output_dir))
        if not os.path.isdir(self.output_dir):
            os.makedirs(self.output_dir)
        if not os.path.isdir(self.cache_dir):
            os.makedirs(self.cache_dir)
        if not os.path.isdir(self.emo_cache_dir):
            os.makedirs(self.emo_cache_dir)

    def get_tts_params(self):
        return self.tts_params

    def set_tts_params(self, **params):
        if params:
            self.tts_params.update(params)

    def get_emo_cache_file(self, text, params):
        data = (
            text.lower() + str(self.voice_id) + str(self.get_tts_params()) + str(params)
        )
        data = data.encode("utf-8")
        hashcode = hashlib.sha1(data).hexdigest()[:40]
        filename = os.path.join(self.emo_cache_dir, hashcode + ".wav")
        return filename

    def get_tts_id(self, text, tts_params):
        if tts_params and "id" in tts_params:
            return tts_params["id"]
        text = text.replace("|p|", "").strip()  # remove |p| in the text
        data = text.lower() + str(self.voice_id) + str(tts_params)
        data = data.encode("utf-8")
        suffix = hashlib.sha1(data).hexdigest()[:6]
        text = strip_xmltag(text)
        if text:
            text = slugify(text, max_length=200)
        else:
            text = "<empty>"
        return text + "-" + suffix

    def save_to_cache(self, tts_data):
        if not os.path.isdir(self.cache_dir):
            os.makedirs(self.cache_dir)
        audio_file_cache = os.path.join(
            self.cache_dir, "%s.%s" % (tts_data.id, tts_data.format)
        )
        timing_cache_file = os.path.join(self.cache_dir, "%s.timing" % tts_data.id)
        shutil.copy(tts_data.wavout, audio_file_cache)
        tts_data.raw_nodes = tts_data.phonemes + tts_data.words + tts_data.markers
        tts_data.raw_nodes = sorted(
            tts_data.raw_nodes, key=lambda node: (node["start"], node["type"])
        )
        with open(timing_cache_file, "w") as f:
            for node in tts_data.raw_nodes:
                f.write(json.dumps(node))
                f.write("\n")

    def load_from_cache(self, tts_data):
        audio_file_cache = os.path.join(
            self.cache_dir, "%s.%s" % (tts_data.id, tts_data.format)
        )
        timing_cache_file = os.path.join(self.cache_dir, "%s.timing" % tts_data.id)
        if not os.path.exists(audio_file_cache) or not os.path.exists(
            timing_cache_file
        ):
            return False
        logger.info("Audio cache %r", audio_file_cache)
        logger.info("Timing info cache %r", timing_cache_file)
        try:
            shutil.copy(audio_file_cache, tts_data.wavout)
            tts_data.raw_nodes = []
            with open(timing_cache_file) as f:
                for line in f:
                    node = json.loads(line)
                    tts_data.raw_nodes.append(node)
        except Exception as ex:
            logger.error("Can't load tts cache. %s", ex)
            # clean false cache data
            os.unlink(audio_file_cache)
            os.unlink(timing_cache_file)
            return False

        tts_data.phonemes = [
            node for node in tts_data.raw_nodes if node["type"] == "phoneme"
        ]
        tts_data.markers = [
            node for node in tts_data.raw_nodes if node["type"] == "marker"
        ]
        tts_data.words = [node for node in tts_data.raw_nodes if node["type"] == "word"]
        return True

    def set_voice(self, voice):
        raise NotImplementedError("set_voice is not implemented")

    def do_tts(self, tts_data):
        raise NotImplementedError("do_tts is not implemented")

    def _adjust_phonemes_timing(self, phonemes, ratio):
        for p in phonemes:
            p["start"] = p["start"] * ratio
            p["end"] = p["end"] * ratio

    def tts(self, text, wavout=None, **kwargs):
        format = kwargs.get("format", "wav")
        if wavout is None:
            id = str(uuid.uuid1())
            wavout = os.path.join(self.output_dir, "%s.%s" % (id, format))
        if not is_ssml(text):
            text = auto_emphasis(text)
        tts_data = TTSData(text, wavout, format)
        tts_params = self.get_tts_params() or {}
        if kwargs:
            # do not override initial TTS params
            tts_params.update(**kwargs)
        tts_data.id = self.get_tts_id(text, tts_params)
        self.do_tts(tts_data)
        emotion = kwargs.get("emotion")
        if emotion is not None:
            orig_duration = tts_data.get_duration()
            cache_file = self.get_emo_cache_file(text, kwargs)
            try:
                ofile = "{}/emo_tmp.wav".format(os.path.dirname(tts_data.wavout))
                if os.path.isfile(cache_file):
                    shutil.copy(cache_file, ofile)
                    logger.info(
                        "Get cached emotive speech tts for {} {}".format(
                            text, cache_file
                        )
                    )
                else:
                    emotive_speech(tts_data.wavout, ofile, **kwargs)
                    shutil.copy(ofile, cache_file)
                shutil.move(ofile, tts_data.wavout)
                emo_duration = tts_data.get_duration()
                self._adjust_phonemes_timing(
                    tts_data.phonemes, emo_duration / orig_duration
                )
            except Exception as ex:
                logger.exception(ex)
        return tts_data

    def get_phonemes(self, nodes, time_factor=1000.0):
        phonemes = []
        visemes_in_nodes = [node for node in nodes if node["type"] == "viseme"]
        if not visemes_in_nodes:
            return []
        last_viseme = visemes_in_nodes[-1]
        for i, node in enumerate(visemes_in_nodes):
            phoneme = {"type": "phoneme"}
            phoneme["start"] = node["time"] / time_factor
            if i < len(visemes_in_nodes) - 1:
                phoneme["end"] = visemes_in_nodes[i + 1]["time"] / time_factor
            else:
                phoneme["end"] = (
                    last_viseme["time"] / time_factor
                )  # use the last node is the sil viseme node

            phoneme["name"] = node["value"]
            phonemes.append(phoneme)
        return phonemes

    def get_markers(self, nodes, time_factor=1000.0):
        markers = []
        markers_in_nodes = [node for node in nodes if node["type"] == "ssml"]
        for node in markers_in_nodes:
            marker = {"type": "marker"}
            marker["start"] = node["time"] / time_factor
            marker["name"] = node["value"]
            markers.append(marker)
        return markers

    def get_words(self, nodes, time_factor=1000.0):
        words = []
        visemes_in_nodes = [node for node in nodes if node["type"] == "viseme"]
        if not visemes_in_nodes:
            return []
        last_viseme = visemes_in_nodes[-1]
        words_in_nodes = [node for node in nodes if node["type"] == "word"]
        for i, node in enumerate(words_in_nodes):
            word = {"type": "word"}
            word["start"] = node["time"] / time_factor
            if i < len(words_in_nodes) - 1:
                word["end"] = words_in_nodes[i + 1]["time"] / time_factor
            else:
                word["end"] = (
                    last_viseme["time"] / time_factor
                )  # use the last node is the sil viseme node
            word["name"] = node["value"]
            word["word_start"] = node["start"]
            word["word_end"] = node["end"]
            words.append(word)
        return words


class ChineseTTSBase(TTSBase):
    def __init__(self):
        super(ChineseTTSBase, self).__init__()
        from ttsserver.phoneme_proxy import PollyPhonemeProxy

        self.phoneme_proxy = PollyPhonemeProxy("Zhiyu", self.cache_dir)

    def do_tts(self, tts_data):
        super(ChineseTTSBase, self).do_tts(tts_data)
        tts_data.phonemes = self.phoneme_proxy.get_phonemes(tts_data)
