import logging
import shutil
from collections import defaultdict
import os
import sys

CWD = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, os.path.join(CWD, ".."))

from ttsserver.ttsbase import TTSBase
from ttsserver.phoneme_process.process import audio2phonemes

logger = logging.getLogger("hr.ttsserver.voices.singing")


class SingingTTS(TTSBase):
    def __init__(self, voice):
        super(SingingTTS, self).__init__()
        self.voice = voice
        self.voice_id = "singing:%s" % self.voice

    def do_tts(self, tts_data):
        self.set_tts_params(id=tts_data.text)
        tts_data.id = tts_data.text
        self.tts_data = tts_data
        loaded = self.load_from_cache(self.tts_data)
        if not loaded:
            audio_file_cache = os.path.join(
                self.cache_dir, "%s.%s" % (tts_data.id, tts_data.format)
            )
            shutil.copy(audio_file_cache, tts_data.wavout)
            tts_data.phonemes = audio2phonemes(tts_data.wavout)
            self.save_to_cache(self.tts_data)
            logger.info("Saved to cache")


def load_voices():
    voices = defaultdict(dict)
    voices["singing"]["singing"] = SingingTTS("singing")
    logger.info("Added voices: %s" % ", ".join(list(voices["singing"].keys())))
    return voices


voices = load_voices()

if __name__ == "__main__":
    logging.basicConfig()
    api = SingingTTS()
    api.set_output_dir(CWD)
    import os

    api.tts("0001-hello-everybody-i-am-sophia-921b42")
