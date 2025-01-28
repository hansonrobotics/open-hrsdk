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
from queue import Queue
from typing import List

from google.cloud.speech import StreamingRecognitionResult
from pydantic import BaseModel

from speech_recognizer.audio import Audio


class TopResult(BaseModel):
    """
    Top result of a speech recognition results.

    Attributes:
    is_final (bool): Indicates if the result is final.
    transcript (str): The transcribed text from the speech.
    stability (float): The stability of the recognition result.
    language (str): The language of the recognized speech.
    """

    is_final: bool
    transcript: str
    stability: float
    language: str


# Mapping of language codes to standardized format
LANGUAGE_CODE_MAPPING = {
    "ar-x-gulf": "ar-AE",
    "ar-ae": "ar-AE",
    "ar-sa": "ar-SA",
    "cmn-hans-cn": "cmn-Hans-CN",
    "cs-cz": "cs-CZ",
    "de-de": "de-DE",
    "el-gr": "el-GR",
    "en-us": "en-US",
    "en-us": "en-US",
    "es-es": "es-ES",
    "fr-fr": "fr-FR",
    "hi-in": "hi-IN",
    "hu-hu": "hu-HU",
    "it-it": "it-IT",
    "ja-jp": "ja-JP",
    "ko-kr": "ko-KR",
    "nb-no": "nb-NO",
    "nl-nl": "nl-NL",
    "pl-pl": "pl-PL",
    "pt-br": "pt-BR",
    "ru-ru": "ru-RU",
    "tr-tr": "tr-TR",
    "yue-hant-hk": "yue-Hant-HK",
}


def format_language_code(language_code: str) -> str:
    """
    Format the given language code to a standardized format.

    Parameters:
    language_code (str): The language code to format.

    Returns:
    str: The formatted language code.
    """
    if language_code in LANGUAGE_CODE_MAPPING:
        return LANGUAGE_CODE_MAPPING.get(language_code)
    elif "-" in language_code:
        return "-".join(
            [
                part if i == 0 else part.upper()
                for i, part in enumerate(language_code.split("-"))
            ]
        )
    else:
        return language_code


def save_audio_data(
    audio_home_dir: str, audio_queue: Queue, language: str, audio_rate: int
) -> str:
    """
    Save audio data from the audio queue to a file.

    Parameters:
    audio_home_dir (str): The directory where the audio file will be saved.
    audio_queue (Queue): The queue containing audio data.
    language (str): The language code of the audio.
    audio_rate (int): The audio sample rate.

    Returns:
    str: The path to the saved audio file.
    """
    audio_data = b""
    while not audio_queue.empty():
        audio_data += audio_queue.get()

    return Audio(home_dir=audio_home_dir, lang=language).save(audio_data, audio_rate)


def verify_credential() -> bool:
    file_path = os.environ.get("GOOGLE_APPLICATION_CREDENTIALS")
    if file_path and os.path.isfile(file_path) and os.stat(file_path).st_size != 0:
        return True
    return False


def extract_top_recognition_result(
    results: List[StreamingRecognitionResult],
) -> TopResult:
    top_result = sorted(results, key=lambda r: r.stability, reverse=True)[0]
    top_alternative = top_result.alternatives[0]
    transcript = top_alternative.transcript
    stability = top_result.stability
    language_code = top_result.language_code
    language = format_language_code(language_code)

    return TopResult(
        is_final=top_result.is_final,
        transcript=transcript,
        stability=stability,
        language=language,
    )
