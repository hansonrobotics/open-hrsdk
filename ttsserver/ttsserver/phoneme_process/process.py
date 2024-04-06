import logging
import tempfile
import os
import subprocess

import argparse
import json

PHONEME_MAPPING = {
    "A": "21",
    "B": "6",
    "C": "1",
    "D": "2",
    "E": "3",
    "F": "7",
    "G": "18",
    "H": "14",
    "X": "0",
}

logger = logging.getLogger("hr.ttsserver.phonemes_process.process")


def convert_visemes(input_file, output_file=None):
    with open(input_file) as f:
        data = json.load(f)
    phonemes = []
    for mouthCue in data["mouthCues"]:
        phoneme = {}
        phoneme["start"] = mouthCue["start"]
        phoneme["end"] = mouthCue["end"]
        phoneme["name"] = PHONEME_MAPPING[mouthCue["value"]]
        phoneme["type"] = "phoneme"
        phonemes.append(phoneme)
    if output_file:
        with open(output_file, "w") as f:
            f.write("\n".join([json.dumps(p) for p in phonemes]))
    return phonemes


def audio2phonemes(wave_file):
    cwd = os.path.dirname(os.path.abspath(__file__))
    temp_file_descriptor, temp_file_path = tempfile.mkstemp(suffix=".json", dir="/tmp")

    phonemes = []
    command = [
        "./rhubarb",
        "-f",
        "json",
        "-r",
        "phonetic",
        "-o",
        temp_file_path,
        wave_file,
    ]
    process = subprocess.Popen(command, stdout=subprocess.PIPE, cwd=cwd)
    out, err = process.communicate()

    if err:
        logger.error("Generating phonemes error %s", err.decode())
    else:
        logger.info("Generating phonemes %s", out.decode())
        phonemes = convert_visemes(temp_file_path)
        logger.info("Generated phonemes")

    os.close(temp_file_descriptor)
    os.unlink(temp_file_path)
    return phonemes


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-i", dest="input", required=True, nargs="+", help="input file name"
    )
    args = parser.parse_args()

    cwd = os.path.dirname(os.path.abspath(__file__))

    for wave_file in args.input:
        temp_file_descriptor, temp_file_path = tempfile.mkstemp(
            suffix=".json", dir="/tmp"
        )

        command = [
            "./rhubarb",
            "-f",
            "json",
            "-r",
            "phonetic",
            "-o",
            temp_file_path,
            wave_file,
        ]
        process = subprocess.Popen(command, stdout=subprocess.PIPE, cwd=cwd)
        out, err = process.communicate()

        if err:
            print(err.decode())
        else:
            print(out.decode())
            output = f"{os.path.splitext(wave_file)[0]}.timing"
            convert_visemes(temp_file_path, output)
            print(f"Generated timing file {output}\n")

        os.close(temp_file_descriptor)
        os.unlink(temp_file_path)
