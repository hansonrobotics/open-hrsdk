#!/usr/bin/python
# -*- coding: utf-8 -*-
import numpy as np
from . import preprocess as prep
from . import batchprocess as bp
from . import synthesis as syn
import os
import sys
import argparse
from optparse import OptionParser

DEFAULT_PARAMS = {
    "chunk_size": 1024,
    "semitones": 1.0,
    "cutfreq": 4000,
    "gain": 3.0,
    "qfactor": 1.0,
    "speed": 8.5,
    "depth": 60,
    "tempo": 1.0,
    "intensity": 3.0,
    "parameter_control": 1.0,
}

PRESET_EMO_PARAMS = {
    "happy": {
        "semitones": 1.5,
        "tempo": 1.1,
    },
    "sad": {
        "semitones": -1.5,
        "gain": 0.25,
        "cutfreq": 3500.0,
        "tempo": 0.95,
    },
    "happy_tensed": {
        "semitones": 2.0,
        "tempo": 1.18,
    },
    "afraid": {"tempo": 1.05},
}


def emotive_speech(fname, ofile, emotion, **kwargs):
    """
    A Caller Module
    Parameter:  fname
                ofile
                emotion
    Returns: output
    """
    params = {}
    params.update(DEFAULT_PARAMS)
    params.update(kwargs)
    chunk_size = int(params["chunk_size"])
    semitones = float(params["semitones"])
    cutfreq = float(params["cutfreq"])
    gain = float(params["gain"])
    qfactor = float(params["qfactor"])
    speed = float(params["speed"])
    depth = float(params["depth"])
    tempo = float(params["tempo"])
    intensity = float(params["intensity"])
    parameter_control = float(params["parameter_control"])

    fs, x = prep.wave_file_read(fname)
    time_stamps = bp.process_variables(x, fs, chunk_size)[0]
    consecutive_blocks = bp.process_variables(x, fs, chunk_size)[1]
    fundamental_frequency_in_blocks = bp.batch_analysis(x, fs, chunk_size)[0]
    voiced_samples = bp.batch_analysis(x, fs, chunk_size)[1]
    rms = bp.batch_analysis(x, fs, chunk_size)[2]
    selected_inflect_block = bp.batch_preprocess(
        fundamental_frequency_in_blocks, voiced_samples, rms
    )
    output = bp.batch_synthesis(
        fs,
        consecutive_blocks,
        time_stamps,
        selected_inflect_block,
        emotion,
        semitones,
        cutfreq,
        gain,
        qfactor,
        speed,
        depth,
        tempo,
        intensity,
        parameter_control,
    )
    output.build(fname, ofile)
    return output


if __name__ == "__main__":

    parser = argparse.ArgumentParser(
        description="Emotive Speech Generation",
        formatter_class=argparse.RawTextHelpFormatter,
    )

    parser.add_argument("f", metavar="filename", help="Absolute wavfile Directory")

    parser.add_argument("emotion", choices=list(PRESET_EMO_PARAMS.keys()))

    parser.add_argument(
        "-c",
        metavar="chunk_size",
        type=int,
        required=False,
        help="""Chunk Size:
        Default Value=1024
        [...,256,512,1024,2048,4096...,]
        """,
    )

    parser.add_argument(
        "-s",
        metavar="semitones",
        type=float,
        required=False,
        help="""Semitones
        Default Values(H=1.5,HT=2.0,S=-1.5)
        """,
    )

    parser.add_argument(
        "-r",
        metavar="cutfreq",
        type=float,
        required=False,
        help="""Cut-Frequency
        """,
    )

    parser.add_argument(
        "-g",
        metavar="gain",
        type=float,
        required=False,
        help="""Gain for Treble
        """,
    )

    parser.add_argument(
        "-q",
        metavar="qfactor",
        type=float,
        required=False,
        help="""Q-factor
        """,
    )

    parser.add_argument(
        "-v",
        metavar="speed",
        type=float,
        required=False,
        help="""Tremelo Speed
        Default Value(speed=8.5)
        WORKS ONLY FOR AFRAID!
        """,
    )

    parser.add_argument(
        "-d",
        metavar="depth",
        type=float,
        required=False,
        help="""Tremelo Depth
        Default Value(depth=60)
        WORKS ONLY FOR AFRAID!
        """,
    )

    parser.add_argument(
        "-o",
        metavar="tempo",
        type=float,
        required=False,
        help="""Tempo
        Default Values(S=0.95,A=1.05,H=1.1,HT=1.18
        """,
    )

    parser.add_argument(
        "-i",
        metavar="intensity",
        type=float,
        required=False,
        help="""Gain for Intensity
        Default Value(3.0db)
        DOESN'T WORK FOR SAD PATCH!
        """,
    )

    parser.add_argument(
        "-p",
        metavar="parameter_control",
        type=float,
        required=False,
        help="Parameter Control",
    )

    args = parser.parse_args()

    fname = args.f
    emotion = args.emotion
    output_dir = os.path.join(os.path.dirname(fname), emotion)
    ofile = os.path.join(output_dir, os.path.basename(fname))
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)

    profile = os.path.join(output_dir, "profile.txt")
    file = open(profile, "w")
    file.write(
        "Args-->  "
        + "\n"
        + str(args)
        + "\n"
        + "\n"
        + "See `python emotivespeech.py -h` for help  "
    )
    file.close()

    emotive_speech(fname, ofile, emotion, **PRESET_EMO_PARAMS[emotion])
