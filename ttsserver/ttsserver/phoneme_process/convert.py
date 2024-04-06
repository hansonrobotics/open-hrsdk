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


def convert_visemes(input_file, output_file):
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
    with open(output_file, "w") as f:
        f.write("\n".join([json.dumps(p) for p in phonemes]))


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("-i", dest="input", required=True, help="input file name")
    parser.add_argument("-o", dest="output", required=True, help="output file name")
    args = parser.parse_args()

    convert_visemes(args.input, args.output)
