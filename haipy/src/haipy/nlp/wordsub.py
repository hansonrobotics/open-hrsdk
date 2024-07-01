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
import re


class WordSub(dict):
    """All-in-one multiple-string-substitution class."""

    def _wordToRegex(self, word):
        """Convert a word to a regex object which matches the word."""
        if word != "" and word[0].isalpha() and word[-1].isalpha():
            return "\\b%s\\b" % re.escape(word)
        else:
            return r"\b%s\b" % re.escape(word)

    def _update_regex(self):
        """Build re object based on the keys of the current
        dictionary.

        """
        self._regex = re.compile(
            "|".join(map(self._wordToRegex, list(self.keys()))), flags=re.IGNORECASE
        )
        self._regexIsDirty = False

    def __init__(self, defaults={}):
        """Initialize the object, and populate it with the entries in
        the defaults dictionary.

        """
        self._regex = None
        self._regexIsDirty = True
        for k, v in list(defaults.items()):
            self[k] = v

    def __call__(self, match):
        """Handler invoked for each regex match."""
        matched = match.group(0)
        if matched in self:
            value = self[matched]
        elif matched.lower() in self:
            value = self[matched.lower()]
        if match.span()[0] == 0:
            value = value.title()
        return value

    def __setitem__(self, i, y):
        self._regexIsDirty = True
        super(WordSub, self).__setitem__(i, y)  # key = value
        super(WordSub, self).__setitem__(i.lower(), y)

    def sub(self, text):
        """Translate text, returns the modified text."""
        if self._regexIsDirty:
            self._update_regex()
        return self._regex.sub(self, text)


# TODO: this list is far from complete
defaultNormal = {
    "wanna": "want to",
    "gonna": "going to",
    "I'm": "I am",
    "I'd": "I would",
    "I'll": "I will",
    "I've": "I have",
    "you'd": "you would",
    "you're": "you are",
    "you've": "you have",
    "you'll": "you will",
    "he's": "he is",
    "he'd": "he would",
    "he'll": "he will",
    "she's": "she is",
    "she'd": "she would",
    "she'll": "she will",
    "we're": "we are",
    "we'd": "we would",
    "we'll": "we will",
    "we've": "we have",
    "they're": "they are",
    "they'd": "they would",
    "they'll": "they will",
    "they've": "they have",
    "y'all": "you all",
    "can't": "can not",
    "cannot": "can not",
    "couldn't": "could not",
    "wouldn't": "would not",
    "shouldn't": "should not",
    "isn't": "is not",
    "ain't": "is not",
    "don't": "do not",
    "aren't": "are not",
    "won't": "will not",
    "weren't": "were not",
    "wasn't": "was not",
    "didn't": "did not",
    "hasn't": "has not",
    "hadn't": "had not",
    "haven't": "have not",
    "where's": "where is",
    "where'd": "where did",
    "where'll": "where will",
    "who's": "who is",
    "who'd": "who did",
    "who'll": "who will",
    "what's": "what is",
    "what'd": "what did",
    "what'll": "what will",
    "when's": "when is",
    "when'd": "when did",
    "when'll": "when will",
    "why's": "why is",
    "why'd": "why did",
    "why'll": "why will",
    "it's": "it is",
    "it'd": "it would",
    "it'll": "it will",
}

first2second = {
    "I": "you",
    "me": "you",
    "my": "your",
    "mine": "yours",
    "myself": "yourself",
    "am": "are",
    "I was": "you were",
}


def text2priming(text, subbers=[]):
    """Convert text to priming text

    Parameters
    text: the original text
    subbers: list of word subs
    """
    sentences = []
    sentence_text = ""
    sentence_mark = ""
    # split to sentences
    for chunk in re.split(r"([.!?])", text):
        chunk = chunk.strip()
        if chunk not in [".", "!", "?"]:
            sentence_text = chunk
        else:
            sentence_mark = chunk
        if sentence_mark:
            sentences.append(f"{sentence_text}{sentence_mark}")
            sentence_text = ""
            sentence_mark = ""
    # no punctuation marks
    if not sentences and sentence_text:
        sentences.append(sentence_text)

    primings = []
    for sentence in sentences:
        sentence = sentence.strip()
        if not sentence:
            continue
        tokens = [token.lower() for token in sentence.split()]
        if (
            "you" in tokens
            or "your" in tokens
            or "yourself" in tokens
            or "yours" in tokens
        ):
            # ignore sentence contains you
            continue
        priming = sentence
        for sub in subbers:
            priming = sub.sub(priming)
        priming = priming.strip()
        primings.append(priming)
    return " ".join(primings)
