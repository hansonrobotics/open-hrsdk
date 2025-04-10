# -*- coding: utf-8 -*-
#
# Licensed under the GNU LGPL v2.1 - http://www.gnu.org/licenses/lgpl.html
"""This module contains function of computing rank scores for documents in
corpus and helper class `BM25` used in calculations. Original algorithm
descibed in [1]_, also you may check Wikipedia page [2]_.


.. [1] Robertson, Stephen; Zaragoza, Hugo (2009).  The Probabilistic Relevance Framework: BM25 and Beyond,
       http://www.staff.city.ac.uk/~sb317/papers/foundations_bm25_review.pdf
.. [2] Okapi BM25 on Wikipedia, https://en.wikipedia.org/wiki/Okapi_BM25



Examples
--------

.. sourcecode:: pycon

    >>> corpus = [
    ...     ["black", "cat", "white", "cat"],
    ...     ["cat", "outer", "space"],
    ...     ["wag", "dog"]
    ... ]
    >>> result = get_bm25_weights(corpus)


Data:
-----
.. data:: PARAM_K1 - Free smoothing parameter for BM25.
.. data:: PARAM_B - Free smoothing parameter for BM25.
.. data:: EPSILON - Constant used for negative idf of document in corpus.

"""
import logging
import math

# https://www.elastic.co/blog/practical-bm25-part-3-considerations-for-picking-b-and-k1-in-elasticsearch
PARAM_K1 = 1.2
PARAM_B = 0.75
EPSILON = 0.25
logger = logging.getLogger(__name__)


class BM25(object):
    """Implementation of Best Matching 25 ranking function.

    Attributes
    ----------
    corpus_size : int
        Size of corpus (number of documents).
    avgdl : float
        Average length of document in `corpus`.
    doc_freqs : list of dicts of int
        Dictionary with terms frequencies for each document in `corpus`. Words used as keys and frequencies as values.
    idf : dict
        Dictionary with inversed documents frequencies for whole `corpus`. Words used as keys and frequencies as values.
    doc_len : list of int
        List of document lengths.
    """

    def __init__(self, docs):
        """
        Parameters
        ----------
        docs: dict of document of corpus
            Given docs.

        """
        index_names = list(docs.keys())
        values = list(docs.values())
        # TODO: tokenize
        corpus = [sum([v.split() for v in value], []) for value in values]
        self._index_names = index_names
        self.corpus_size = len(corpus)
        self.avgdl = 0
        self.doc_freqs = []
        self.idf = {}
        self.doc_len = []
        self._initialize(corpus)

    def _initialize(self, corpus):
        """Calculates frequencies of terms in documents and in corpus. Also computes inverse document frequencies."""
        if not corpus:
            logger.warning("Empty corpus")
            return
        nd = {}  # word -> number of documents with word
        num_doc = 0
        for document in corpus:
            self.doc_len.append(len(document))
            num_doc += len(document)

            frequencies = {}
            for word in document:
                if word not in frequencies:
                    frequencies[word] = 0
                frequencies[word] += 1
            self.doc_freqs.append(frequencies)

            for word, freq in frequencies.items():
                if word not in nd:
                    nd[word] = 0
                nd[word] += 1

        self.avgdl = float(num_doc) / self.corpus_size
        # collect idf sum to calculate an average idf for epsilon value
        idf_sum = 0
        # collect words with negative idf to set them a special epsilon value.
        # idf can be negative if word is contained in more than half of
        # documents
        negative_idfs = []
        for word, freq in nd.items():
            idf = math.log(self.corpus_size - freq + 0.5) - math.log(freq + 0.5)
            self.idf[word] = idf
            idf_sum += idf
            if idf < 0:
                negative_idfs.append(word)
        self.average_idf = float(idf_sum) / len(self.idf)

        eps = EPSILON * self.average_idf
        for word in negative_idfs:
            self.idf[word] = eps

    def get_score(self, document, index):
        """Computes BM25 score of given `document` in relation to item of corpus selected by `index`.

        Parameters
        ----------
        document : list of str
            Document to be scored.
        index : int
            Index of document in corpus selected to score with `document`.

        Returns
        -------
        float
            BM25 score.

        """
        score = 0
        doc_freqs = self.doc_freqs[index]
        for word in document:
            if word not in doc_freqs:
                continue
            score += (
                self.idf[word]
                * doc_freqs[word]
                * (PARAM_K1 + 1)
                / (
                    doc_freqs[word]
                    + PARAM_K1
                    * (1 - PARAM_B + PARAM_B * self.doc_len[index] / self.avgdl)
                )
            )
        return score

    def get_scores(self, document):
        """Computes and returns BM25 scores of given `document` in relation to
        every item in corpus.

        Parameters
        ----------
        document : list of str
            Document to be scored.

        Returns
        -------
        list of dict
            Each contains a doc name and BM25 score.

        """
        scores = []
        for index in range(self.corpus_size):
            score = self.get_score(document, index)
            name = self._index_names[index]
            if score > 0:
                name = self._index_names[index]
                scores.append({"id": name, "score": score})
        scores = sorted(scores, key=lambda x: x["score"], reverse=True)
        return scores


if __name__ == "__main__":
    import pandas as pd

    df = pd.read_csv(
        "data/blenderbot_training_corpora/finetuning2/finetuning_train.csv"
    )
    corpus = {"doc%s" % i: row.text.split() for i, row in df.iterrows()}
    bm25 = BM25(corpus)
    while True:
        question = input("question: ")
        results = bm25.get_scores(question.split())[:3]
        if results:
            print("results:", results)
            print(corpus[results[0]["id"]])
