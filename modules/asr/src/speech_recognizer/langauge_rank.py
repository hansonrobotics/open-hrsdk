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
class LanguageDecisionMaker:
    def __init__(self):
        self.cfg = None
        self.last_language = None
        self.english_characters_penalty = ['cmn-Hans-CN', 'yue-Hant-HK'] # detects english langauge words

    def percentage_of_alpha_chars(self, text):
        return sum(1 for char in text if 'A' <= char <= 'Z' or 'a' <= char <= 'z') / float(len(text))

    def decide_language(self, candidates):
        if not self.cfg:    
            return None, None, None
        # Determine the main language from the candidates
        main_language = None
        for candidate in candidates:
            if candidate.get('main', False):
                main_language = candidate['lang']
                break
        
        # If no main language is set, we assume no specific bias for main language
        if not main_language:
            main_language = ""

        # Apply confidence thresholds and biases
        filtered_candidates = []
        for candidate in candidates:
            lang = candidate['lang']
            confidence = candidate['confidence']

            # Apply main language bias if this language is the main one
            if lang == main_language:
                confidence += self.cfg.main_language_bias
            
            # Apply previous language bias if this language matches the last selected one
            if lang == self.last_language:
                confidence += self.cfg.previous_language_bias
            # punsih the English letter in chinese transcription:
            if lang in self.english_characters_penalty:
                if self.percentage_of_alpha_chars(candidate['transcript']) > 0.4:
                    confidence -= 0.3
            # Check minimum confidence thresholds
            if lang == main_language:
                if confidence >= self.cfg.min_confidence_for_main:
                    filtered_candidates.append((confidence, candidate))
            else:
                if confidence >= self.cfg.min_confidence_for_alternatives:
                    filtered_candidates.append((confidence, candidate))

        # Sort by adjusted confidence and pick the best candidate
        if filtered_candidates:
            filtered_candidates.sort(reverse=True, key=lambda x: x[0])
            best_candidate = filtered_candidates[0][1]
            self.last_language = best_candidate['lang']
            return best_candidate['transcript'], best_candidate['confidence'], best_candidate['lang']
        
        # No valid candidates after filtering
        return None, None, None
