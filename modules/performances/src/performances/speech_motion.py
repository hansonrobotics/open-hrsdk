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

import os
import yaml
import requests
import random
import re
import logging
from copy import deepcopy
import random

logger = logging.getLogger('hr')
if not logger.handlers:
    logging.basicConfig()

class SpeechMotionUtils:
    SENTENCE_DELIMITERS = '(?<!\d)[\.\?\!](?!\d)'

    @staticmethod
    def match_NLU_to_text_words(text_words, tokens):
        try:
            if len(text_words) < 1:
                return []
            matched = [[] for i in range(len(text_words))]
            j = 0 # Current NLU word
            for i,w in enumerate(text_words):
                while j < len(tokens):
                    t = SpeechMotionUtils.split_text(tokens[j]['text'], True)
                    # non verbal token
                    if len(t) == 0:
                        j += 1
                        continue
                    t = t[0]
                    # Most common case: words matching
                    if t == w:
                        matched[i] = [deepcopy(tokens[j])]
                        j += 1
                        break
                    # token i part of word
                    if t in w:
                        matched[i].append(deepcopy(tokens[j]))
                        j += 1
                        continue
                    # Exception
                    break
            for i, m in enumerate(matched):
                if len(m) == 0:
                    #logger.error("Cant match all NLU words for {}".format(text_words))
                    return matched
        except Exception as e:
            #logger.error("Exception {} matching {}".format(e,text_words))
            pass
        return matched

    @staticmethod
    def create_speech_data_from_tts_data(ttsdata, offset=0):
        text, text_words, words = SpeechMotionUtils.match_tts(ttsdata)
        if not text:
            return
        try:
            sentences = SpeechMotionUtils.split_text(text['name'])
            current_sentence_start = 0
            current_word = 0
            nodes = []
            for i in range(0,len(sentences)):
                s = sentences[i]
                s_words = SpeechMotionUtils.split_text(s, True)
                tts_words = deepcopy(words[current_word:current_word + len(s_words)])
                try:
                    # Try find end time by last word matched
                    end_time = tts_words[-1]['end']
                except:
                    # last
                    if i == len(sentences)-1:
                        end_time = text['duration']
                    else:
                        # try get next sentence first word time
                        try:
                            end_time = words[current_word+len(s_words)]['start']-0.01
                        except:
                            # merge the to next sentence at worst case.
                            sentences[i+1] = s + ' ' + sentences[i+1]
                            continue
                sentence_tts_data = deepcopy([i for i in ttsdata if current_sentence_start <= i['start'] <= end_time])
                for st in sentence_tts_data:
                    st['start'] -= current_sentence_start
                    st['end'] -= current_sentence_start
                for w in tts_words:
                    if w is not None:
                        w['start'] -= current_sentence_start
                        w['end'] -= current_sentence_start
                nodes.append(
                    SpeechMotionUtils.create_node_data(name='speech', duration=end_time - current_sentence_start, text=s,
                                                start_time=current_sentence_start,
                                                tts_data=sentence_tts_data, words=tts_words, text_words=s_words))
                current_word += len(s_words)
                current_sentence_start = end_time
            return nodes
        except:
            logger.error("Word Missmatch {}".format(text['name']))
            return []

    @staticmethod
    def create_node_data(**kwargs):
        data = {
            'name': type,
            'start_time': 0.0,
            'duration': 0.0,
        }
        if kwargs.get('name') == 'speech':
            data['lang'] = 'en-US'
        data.update(kwargs)
        return data

    @staticmethod
    def split_text(text, to_words=False):
        # splits text to words, or sentences
        text = text.replace(u'’',"'")
        text = re.sub(r'\|.*?\|','', text)
        try:
            text = str(text)
        except:
            pass
        delimiters = SpeechMotionUtils.SENTENCE_DELIMITERS
        s = re.split(delimiters, text)
        if to_words:
            # making sure
            txt = " ".join(s)
            s = re.findall(r'[\w.\']+',txt)
        words = [w.lower().strip("?!.;-'+ \r\n\t") for w in s if w and w.strip("?!.;-'+ \r\n\t")]
        return words

    @staticmethod
    def match_tts(tts_data, s=None):
        # If sentence not provided find it in TTS data
        if s is None:
            for i in tts_data:
                if i['type'] == 'text':
                    s = i['name']
        try:
            s = str(s)
        except:
            pass
        text_words = SpeechMotionUtils.split_text(s, True)
        words = [None] * len(text_words)
        j = 0
        text = None
        for ii in tts_data:
            if ii['type'] == 'text':
                text = ii
            if ii['type'] == 'word':
                # need to minimize look ahead,
                for jj in range(j, min(j+5,len(words))):
                    if ii['name'] == text_words[jj]:
                        j = jj + 1
                        words[jj] = ii
                        break
        for iw, w in enumerate(words):
            if w is None:
                logger.info(u'Cant find TTS dor word {}'.format(text_words[iw]))
        return text, text_words, words

class AnimationActionLibrary:

    def __init__(self, library):
        # Load from other storage later
        self.library = library


    def pick_animation(self, aa, max_length=0):
        try:
            actions = self.library[aa]
            if max_length > 0:
                actions = [a for a in actions if a['duration'] > 0]
            animation = deepcopy(random.choice(actions))
        except:
            return None
        return animation


class SpeechMotionController:

    def __init__(self, animation_library, nlu_server='http://127.0.0.1:8210/da', lipsync_delay=0.1, arms_main=False, min_arms_hold_time = 3, max_arms_hold_time = 5, animated_shoulders = False):
        self.lipsync_delay = lipsync_delay
        self.nlu_cache = {}
        self.nlu_timeout = 2
        self.keyword_rules = []
        self.da_rules = []
        self.library = AnimationActionLibrary(animation_library)
        self.probability_modifier = 0  # 0 unchanged, -1 none rules applied and 1 all rules applied
        self.min_arms_hold_time = min_arms_hold_time
        self.max_arms_hold_time = max_arms_hold_time
        self.animated_arms = True
        self.animated_shoulders = animated_shoulders
        self.nlu_server = nlu_server
        self.skip_keyword_rules = False
        pass

    def arms_interval(self):
        return random.uniform(self.min_arms_hold_time, self.max_arms_hold_time)

    # Currently timeline format with TTS nodes are supported. All other nodes will be left untouched.
    def animate_motion(self, motion, animate_arms=True):
        animated = []
        arms = self.animated_arms
        self.animated_arms = animate_arms
        self.motion_length = 0
        for n in motion:
            if n['name'] == 'speech':
                animated += self.animate_speech_node(n)
            if n.get('start_time', 0) + n.get('duration', 0) > self.motion_length:
                self.motion_length =  n.get('start_time', 0) + n.get('duration', 0)
        # Restore original setting
        self.animated_arms = arms
        return self.filter_animations(animated)

    def apply_probability_modifier(self, probability):
        return probability + probability * self.probability_modifier if self.probability_modifier < 0 \
            else probability + (1 - probability) * self.probability_modifier

    def neutral_arms(self, time):
        return SpeechMotionUtils.create_node_data(name='arm_animation', duration=1, speed=1, magnitude=1, start_time=time, arm_animation='MAIN-2')

    def filter_animations(self, animated):
        # Needs to shuffle, so its not dependant on rule order
        if len(animated) == 0:
            return animated
        random.shuffle(animated)
        prioritized = sorted(animated, key=lambda k: 0 if 'sc_meta' not in k.keys() else k['sc_meta']['priority'],
                             reverse=True)
        i = 0
        arm_animations = []

        # Remove conflicting gestures by priority on channel
        while i < len(prioritized):
            if 'sc_meta' in prioritized[i].keys():
                try:
                    channels = prioritized[i]['sc_meta']['channel']
                    # Gesture begin time
                    block_time = [prioritized[i]['sc_meta']['begin'] + prioritized[i]['start_time'],
                                  prioritized[i]['sc_meta']['end'] + prioritized[i]['start_time'],
                                  ]
                    j = i + 1
                    while j < len(prioritized):
                        try:
                            if len(set(channels).intersection(prioritized[j]['sc_meta']['channel'])) > 0:
                                # Same channel, so check if any lower priority animations are interupting
                                if max(block_time[0], prioritized[j]['start_time']) <= \
                                        min(block_time[1], prioritized[j]['start_time'] + prioritized[j]['duration']):
                                    prioritized.pop(j)
                                    continue
                            j += 1
                        except:
                            j += 1
                except Exception as e:
                    logger.error("Filter animation error {}".format(e))
            # capture all arm animation ending times, in case need to reset to neutral pose
            if prioritized[i].get('name','') == 'arm_animation':
                arm_animations.append(prioritized[i])
            # update length of motion if it exceeds previous
            if prioritized[i]['start_time'] + prioritized[i]['duration'] > self.motion_length:
                self.motion_length = prioritized[i]['start_time'] + prioritized[i]['duration']
            i += 1
        # Need to reset to neutral after 5 seconds or so
        if len(arm_animations) > 0:
            #Sort by start time
            arm_animations = sorted(arm_animations, key=lambda k: k['start_time'])
            prev = {}
            start_with_main = False
            for a in arm_animations:
                curr_name = a['arm_animation']
                if not start_with_main:
                    # Check for first arm animation, and if its not MAIN it together it will neutralize the psoe
                    start_with_main = True
                    if not 'MAIN' in curr_name:
                        prioritized.append(self.neutral_arms(max(0,a['start_time']-0.5)))

                prev_name = prev.get('arm_animation', '')
                # Return to neutral pose for main animations only
                if 'MAIN' in prev_name:
                   # If there is transition in MAIN, he neutral should be added only if othe MAIN starts after more than max hold time
                    if 'MAIN' in curr_name:
                        if prev['start_time'] + prev['duration'] + self.max_arms_hold_time < a['start_time']:
                            # Neutralize arms after interval
                            prioritized.append(self.neutral_arms(prev['start_time'] + prev['duration'] + self.arms_interval()))
                    # If next animation not a MAIN then always return to neutral, either at a time of the animation or (3-5secs later) or then the next animation starts
                    else:
                        prioritized.append(self.neutral_arms(min(prev['start_time'] + prev['duration'] + self.arms_interval(), a['start_time'])))
                prev = a
            if 'MAIN' in prev.get('arm_animation', ''):
                prioritized.append(self.neutral_arms(min(prev['start_time'] + prev['duration'] + self.arms_interval(), self.motion_length)))

        return prioritized

    # returns update speech node merged with animations
    def animate_speech_node(self, node):
        try:
            if not node.get('NLUData', False):
                self.get_nlu_data(node)
        except Exception as e:
            logger.exception("NLU data exception {}".format(e))

        added = []
        if not self.skip_keyword_rules:
            added += self.apply_keyword_rules(node)
        added += self.apply_da_rules(node)

        # # Add the speech for visualization
        # added.append(Node.create_empty_node_data(type='speech', duration=node['duration'],
        #                                          text=node['text'], lang='NONE', start_time=node['start_time']))
        return added

    # tries get NLUData, Should execute on another thread with timeout
    def get_nlu_data(self, node):
        r = requests.get(self.nlu_server, {'language': 'en-US', 'text': node['text']})
        node['nlu_data'] = yaml.safe_load(r.text)
        node['nlu_data']['word_tokens'] = SpeechMotionUtils.match_NLU_to_text_words(node['text_words'],
                                                                                    node['nlu_data']['tokens'])
        lemma = [(j, i['lemma'], i['pos']) for j,t  in enumerate(node['nlu_data']['word_tokens']) for i in t]
        node['nlu_data']['words_index'] = [x[0] for x in lemma]
        node['nlu_data']['lemma_words'] = [x[1] for x in lemma]
        node['nlu_data']['pos_words'] = [x[2] for x in lemma]

    def update_keyword_rules(self, rules):
        sorted_rules = sorted(rules, key=lambda k: k.get('priority',0))
        for r in sorted_rules:
            r['words'] = [SpeechMotionUtils.split_text(k.strip(), to_words=True) for k in r['keywords'].split(',') if
                          len(k.strip()) > 0]
            r['index_words'] = [str(w.encode('ascii', 'ignore').decode()) for l in r['words'] for w in l]
        self.keyword_rules = sorted_rules

    def get_keyword_rules(self):
        return self.keyword_rules

    def apply_keyword_rules(self, node):
        added = []
        for rule in self.keyword_rules:
            try:
                added += self.process_keyword_rule(node, rule)
            except Exception as e:
                #logger.error("ERROR processing rule {} ith exception {}".format(rule, e))
                pass
        return added

    @staticmethod
    def subset(words, sentence):
        matches = [sentence[i:i + len(words)] == words for i in range(0, len(sentence) - len(words) + 1)]
        try:
            return matches.index(True)+len(words)-1
        except:
            return False

    @staticmethod
    def negative_word_around(node, w):
        # skip rules if they have negative words: no, not, can't don't
        negatives = ['no', 'not', 'don\'t', 'can\'t']
        if w > 0:
            if node['text_words'][w - 1] in negatives:
                return True
        if w + 1 < len(node['words']):
            if node['text_words'][w + 1] in negatives:
                return True
        return False


    def process_keyword_rule(self, node, rule):

        nodes = []
        pos = rule.get('pos', False)
        lemma = rule.get('lemma', False)
        if pos and lemma:
            # Fast check
            matched = set(node['nlu_data']['lemma_words']).intersection(rule['index_words'])
            if not matched:
                return nodes
            for word in matched:
                try:
                    lw = node['nlu_data']['lemma_words'].index(word)
                    w = node['nlu_data']['words_index'][lw]
                    if self.negative_word_around(node, w):
                        continue
                    nodes += self.apply_rule(rule, node, node['words'][w])
                except Exception as e:
                    logger.error("Error {} processing lemmas {} in  {}".format(e, w, node['text_words']))

        # Check if there any overlapping words, for fast checking
        if not set(node['text_words']).intersection(rule['index_words']):
            return nodes
        for phrase in rule['words']:
            w = self.subset(phrase, node['text_words'])
            if w is not False:
                if self.negative_word_around(node, w):
                    continue

                if pos:
                    skip = True
                    # Need to match single POS for any of the phrase words
                    for i in range (w-len(phrase)+1, w+1):
                        for t in node['nlu_data']['word_tokens'][i]:
                            if t['pos'] == pos:
                                skip = False
                    if skip:
                        continue
                try:
                    nodes += self.apply_rule(rule, node, node['words'][w])
                except:
                    logger.info("Word {} {} failed.".format(w, node['words'][w]))
        return nodes


    def apply_rule(self, rule, node, word, ):
        # Adds animations to rule KW and DA rule
        nodes = []
        if self.apply_probability_modifier(rule['probability']) > random.random():
            # Add keyword gestures
            animation = self.library.pick_animation(rule.get('browAA','-'))
            if animation:
                nodes.append(self.create_gesture(animation, node, word, rule))
            animation = self.library.pick_animation(rule.get('headAA','-'))
            if animation:
                nodes.append(self.create_gesture(animation, node, word, rule))
            animation = self.library.pick_animation(rule.get('eyesAA','-'))
            if animation:
                nodes.append(self.create_gesture(animation, node, word, rule))
            animation = self.library.pick_animation(rule.get('eyelidsAA', 'none'))
            if animation:
                nodes.append(self.create_gesture(animation, node, word, rule))
            if self.animated_arms:
                animation = self.library.pick_animation(rule.get('armsAA','-'))
                if animation:
                    nodes.append(self.create_gesture(animation, node, word, rule, head_gestrue=False))
            if self.animated_shoulders:
                animation = self.library.pick_animation(rule.get('shouldersAA','-'))
                if animation:
                    nodes.append(self.create_gesture(animation, node, word, rule, head_gestrue=False))

            # Special case for procedural animations:
            # if rule['headAA'] == 'head-tilt':
            #     nodes += self.create_head_tilt(node, word, rule)
        return nodes


    def update_da_rules(self, rules):
        sorted_rules = sorted(rules, key=lambda k: k['priority'])
        self.da_rules = sorted_rules


    def get_da_rules(self):
        return self.da_rules


    def apply_da_rules(self, node):
        added = []

        for rule in self.da_rules:
            try:
                added += self.process_da_rule(node, rule)
            except Exception as e:
                logger.error("ERROR with DA rule: {}, exception {}".format(rule, e))
        return added


    def process_da_rule(self, node, rule):
        try:
            for da in node['nlu_data']['dialog_act_ranking']:
                if da['confidence'] > rule['threshold']:
                    if da['name'] == rule['act']:
                        word = random.choice(node['words'])
                        if rule['apply'] == 'first':
                            word = node['words'][0]
                        if rule['apply'] == 'last':
                            word = node['words'][-1]
                        if rule['apply'] == 'most important':
                            try:
                                m = w = -1
                                for i, k in enumerate(node['nlu_data']['tokens']):
                                    if k['attention'] > m:
                                        m = k['attention']
                                        w = i
                                word = node['words'][w]
                            except:
                                pass
                        return self.apply_rule(rule, node, word)

                else:
                    break
        except:
            pass
        # nodes += self.apply_rule(rule, node, word)
        return []

    def create_gesture(self, animation, node, word, rule, head_gestrue=True):
        start_time = node['start_time'] + word['end'] - animation['max']+self.lipsync_delay
        start_time = max(0, start_time)
        # speech_controller metadata
        sc_meta = animation
        sc_meta['priority'] = rule['priority']
        if rule.get('act', False):
            debug = 'Dialog ACT: {}'.format(rule.get('act'))
        else:
            debug = 'Keyword rule for word {} lemma {}'.format(word['name'], rule.get('lemma', False))
        if head_gestrue:
            gesture = SpeechMotionUtils.create_node_data(name='gesture', duration=animation['duration'], speed=round(random.uniform(animation.get('speed_min',1.0), animation.get('speed_max',1)),2),
                                                  magnitude=[animation.get('magnitude_min',1), animation.get('magnitude_max',1)],
                                                  start_time=start_time, gesture=animation['name'], sc_meta=sc_meta,  debug=debug)
        else:
            gesture = SpeechMotionUtils.create_node_data(name='arm_animation', duration=animation['duration'], speed=round(random.uniform(animation.get('speed_min',1.0), animation.get('speed_max',1)),2),
                                                  magnitude=[animation.get('magnitude_min',1), animation.get('magnitude_max',1)],
                                                  start_time=start_time, arm_animation=animation['name'], sc_meta=sc_meta, debug=debug)
        return gesture


    def create_head_tilt(self, node, word, rule):
        start_time = max(0, node['start_time'] + word['end'] - 0.2)
        duration = round(0.5 + 1 * random.random(), 2)
        rad = round((0.05 + 0.1 * random.random()) * random.choice([1, -1]), 2)
        sc_meta = {
            'begin': 0.1,
            'max': 0.2,
            'end': duration + 0.05,
            'channel': ['head_tilt'],
            'priority': rule['priority']
        }
        nodes = []
        nodes.append(SpeechMotionUtils.create_node_data(name='head_rotation', duration=duration, speed=2,
                                                 angle=rad, start_time=start_time, sc_meta=sc_meta))
        # Reset
        nodes.append(SpeechMotionUtils.create_node_data(name='head_rotation', duration=0.1, speed=2,
                                                 angle=0, start_time=start_time + duration))
        return nodes

        # Cleanup metadata, and ttsdata, not required


    @staticmethod
    def clean_nodes(nodes):
        for n in nodes:
            try:
                del (n['sc_meta'])
            except KeyError:
                pass
            try:
                del (n['tts_data'])
            except KeyError:
                pass
            try:
                del (n['words'])
            except KeyError:
                pass
            try:
                del (n['nlu_data'])
            except KeyError:
                pass
            try:
                del (n['text_words'])
            except KeyError:
                pass

class SpeechMotionAPI():
    def __init__(self, cfg = None, nlu_server="http://127.0.0.1:8210/da"):
        if cfg is None:
            cfg=os.path.join(os.path.dirname(__file__), 'config.yaml')
        with open(cfg, 'r') as stream:
            try:
                self.config = yaml.safe_load(stream)
            except yaml.YAMLError as exc:
                logger.error("error reading config. Exception: {}".format(exc))
                return
        self.controller = SpeechMotionController(self.config['animation_actions'], nlu_server=nlu_server)
        self.controller.update_keyword_rules(yaml.safe_load(self.config['speech_motions']['keyword_rules']))
        self.controller.update_da_rules(yaml.safe_load(self.config['speech_motions']['da_rules']))

    def get_animations(self, ttsdata):
        # no generation on interacting
        nodes = SpeechMotionUtils.create_speech_data_from_tts_data(ttsdata)
        animated = self.controller.animate_motion(nodes, animate_arms=True)
        SpeechMotionController.clean_nodes(animated)
        return animated
