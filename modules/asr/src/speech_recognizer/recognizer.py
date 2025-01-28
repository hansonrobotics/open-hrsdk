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

import logging


logger = logging.getLogger("hr.asr.speech_recognizer")
import pyaudio
import threading
from queue import Queue
from google.cloud import speech
import threading
from queue import Queue
from google.cloud import speech
from typing import List, Dict
# from vad import SileroVad
import time
import rospy # Only determine if hybrid mode is on.
from .langauge_rank import LanguageDecisionMaker


RATE = 16000
CHUNK_SIZE = 1600
CHUNK_TIME = CHUNK_SIZE / RATE
SPEECH_EVENTS = {
    2: 'start',
    3: 'end',
}

class settings:
    skip_number_of_chunks_at_start = 1


class GoogleStreamingTranscriber(threading.Thread):
    def __init__(self, shared_input, shared_results, boost_phrases, language_code = "en-US", start_at=0, main=False):
        super().__init__(daemon=True)
        self.shared_input = shared_input
        self.results = {
            'events': [],
            'alternatives': [],
            'final': [],
            'finished_at': 0,
            'finished': False,
            'duration': 0,
            'main' : main,
            'lang': language_code
        }
        punctuation = ['ar-DZ', 'ar-BH', 'ar-EG', 'ar-IQ', 'ar-IL', 'ar-JO', 'ar-KW', 'ar-LB', 'ar-MR', 'ar-MA', 'ar-OM', 'ar-QA', 'ar-SA', 'ar-PS', 'ar-TN', 'ar-AE', 'ar-YE', 'cmn-Hans-CN', 'cmn-Hans-HK', 'cmn-Hant-TW', 'cs-CZ', 'da-DK', 'nl-NL', 'en-AU', 'en-IN', 'en-SG', 'en-GB', 'en-US', 'fi-FI', 'fr-FR', 'de-DE', 'hi-IN', 'id-ID', 'it-IT', 'ja-JP', 'ko-KR', 'pt-BR', 'ru-RU', 'es-ES', 'es-US', 'sv-SE', 'tr-TR', 'vi-VN']
        # Keep a reference to the dictionary.
        shared_results[f'google_{language_code}'] = self.results
        self.main = main
        self.language_code = language_code
        self.input_queue = Queue()
        self.client = speech.SpeechClient()


        phrase_set = speech.PhraseSet(
            name="custom phraseset",
            phrases=[
                speech.PhraseSet.Phrase(value=phrase, boost=20)
                for phrase in boost_phrases
            ],
            boost=20,
        )
        speech_adaptation = speech.SpeechAdaptation(phrase_sets=[phrase_set])

        self.streaming_config = speech.StreamingRecognitionConfig(
            config=speech.RecognitionConfig(
                encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
                sample_rate_hertz=16000,
                language_code=language_code,
                enable_word_time_offsets=True,
                enable_automatic_punctuation=language_code in punctuation,
                adaptation=speech_adaptation,
            ),
            interim_results= True,
            enable_voice_activity_events=True,
        )
        self.started_at = start_at
        # Restart ASR after 3 minutes and with 10 seconds of inactivity
        self.last_result = 0
        self.current_chunk = start_at



    def run(self):
        try:
            requests = self._request_generator()
            responses = self.client.streaming_recognize(
                config=self.streaming_config,
                requests=requests,
            )

            for response in responses:
                if response.speech_event_type:
                    # only care about start and end of speech
                    if response.speech_event_type not in SPEECH_EVENTS:
                        continue
                    event = SPEECH_EVENTS[response.speech_event_type]
                    # If no given time the 
                    event_time = 0
                    if response.speech_event_time:
                        event_time = response.speech_event_time.total_seconds()
                    self.results['events'].append([event, event_time, time.time()])

                for result in response.results:
                    # Check last result
                    self.last_result = (self.current_chunk - self.started_at) * CHUNK_TIME
                    if result.is_final: 
                        if result.alternatives[0].transcript == '':
                            continue
                        self.results['final'] = result.alternatives[0]
                        w_duration = 0
                        for w in result.alternatives[0].words:
                            start = w.start_time.total_seconds() if w.start_time else 0
                            end = w.end_time.total_seconds() if w.end_time else start
                            w_duration += end - start
                        self.results['duration'] = w_duration
                        # Someone is hearing something. Perhaps need to cancel all speach recognition
                        if self.shared_input['state'] == ASRState.HEARING:
                            # If we quiting then finish before the last piece processed
                            self.results['finished_at'] = self.current_chunk - 1 
                            self.results['finished'] = True
                    else:
                        if len(result.alternatives[0].transcript) > 0:
                            stability = result.stability
                            self.results['alternatives'].append({'transcript': result.alternatives[0].transcript, 'stability': stability, 'ts': time.time(), 'lang':self.language_code}) 
                if self.results['finished']:
                    break
            # Finished
            self.results['finished'] = True
            if self.results['finished_at'] == 0:
                self.results['finished_at'] = self.current_chunk
            # a = Audio('./records', self.shared_input, lang=self.language_code)
            # a.save(self.started_at, self.results['finished_at'])
        except Exception as e:
            logger.exception(f"Error in GoogleStreamingTranscriber {e}")
            self.results['finished'] = True

        

    def _request_generator(self):
        while True:
            index = self.input_queue.get()
            if index is None:
                return
            if index < self.current_chunk:
                continue
            current_time = (self.current_chunk-self.started_at) * CHUNK_TIME
            if current_time > 180 and self.last_result < current_time - 10:
                # restart ASR after 3 minutes and with 10 seconds of inactivity
                return
            

            self.current_chunk += 1
            chunk = self.shared_input[index]
            # Check if more chunks are available
            while self.current_chunk < index:
                self.current_chunk += 1
                chunk += self.shared_input[self.current_chunk]
            yield speech.StreamingRecognizeRequest(audio_content=chunk)


class MicrophoneStream(object):
    """Opens a recording stream and exposes the buffer for audio data."""

    def __init__(self, rate, chunk):
        self._rate = rate
        self._chunk = chunk
        self.buffer = Queue()  # Directly expose the buffer as a public attribute
        self.closed = True
        self._audio_interface = None
        self._audio_stream = None
        self._thread = None

    def _fill_buffer(self, in_data, frame_count, time_info, status_flags):
        """Continuously collect data from the audio stream, into the buffer."""
        self.buffer.put(in_data)
        return None, pyaudio.paContinue

    def start(self):
        """Starts the audio stream and begins filling the buffer on a new thread."""
        if self.closed:
            self._audio_interface = pyaudio.PyAudio()
            self._audio_stream = self._audio_interface.open(
                format=pyaudio.paInt16,
                channels=1,
                rate=self._rate,
                input=True,
                frames_per_buffer=self._chunk,
                stream_callback=self._fill_buffer,
            )
            self.closed = False
            self._thread = threading.Thread(target=self._audio_stream.start_stream)
            self._thread.start()

    def stop(self):
        """Stops the audio stream and terminates the thread."""
        if not self.closed:
            self._audio_stream.stop_stream()
            self._audio_stream.close()
            self._audio_interface.terminate()
            self.closed = True
            # Signal to clear buffer
            self.buffer.put(None)
            if self._thread:
                self._thread.join()


class ASRState ():
    IDLE = 0 # not running
    STARTED = 1 # started, but no speech is heard
    HEARING = 2 # speech is heard, waiting for speech to finish. 
    DECIDING = 3 # speech is finished, but need to wait for alternative languages to finish and decide the language



    
class MultiRecognizer(threading.Thread):
    def __init__(self, main_lang, alternatives=[], start_immediate=False):
        super().__init__(daemon=True)
        self.shared_audio_buffer = {
            'state': ASRState.IDLE
        } # Keys are incremental and its shared between all consumers. other keys may used to share information between threads
        self.shared_results = {} # Each thread will make new entry as a reference and write it.
        self.mic = MicrophoneStream(rate=16000, chunk=1600)
        self.google_recognizers = {}
        self.mic.start()
        self.main_lang = main_lang
        self.alternatives = alternatives
        # self.vad = SileroVad('silero_vad.onnx',0.5, 1)
        # self.vad.start()
        self.languages = [main_lang] + alternatives
        # chunk counter
        self.i = 0
        self.vad_enabled = False
        self.last_started = 0
        self.start_event = threading.Event()
        if start_immediate:
            self.start_event.set()
        self.state = ASRState.IDLE
        self.new_audio_event = threading.Event()
        # Thread to manage audio to queue
        self.audio_received = threading.Thread(daemon=True, target=self.put_audio_chunk)
        self.audio_received.start() 
        self.next_start = 1
        self.final_cb = None
        self.alternatives_cb = None
        self.event_cb = None
        self.audio_cb = None
        self.final = None
        self.phrase_engine = None # probably better would be simplify
        self.phrases = []
        self.reset = 0 # There are two types of reset. Soft reset stops straming and waits for final results. Set value to 1 to stop and 2 aborts recognition. Nothing will be publsihed.
        # Currently enabled flag. If disable recognition will not start until enabled is called
        self.enabled = False 
        self.paused = False # While enable recognition can be pasued and resumed. Has same effect except its not managed by recongiguration params. 
        self.decision_timeout = 0.3
        self.final_result_timeout = 0.0
        self.hybrid_all_answers = False
        self.decision_maker = LanguageDecisionMaker()
        


    def setup_cb(self, final_cb=None, alternatives_cb=None, event_cb=None, audio_cb=None):
        self.final_cb = final_cb
        self.alternatives_cb = alternatives_cb
        self.event_cb = event_cb
        self.audio_cb = audio_cb

    def update_config_from_node(self, config, other_langs):
        self.disable()
        self.main_lang = config.language
        self.alternatives = other_langs
        self.languages = [self.main_lang, config.alt_language] + self.alternatives
        # Remove duplicates
        self.languages = list(dict.fromkeys(self.languages))
        self.phrases = self.phrase_engine.get_phrases()
        self.enabled = config.enable
        self.decision_maker.cfg = config
        self.decision_timeout = config.decision_timeout
        self.final_result_timeout = config.final_result_timeout
        self.hybrid_all_results = config.hybrid_all_results
        pass

        
    def enable(self, after=0):
        if not self.enabled:
            self.paused = False
        self.enabled = True
        # Get the next start time
        self.next_start = self.i + 1 + after
        self.start_event.set()

    def disable(self):
        self.enabled = False
        # Should just stop everything and do not publish anything after this call
        self.start_event.clear()
        self.reset = 1

    def pause(self):
        self.paused = True
        self.start_event.clear()
        self.reset = 1
    
    def resume(self, after=0):
        self.paused = False
        if self.enabled:
            self.reset = 0
            self.next_start = self.i + 1 + after
            self.start_event.set()

    # def stop(self):
    #     # Stops streaming, publishes last result
    #     self.start_event.clear()
    #     self.reset = 1
        

    def put_audio_chunk(self):
        while True:   
            chunk = self.mic.buffer.get()
            self.shared_audio_buffer[self.i+1] = chunk
            self.i += 1
            try:
                # in case exception if recognizers changed is not a big deal as it would pickup oon next sound input
                for recognizer in self.google_recognizers.values():    
                        recognizer.input_queue.put(self.i)
            except Exception as e:
                logger.warn(e)
            # cleanup buffer every 20s remove older than 6 min audio data
            if self.i % 200 == 0 and self.i > 3600:
                for k in list(self.shared_audio_buffer.keys()):
                    if k is int and k < self.i - 3600:
                        del self.shared_audio_buffer[k]

    def run(self):
        self.i = 0
        last_event = '' # Track event
        last_interim_length = 0 # Publish longest interrim result available
        last_interim_result = None
        decsision_start  = 0
        while True:
            time.sleep(0.02)
            self.shared_audio_buffer['state'] = self.state  
            if self.state  == ASRState.IDLE:
                last_interim_length = 0
                last_interim_result = None
                # Do nothing until its enabled
                if self.start_event.wait():
                    self.state  = ASRState.STARTED
                    self.reset = 0
                    self.last_started = self.i
                    # recognizers start time starts after the next chunk. Perhaps need to start + 2 because current chunk may contain some self speech.
                    starting = self.next_start
                    # start recognizers
                    self.google_recognizers = \
                        {lang:GoogleStreamingTranscriber(self.shared_audio_buffer, self.shared_results, self.phrases, language_code=lang, start_at=starting) 
                            for lang in self.languages}
                    #Start recognizers
                    for asr in self.google_recognizers.values():
                        asr.start()
            if self.state  == ASRState.STARTED:
                # get main recognizer results and see if event has been recorded. Assume first event is real event
                for asr in self.google_recognizers.values():
                    if len(asr.results['alternatives']) > 0:
                        self.state  = ASRState.HEARING
                    # If any asr finished then stop all recognizers
                    if asr.results['finished']:
                        self.state  = ASRState.HEARING
                # Advance state
                if self.reset == 1:
                    self.state  = ASRState.HEARING 
            if self.state  == ASRState.HEARING:
                if last_event != 'start':
                    last_event = 'start'
                    self.event_cb('speechstart')
                # check if main language is finished cancel all others
                finished = False
                for lang, results in self.shared_results.items():
                    if results['finished']:
                        finished = True
                # publish interrim results
                last_interim_time = 0
                for lang, asr in self.google_recognizers.items():
                    for alt in list(asr.results['alternatives']): 
                        last_interim_time = max(alt['ts'], last_interim_time)
                        if len(alt['transcript']) > last_interim_length:
                            last_interim_length = len(alt['transcript'])
                            last_interim_result = alt
                            self.alternatives_cb(alt['transcript'], alt['stability'], lang)
                # Check if 
                if self.final_result_timeout > 0.1 and last_interim_result:
                    if time.time() - last_interim_time > self.final_result_timeout and last_interim_result['lang'] != 'en-US':
                        finished = True
                        logger.warn(f"Final result timeout {last_interim_result['transcript']} {last_interim_result['lang']} {last_interim_result['stability']}")
                # Finish on reset
                finished = finished or self.reset == 1
                if finished:
                    self.state  = ASRState.DECIDING
                    for asr in self.google_recognizers.values():
                        asr.input_queue.put(None)
                else:
                    if last_event != 'start':
                        last_event = 'start'
                        self.event_cb('speechstart')                    

            if self.state  == ASRState.DECIDING:
                if decsision_start < 1:
                    decsision_start = time.time()
                
                # Check if all recognizers are finished, or timeout reached
                if all([r['finished'] for r in self.shared_results.values()]) or time.time() - decsision_start > self.decision_timeout:
                    # Reset the decision
                    decsision_start = 0
                    if last_event != 'stop':
                        last_event = 'stop'
                        self.event_cb('speechstop') 
                    # Decide the language
                    candidates = []
                    for lang, results in self.shared_results.items():
                        try:
                            logger.info(f'{lang}: {round(results["final"].confidence,2)} : {round(results["duration"],1)} -  {results["final"].transcript} - finished at {results["finished_at"]}')
                            candidate = {
                                'confidence': round(results["final"].confidence,2),
                                'transcript': results["final"].transcript,
                                'finished_at': results["finished_at"],
                                'lang': results["lang"],
                                'duration': results['duration'],
                                'main': results['main']

                            }
                            candidates.append(candidate)
                        except Exception:
                            logger.warn(f'{lang}: No final result')

                    # Use the decision maker to decide on the best candidate
                    transcript, confidence, lang = self.decision_maker.decide_language(candidates)
                    if self.hybrid_all_results:
                        hybrid = rospy.get_param("/hr/interaction/chatbot/hybrid_mode", False)
                        if hybrid:
                            for candidate in candidates:
                                if candidate['transcript']:
                                    self.final_cb(candidate['transcript'], candidate['confidence'], "", lang)
                        else:
                            if transcript:
                                self.final_cb(transcript, confidence, "", lang)
                    else:       
                        # By default publish only one
                        if transcript:
                            self.final_cb(transcript, confidence, "", lang)
                        else:
                            logger.warn("No proper speech detected")
                    try:
                        if self.shared_results[f'google_{self.main_lang}']['finished_at'] > 0:
                            self.next_start = self.shared_results[f'google_{self.main_lang}']['finished_at']
                        else:
                            self.next_start = self.i
                    except KeyError:
                        # In case main language was changed
                        self.next_start = self.i
                    self.google_recognizers = {}
                    self.shared_results = {}
                    self.state  = ASRState.IDLE
                    if self.enabled and not self.paused:
                        self.start_event.set()
                


