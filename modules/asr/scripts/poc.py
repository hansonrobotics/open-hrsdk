import queue
from google.cloud import speech
import sounddevice
# Can use numpy for easier manipulation of audio data
import numpy as np
# Audio recording parameters
RATE = 16000
CHUNK = int(RATE / 10)  # 100ms



class Singleton(type):
    """ Singleton pattern for objects"""
    _instances = {}
    def __call__(cls, *args, **kwargs):
        if cls not in cls._instances:
            cls._instances[cls] = super(Singleton, cls).__call__(*args, **kwargs)
        return cls._instances[cls]

class MicrophoneStream(sounddevice.InputStream):
    """Opens a recording stream as a generator yielding the audio chunks."""

    def __init__(self, **kwargs):
        if not kwargs.get('callback',None):
            kwargs['callback'] = self.fill_buffer
        super().__init__(**kwargs)
        self._buff = queue.Queue()


    def fill_buffer(self, in_data, frame_count, time_info, status_flags):
        # Dont forget to coppy data from buffer as it may get changed after!!!
        self._buff.put(in_data.copy())


    def generator(self):
        i = 0
        print(f'{self.samplerate=}')

        while not self.closed:
            chunk = self._buff.get()
            if chunk is None:
                return
            # Gets all chunks remaining to be streamedori
            data = chunk
            # Now consume whatever other data's still buffered.
            while True:
                try:
                    chunk = self._buff.get(block=False)
                    if chunk is None:
                        return
                    np.append(data, chunk)
                except queue.Empty:
                    break
            yield data


class MaxLegthGenerator(object, metaclass=Singleton):
    # Logic to make the straming
    def __init__(self, microphone,  samplerate, max_length = 300, break_after=250, bridge=10, max_carry_over=50):
        # Flag to determine then new stream is passed
        self.new_stream = True
        self.finished = False
        self._mic = microphone
        self.samplerate = samplerate
        # Sample rate needed to know at what time we need to start streaming again
        self._max_length = max_length
        self._break_after = break_after
        self._bridge = bridge
        self._max_carry_over = max_carry_over
        # Empty data
        self._audio_data = np.empty((0,), dtype='int16')
        # Keep track if speech in the intermi recognition mode
        self._interim_start = -1
        self._last_interim = -1
        self._last_final = -1
        self.prev_bridge = 0

    def _find_brige(self):
        return self._bridge

    def reset_stream(self):
        self.new_stream = False
        self._audio_data = np.empty((0,), dtype='int16')
        self._last_interim = -1
        self._last_final = -1
        self._interim_start = -1
        self.finished = False

    def __iter__(self):
        return self

    def __next__(self):
        # Quit streaming if max length reache
        if not self.new_stream and len(self._audio_data) > self._max_length*self.samplerate -1:
            self.finished = True
            raise StopIteration
        # Otherwise waits for more data
        for d in self._mic:
            data = np.empty((0,), dtype='int16')
            if self.new_stream:
                offset = self._get_next_offset()
                print(f'{offset=}')
                samples = int(offset*self.samplerate)
                if samples > 0:
                    # get required chunks for carry over
                    try:
                        data = self._audio_data[-samples:]
                    except Exception as e:
                        print(f'Error {e}')
                        pass
                self.reset_stream()
            try:
                data = np.append(data, d)
                self._audio_data = np.append(self._audio_data,data)
            except Exception as e:
                print(e)
            # Yield binary data (used for streaming
            return data.tobytes()




    def result_callback(self, final, result_time = 0):
        if self.finished:
            return
        print(f'{final=}')
        # interim result received
        if not final:
            if not(self._last_interim > 0 and self._last_interim > result_time - self._bridge):
                self._interim_start = result_time
            self._last_interim = result_time
        # Final result
        else:
            self._last_final = result_time
            # Interim times need to reset after final result
            self._last_interim = -1
            self._interim_start = -1

    def _get_next_offset(self):
        # By default use bridge as carry over data
        offset =self._bridge
        # Total previous stream time
        total_time = self._audio_data.size / self.samplerate
        print(f'{total_time=}')
        print(f'{offset=}')
        print(f'{self._last_final=}\t{self._interim_start=}\t{self._last_interim=}\t{self._max_carry_over=}')

        if total_time == 0:
            return 0
        # If havent received any interim results after last final result
        if self._interim_start < 0:
            # If last final result came after bridge time
            return min(offset, total_time-self._last_final)
        else:
            if total_time - self._last_interim < offset:
                # Add bridge to interim start so the begining of the speech is captured correctly
                offset = total_time - self._interim_start+self._bridge
            return min(offset, self._max_carry_over)


def listen_print_loop(responses, generator):
    """Iterates through server responses and prints them.

    The responses passed is a generator that will block until a response
    is provided by the server.

    Each response may contain multiple results, and each result may contain
    multiple alternatives; for details, see https://goo.gl/tjCPAU.  Here we
    print only the transcription for the top alternative of the top result.

    In this case, responses are provided for interim results as well. If the
    response is an interim one, print a line feed at the end of it, to allow
    the next result to overwrite it, until the response is a final one. For the
    final one, print a newline to preserve the finalized transcription.
    """
    num_chars_printed = 0
    for response in responses:
        if not response.results:
            continue

        # The `results` list is consecutive. For streaming, we only care about
        # the first result being considered, since once it's `is_final`, it
        # moves on to considering the next utterance.
        result = response.results[0]
        if not result.alternatives:
            continue

        # Display the transcription of the top alternative.
        transcript = result.alternatives[0].transcript
        # Inform strem on timing of results:
        result_seconds = result_micros = 0
        if result.result_end_time.seconds:
            result_seconds = result.result_end_time.seconds
        if result.result_end_time.microseconds:
            result_micros = result.result_end_time.microseconds
        # Audio generator needs to be informed on result times
        generator.result_callback(result.is_final, result_seconds + result_micros / 1000000.0)
        print(transcript)



def main():
    # See http://g.co/cloud/speech/docs/languages
    # for a list of supported languages.
    language_code = "en-US"  # a BCP-47 language tag

    client = speech.SpeechClient()
    config = speech.RecognitionConfig(
        encoding=speech.RecognitionConfig.AudioEncoding.LINEAR16,
        sample_rate_hertz=RATE,
        language_code=language_code,
    )

    streaming_config = speech.StreamingRecognitionConfig(
        config=config, interim_results=True
    )
    stream = MicrophoneStream( samplerate=RATE, blocksize = CHUNK, device='default',

                                            dtype='int16', channels=1)
    audio_generator = MaxLegthGenerator(stream.generator(), samplerate=RATE, max_length=20, break_after=6, bridge=3, max_carry_over=10)
    try:
        with stream:
            #audio_generator = MaxLegthGenerator(stream.generator(), samplerate=RATE)
            # debug
            while True:
                # Start new stream
                audio_generator.new_stream = True
                requests = (
                    speech.StreamingRecognizeRequest(audio_content=content)
                    for content in audio_generator
                )
                # Return only when first response available
                responses = client.streaming_recognize(streaming_config, requests)
                # Now, put the transcription responses to use.
                listen_print_loop(responses, audio_generator)

    except Exception as e:
        print(f'Exception detected {e}')
    except KeyboardInterrupt:
        print("FINISHED")
if __name__ == "__main__":
    main()
    # stream = AudioStream()
    # print("started")
    # time.sleep(5)
    # write("TT3.wav", RATE, stream.data)