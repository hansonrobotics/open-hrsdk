#!/usr/bin/env python
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

TEST_DATA = [{'duration': 3.605771,
             'end': 3.605771,
             'name': 'Hello there. I am Sophia. What is your name?',
             'start': 0.0,
             'type': 'text'},
            {'end': 0.33539584279060364,
             'name': 'hello',
             'start': 0.030000001192092896,
             'type': 'word'},
            {'end': 0.6391458511352539,
             'name': 'there',
             'start': 0.33539584279060364,
             'type': 'word'},
            {'end': 1.1891666650772095,
             'name': 'i',
             'start': 1.0391458421945572,
             'type': 'word'},
            {'end': 1.2893333435058594,
             'name': 'am',
             'start': 1.1891666650772095,
             'type': 'word'},
            {'end': 1.8942708373069763,
             'name': 'sophia',
             'start': 1.2893333435058594,
             'type': 'word'},
            {'end': 2.4396458864212036,
             'name': 'what',
             'start': 2.294270873069763,
             'type': 'word'},
            {'end': 2.5999792218208313,
             'name': 'is',
             'start': 2.4396458864212036,
             'type': 'word'},
            {'end': 2.777625024318695,
             'name': 'your',
             'start': 2.5999792218208313,
             'type': 'word'},
            {'end': 3.205770790576935,
             'name': 'name',
             'start': 2.777625024318695,
             'type': 'word'}]



from performances.speech_motion import SpeechMotionAPI
import yaml
import os
if __name__ == '__main__':
    if os.path.dirname(__file__):
        fn = os.path.dirname(__file__)+'/test.yaml'
    else:
        fn = 'test.yaml'
    with open(fn, 'r') as f:
        data = yaml.safe_load(f)
        data = yaml.safe_load(data['data'])
    api =SpeechMotionAPI(nlu_server='http://127.0.0.1:8102/v1.0/nlu/da')
    import time
    print('start')
    t = time.time()
    animations = api.get_animations(data)
    print(time.time() -t)
    #print(animations)


