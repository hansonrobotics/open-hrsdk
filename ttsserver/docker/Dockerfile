##
## Copyright (C) 2017-2024 Hanson Robotics
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
##################################
FROM alpine:3.14 as open-hrsdk

RUN apk add --no-cache git git-lfs
RUN git lfs install --skip-smudge
RUN git clone https://github.com/hansonrobotics/open-hrsdk.git

ADD https://api.github.com/repos/hansonrobotics/open-hrsdk/git/refs/heads/ open-hrsdk.json
ADD https://api.github.com/repos/hansonrobotics/open-hrsdk/git/matching-refs/tags/ open-hrsdk-tags.json
RUN cd /open-hrsdk && git fetch && git reset --hard origin/main
##################################

FROM ubuntu:20.04

RUN apt-get update && apt-get install -y \
        build-essential \
        libopenblas-base \
        locales \
        mpg123 \
        python3-dev \
        python3-virtualenv \
        sox \
        swig  \
    && rm -rf /var/lib/apt/lists/*

RUN locale-gen en_US.UTF-8
RUN echo "LANG=en_US.UTF-8" > /etc/default/locale

RUN virtualenv -p /usr/bin/python3 /opt/hansonrobotics/py3env

RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y \
        ffmpeg \
        libsox-fmt-all \
    && rm -rf /var/lib/apt/lists/*

RUN /opt/hansonrobotics/py3env/bin/python -m pip install \
    "numpy" \
    "typing_extensions"

RUN /opt/hansonrobotics/py3env/bin/python -m pip install \
    "azure-cognitiveservices-speech==1.17.0" \
    "boto3" \
    "bs4" \
    "coloredlogs>=14.0" \
    "flask" \
    "lxml" \
    "pinyin==0.2.5" \
    "pydub" \
    "python-slugify>=5.0.2" \
    "pyyaml" \
    "requests" \
    "scipy" \
    "sox>=1.2.9" \
    "urllib3>1.25.4" \
    "websocket-client==1.2.1"

WORKDIR /ttsserver
COPY --from=open-hrsdk /open-hrsdk/ttsserver /ttsserver

RUN /opt/hansonrobotics/py3env/bin/python setup.py install

EXPOSE 10002
