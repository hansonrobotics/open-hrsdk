from setuptools import setup

setup(
    name="ttsserver",
    version="0.5.0",
    packages=["ttsserver", "ttsserver.espp"],
    description=("Hanson Robotics TTS Server"),
    url="https://github.com/hansonrobotics/ttsserver",
    author="Wenwei Huang",
    author_email="wenwei@hansonrobotics.com",
    entry_points={"console_scripts": ["run_tts_server=ttsserver.server:main"]},
)
