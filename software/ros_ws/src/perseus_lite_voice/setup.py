import os
from glob import glob

from setuptools import find_packages, setup

package_name = "perseus_lite_voice"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "models"), glob("models/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Nigel_H-S",
    maintainer_email="nigel.hungerfordsymes@gmail.com",
    description=(
        "Voice assistant ROS 2 node for Perseus Lite: Ollama LLM chat, Piper TTS, "
        "Whisper STT, openwakeword wake-word activation, YOLOv8 vision over "
        "/image_raw, and a voice-to-robot intent bridge."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "perseus_lite_voice = perseus_lite_voice.voice_node:main",
        ],
    },
)
