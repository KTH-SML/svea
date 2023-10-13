# -*- coding: utf-8 -*-
from setuptools import setup, find_packages
from setuptools.command.build_ext import build_ext
import platform, subprocess

try:
    # read the contents of your README file
    from os import path

    this_directory = path.abspath(path.dirname(__file__))
    with open(path.join(this_directory, "README.md"), encoding="utf-8") as f:
        long_description = f.read()
except:
    long_description = None

system = platform.system()
bits = platform.architecture()[0]
bit = "x64" if bits == "64bit" else "x86"


class Build(build_ext):
    def run(self):
        if system == "Linux":
            subprocess.call("ls ", shell=True)
            subprocess.call("cd ./src && make clean && make", shell=True)
            subprocess.call("cp ./bin/*.so ./arducam_config_parser/", shell=True)
        elif system == "Windows":
            subprocess.call(
                "cd ./msvc && msbuild arducam_config_parser.sln /t:Rebuild /p:Configuration=Release",
                shell=True,
            )
            subprocess.call(
                "cp ./bin/*/Release/*.dll ./arducam_config_parser/", shell=True
            )


setup_kwargs = {
    "name": "arducam_config_parser",
    "version": "0.1.1",
    "description": "arducam_config_parser is a python wrapper for dynamic libraries. ",
    "long_description": long_description,
    "long_description_content_type": "text/markdown",  # This is important!
    "author": "ArduCam",
    "author_email": "support@arducam.com",
    "url": "https://github.com/ArduCAM/arducam_config_parser",
    "packages": find_packages(),
    "package_data": {"": ["*"]},
    "include_package_data": True,
    "platforms": ["Windows", "Linux"],
    "cmdclass": {"build_ext": Build},
    "classifiers": [
        "Programming Language :: Python",
        "Operating System :: Microsoft :: Windows",
        "Operating System :: Unix",
    ],
}


setup(**setup_kwargs)
