from glob import glob
import re
import os
from setuptools import setup, find_packages
import setuptools_scm

TAG_REGEX = r"^firmware-(?:[\w-]+-)?(?P<version>[vV]?\d+(?:\.\d+){0,2}[^\+]*)(?:\+.*)?$"


def get_version():
    return setuptools_scm.get_version(
      root="..", 
      tag_regex=TAG_REGEX,
      git_describe_command="git describe --tags --match firmware-*")


setup(
    name='tarpn-bbd-firmware',
    version=get_version(),
    packages=['tarpn.bbd.firmware'],
    data_files=[
        ("extra", ["bootloader/optiboot_atmega328.hex", ".pio/build/ATmega328P_RPi/firmware.hex"]),
        ("", ["VERSION"])
    ],
    url='https://github.com/tarpn/tarpn-battery-backup-deluxe',
    license='MIT License',
    author='David Arthur',
    author_email='mumrah@gmail.com',
    description='Firmware for TARPN Battery Board Deluxe',
    python_requires='>=3.7',
)
