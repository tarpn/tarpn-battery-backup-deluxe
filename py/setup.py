from glob import glob
import os
from setuptools import setup, find_packages

setup(
    name='tarpn-bbd-agent',
    use_scm_version = {
        "root": "..",
        "relative_to": __file__,
        "git_describe_command": "git describe --tags --match v*.*",
    },
    setup_requires=['setuptools_scm'],
    packages=find_packages(),
    data_files=[
        ("config", ["config/bbd.ini.sample", "config/logging.ini"]),
        ("extra", glob("extra/*")),
        ("logs", ["extra/.empty.txt"])
    ],
    url='https://github.com/tarpn/tarpn-battery-backup-deluxe',
    license='MIT License',
    author='David Arthur',
    author_email='mumrah@gmail.com',
   
    description='Prometheus metrics scraper for TARPN Battery Board Deluxe',
   
    entry_points={
             'console_scripts': [
                 'bbd-agent = tarpn.bbd.agent:main',
                 'bbd-firmware-update = tarpn.bbd.firmware_update:main'
             ]},
    python_requires='>=3.7',

    install_requires=[
        'tarpn-bbd-firmware~=2.0',                     # The firmware package
        'pyserial~=3.4',                               # Serial interface
        'prometheus-client~=0.15.0',                   # Prometheus client library
        "RPi.GPIO==0.7.1;platform_machine=='armv7l' or platform_machine=='aarch64'",  # Control the RPi GPIOs (for firmware upload)
        "importlib_metadata~=6.0",
        # Tests
        'pytest==6.0.1',
        'pytest-runner==5.2',
    ]
)
