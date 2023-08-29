import os
from setuptools import setup, find_packages

setup(
    name='tarpn-bbd',
    use_scm_version = {
        "root": "..",
        "relative_to": __file__,
        "local_scheme": "node-and-timestamp"
    },
    setup_requires=['setuptools_scm'],
    packages=find_packages(),
    data_files=[
        ("config", ["config/bbd.ini.sample", "config/logging.ini"]),
        ("extra", ["extra/tarpn-bbd-supervisor.conf"]),
        ("logs", ["extra/.empty.txt"])
    ],
    url='https://github.com/tarpn/tarpn-battery-backup-deluxe',
    license='MIT License',
    author='David Arthur',
    author_email='mumrah@gmail.com',
   
    description='Prometheus metrics scraper for TARPN Battery Board Deluxe',
   
    entry_points={
             'console_scripts': [
                 'tarpn-bbd-scrape = tarpn_bbd:main'
             ]},
    python_requires='>=3.7',

    install_requires=[
        'pyserial~=3.4',             # Serial interface
        'prometheus-client~=0.15.0', # Prometheus client library
        'python-json-logger~=2.0.7', # Makes JSON logs
        'importlib-metadata==4.10',  # only until we drop 3.7 support
    ]
)
