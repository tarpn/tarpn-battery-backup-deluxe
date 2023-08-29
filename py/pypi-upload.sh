#!/bin/bash

source .twine.env

./venv/bin/twine upload --repository-url https://pypi.mumrah.synology.me/ -s dist/*
