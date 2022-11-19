SHELL=/bin/bash

.PHONY: version
version:
	./venv/bin/python setup.py --version

.PHONY: init
init: 
	python3 -m venv venv
	. venv/bin/activate; pip install -U platformio setuptools_scm twine

.PHONY: build
build: 
	. venv/bin/activate; pio run -e ATmega328P_RPi 
	ls .pio/build/ATmega328P_RPi/firmware.hex

.PHONY: dist
dist: build  
	./venv/bin/python setup.py egg_info sdist
	./venv/bin/pip wheel --no-index --no-deps --wheel-dir dist dist/*.tar.gz

.PHONY: clean
clean: 
	. venv/bin/activate; pio run -e ATmega328P_RPi -t clean
	rm -rf dist build *.egg-info

.PHONY: upload
upload:
	. ../.twine.env; ./venv/bin/twine upload --repository-url https://pypi.mumrah.synology.me/ -s dist/*
