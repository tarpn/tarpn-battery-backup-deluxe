SHELL=/bin/bash

.PHONY: init
init:
	python3 -m venv venv
	./venv/bin/pip install wheel twine

.PHONY: dist
dist: 
	./venv/bin/python setup.py egg_info sdist; 
	./venv/bin/pip wheel --no-index --no-deps --wheel-dir dist dist/*.tar.gz
	ls dist

.PHONY: upload
upload:
	./pypi-upload.sh


.PHONY: clean
clean:
	rm -rf dist
	find . | grep -E "(__pycache__|\.pyc|\.pyo$$)" | xargs rm -rf

