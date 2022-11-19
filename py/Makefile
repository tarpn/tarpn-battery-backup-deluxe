SHELL=/bin/bash

.PHONY: init
init:
	python3 -m venv venv
	./venv/bin/pip install wheel twine
	./venv/bin/python setup.py develop

.PHONY: dist
dist: 
	./venv/bin/python setup.py egg_info sdist
	./venv/bin/pip wheel --no-index --no-deps --wheel-dir dist dist/*.tar.gz
	ls dist

.PHONY: upload
upload:
	. ../.twine.env; ./venv/bin/twine upload --repository-url https://pypi.mumrah.synology.me/ -s dist/*

.PHONY: clean
clean:
	rm -rf *.egg-info
	rm -rf dist
	find . | grep -E "(__pycache__|\.pyc|\.pyo$$)" | xargs rm -rf

