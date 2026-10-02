PY ?= $(CURDIR)/.venv/bin/python
.PHONY: test test-full venv
venv:
	python3 -m venv .venv && .venv/bin/pip install -q numpy pillow matplotlib lz4
test:
	$(PY) tools/test_core.py --quick
test-full:
	$(PY) tools/test_core.py
