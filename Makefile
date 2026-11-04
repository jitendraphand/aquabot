PY ?= python3
export PYTHONPATH := $(CURDIR)
export MUJOCO_GL ?= egl

.PHONY: all cad videos qa qa-fast firmware-test test readme clean

all: cad
	$(PY) -m scripts.build_all

cad:
	$(PY) -m scripts.build_all --no-render

videos:
	$(PY) -m scripts.render_videos
	$(PY) -m scripts.build_all --only gifs

qa:
	$(PY) -m aquabot.qa.run --fix
	$(PY) -m scripts.update_readme

qa-fast:
	$(PY) -m aquabot.qa.run --fast

firmware-test:
	$(PY) -m aquabot.firmware_config
	g++ -std=c++17 -O2 -Wall -Wextra -Werror -DHOST_BUILD -Ifirmware/test/stubs -Ifirmware/aquabot \
	    firmware/test/host_build.cpp firmware/aquabot/kinematics.cpp firmware/aquabot/gait.cpp firmware/aquabot/hal.cpp -o build/fw_host
	./build/fw_host > /dev/null && echo "firmware host build OK"

test:
	$(PY) -m pytest -q tests

readme:
	$(PY) -m scripts.update_readme

clean:
	rm -rf build
