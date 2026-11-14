#!/bin/bash
# Front camera ("H264 USB Camera") on its own; options: see camera.sh --help.
exec "$(dirname "$(readlink -f "$0")")/camera.sh" front "$@"
