#!/bin/bash
# Bottom camera (Horus B "2M WebCam") on its own; options: see camera.sh --help.
exec "$(dirname "$(readlink -f "$0")")/camera.sh" bottom "$@"
