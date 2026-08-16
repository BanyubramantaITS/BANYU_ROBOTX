"""Vendored RoboCommand protobuf modules — see VENDORED.md.

The generated code uses absolute top-level imports (`import common_pb2`,
`from robotx import rx_common_pb2`), so this directory has to be a sys.path
root, not a package prefix. Importing this package installs it.
"""

import os
import sys

sys.path.insert(0, os.path.dirname(__file__))

import common_pb2  # noqa: E402
from robotx import (  # noqa: E402
    rx_commands_pb2,
    rx_common_pb2,
    rx_course_pb2,
    rx_reports_pb2,
    rx_requests_pb2,
)

__all__ = [
    'common_pb2',
    'rx_commands_pb2',
    'rx_common_pb2',
    'rx_course_pb2',
    'rx_reports_pb2',
    'rx_requests_pb2',
]
