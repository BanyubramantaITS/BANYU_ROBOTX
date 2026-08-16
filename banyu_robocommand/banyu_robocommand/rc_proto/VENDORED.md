# Vendored RoboCommand schema (do not hand-edit)

Copied verbatim from `robonation/robocommand`, `RobotX_2026/gen/python/`.

- Upstream: https://github.com/robonation/robocommand
- Commit: `f6457fa47489647a8650c1d3eb74efa84ce4b84a` (2026-08-07)
- License: Apache-2.0 (see upstream `LICENSE`)

Why vendored instead of generated at build time: the `.proto` files declare
`edition = "2024"`, which `protoc` 31.1 refuses ("later than the protoc maximum
supported edition 2023") and which jammy's `python3-protobuf` 3.12.4 cannot
import (`ImportError: cannot import name 'runtime_version'`). The checked-in
generated code works only against `protobuf==7.35`. Handbook §3.4.4 forbids
changing package names, field numbers, enum values, or topic names anyway.

Re-vendor (do not patch) when RoboNation bumps the schema:

    rsync -a --exclude __pycache__ <clone>/RobotX_2026/gen/python/ ./

`common_pb2.py` and `robotx/` are imported as top-level modules
(`import common_pb2`, `from robotx import ...`), so this directory is put on
`sys.path` by `rc_proto/__init__.py` rather than being imported as a package.
