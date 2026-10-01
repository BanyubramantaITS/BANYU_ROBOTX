RoboCommand schema, copied unchanged from RoboNation's release
https://github.com/robonation/robocommand (RobotX_2026/proto), commit
f6457fa47489647a8650c1d3eb74efa84ce4b84a (2026-08-07). Apache-2.0, see LICENSE.

Kept as the reference `src/rx_report.cpp` encodes against; handbook 3.4.4 requires this schema
unchanged. RoboNation's generated C++ needs protobuf 35 (edition 2024), which Ubuntu 24.04 does
not ship, so the one message this package sends is encoded by hand and pinned to bytes produced
by their official generated classes (test/test_rx_report.cpp).
