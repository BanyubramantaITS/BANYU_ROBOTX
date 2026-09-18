# banyu_msgs contracts

The action interfaces use ROS order `Goal --- Result --- Feedback`. Every
terminal result begins with `bool success` and `string reason`; operation-
specific terminal fields follow. Feedback contains progress only, including
its optional `builtin_interfaces/Time stamp`. GPS coordinates are WGS84
latitude/longitude in degrees. Distances and depths are metres; depth is
positive downward and a surface target is normally zero. Headings are ENU
radians, measured counter-clockwise from east toward north. Image offsets and
alignment values are unitless; image offsets use the normalized range `[-1, 1]`.
Ring colors and pipeline identifiers are mission-defined strings.
`ring_order` contains mission-defined numeric ring identifiers, and `crc32`
is the unsigned report CRC.

`VehicleStatus` is deliberately MAVROS-neutral. Its `header.stamp` is the
status publication time; the freshness booleans describe the corresponding
FCU, RC/operator, and GPS data validity. `authority` is an implementation-
neutral authority label. `AcknowledgeRingRepair` accepts one mission ring
index and echoes the accepted index only when the acknowledgement succeeds.

`MissionReport` is the transport-neutral report payload for the
`SendMissionReport` action boundary. `ring_sequence` uses the mission-defined
numeric ring identifiers, `source_timestamp` identifies when the report was
formed, and `crc32` is the unsigned report checksum. `MissionReportAck` echoes
the mission ID and CRC; `acknowledged` indicates acceptance, while `reason` is
optional and is empty when no explanation is needed. Neither message carries
radio, MAVROS, or vehicle-control fields.
