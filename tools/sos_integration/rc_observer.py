#!/usr/bin/env python3
"""RoboCommand-side observer: decodes every robocommand/robotx/# MQTT message into JSONL.

Usage (GCS repo venv): python rc_observer.py OUT.jsonl [broker_host=127.0.0.1]
GCS_REPO points at a robocommand-rx-ui-itsn checkout for the protobuf decoder.
"""
import json
import os
import sys
import time

sys.path.insert(0, os.environ.get('GCS_REPO', os.path.expanduser('~/Desktop/robocommand-rx-ui-itsn')))

import paho.mqtt.client as mqtt  # noqa: E402
from google.protobuf.json_format import MessageToDict  # noqa: E402

from robocommand_ui.proto_helper import parse_mqtt_payload  # noqa: E402

out = open(sys.argv[1], 'a', buffering=1)


def on_connect(client, userdata, flags, reason_code, properties=None):
    client.subscribe('robocommand/robotx/#')
    print('[observer] subscribed robocommand/robotx/#', flush=True)


def on_message(client, userdata, msg):
    name, parsed = parse_mqtt_payload(msg.topic, msg.payload)
    record = {'t': time.time(), 'topic': msg.topic, 'type': name, 'retain': bool(msg.retain)}
    if parsed is None:
        record['error'] = 'undecodable'
    else:
        record['msg'] = MessageToDict(parsed, preserving_proto_field_name=True)
        oneofs = parsed.DESCRIPTOR.oneofs_by_name
        record['body'] = parsed.WhichOneof('body') if 'body' in oneofs else None
    out.write(json.dumps(record) + '\n')
    if record.get('body') != 'heartbeat':
        print(f"[observer] {msg.topic} {name} {record.get('body')}", flush=True)


client = mqtt.Client(mqtt.CallbackAPIVersion.VERSION2)
client.on_connect = on_connect
client.on_message = on_message
client.connect(sys.argv[2] if len(sys.argv) > 2 else '127.0.0.1', 1883)
client.loop_forever()
