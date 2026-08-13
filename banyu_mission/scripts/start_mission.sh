#!/bin/bash

set -xe

ros2 service call /banyu/mission/start \
  std_srvs/srv/SetBool "{data: true}"