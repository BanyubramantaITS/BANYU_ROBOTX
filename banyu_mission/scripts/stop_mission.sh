#!/bin/bash

set -xe

ros2 service call /banyu/mission/abort \
  std_srvs/srv/SetBool "{data: true}"