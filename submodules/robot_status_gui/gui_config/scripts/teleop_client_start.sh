#!/usr/bin/env bash
screen -S teleop_client -dm ./src/ServiceRobotControlStack/control_stack/src/teleop/client.py robot0 192.168.0.102 --http_port 5000 &