#!/usr/bin/env bash
rosnode kill /goal_publisher
lsof -i :8001 | tail -n +2 | awk '{system("kill -9 " $2)}'