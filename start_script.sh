#!/bin/bash

/usr/bin/flock -n /home/ubuntu/NewGaitMaven.lock -c "/home/ubuntu/NewGaitMaven/measurement.sh &" > /dev/null 2>&1

exit $?
