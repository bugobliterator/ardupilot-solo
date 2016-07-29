[![CircleCI](https://circleci.com/gh/heleport/ardupilot.svg?style=svg)](https://circleci.com/gh/heleport/ardupilot)

# Automated builds

Builds are done on CircleCI, firmware is uploaded to `s3://heleport-dev/ardupilot/&lt;branchname&gt;/ubuntu`, eg:
* https://s3-us-west-1.amazonaws.com/heleport-dev/ardupilot/visualpos/ubuntu/ArduCopter-v2.px4
* https://s3-us-west-1.amazonaws.com/heleport-dev/ardupilot/visualpos/ubuntu/ArduCopter.elf

# Load new firmware

## Build Arducopter

## Update firmware

* `scp ArduCopter-v2.px4 solo:/firmware`
* Restart solo

## Connect with mavlink

* `pip install mavproxy`
* `mavproxy.py —master 0.0.0.0:14550`

## Set params

* `param set EK2_GPS_TYPE 0`
* `param set LOG_BACKEND_TYPE 1`
* `param set LOG_BITMASK 131070`
* `param set LOG_DISARMED 1`
* `param set LOG_REPLAY 1`