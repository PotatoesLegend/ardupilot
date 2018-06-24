#!/bin/bash

./waf configure --board=px4-v2
./waf copter
./waf --targets bin/arducopter --upload
python ./setup_roscopter.py
