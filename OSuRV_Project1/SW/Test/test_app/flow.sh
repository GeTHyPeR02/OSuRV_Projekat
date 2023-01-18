#!/bin/bash

exit 0


./waf configure

# Test robot.
./waf build && ./build/test_servos w 0 25
./waf build && ./build/test_servos w 0 125

# Test chassis.
# Speed.
./waf build && ./build/test_bldc +20
./waf build && ./build/test_bldc -20
# Steer.
./waf build && ./build/test_servos w 1 75
./waf build && ./build/test_servos w 1 95
./waf build && ./build/test_servos w 1 55

