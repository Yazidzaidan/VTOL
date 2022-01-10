#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
© Copyright 2017, Peter Barker
play_tune.py: GUIDED mode "simple goto" example (Copter Only)

Demonstrates how to play a custom tune on a vehicle using the vehicle's buzzer

Full documentation is provided at http://python.dronekit.io/examples/play_tune.html
"""

from __future__ import print_function
import time
from dronekit import connect


# Set up option parsing to get connection string
import argparse
parser = argparse.ArgumentParser(description='Play tune on vehicle buzzer.')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, SITL automatically started and used.")
parser.add_argument('--tune', type=str, help="tune to play", default="AAAA")
args = parser.parse_args()

connection_string = '/dev/ttyACM0'



# Connect to the Vehicle
print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

while True:
    print(vehicle.channels['8'])

