#!/bin/bash
#indoor normal ligth

v4l2-ctl -d /dev/video0 --set-ctrl=sharpness=140
v4l2-ctl -d /dev/video0 --set-ctrl=focus_auto=0
v4l2-ctl -d /dev/video0 --set-ctrl=focus_absolute=10
v4l2-ctl -d /dev/video0 --set-ctrl=power_line_frequency=1
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_auto=1
v4l2-ctl -d /dev/video0 --set-ctrl=contrast=150
v4l2-ctl -d /dev/video0 --set-ctrl=saturation=120
v4l2-ctl -d /dev/video0 --set-ctrl=white_balance_temperature_auto=1
v4l2-ctl -d /dev/video0 --set-ctrl=gain=$2
v4l2-ctl -d /dev/video0 --set-ctrl=zoom_absolute=100
v4l2-ctl -d /dev/video0 --set-ctrl=exposure_absolute=$1
exit 0
