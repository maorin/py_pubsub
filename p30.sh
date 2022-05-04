set -x

sudo modprobe v4l2loopback

v4l2-ctl --list-devices

scrcpy --v4l2-sink=/dev/video0 -m 1024 --no-display