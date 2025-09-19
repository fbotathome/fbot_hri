#! /bin/bash
foxglove-studio &
sleep 5
WINDOW_ID=$(xwininfo -name "Dashboard | Foxglove" | grep "Window id:" | awk '{print $4}')

echo "Window ID: $WINDOW_ID"

x11vnc -id $WINDOW_ID -forever -noremote -clip 2560x1600+0+80

