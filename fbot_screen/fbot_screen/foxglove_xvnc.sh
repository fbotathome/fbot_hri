#! /bin/bash
foxglove-studio &
sleep 5
WINDOW_ID=$(xwininfo -name "Dashboard | Foxglove" | grep "Window id:" | awk '{print $4}')
WINDOW_WIDTH=$(xwininfo -name "Dashboard | Foxglove" | grep "Width:" | awk '{print $2}')
WINDOW_HEIGHT=$(xwininfo -name "Dashboard | Foxglove" | grep "Height:" | awk '{print $2}')

echo "Window ID: $WINDOW_ID"

CLIP=$WINDOW_WIDTH'x'$WINDOW_HEIGHT



x11vnc -id $WINDOW_ID -forever -noremote -noxdamage -clip  $CLIP+0+80

