#! /bin/bash
foxglove-studio &
sleep 5

WINDOW_ID=$(xwininfo -name "Foxglove" | grep "Window id:" | awk '{print $4}')
echo "Window ID: $WINDOW_ID"

xdotool windowsize $WINDOW_ID 1920 1080
sleep 1

CROP_WIDTH=1920
CROP_HEIGHT=870
OFFSET_X=0
OFFSET_Y=140

CLIP="${CROP_WIDTH}x${CROP_HEIGHT}+${OFFSET_X}+${OFFSET_Y}"

x11vnc -id $WINDOW_ID -forever -noremote -noxdamage -clip  $CLIP -speeds -defer -scale 1920x1200 -viewonly
