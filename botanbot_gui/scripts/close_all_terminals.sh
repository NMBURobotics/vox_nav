#!/bin/bash
xdotool search --class "terminal" | while read id
do
      xdotool windowactivate "$id" &>/dev/null
      xdotool key ctrl+shift+q
      sleep 0.05
done 