#! /usr/bin/env bash
export cmd="docker run -it --user=1000:1000 --device=/dev/ttyUSB0 --env=DISPLAY --workdir=/home/sharath --volume=/home/sharath:/home/sharath --volume=/etc/group:/etc/group:ro --volume=/etc/passwd:/etc/passwd:ro --volume=/etc/shadow:/etc/shadow:ro --volume=/etc/sudoers.d:/etc/sudoers.d:ro --volume=/tmp/.X11-unix:/tmp/.X11-unix:rw osrf/ros:foxy-desktop" 
gnome-terminal --window-with-profile="orange" -- bash -c "$cmd ;exec bash"
