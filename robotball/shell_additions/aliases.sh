#!/bin/bash

## Basic stuff
alias cd..='cd ..'
alias re-source='source ~/.bashrc'

## Short commands for common tools.
# With this alias, when returning from Ranger, current directory will be the last one you were positioned in while in Ranger.
alias ra='. ranger'
# If you don-t want this behaviour, uncomment the alias bellow, and comment out the one above.
# alias ra='ranger'

## Commands for easier working with Arduino.
ard_compile() {
  arduino-cli compile --fqbn arduino:megaavr:nona4809 --libraries ~/catkin_ws/src/robotball/robotball_arduino/customLibraries ~/catkin_ws/src/robotball/robotball_arduino/$1
}

ard_upload() {
  arduino-cli upload -v -p $(readlink -f /dev/tty_arduino) --fqbn arduino:megaavr:nona4809 ~/catkin_ws/src/robotball/robotball_arduino/$1
}

ard_auto() {
  ard_compile $1 && ard_upload $1
}

alias ard_msg='rosrun rosserial_arduino make_libraries.py /home/raspi/Arduino/libraries'
