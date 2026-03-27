#!/bin/bash
# Script per lanciare la chat interattiva della LLM app

source /home/leandro/ros2_arise_vulcanexus/ros2_ws/install/setup.bash

# Se disponibile, usa gnome-terminal, altrimenti fallback a konsole, xfce4-terminal, o terminator
if command -v gnome-terminal &> /dev/null; then
    gnome-terminal -- bash -c "source /home/leandro/ros2_arise_vulcanexus/ros2_ws/install/setup.bash && ros2 run voice_command_interpreter nl_savepose_parser; exec bash"
elif command -v konsole &> /dev/null; then
    konsole -e bash -c "source /home/leandro/ros2_arise_vulcanexus/ros2_ws/install/setup.bash && ros2 run voice_command_interpreter nl_savepose_parser; exec bash"
elif command -v xfce4-terminal &> /dev/null; then
    xfce4-terminal -e "bash -c 'source /home/leandro/ros2_arise_vulcanexus/ros2_ws/install/setup.bash && ros2 run voice_command_interpreter nl_savepose_parser; exec bash'"
elif command -v terminator &> /dev/null; then
    terminator -e "bash -c 'source /home/leandro/ros2_arise_vulcanexus/ros2_ws/install/setup.bash && ros2 run voice_command_interpreter nl_savepose_parser; exec bash'"
else
    # Fallback: lancialo nello stesso terminale
    bash -c "source /home/leandro/ros2_arise_vulcanexus/ros2_ws/install/setup.bash && ros2 run voice_command_interpreter nl_savepose_parser"
fi
