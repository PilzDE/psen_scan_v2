#!/bin/bash

# Executing this script at package root will check
# for warnings/errors from rosdoc_lite and return 1 if there are findings
# and 0 otherwise. This is intended to be used by the continuous integration.

source /opt/ros/noetic/setup.bash

if rosdoc_lite . -o ../doc 2>&1 >/dev/null | grep -v -E -f .rosdoc_lite_ignore
then echo "rosdoc_lite did finish with warnings/errors"; exit 1
else echo "rosdoc_lite finished without warnings/errors"; exit 0
fi
