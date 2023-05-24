#!/usr/bin/env bash

SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )

# Create urdf from xacro
xacro $1 > /tmp/tmp_urdf.urdf
# Create mjcf from urdf
compile /tmp/tmp_urdf.urdf /tmp/tmp_mjcf.xml
# Add missing elements to the mjcf file
python3 $SCRIPT_DIR/add_actuators.py /tmp/tmp_urdf.urdf /tmp/tmp_mjcf.xml $2
# Clean temporary files
rm /tmp/tmp_urdf.urdf /tmp/tmp_mjcf.xml