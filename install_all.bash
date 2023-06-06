#!/bin/bash
if [ "$#" -ne 1 ]; then
    echo "Please provide a rosinstall-compatible file as an argument" >&2
    exit 2
fi

workspace_src_dir=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && cd .. && pwd )
rosinstall $workspace_src_dir $1
rosdep install -riy --from-paths $workspace_src_dir
