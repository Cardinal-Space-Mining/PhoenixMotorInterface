#!/bin/bash

# Get the current directory using pwd
base_dir=$(pwd)

lib_to_modify="$base_dir/install/phoenix5_driver/lib/phoenix5_driver/libCTRE_Phoenix.so"
exe_to_modify="$base_dir/install/phoenix5_driver/lib/phoenix5_driver/main"
new_rpath="$base_dir/install/phoenix5_driver/lib/phoenix5_driver"

# Use patchelf to set the new RPATH
patchelf --set-rpath "$new_rpath" "$lib_to_modify"
patchelf --set-rpath "$new_rpath" "$exe_to_modify"