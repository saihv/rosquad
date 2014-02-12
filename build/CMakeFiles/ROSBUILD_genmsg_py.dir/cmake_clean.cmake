FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/rosquad/msg"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/rosquad/msg/__init__.py"
  "../src/rosquad/msg/_Mavlink_RAW_IMU.py"
  "../src/rosquad/msg/_State.py"
  "../src/rosquad/msg/_Control.py"
  "../src/rosquad/msg/_RC.py"
  "../src/rosquad/msg/_VFR_HUD.py"
  "../src/rosquad/msg/_Attitude.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
