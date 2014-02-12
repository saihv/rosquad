FILE(REMOVE_RECURSE
  "../msg_gen"
  "../msg_gen"
  "../src/rosquad/msg"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/rosquad/Mavlink_RAW_IMU.h"
  "../msg_gen/cpp/include/rosquad/State.h"
  "../msg_gen/cpp/include/rosquad/Control.h"
  "../msg_gen/cpp/include/rosquad/RC.h"
  "../msg_gen/cpp/include/rosquad/VFR_HUD.h"
  "../msg_gen/cpp/include/rosquad/Attitude.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
