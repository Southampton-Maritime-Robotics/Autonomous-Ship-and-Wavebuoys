FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ASV/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_cpp"
  "../msg_gen/cpp/include/ASV/status.h"
  "../msg_gen/cpp/include/ASV/compass.h"
  "../msg_gen/cpp/include/ASV/arduino.h"
  "../msg_gen/cpp/include/ASV/position.h"
  "../msg_gen/cpp/include/ASV/gps.h"
  "../msg_gen/cpp/include/ASV/rudder.h"
  "../msg_gen/cpp/include/ASV/headingd.h"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_cpp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
