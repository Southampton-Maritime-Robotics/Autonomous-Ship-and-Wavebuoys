FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ASV/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_lisp"
  "../msg_gen/lisp/status.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_status.lisp"
  "../msg_gen/lisp/compass.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_compass.lisp"
  "../msg_gen/lisp/arduino.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_arduino.lisp"
  "../msg_gen/lisp/position.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_position.lisp"
  "../msg_gen/lisp/gps.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_gps.lisp"
  "../msg_gen/lisp/rudder.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_rudder.lisp"
  "../msg_gen/lisp/headingd.lisp"
  "../msg_gen/lisp/_package.lisp"
  "../msg_gen/lisp/_package_headingd.lisp"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_lisp.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
