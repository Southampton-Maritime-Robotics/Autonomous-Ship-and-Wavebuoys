FILE(REMOVE_RECURSE
  "../msg_gen"
  "../src/ASV/msg"
  "../msg_gen"
  "CMakeFiles/ROSBUILD_genmsg_py"
  "../src/ASV/msg/__init__.py"
  "../src/ASV/msg/_status.py"
  "../src/ASV/msg/_compass.py"
  "../src/ASV/msg/_arduino.py"
  "../src/ASV/msg/_position.py"
  "../src/ASV/msg/_gps.py"
  "../src/ASV/msg/_rudder.py"
  "../src/ASV/msg/_headingd.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_genmsg_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
