FILE(REMOVE_RECURSE
  "../msg_gen"
  "../srv_gen"
  "../msg_gen"
  "../srv_gen"
  "../src/CDIO/msg"
  "../src/CDIO/srv"
  "CMakeFiles/ROSBUILD_gensrv_py"
  "../src/CDIO/srv/__init__.py"
  "../src/CDIO/srv/_chapter2_srv1.py"
  "../src/CDIO/srv/_Get_battery_srv.py"
)

# Per-language clean rules from dependency scanning.
FOREACH(lang)
  INCLUDE(CMakeFiles/ROSBUILD_gensrv_py.dir/cmake_clean_${lang}.cmake OPTIONAL)
ENDFOREACH(lang)
