#
# Copyright 2015-2019 CNRS-UM LIRMM, CNRS-AIST JRL
#

set(mc_vrep_src
  vrep_simulation.cpp
  vrep_cli.cpp
  mc_vrep.cpp
)

add_executable(mc_vrep ${mc_vrep_src})
target_link_libraries(mc_vrep PUBLIC vrep-api-wrapper::vrep-api-wrapper mc_rtc::mc_control)

install(TARGETS mc_vrep
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

# FIXME If we ever use the stack in CoppeliaSim API we might need the stack stuff?
add_library(simExtMcRtc SHARED vrep_plugin.cpp "${VREP_PATH}/programming/common/simLib.cpp")
target_include_directories(simExtMcRtc PUBLIC "${VREP_PATH}/programming/include/" "${VREP_PATH}/programming/common")
target_link_libraries(simExtMcRtc PUBLIC mc_rtc::mc_control)

install(TARGETS simExtMcRtc
  RUNTIME DESTINATION "${VREP_PATH}"
  LIBRARY DESTINATION "${VREP_PATH}"
  ARCHIVE DESTINATION "${VREP_PATH}")
