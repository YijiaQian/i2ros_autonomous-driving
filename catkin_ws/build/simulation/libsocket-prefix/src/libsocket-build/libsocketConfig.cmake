set(libsocket_INCLUDE_DIRS "/home/joshua/project/project_i2ros/catkin_ws/build/simulation/libsocket-prefix/src/libsocket/headers")

set(libsocket_BINARY_DIR "/home/joshua/project/project_i2ros/catkin_ws/build/simulation/libsocket-prefix/src/libsocket-build")

include(${libsocket_BINARY_DIR}/libsocketTargets.cmake)

set(libsocket_LIBRARIES socket++)
