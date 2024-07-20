set(libsocket_INCLUDE_DIRS "/home/jonas/i2ROS/project/ziou/AutonomousDriving/build/simulation/libsocket-prefix/src/libsocket/headers")

set(libsocket_BINARY_DIR "/home/jonas/i2ROS/project/ziou/AutonomousDriving/build/simulation/libsocket-prefix/src/libsocket-build")

include(${libsocket_BINARY_DIR}/libsocketTargets.cmake)

set(libsocket_LIBRARIES socket++)
