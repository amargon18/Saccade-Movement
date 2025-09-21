project(SACCADE)

find_package(YARP REQUIRED)

add_executable(saccade saccade.cpp)

target_link_libraries(saccade ${YARP_LIBRARIES})