# requires minimum cmake version
cmake_minimum_required(VERSION 3.5)

# produce the cmake var PROJECT_NAME
project(tutorial_gaze-interface)

# mandatory use of these packages
find_package(YARP 3.3.0 REQUIRED)
find_package(ICUBcontrib REQUIRED)

# extend the current search path used by cmake to load helpers
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

# helpers defining certain macros
include(ICUBcontribHelpers)
include(ICUBcontribOptions)

# specify to install in $ICUBcontrib_DIR/bin
icubcontrib_set_default_prefix()



# detector
add_executable(${PROJECT_NAME}-detector ${CMAKE_SOURCE_DIR}/src/detector.cpp)
target_compile_definitions(${PROJECT_NAME}-detector PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME}-detector ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME}-detector DESTINATION bin)

# tracker
add_executable(${PROJECT_NAME}-tracker ${CMAKE_SOURCE_DIR}/src/tracker.cpp)
target_compile_definitions(${PROJECT_NAME}-tracker PRIVATE _USE_MATH_DEFINES)
target_link_libraries(${PROJECT_NAME}-tracker ${YARP_LIBRARIES})
install(TARGETS ${PROJECT_NAME}-tracker DESTINATION bin)

# generate ad-hoc project to perform "make uninstall"
icubcontrib_add_uninstall_target()

file(GLOB scripts ${CMAKE_SOURCE_DIR}/app/scripts/*.xml)
yarp_install(FILES ${scripts} DESTINATION ${ICUBCONTRIB_APPLICATIONS_INSTALL_DIR})
