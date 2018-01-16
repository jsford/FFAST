cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

project(HokuyoAIST VERSION 3.0.2)
string(TOLOWER ${PROJECT_NAME} PROJECT_NAME_LOWER)
include(${PROJECT_SOURCE_DIR}/cmake/hokuyoaist_utils.cmake)
set(PROJECT_DESCRIPTION "Hokuyo range sensor driver.")
set(PROJECT_VENDOR "Geoffrey Biggs, AIST")

# Add an "uninstall" target
configure_file("${PROJECT_SOURCE_DIR}/cmake/uninstall_target.cmake.in"
    "${PROJECT_BINARY_DIR}/uninstall_target.cmake" IMMEDIATE @ONLY)
add_custom_target(uninstall "${CMAKE_COMMAND}" -P
    "${PROJECT_BINARY_DIR}/uninstall_target.cmake")

set(LIB_TYPE SHARED)

# Set up installation directories
set(BIN_INSTALL_DIR "bin")
set(LIB_INSTALL_DIR "lib")
set(INC_INSTALL_DIR
    "include/${PROJECT_NAME_LOWER}-${PROJECT_VERSION_MAJOR}")
set(SHARE_INSTALL_DIR
    "share/${PROJECT_NAME_LOWER}-${PROJECT_VERSION_MAJOR}")

# Dependencies
find_package(Flexiport)
# May need to link to the rt library
include(CheckFunctionExists)
set(CMAKE_REQUIRED_INCLUDES time.h)
set(CMAKE_REQUIRED_LIBRARIES rt)
CHECK_FUNCTION_EXISTS(clock_gettime HAVE_CLOCK_GETTIME)
set(CMAKE_REQUIRED_INCLUDES)
set(CMAKE_REQUIRED_LIBRARIES)

# Subdirectories
add_subdirectory(cmake)
#add_subdirectory(src)
add_subdirectory(include)
add_subdirectory(hokuyoaist_src)

# pull in the pods macros. See cmake/pods.cmake for documentation
set(POD_NAME hokuyoaist)
include(cmake/pods.cmake)

# automatically build LCM types.  This also defines a number of CMake
# variables, see cmake/lcmtypes.cmake for details
include(cmake/lcmtypes.cmake)
lcmtypes_build()


include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_BINARY_DIR}/include)
include_directories(${Flexiport_INCLUDE_DIRS})
include_directories(include lcmtypes/cpp/)

# Laser data reader example
add_executable(${PROJECT_NAME_LOWER}_example src/example.cpp)
pods_use_pkg_config_packages(${PROJECT_NAME_LOWER}_example lcm lcmtypes_${POD_NAME})

target_link_libraries(${PROJECT_NAME_LOWER}_example
    ${PROJECT_NAME_LOWER})
install(TARGETS ${PROJECT_NAME_LOWER}_example
    DESTINATION ${BIN_INSTALL_DIR}
    COMPONENT examples)

# LCM publisher
add_executable(hokuyo_publisher src/hokuyo_publisher.cpp)
pods_use_pkg_config_packages(hokuyo_publisher lcm lcmtypes_${POD_NAME})

target_link_libraries(hokuyo_publisher
    ${PROJECT_NAME_LOWER})
install(TARGETS ${PROJECT_NAME_LOWER}_example
    DESTINATION ${BIN_INSTALL_DIR}
    COMPONENT examples)

# LCM listener
add_executable(listener src/listener.cpp)
pods_use_pkg_config_packages(listener lcm lcmtypes_${POD_NAME})

# Install example sources
install(FILES src/CMakeLists.txt.example
    DESTINATION ${SHARE_INSTALL_DIR}/examples
    RENAME CMakeLists.txt
    COMPONENT examples)

# For packaging
set(EXAMPLE_EXECUTABLES ${PROJECT_NAME_LOWER}_example HokuyoAIST_Example
    ${PROJECT_NAME_LOWER}_getid GetID
    PARENT_SCOPE)


# make executables public
pods_install_executables(${PROJECT_NAME_LOWER}_example)
pods_install_executables(hokuyo_publisher)
pods_install_executables(listener)


# Package creation
include(InstallRequiredSystemLibraries)
set(PROJECT_EXECUTABLES ${EXAMPLE_EXECUTABLES})
set(cpack_options "${PROJECT_BINARY_DIR}/cpack_options.cmake")
configure_file("${PROJECT_SOURCE_DIR}/cmake/cpack_options.cmake.in"
    ${cpack_options} @ONLY)
set(CPACK_PROJECT_CONFIG_FILE ${cpack_options})
include(CPack)
