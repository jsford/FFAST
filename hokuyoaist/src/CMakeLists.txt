include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_BINARY_DIR}/include)
include_directories(${Flexiport_INCLUDE_DIRS})
include_directories(${PROJECT_SOURCE_DIR}/lcmtypes/cpp)
include_directories(${PROJECT_BINARY_DIR}/lcmtypes/cpp)

# Laser data reader example
add_executable(${PROJECT_NAME_LOWER}_example example.cpp)
#pods_use_pkg_config_packages(example lcm lcmtypes_${POD_NAME})

# make executables public
#pods_install_executables(${PROJECT_NAME_LOWER}_example)

target_link_libraries(${PROJECT_NAME_LOWER}_example
    ${PROJECT_NAME_LOWER})
install(TARGETS ${PROJECT_NAME_LOWER}_example
    DESTINATION ${BIN_INSTALL_DIR}
    COMPONENT examples)

# Install example sources
install(FILES CMakeLists.txt.example
    DESTINATION ${SHARE_INSTALL_DIR}/examples
    RENAME CMakeLists.txt
    COMPONENT examples)

# For packaging
set(EXAMPLE_EXECUTABLES ${PROJECT_NAME_LOWER}_example HokuyoAIST_Example
    ${PROJECT_NAME_LOWER}_getid GetID
    PARENT_SCOPE)

