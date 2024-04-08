option(BUILD_SHARED_LIBS "Build shared library" OFF)

add_library(serial_port serial_port.cpp)

target_compile_features(serial_port PUBLIC cxx_std_17)

target_include_directories(
    serial_port
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/../include>"
    "$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}/${PROJECT_NAME}>"
)

install(
  TARGETS serial_port
  EXPORT serial_port_targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/../include/serial_port
  DESTINATION include
)

install(
  EXPORT serial_port_targets
  FILE serial_port_targets.cmake
  NAMESPACE serial_port::
  DESTINATION lib/cmake/serial_port
)

include(CMakePackageConfigHelpers)

configure_package_config_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/Config.cmake.in
  "${CMAKE_CURRENT_BINARY_DIR}/serial_port_config.cmake"
  INSTALL_DESTINATION lib/cmake/serial_port
)

write_basic_package_version_file(
  "${CMAKE_CURRENT_BINARY_DIR}/serial_port_config_version.cmake"
  VERSION "${serial_port_VERSION_MAJOR}.${serial_port_VERSION_MINOR}"
  COMPATIBILITY AnyNewerVersion
)

install(
  FILES
    "${CMAKE_CURRENT_BINARY_DIR}/serial_port_config.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/serial_port_config_version.cmake"
  DESTINATION lib/cmake/serial_port
)

export(
  EXPORT serial_port_targets
  FILE "${CMAKE_CURRENT_BINARY_DIR}/cmake/serial_port_targets.cmake"
  NAMESPACE serial_port::
)
