# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.20)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# add the component directories that we want to use
set(EXTRA_COMPONENT_DIRS
  "../../../components/"
  "../../../components/espp/components"
)

set(
  COMPONENTS
  "main esptool_py task i2c ft3168"
  CACHE STRING
  "List of components to include"
  )

project(tca9554_example)

set(CMAKE_CXX_STANDARD 20)
