#
# Copyright (c) 2010, 2011, 2020, CNRS Authors: Joseph Mirabel, Guilhem Saurel
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# 1. Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

cmake_minimum_required(VERSION 3.22)

set(PROJECT_NAME hpp-manipulation-urdf)
set(PROJECT_DESCRIPTION "Implementation of a parser for hpp-manipulation.")

set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CXX_DISABLE_WERROR TRUE)

# Check if the submodule cmake have been initialized
set(JRL_CMAKE_MODULES "${CMAKE_CURRENT_LIST_DIR}/cmake")
if(EXISTS "${JRL_CMAKE_MODULES}/base.cmake")
  message(STATUS "JRL cmakemodules found in 'cmake/' git submodule")
else()
  find_package(jrl-cmakemodules QUIET CONFIG)
  if(jrl-cmakemodules_FOUND)
    get_property(
      JRL_CMAKE_MODULES
      TARGET jrl-cmakemodules::jrl-cmakemodules
      PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    message(STATUS "JRL cmakemodules found on system at ${JRL_CMAKE_MODULES}")
  else()
    message(STATUS "JRL cmakemodules not found. Let's fetch it.")
    include(FetchContent)
    FetchContent_Declare(
      "jrl-cmakemodules"
      GIT_REPOSITORY "https://github.com/jrl-umi3218/jrl-cmakemodules.git")
    FetchContent_MakeAvailable("jrl-cmakemodules")
    FetchContent_GetProperties("jrl-cmakemodules" SOURCE_DIR JRL_CMAKE_MODULES)
  endif()
endif()

include("${JRL_CMAKE_MODULES}/hpp.cmake")

compute_project_args(PROJECT_ARGS LANGUAGES CXX)
project(${PROJECT_NAME} ${PROJECT_ARGS})

add_project_dependency(TinyXML2 REQUIRED FIND_EXTERNAL TinyXML)

add_project_dependency("hpp-manipulation" REQUIRED)
if(BUILD_TESTING)
  add_project_dependency(example-robot-data REQUIRED)
endif()

# Declare Headers
set(${PROJECT_NAME}_HEADERS
    include/hpp/manipulation/parser/parser.hh
    include/hpp/manipulation/parser/factories/sequence.hh
    include/hpp/manipulation/parser/factories/ignoretag.hh
    include/hpp/manipulation/srdf/util.hh
    include/hpp/manipulation/srdf/factories.hh
    include/hpp/manipulation/srdf/factories/contact.hh
    include/hpp/manipulation/srdf/factories/gripper.hh
    include/hpp/manipulation/srdf/factories/handle.hh
    include/hpp/manipulation/srdf/factories/position.hh
    include/hpp/manipulation/srdf/factories/robot.hh)

set(${PROJECT_NAME}_SOURCES
    src/parser/parser.cc
    src/parser/factories/sequence.cc
    src/srdf/util.cc
    src/srdf/factories/handle.cc
    src/srdf/factories/gripper.cc
    src/srdf/factories/contact.cc
    src/srdf/factories/position.cc
    src/srdf/factories/robot.cc)

add_library(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES}
                                   ${${PROJECT_NAME}_HEADERS})
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>)
target_link_libraries(${PROJECT_NAME} hpp-manipulation::hpp-manipulation)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${TARGETS_EXPORT_NAME}
  DESTINATION lib)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

pkg_config_append_libs(${PROJECT_NAME})
