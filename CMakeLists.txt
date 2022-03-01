#
# Copyright (c) 2010, 2011, 2020, CNRS
# Authors: Joseph Mirabel, Guilhem Saurel
#
#

# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are
# met:
#
# 1. Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright
# notice, this list of conditions and the following disclaimer in the
# documentation and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
# A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
# HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
# SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
# LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
# DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
# THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
# DAMAGE.

CMAKE_MINIMUM_REQUIRED(VERSION 3.1)

SET(PROJECT_NAME hpp-manipulation-urdf)
SET(PROJECT_DESCRIPTION "Implementation of a parser for hpp-manipulation.")

SET(PROJECT_USE_CMAKE_EXPORT TRUE)
SET(CXX_DISABLE_WERROR TRUE)

INCLUDE(cmake/hpp.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake/find-external/TinyXML)
FIND_PACKAGE(TinyXML REQUIRED)

ADD_PROJECT_DEPENDENCY("hpp-manipulation" REQUIRED)

# Declare Headers
SET(${PROJECT_NAME}_HEADERS
  include/hpp/manipulation/parser/parser.hh
  include/hpp/manipulation/parser/factories/sequence.hh
  include/hpp/manipulation/parser/factories/ignoretag.hh

  include/hpp/manipulation/srdf/util.hh
  include/hpp/manipulation/srdf/factories.hh
  include/hpp/manipulation/srdf/factories/contact.hh
  include/hpp/manipulation/srdf/factories/gripper.hh
  include/hpp/manipulation/srdf/factories/handle.hh
  include/hpp/manipulation/srdf/factories/position.hh
  include/hpp/manipulation/srdf/factories/robot.hh
  )


SET(${PROJECT_NAME}_SOURCES
  src/parser/parser.cc
  src/parser/factories/sequence.cc

  src/srdf/util.cc
  src/srdf/factories/handle.cc
  src/srdf/factories/gripper.cc
  src/srdf/factories/contact.cc
  src/srdf/factories/position.cc
  src/srdf/factories/robot.cc
  )

ADD_LIBRARY(${PROJECT_NAME} SHARED ${${PROJECT_NAME}_SOURCES} ${${PROJECT_NAME}_HEADERS})
TARGET_INCLUDE_DIRECTORIES(${PROJECT_NAME} PUBLIC $<INSTALL_INTERFACE:include>)
TARGET_LINK_LIBRARIES(${PROJECT_NAME} hpp-manipulation::hpp-manipulation)

INSTALL(TARGETS ${PROJECT_NAME} EXPORT ${TARGETS_EXPORT_NAME} DESTINATION lib)

ADD_SUBDIRECTORY(tests)

PKG_CONFIG_APPEND_LIBS(${PROJECT_NAME})

INSTALL(FILES package.xml DESTINATION share/${PROJECT_NAME})
