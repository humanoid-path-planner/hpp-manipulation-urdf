#
# Copyright (c) 2010, 2011, 2020, CNRS
# Authors: Joseph Mirabel, Guilhem Saurel
#
#
# This file is part of hpp-manipulation-urdf
# hpp-manipulation-urdf is free software: you can redistribute it
# and/or modify it under the terms of the GNU Lesser General Public
# License as published by the Free Software Foundation, either version
# 3 of the License, or (at your option) any later version.
#
# hpp-manipulation-urdf is distributed in the hope that it will be
# useful, but WITHOUT ANY WARRANTY; without even the implied warranty
# of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
# General Lesser Public License for more details.  You should have
# received a copy of the GNU Lesser General Public License along with
# hpp-manipulation-urdf  If not, see
# <http://www.gnu.org/licenses/>.

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
