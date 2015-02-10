#
# Copyright (c) 2010, 2011 CNRS
# Authors: Joseph Mirabel
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

CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

SET(CXX_DISABLE_WERROR TRUE)
INCLUDE(cmake/base.cmake)
INCLUDE(cmake/cpack.cmake)

SET(PROJECT_NAME hpp-manipulation-urdf)
SET(PROJECT_DESCRIPTION "Implementation of a parser for hpp-manipulation.")
SET(PROJECT_URL "https://github.com/billx09/hpp-manipulation-urdf")

SETUP_PROJECT()

# Activate hpp-util logging if requested
SET (HPP_DEBUG FALSE CACHE BOOL "trigger hpp-util debug output")
IF (HPP_DEBUG)
  SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -DHPP_DEBUG")
ENDIF()

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

ADD_REQUIRED_DEPENDENCY("tinyxml2 >= 2.2")
ADD_REQUIRED_DEPENDENCY("hpp-util")
ADD_REQUIRED_DEPENDENCY("hpp-model")
ADD_REQUIRED_DEPENDENCY("hpp-model-urdf")
ADD_REQUIRED_DEPENDENCY("hpp-manipulation")
ADD_REQUIRED_DEPENDENCY("resource_retriever")

PKG_CONFIG_APPEND_LIBS (${PROJECT_NAME})
ADD_SUBDIRECTORY(src)
ADD_SUBDIRECTORY(tests)

SETUP_PROJECT_FINALIZE()
SETUP_PROJECT_CPACK()
