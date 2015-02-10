# Copyright (c) 2014, LAAS-CNRS
# Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
#
# This file is part of hpp-manipulation-urdf.
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
# hpp-manipulation-urdf. If not, see <http://www.gnu.org/licenses/>.

SET(LIBRARY_NAME ${PROJECT_NAME})

SET(${LIBRARY_NAME}_SOURCES
  parser/parser.cc
  parser/factories/sequence.cc

  srdf/util.cc
  srdf/factories/handle.cc
  srdf/factories/gripper.cc
  srdf/factories/contact.cc
  srdf/factories/position.cc
  srdf/factories/robot.cc
  )

ADD_LIBRARY(${LIBRARY_NAME}
  SHARED
  ${${LIBRARY_NAME}_SOURCES}
  )

PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} tinyxml2)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-util)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-model-urdf)
PKG_CONFIG_USE_DEPENDENCY(${LIBRARY_NAME} hpp-manipulation)

INSTALL(TARGETS ${LIBRARY_NAME} DESTINATION lib)
