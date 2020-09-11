# Copyright (c) 2014, 2020, LAAS-CNRS
# Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
# Authors: Guilhem Saurel (guilhem.saurel@laas.fr)
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

# ADD_TESTCASE(NAME)
# ------------------------
#
# Define a test named `NAME'.
#
# This macro will create a binary from `NAME.cc' and add it to the test suite.
#
MACRO(ADD_TESTCASE NAME GENERATED)
  IF (${GENERATED} STREQUAL TRUE)
   ADD_UNIT_TEST(${NAME} ${CMAKE_CURRENT_BINARY_DIR}/${NAME}.cc)
  ELSE()
    ADD_UNIT_TEST(${NAME} ${NAME}.cc)
  ENDIF()

  PKG_CONFIG_USE_DEPENDENCY(${NAME} tinyxml)
  TARGET_LINK_LIBRARIES(${NAME} ${PROJECT_NAME})
ENDMACRO(ADD_TESTCASE)

CONFIG_FILES(srdfparser.cc)
ADD_TESTCASE(srdfparser TRUE)
