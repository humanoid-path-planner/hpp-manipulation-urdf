// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//
// This file is part of hpp-manipulation-urdf.
// hpp-manipulation-urdf is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
//
// hpp-manipulation-urdf is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Lesser Public License for more details.  You should have
// received a copy of the GNU Lesser General Public License along with
// hpp-manipulation-urdf. If not, see <http://www.gnu.org/licenses/>.

#include "hpp/manipulation/srdf/factories/robot.hh"
#include <hpp/manipulation/device.hh>

namespace hpp {
  namespace manipulation {
    namespace srdf {
      bool RobotFactory::finishAttributes ()
      {
        if (!root ()->device ()) {
          hppDout (error, "There is no Device");
          return false;
        }
        if (root ()->device ()->name ().compare (name ()) != 0) {
          hppDout (warning, "Device and XML robot tag have different names.");
        }
        return true;
      }
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp
