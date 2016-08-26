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

#include "hpp/manipulation/srdf/factories/position.hh"

#include <pinocchio/spatial/se3.hpp>

namespace hpp {
  namespace manipulation {
    namespace srdf {
      typedef Eigen::Quaternion<value_type> Quaternion_t;

      Transform3f PositionFactory::position () const
      {
        std::vector <float> v = values ();
        Transform3f p = Transform3f (
            Quaternion_t(v[3], v[4], v[5], v[6]).matrix(), // w, x, y, z
            vector3_t (v[0], v[1], v[2]));
        return p;
      }
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp
