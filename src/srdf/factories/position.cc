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

#include <hpp/util/debug.hh>

#include <pinocchio/spatial/se3.hpp>

namespace hpp {
  namespace manipulation {
    namespace srdf {
      typedef Eigen::Quaternion<value_type> Quaternion_t;

      void PositionFactory::finishTags ()
      {
        if (values().size() != 0)
          computeTransformFromText ();
        else
          computeTransformFromAttributes ();
      }

      void PositionFactory::computeTransformFromText ()
      {
        const std::vector<float>& v = values ();
        Quaternion_t q(v[3], v[4], v[5], v[6]); // w, x, y, z
        if (std::fabs(1 - q.squaredNorm()) < 1e-4) {
          hppDout (warning, "Quaternion is not normalized.");
        }
        q.normalize();
        position_ = Transform3f (q.matrix(), vector3_t (v[0], v[1], v[2]));
      }

      void PositionFactory::computeTransformFromAttributes ()
      {
        vector3_t xyz (0,0,0);
        if (hasAttribute ("xyz"))
          parser::readSequence (getAttribute ("xyz"), xyz, 3);

        bool a_rpy  = hasAttribute ("rpy" );
        bool a_wxyz = hasAttribute ("wxyz");
        bool a_xyzw = hasAttribute ("xyzw");
        if ((int)a_rpy + (int)a_wxyz + (int)a_xyzw > 1)
          throw std::invalid_argument ("Tag " + tagName () + " must have only one of rpy, wxyz, xyzw");

        matrix3_t R (matrix3_t::Identity());
        if (a_rpy) {
          vector3_t rpy;
          parser::readSequence (getAttribute ("rpy"), rpy, 3);

          typedef Eigen::AngleAxis<value_type> AngleAxis;
          R = AngleAxis(rpy[2], vector3_t::UnitZ())
            * AngleAxis(rpy[1], vector3_t::UnitY())
            * AngleAxis(rpy[0], vector3_t::UnitX());
        } else if (a_wxyz || a_xyzw) {
          Quaternion_t q;

          if (a_wxyz) {
            vector_t wxyz (4);
            parser::readSequence (getAttribute ("wxyz"), wxyz, 4);
            q.w() = wxyz[0];
            q.vec() = wxyz.tail<3>();
          } else if (a_xyzw) {
            vector_t xyzw (4);
            parser::readSequence (getAttribute ("xyzw"), xyzw, 4);
            q.w() = xyzw[3];
            q.vec() = xyzw.head<3>();
          }

          if (std::fabs(1 - q.squaredNorm()) < 1e-4) {
            hppDout (warning, "Quaternion is not normalized.");
          }
          q.normalize();

          R = q.matrix();
        }

        position_ = Transform3f (R, xyz);
      }
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp
