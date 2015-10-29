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

#include "hpp/manipulation/srdf/factories/contact.hh"

#include <boost/assign/list_of.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/pointer.hh>

#include <hpp/model/joint.hh>
#include <hpp/model/body.hh>
#include <hpp/model/collision-object.hh>

#include <hpp/manipulation/device.hh>

namespace hpp {
  namespace manipulation {
    namespace srdf {
      void ContactFactory::finishTags ()
      {
        DevicePtr_t device = HPP_DYNAMIC_PTR_CAST (Device, root ()->device ());
        if (!device) {
          hppDout (error, "Failed to create contacts");
          return;
        }

        /// Get the link
        ObjectFactory* o = NULL;
        getChildOfType ("link", o);
        linkName_ = root ()->prependPrefix (o->name ());
        JointPtr_t joint = device->getJointByBodyName (linkName_);
        Transform3f M; M.setIdentity ();
        if (o->hasAttribute ("index")) {
          // If there is an index, we consider the position are given relatively
          // to the "index"th collision object,
          const model::ObjectVector_t& objVector =
            joint->linkedBody ()->innerObjects (model::COLLISION);
          if (o->hasAttribute ("index"))
            objectName_ = linkName_ + "_" + o->getAttribute ("index");
          else
            objectName_ = linkName_ + "_0";
          /// In this case, coordinates are expressed in the body frame.
          bool found = false;
          for (model::ObjectVector_t::const_iterator it = objVector.begin ();
              it != objVector.end (); ++it) {
            if (root ()->prependPrefix ((*it)->name ())
                .compare (objectName_) == 0) {
              M = (*it)->positionInJointFrame ();
              found = true;
              break;
            }
          }
          if (!found) {
            hppDout (error, "Body " << objectName_ << " not found in link " << linkName_);
          }
        } else {
          // If there is no index, the position are relative to the link.
          M = joint->linkInJointFrame ();
        }

        getChildOfType ("point", o);
        PointFactory* pts = o->as <PointFactory>();

        /// First build the sequence of points
        const PointFactory::OutType& v = pts->values();
        if (v.size() % 3 != 0) throw std::length_error ("Point sequence size should be a multiple of 3.");
        std::vector < fcl::Vec3f > points;
        for (size_t i = 0; i < v.size (); i+=3)
          points.push_back (M.transform (fcl::Vec3f (v[i], v[i+1], v[i+2])));

        try {
          /// Construct shapes
          getChildOfType ("shape", o);
          ShapeFactory* shapes = o->as <ShapeFactory>();

          const ShapeFactory::OutType& shapeIdxs = shapes->values();
          std::size_t i_shape = 0;
          while (i_shape < shapeIdxs.size ()) {
            if (i_shape + shapeIdxs[i_shape] > shapeIdxs.size ())
              throw std::out_of_range ("shape should be a sequence of unsigned "
                  "integer: N iPoints_1 ... iPoints_N M iPoints_1 ... iPoints_M");
            Shape_t shape (shapeIdxs [i_shape]);
            for (size_t i = 1; i < shapeIdxs [i_shape] + 1; ++i)
              shape[i] = points [shapeIdxs [i_shape + i]];
            i_shape += shapeIdxs[i_shape] + 1;
            shapes_.push_back (JointAndShape_t (joint, shape));
          }
        } catch (const std::invalid_argument& e) {
          /// Backward compatibility
          try {
            getChildOfType ("triangle", o);
          } catch (const std::invalid_argument& eTri) {
            throw std::invalid_argument (e);
          }
          TriangleFactory* tri = o->as <TriangleFactory>();
          /// Group points by 3 to form triangle
          const TriangleFactory::OutType& indexes = tri->values();
          if (indexes.size() % 3 != 0) throw std::length_error ("Triangle sequence size should be a multiple of 3.");
          if (*std::max_element (indexes.begin (), indexes.end ()) >= points.size ())
            throw std::out_of_range ("triangle should be a sequence of unsigned integer lower than the number of points.");
          for (size_t i_tri = 0; i_tri < indexes.size (); i_tri+=3) {
            /// For each of the point indexes
            Shape_t t = boost::assign::list_of
              (points [indexes [i_tri  ]])
              (points [indexes [i_tri+1]])
              (points [indexes [i_tri+2]]);
            shapes_.push_back (JointAndShape_t (joint, t));
          }
          hppDout (info, "Triangles are deprecated and will be removed."
              " Use tag shape instead.");
        }

        device->add (root ()->prependPrefix (name ()), shapes_);
      }
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp
