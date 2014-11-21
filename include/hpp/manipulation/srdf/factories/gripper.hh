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

#ifndef HPP_MANIPULATION_SRDF_FACTORIES_GRIPPER_HH
# define HPP_MANIPULATION_SRDF_FACTORIES_GRIPPER_HH

# include <hpp/manipulation/fwd.hh>
# include <hpp/model/fwd.hh>
# include <fcl/math/transform.h>
# include <fcl/shape/geometric_shapes.h>

# include "hpp/manipulation/srdf/parser.hh"

namespace hpp {
  namespace manipulation {
    namespace srdf {
      /// \brief Build an object of type hpp::model::Gripper.
      class GripperFactory : public ObjectFactory {
        public:
          GripperFactory (ObjectFactory* parent, const XMLElement* element) :
            ObjectFactory (parent, element) {}

          virtual void finishTags ();

          GripperPtr_t gripper () const;

        protected:
          GripperPtr_t gripper_;

          /// The element required to build the gripper.
          Transform3f localPosition_;
          std::string linkName_;
          std::list <std::string> collisionLinks_;
      };
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_SRDF_FACTORIES_GRIPPER_HH
