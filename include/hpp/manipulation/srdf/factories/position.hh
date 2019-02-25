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

#ifndef HPP_MANIPULATION_SRDF_FACTORIES_POSITION_HH
# define HPP_MANIPULATION_SRDF_FACTORIES_POSITION_HH

# include <hpp/manipulation/fwd.hh>

# include <pinocchio/spatial/se3.hpp>
# include "hpp/manipulation/parser/factories/sequence.hh"

namespace hpp {
  namespace manipulation {
    namespace srdf {
      /// \addtogroup factories
      /// \{

      /// \brief Build a fcl::Transform.
      ///
      /// The sequence of number in the XML text must:
      /// \li be of length 7;
      /// \li begin with the translation (3 coordinates);
      /// \li end with a quaternion (4 coordinates).
      class PositionFactory : public parser::SequenceFactory <float> {
        public:
          PositionFactory (ObjectFactory* parent, const parser::XMLElement* element) :
            SequenceFactory <float> (parent, element, 7) {}

          virtual void finishTags ();

          const Transform3f& position () const
          {
            return position_;
          }

        private:
          void computeTransformFromText ();
          void computeTransformFromAttributes ();

          Transform3f position_;
      };

      /// \}
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_SRDF_FACTORIES_POSITION_HH
