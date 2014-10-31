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

#ifndef HPP_MANIPULATION_SRDF_FACTORIES_HH
# define HPP_MANIPULATION_SRDF_FACTORIES_HH

# include <hpp/manipulation/fwd.hh>
# include <hpp/model/fwd.hh>
# include <fcl/math/transform.h>
# include "hpp/manipulation/srdf/parser.hh"

namespace hpp {
  namespace manipulation {
    namespace srdf {
      /// This class only check if the robot name and
      /// the attribute "name" of tag "robot" are the same.
      class RobotFactory : public ObjectFactory {
        public:
          RobotFactory (ObjectFactory* parent, const XMLElement* element) :
            ObjectFactory (parent, element) {}

          /// \return true iif the robot name and
          /// the attribute "name" of tag "robot" are equal.
          bool finishAttributes ();
      };

      template <typename ValueType>
      class SequenceFactory : public ObjectFactory {
        public:
          SequenceFactory (ObjectFactory* parent, const XMLElement* element, const unsigned int nbValue = 0) :
            ObjectFactory (parent, element), size_ (nbValue)
        {
          if (size_ > 0)
            values_.resize (size_);
        }

          virtual void addTextChild (const XMLText* text);

          std::vector <ValueType> values () const
          {
            return values_;
          }

        private:
          std::vector <ValueType> values_;
          unsigned int size_;
      };

      /// \brief Build a fcl::Transform.
      ///
      /// The sequence of number in the XML text must:
      /// \li be of length 7;
      /// \li begin with the translation (3 coordinates);
      /// \li end with a quaternion (4 coordinates).
      class PositionFactory : public SequenceFactory <float> {
        public:
          PositionFactory (ObjectFactory* parent, const XMLElement* element) :
            SequenceFactory <float> (parent, element, 7) {}

          Transform3f position () const;
      };

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

      /// \brief Build an object of type hpp::manipulation::Handle.
      class HandleFactory : public ObjectFactory {
        public:
          HandleFactory (ObjectFactory* parent, const XMLElement* element) :
            ObjectFactory (parent, element) {}

          virtual void finishTags ();

          HandlePtr_t handle () const;

        protected:
          HandlePtr_t handle_;

          /// The element required to build the handle
          Transform3f localPosition_;
          std::string linkName_;
      };
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_SRDF_FACTORIES_HH
