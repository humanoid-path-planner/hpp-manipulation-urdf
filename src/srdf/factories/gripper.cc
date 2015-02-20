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

#include <hpp/util/debug.hh>
#include <hpp/util/pointer.hh>

#include <hpp/model/gripper.hh>

#include <hpp/manipulation/device.hh>
#include "hpp/manipulation/srdf/factories/position.hh"
#include "hpp/manipulation/srdf/factories/gripper.hh"

namespace hpp {
  namespace manipulation {
    namespace srdf {
      void GripperFactory::finishTags ()
      {
        ObjectFactoryList factories = getChildrenOfType ("position");
        if (factories.empty ()) {
          factories = getChildrenOfType ("handle_position_in_joint");
          hppDout (warning, "Use tag position instead of handle_position_in_joint");
        }
        if (factories.size () != 1) {
          hppDout (error, "gripper should have exactly one <position>");
          return;
        }
        PositionFactory* pf = factories.front ()->as <PositionFactory> ();
        localPosition_ = pf->position ();

        factories = getChildrenOfType ("link");
        if (factories.size () != 1) {
          hppDout (error, "gripper should have exactly one <link>");
          return;
        }
        linkName_ = root ()->prependPrefix (factories.front ()->name ());

        factories = getChildrenOfType ("disable_collision");
        for (ObjectFactoryList::const_iterator it = factories.begin ();
            it != factories.end (); ++it)
          collisionLinks_.push_back (
              root ()->prependPrefix ((*it)->getAttribute ("link")));

        /// We have now all the information to build the handle.
        if (!root ()->device ()) {
          hppDout (error, "Failed to create gripper");
          return;
        }
        model::JointVector_t joints;
        for (std::list <std::string>::const_iterator it = collisionLinks_.begin ();
            it != collisionLinks_.end (); ++it) {
          joints.push_back (root ()->device ()->getJointByBodyName (*it));
        }
        JointPtr_t joint = root ()->device ()->getJointByBodyName (linkName_);
        gripper_ = model::Gripper::create (
            root ()->prependPrefix (name ()), joint, localPosition_, joints);
        root ()->device ()->add (gripper_->name (), gripper_);
      }

      GripperPtr_t GripperFactory::gripper () const
      {
        return gripper_;
      }
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp
