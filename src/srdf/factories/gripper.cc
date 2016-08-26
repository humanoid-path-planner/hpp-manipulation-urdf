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

#include "hpp/manipulation/srdf/factories/gripper.hh"

#include <pinocchio/multibody/model.hpp>

#include <hpp/util/debug.hh>
#include <hpp/util/pointer.hh>

#include <hpp/pinocchio/gripper.hh>

#include <hpp/manipulation/device.hh>
#include "hpp/manipulation/srdf/factories/position.hh"

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

        /// Get the clearance
        value_type clearance = 0;
        if (hasAttribute ("clearance")) {
          if (TIXML_SSCANF (
                getAttribute ("clearance").c_str (), "%lf", &clearance
                ) != 1 ) {
            hppDout (error, "Could not cast attribute clearance of tag "
                << name () << " to double");
          }
        } else {
          hppDout (warning, "Missing attribute clearance of tag "
              << name () << ". Assuming 0");
        }

        /// We have now all the information to build the handle.
        DevicePtr_t d = root()->device();
        if (!d) {
          hppDout (error, "Failed to create gripper");
          return;
        }
        const se3::Model& model = d->model();
        if (!model.existBodyName(linkName_))
          throw std::invalid_argument ("Link " + linkName_ + " not found. Cannot create gripper");
        const se3::Frame& linkFrame = model.frames[model.getFrameId(linkName_)];
        assert(linkFrame.type == se3::BODY);
	// Gripper position is expressed in link frame. We need to compute
	// the position in joint frame.
        d->model().addFrame (se3::Frame(
              root ()->prependPrefix (name ()),
              linkFrame.parent,
              linkFrame.placement * localPosition_,
              se3::OP_FRAME
              ));
        gripper_ = pinocchio::Gripper::create
	  (root ()->prependPrefix (name ()), root()->device());
        gripper_->clearance (clearance);
        d->add (gripper_->name (), gripper_);
      }

      GripperPtr_t GripperFactory::gripper () const
      {
        return gripper_;
      }
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp
