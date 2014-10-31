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

#include "hpp/manipulation/srdf/factories.hh"

#include <hpp/util/debug.hh>
#include <hpp/util/pointer.hh>
#include <hpp/manipulation/handle.hh>
#include <hpp/manipulation/axial-handle.hh>

#include <hpp/model/device.hh>
#include <hpp/model/gripper.hh>
#include <hpp/manipulation/object.hh>

namespace hpp {
  namespace manipulation {
    namespace srdf {
      bool RobotFactory::finishAttributes ()
      {
        if (!root ()->device ()) {
          hppDout (error, "There is no Device");
          return false;
        }
        if (!root ()->device ()->name ().compare (name ()) != 0) {
          hppDout (error, "Device and XML robot tag have different names.");
          return false;
        }
        return true;
      }

      void PositionFactory::addTextChild (const XMLText* text)
      {
        std::stringstream t(text->Value ());
        std::string segment;
        std::vector<std::string> values;

        while(std::getline(t, segment, ' '))
        {
          if (segment.empty ())
            continue;
          values.push_back(segment);
        }
        if (values.size () != 7) {
          throw std::invalid_argument ("Position string must match (\\s*[+-]?[0-9]+(\\.[0-9]*)?\\s*){7}");
        }
        value_type v[7];
        for (size_t i = 0; i < 7; i++) {
          if (!XMLUtil::ToDouble (values[i].c_str (), &(v[i]))) {
            v[i] = 0;
            hppDout (error, "Position is not properly set.");
          }
        }

        p_ = Transform3f (fcl::Quaternion3f (v[3], v[4], v[5], v[6]),
            fcl::Vec3f (v[0], v[1], v[2]));
      }

      Transform3f PositionFactory::position () const
      {
        return p_;
      }

      void HandleFactory::finishTags ()
      {
        std::list <ObjectFactory*> factories = getChildrenOfType ("local_position");
        if (factories.size () != 1) {
          hppDout (error, "handle should have exactly one <local_position>");
          return;
        }
        PositionFactory* pf = factories.front ()->as <PositionFactory> ();
        localPosition_ = pf->position ();
        factories = getChildrenOfType ("link");
        if (factories.size () != 1) {
          hppDout (error, "handle should have exactly one <link>");
          return;
        }
        linkName_ = factories.front ()->name ();

        /// We have now all the information to build the handle.
        ObjectPtr_t o = HPP_DYNAMIC_PTR_CAST (Object, root ()->device ());
        if (!o) {
          hppDout (error, "Failed to create handle");
          return;
        }
        JointPtr_t joint = root ()->device ()->getJointByBodyName (linkName_);
        handle_ = Handle::create (name (), localPosition_, joint);
        o->addHandle (handle_);
      }

      HandlePtr_t HandleFactory::handle () const
      {
        return handle_;
      }

      void GripperFactory::finishTags ()
      {
        ObjectFactoryList factories = getChildrenOfType ("handle_position_in_joint");
        if (factories.size () != 1) {
          hppDout (error, "gripper should have exactly one <handle_position_in_joint>");
          return;
        }
        PositionFactory* pf = factories.front ()->as <PositionFactory> ();
        localPosition_ = pf->position ();

        factories = getChildrenOfType ("link");
        if (factories.size () != 1) {
          hppDout (error, "gripper should have exactly one <link>");
          return;
        }
        linkName_ = factories.front ()->name ();

        factories = getChildrenOfType ("disable_collision");
        for (ObjectFactoryList::const_iterator it = factories.begin ();
            it != factories.end (); it++)
          collisionLinks_.push_back ((*it)->getAttribute ("link"));

        /// We have now all the information to build the handle.
        if (!root ()->device ()) {
          hppDout (error, "Failed to create gripper");
          return;
        }
        model::JointVector_t joints;
        for (std::list <std::string>::const_iterator it = collisionLinks_.begin ();
            it != collisionLinks_.end (); it++) {
          joints.push_back (root ()->device ()->getJointByBodyName (*it));
        }
        JointPtr_t joint = root ()->device ()->getJointByBodyName (linkName_);
        gripper_ = model::Gripper::create (name (), joint, localPosition_, joints);
        root ()->device ()->addGripper (gripper_);
      }

      GripperPtr_t GripperFactory::gripper () const
      {
        return gripper_;
      }
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp
