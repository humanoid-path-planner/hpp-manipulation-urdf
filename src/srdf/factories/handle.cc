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

#include <hpp/manipulation/handle.hh>
#include <hpp/manipulation/axial-handle.hh>

#include <hpp/manipulation/device.hh>

#include "hpp/manipulation/srdf/factories/position.hh"
#include "hpp/manipulation/srdf/factories/handle.hh"

namespace hpp {
  namespace manipulation {
    namespace srdf {
      template < typename HandleType>
      void HandleFactory<HandleType>::finishTags ()
      {
        ObjectFactoryList factories = getChildrenOfType ("position");
        if (factories.empty ()) {
          factories = getChildrenOfType ("local_position");
          hppDout (warning, "Use tag position instead of local_position");
        }
        if (factories.size () != 1) {
          hppDout (error, "handle should have exactly one <position>");
          return;
        }
        PositionFactory* pf = factories.front ()->as <PositionFactory> ();
        localPosition_ = pf->position ();
        factories = getChildrenOfType ("link");
        if (factories.size () != 1) {
          hppDout (error, "handle should have exactly one <link>");
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
        DevicePtr_t d = HPP_DYNAMIC_PTR_CAST (Device, root ()->device ());
        if (!d) {
          hppDout (error, "Failed to create handle");
          return;
        }
        JointPtr_t joint = root ()->device ()->getJointByBodyName (linkName_);
        handle_ = HandleType::create (root ()->prependPrefix (name ()),
            localPosition_, joint);
        handle_->clearance (clearance);
        d->add <HandlePtr_t> (handle_->name (), handle_);
      }

      template < typename HandleType>
      typename HandleFactory<HandleType>::HandleTypePtr_t
        HandleFactory<HandleType>::handle () const
      {
        return handle_;
      }

      template class HandleFactory <Handle>;
      template class HandleFactory <AxialHandle>;
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp
