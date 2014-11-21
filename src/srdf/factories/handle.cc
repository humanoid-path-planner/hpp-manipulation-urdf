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

#include <hpp/model/device.hh>
#include <hpp/manipulation/object.hh>

#include "hpp/manipulation/srdf/factories/position.hh"
#include "hpp/manipulation/srdf/factories/handle.hh"

namespace hpp {
  namespace manipulation {
    namespace srdf {
      void AxialHandleFactory::finishTags ()
      {
        ObjectFactoryList factories = getChildrenOfType ("position");
        if (factories.empty ()) {
          factories = getChildrenOfType ("local_position");
          hppDout (warning, "Use tag position instead of local_position");
        }
        if (factories.size () != 1) {
          hppDout (error, "axial handle should have exactly one <position>");
          return;
        }
        PositionFactory* pf = factories.front ()->as <PositionFactory> ();
        localPosition_ = pf->position ();
        factories = getChildrenOfType ("link");
        if (factories.size () != 1) {
          hppDout (error, "axial handle should have exactly one <link>");
          return;
        }
        linkName_ = factories.front ()->name ();

        /// We have now all the information to build the handle.
        ObjectPtr_t o = HPP_DYNAMIC_PTR_CAST (Object, root ()->device ());
        if (!o) {
          hppDout (error, "Failed to create axial handle");
          return;
        }
        JointPtr_t joint = root ()->device ()->getJointByBodyName (linkName_);
        handle_ = AxialHandle::create (name (), localPosition_, joint);
        o->addHandle (handle_);
      }

      AxialHandlePtr_t AxialHandleFactory::handle () const
      {
        return handle_;
      }

      void HandleFactory::finishTags ()
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
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp
