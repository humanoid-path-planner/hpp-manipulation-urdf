// Copyright (c) 2014, LAAS-CNRS
// Authors: Joseph Mirabel (joseph.mirabel@laas.fr)
//

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
//
// 1. Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright
// notice, this list of conditions and the following disclaimer in the
// documentation and/or other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.

#include "hpp/manipulation/srdf/factories/gripper.hh"

#include <hpp/manipulation/device.hh>
#include <hpp/pinocchio/gripper.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/pointer.hh>
#include <pinocchio/multibody/model.hpp>

#include "hpp/manipulation/srdf/factories/position.hh"

#ifndef TIXML_SSCANF
#define TIXML_SSCANF sscanf
#endif

namespace hpp {
namespace manipulation {
namespace srdf {
void GripperFactory::finishTags() {
  ObjectFactoryList factories = getChildrenOfType("position");
  if (factories.empty()) {
    factories = getChildrenOfType("handle_position_in_joint");
    hppDout(warning, "Use tag position instead of handle_position_in_joint");
  }
  if (factories.size() != 1)
    throw std::invalid_argument("gripper tag " + name() +
                                " should have exactly one <position>");
  PositionFactory* pf = factories.front()->as<PositionFactory>();
  localPosition_ = pf->position();

  factories = getChildrenOfType("link");
  if (factories.size() != 1)
    throw std::invalid_argument("gripper tag " + name() +
                                " should have exactly one <link>");
  linkName_ = root()->prependPrefix(factories.front()->name());
  const std::string& gripperName = root()->prependPrefix(name());
  if (gripperName == linkName_)
    throw std::invalid_argument("Gripper " + gripperName +
                                " cannot have the same name as link " +
                                linkName_ +
                                ". "
                                "Cannot create gripper");

  /// Get the clearance
  value_type clearance = 0;
  if (hasAttribute("clearance")) {
    if (TIXML_SSCANF(getAttribute("clearance").c_str(), "%lf", &clearance) !=
        1) {
      hppDout(error, "Could not cast attribute clearance of tag "
                         << name() << " to double");
    }
  } else {
    hppDout(warning,
            "Missing attribute clearance of tag " << name() << ". Assuming 0");
  }

  /// We have now all the information to build the handle.
  DevicePtr_t d = root()->device();
  if (!d) {
    hppDout(error, "Failed to create gripper");
    return;
  }
  const pinocchio::Model& model = d->model();
  if (!model.existBodyName(linkName_))
    throw std::invalid_argument("Link " + linkName_ +
                                " not found. Cannot create gripper");
  pinocchio::FrameIndex linkFrameId = model.getFrameId(linkName_);
  const ::pinocchio::Frame& linkFrame = model.frames[linkFrameId];
  assert(linkFrame.type == ::pinocchio::BODY);
  // Gripper position is expressed in link frame. We need to compute
  // the position in joint frame.
  if (model.existFrame(gripperName, ::pinocchio::OP_FRAME))
    throw std::runtime_error("Could not add gripper frame of gripper " +
                             gripperName);
  d->model().addFrame(::pinocchio::Frame(
      gripperName, linkFrame.parent, linkFrameId,
      linkFrame.placement * localPosition_, ::pinocchio::OP_FRAME));
  d->createData();
  gripper_ = pinocchio::Gripper::create(gripperName, root()->device());
  gripper_->clearance(clearance);
  d->grippers.add(gripper_->name(), gripper_);
  hppDout(info, "Add gripper "
                    << gripper_->name() << "\n\tattached to joint "
                    << model.names[model.frames[linkFrameId].parent] << " with position "
                    << gripper_->objectPositionInJoint() << "\n\tclearance "
                    << clearance);
}

GripperPtr_t GripperFactory::gripper() const { return gripper_; }
}  // namespace srdf
}  // namespace manipulation
}  // namespace hpp
