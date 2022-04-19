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

#include "hpp/manipulation/srdf/factories/handle.hh"

#include <hpp/manipulation/device.hh>
#include <hpp/manipulation/handle.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/pointer.hh>
#include <pinocchio/multibody/model.hpp>

#include "hpp/manipulation/srdf/factories/position.hh"

namespace hpp {
namespace manipulation {
namespace srdf {
void HandleFactory::finishTags() {
  ObjectFactoryList factories = getChildrenOfType("position");
  if (factories.empty()) {
    factories = getChildrenOfType("local_position");
    hppDout(warning, "Use tag position instead of local_position");
  }
  if (factories.size() != 1)
    throw std::invalid_argument("handle tag " + name() +
                                " should have exactly one <position>");
  PositionFactory* pf = factories.front()->as<PositionFactory>();
  localPosition_ = pf->position();
  factories = getChildrenOfType("link");
  if (factories.size() != 1)
    throw std::invalid_argument("handle tag " + name() +
                                " should have exactly one <link>");
  linkName_ = root()->prependPrefix(factories.front()->name());

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

  /// Get the mask
  factories = getChildrenOfType("mask");
  std::vector<bool> mask(6, true);
  if (factories.size() > 1) {
    hppDout(warning,
            "handle should have at most one <mask>. Using the first one");
  }
  if (!factories.empty()) {
    parser::SequenceFactory<bool>* mf =
        factories.front()->as<parser::SequenceFactory<bool> >();
    mask = mf->values();
  }

  /// Get the mask complement
  factories = getChildrenOfType("mask_complement");
  std::vector<bool> maskComp(6, false);
  if (factories.size() > 1) {
    hppDout(warning,
            "handle should have at most one <mask_complement>. Using the first "
            "one");
  }
  bool maskCompSpecified(false);
  if (!factories.empty()) {
    maskCompSpecified = true;
    parser::SequenceFactory<bool>* mf =
        factories.front()->as<parser::SequenceFactory<bool> >();
    maskComp = mf->values();
  }

  /// We have now all the information to build the handle.
  DevicePtr_t d = HPP_DYNAMIC_PTR_CAST(Device, root()->device());
  if (!d) {
    hppDout(error, "Failed to create handle");
    return;
  }
  const pinocchio::Model& model = d->model();
  if (!model.existBodyName(linkName_))
    throw std::invalid_argument("Link " + linkName_ +
                                " not found. Cannot create handle");
  const ::pinocchio::Frame& linkFrame =
      model.frames[model.getFrameId(linkName_)];
  assert(linkFrame.type == ::pinocchio::BODY);
  JointPtr_t joint(Joint::create(d, linkFrame.parent));
  // Handle position is expressed in link frame. We need to express it in
  // joint frame.
  handle_ = Handle::create(root()->prependPrefix(name()),
                           linkFrame.placement * localPosition_, d, joint);
  handle_->clearance(clearance);
  handle_->mask(mask);
  if (maskCompSpecified) handle_->maskComp(maskComp);
  d->handles.add(handle_->name(), handle_);
}

HandlePtr_t HandleFactory::handle() const { return handle_; }
}  // namespace srdf
}  // namespace manipulation
}  // namespace hpp
