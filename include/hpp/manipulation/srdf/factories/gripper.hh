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

#ifndef HPP_MANIPULATION_SRDF_FACTORIES_GRIPPER_HH
#define HPP_MANIPULATION_SRDF_FACTORIES_GRIPPER_HH

#include <hpp/manipulation/fwd.hh>
#include <pinocchio/spatial/se3.hpp>

#include "hpp/manipulation/parser/parser.hh"

namespace hpp {
namespace manipulation {
namespace srdf {
/// \addtogroup factories
/// \{

/// \brief Build an object of type hpp::pinocchio::Gripper.
class GripperFactory : public parser::ObjectFactory {
 public:
  GripperFactory(ObjectFactory* parent, const parser::XMLElement* element)
      : ObjectFactory(parent, element) {}

  virtual void finishTags();

  GripperPtr_t gripper() const;

 protected:
  GripperPtr_t gripper_;

  /// The element required to build the gripper.
  Transform3f localPosition_;
  std::string linkName_;
  std::list<std::string> collisionLinks_;
};

/// \}
}  // namespace srdf
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_SRDF_FACTORIES_GRIPPER_HH
