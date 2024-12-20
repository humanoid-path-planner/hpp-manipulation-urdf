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

#ifndef HPP_MANIPULATION_SRDF_FACTORIES_POSITION_HH
#define HPP_MANIPULATION_SRDF_FACTORIES_POSITION_HH

#include <hpp/manipulation/fwd.hh>
#include <pinocchio/spatial/se3.hpp>

#include "hpp/manipulation/parser/factories/sequence.hh"

namespace hpp {
namespace manipulation {
namespace srdf {
/// \addtogroup factories
/// \{

/// \brief Build a coal::Transform.
///
/// The sequence of number in the XML text must:
/// \li be of length 7;
/// \li begin with the translation (3 coordinates);
/// \li end with a quaternion (4 coordinates).
class PositionFactory : public parser::SequenceFactory<float> {
 public:
  PositionFactory(ObjectFactory* parent, const parser::XMLElement* element)
      : SequenceFactory<float>(parent, element, 7) {}

  virtual void finishTags();

  const Transform3s& position() const { return position_; }

 private:
  void computeTransformFromText();
  void computeTransformFromAttributes();

  Transform3s position_;
};

/// \}
}  // namespace srdf
}  // namespace manipulation
}  // namespace hpp

#endif  // HPP_MANIPULATION_SRDF_FACTORIES_POSITION_HH
