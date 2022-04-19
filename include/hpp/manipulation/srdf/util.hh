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

#ifndef HPP_MANIPULATION_SRDF_UTIL_HH
#define HPP_MANIPULATION_SRDF_UTIL_HH

#include <hpp/manipulation/fwd.hh>
#include <hpp/manipulation/urdf/deprecated.hh>

namespace hpp {
namespace manipulation {
namespace srdf {
/// Reads the tags from SRDF as defined in \ref
/// hpp_manipulation_urdf_srdf_syntax \param robot robot to modify with the
/// information stored in srdf file, \param prefix prefix to insert before tags
/// of the srdf file to get
///        the corresponding object in the robot model
/// \param modelName part of the filename,
/// \param srdfSuffix part of the filename.
///
/// The srdf file is retrieve by the following string:
/// "package://${package}/srdf/${modelName}${srdfSuffix}.srdf".
///
void loadModelFromFile(const DevicePtr_t& robot, const std::string& prefix,
                       const std::string& package, const std::string& modelName,
                       const std::string& srdfSuffix);

/// Reads the tags from SRDF as defined in \ref
/// hpp_manipulation_urdf_srdf_syntax \param prefix prefix to insert before tags
/// of the srdf file to get
///        the corresponding object in the robot model
/// \param srdfName name of the srdf file; may contain "package://" or
///        "file://".
void loadModelFromFile(const DevicePtr_t& robot, const std::string& prefix,
                       const std::string& srdfName);

/// Reads the tags from SRDF as defined in \ref
/// hpp_manipulation_urdf_srdf_syntax
void loadModelFromXML(const DevicePtr_t& robot, const std::string& prefix,
                      const std::string& srdfString);
}  // namespace srdf
}  // namespace manipulation
}  // namespace hpp
#endif  // HPP_MANIPULATION_SRDF_UTIL_HH
