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

#include "hpp/manipulation/srdf/util.hh"

#include <hpp/util/exception-factory.hh>

#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/manipulation/device.hh>

#include "hpp/manipulation/parser/parser.hh"
#include "hpp/manipulation/srdf/factories/position.hh"

namespace hpp {
  namespace manipulation {
    namespace srdf {
      void loadModelFromFile (const DevicePtr_t& robot,
          const std::string& prefix,
          const std::string& package,
          const std::string& modelName,
          const std::string& srdfSuffix)
      {
        std::string srdfName = "package://" + package + "/srdf/"
          + modelName + srdfSuffix + ".srdf";
        loadModelFromFile (robot, prefix, srdfName);
      }

      void loadModelFromFile (const DevicePtr_t& robot,
          const std::string& prefix,
          const std::string& srdfName)
      {
        parser::Parser p;

        p.prefix(prefix);
        p.parseFile (srdfName, robot);
        hppDout (notice, "Finished parsing semantic informations.");
      }

      void loadModelFromXML (const DevicePtr_t& robot,
          const std::string& prefix,
          const std::string& srdfString)
      {
        parser::Parser p;
        p.prefix(prefix);
        p.parseString (srdfString, robot);
        hppDout (notice, "Finished parsing semantic informations.");
      }
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp
