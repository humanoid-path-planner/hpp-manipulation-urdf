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

#ifndef HPP_MANIPULATION_SRDF_UTIL_HH
# define HPP_MANIPULATION_SRDF_UTIL_HH

# include <hpp/manipulation/fwd.hh>
# include <hpp/manipulation/urdf/deprecated.hh>

namespace hpp {
  namespace manipulation {
    namespace srdf {
      /// Reads the tags from SRDF as defined in \ref hpp_manipulation_urdf_srdf_syntax
      void loadModelFromFile (const DevicePtr_t& robot,
          const std::string& prefix,
          const std::string& package,
          const std::string& modelName,
          const std::string& srdfSuffix);

      /// Reads the tags from SRDF as defined in \ref hpp_manipulation_urdf_srdf_syntax
      void loadModelFromXML (const DevicePtr_t& robot,
          const std::string& prefix,
          const std::string& srdfString);
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp
#endif // HPP_MANIPULATION_SRDF_UTIL_HH
