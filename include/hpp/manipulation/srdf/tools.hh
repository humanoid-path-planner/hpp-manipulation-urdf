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

#ifndef HPP_MANIPULATION_SRDF_TOOLS_HH
# define HPP_MANIPULATION_SRDF_TOOLS_HH

# include <cstring>

namespace hpp {
  namespace manipulation {
    namespace srdf {
      namespace String {
        std::vector <char*> split (char* c, const char* d) {
          std::vector <char*> ret;
          char* split = strtok (c, d);
          while (split) {
            ret.push_back (split);
            split = strtok (NULL, d);
          }
          return ret;
        }
      }
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_SRDF_TOOLS_HH
