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

#ifndef HPP_MANIPULATION_PARSER_FACTORIES_SEQUENCE_HH
# define HPP_MANIPULATION_PARSER_FACTORIES_SEQUENCE_HH

# include "hpp/manipulation/parser/parser.hh"

namespace hpp {
  namespace manipulation {
    namespace parser {
      /// \addtogroup factories
      /// \{
      template <typename Container>
        void readSequence (const std::string& str, Container& out, int size = -1);

      /// \brief  Factory parsing sequence of values.
      /// \tparam ValueType one of (bool, int, unsigned int, double, float)
      ///
      /// A std::vector is built from a sequence of values separeted by
      /// white spaces.
      template <typename ValueType>
      class SequenceFactory : public ObjectFactory {
        public:
          typedef std::vector <ValueType> OutType;
          SequenceFactory (ObjectFactory* parent, const XMLElement* element, const int nbValue = -1) :
            ObjectFactory (parent, element), size_ (nbValue)
        {}

          virtual void addTextChild (const XMLText* text);

          const OutType& values () const
          {
            return values_;
          }

        private:
          std::vector <ValueType> values_;
          int size_;
      };

      /// \}
    } // namespace parser
  } // namespace manipulation
} // namespace hpp

#endif // HPP_MANIPULATION_PARSER_FACTORIES_SEQUENCE_HH
