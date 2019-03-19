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

#include <boost/algorithm/string.hpp>

#include "hpp/manipulation/parser/factories/sequence.hh"

namespace hpp {
  namespace manipulation {
    namespace parser {
      namespace {
        struct StringIsEmpty : public std::unary_function<std::string, bool>{
          bool operator () (std::string s) const {return s.empty ();}
        };

        template <typename ValueType> bool cast (const std::string& str, ValueType* val)
        {
          hppDout (error, "Unkown type.");
          return false;
        }

        template <> bool cast <int> (const std::string& str, int* val)
        {
          if ( TIXML_SSCANF (str.c_str (), "%d", val) == 1 )
            return true;
          return false;
        }

        template <> bool cast <unsigned int> (const std::string& str, unsigned int* val)
        {
          if ( TIXML_SSCANF (str.c_str (), "%u", val) == 1 )
            return true;
          return false;
        }

        template <> bool cast <double> (const std::string& str, double* val)
        {
          if ( TIXML_SSCANF (str.c_str (), "%lf", val) == 1 )
            return true;
          return false;
        }

        template <> bool cast <float> (const std::string& str, float* val)
        {
          if ( TIXML_SSCANF (str.c_str (), "%f", val) == 1 )
            return true;
          return false;
        }

        template <> bool cast <bool> (const std::string& str, bool* val)
        {
          int iVal;
          if (cast <int> (str, &iVal)) {
            *val = (iVal == 0) ? false : true;
            return true;
          }
          if (str.compare ("true") == 0) {
            *val = true;
            return true;
          }
          if (str.compare ("false") == 0) {
            *val = false;
            return true;
          }
          return false;
        }
      }

      template <typename Container>
      void readSequence (const std::string& str, Container& out, int size)
      {
        typedef typename Container::value_type value_type;

        typedef std::vector<std::string> StringList;
        StringList values;

        boost::algorithm::split (values, str,
            boost::algorithm::is_any_of (" \n\t\r"),
            boost::algorithm::token_compress_on);
        std::remove_if (values.begin(), values.end(), StringIsEmpty());
        if (size >= 0 && values.size () != (std::size_t)size) {
          std::ostringstream oss;
          oss << "Wrong sequence size, expecting " << size << ", got "
              << values.size () << " in " << str;
          throw std::invalid_argument (oss.str ().c_str ());
        }
        out.resize (values.size());

        value_type v;
        for (std::size_t i = 0; i < (std::size_t)out.size(); ++i) {
          if (!cast <value_type> (values[i], &v)) {
            throw std::invalid_argument ("Failed to cast string " + values[i]);
          }
          out[i] = v;
        }
      }

      template <typename ValueType>
      void SequenceFactory<ValueType>::addTextChild (const XMLText* text)
      {
        std::string t(text->Value ());

        readSequence (t, values_, size_);
      }

      template class SequenceFactory <bool>;
      template class SequenceFactory <int>;
      template class SequenceFactory <unsigned int>;
      template class SequenceFactory <double>;
      template class SequenceFactory <float>;

      template void readSequence <vector_t > (const std::string&, vector_t &, int);
      template void readSequence <vector3_t> (const std::string&, vector3_t&, int);
    } // namespace parser
  } // namespace manipulation
} // namespace hpp
