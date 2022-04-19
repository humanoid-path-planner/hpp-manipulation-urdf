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

#include "hpp/manipulation/parser/factories/sequence.hh"

#include <boost/algorithm/string.hpp>
#include <hpp/util/debug.hh>

/* tinyxml 2.5.2 is missing this */
#ifndef TIXML_SSCANF
#define TIXML_SSCANF sscanf
#endif

namespace hpp {
namespace manipulation {
namespace parser {
namespace {
struct StringIsEmpty : public std::unary_function<std::string, bool> {
  bool operator()(std::string s) const { return s.empty(); }
};

template <typename ValueType>
bool cast(const std::string& str, ValueType* val) {
  hppDout(error, "Unkown type.");
  return false;
}

template <>
bool cast<int>(const std::string& str, int* val) {
  if (TIXML_SSCANF(str.c_str(), "%d", val) == 1) return true;
  return false;
}

template <>
bool cast<unsigned int>(const std::string& str, unsigned int* val) {
  if (TIXML_SSCANF(str.c_str(), "%u", val) == 1) return true;
  return false;
}

template <>
bool cast<double>(const std::string& str, double* val) {
  if (TIXML_SSCANF(str.c_str(), "%lf", val) == 1) return true;
  return false;
}

template <>
bool cast<float>(const std::string& str, float* val) {
  if (TIXML_SSCANF(str.c_str(), "%f", val) == 1) return true;
  return false;
}

template <>
bool cast<bool>(const std::string& str, bool* val) {
  int iVal;
  if (cast<int>(str, &iVal)) {
    *val = (iVal == 0) ? false : true;
    return true;
  }
  if (str.compare("true") == 0) {
    *val = true;
    return true;
  }
  if (str.compare("false") == 0) {
    *val = false;
    return true;
  }
  return false;
}
}  // namespace

template <typename Container>
void readSequence(const std::string& str, Container& out, int size) {
  typedef typename Container::value_type value_type;

  typedef std::vector<std::string> StringList;
  StringList values;

  boost::algorithm::split(values, str, boost::algorithm::is_any_of(" \n\t\r"),
                          boost::algorithm::token_compress_on);
  values.erase(std::remove_if(values.begin(), values.end(), StringIsEmpty()),
               values.end());
  if (size >= 0 && values.size() != (std::size_t)size) {
    std::ostringstream oss;
    oss << "Wrong sequence size, expecting " << size << ", got "
        << values.size() << " in " << str;
    throw std::invalid_argument(oss.str().c_str());
  }
  out.resize(values.size());

  value_type v;
  for (std::size_t i = 0; i < (std::size_t)out.size(); ++i) {
    if (!cast<value_type>(values[i], &v)) {
      throw std::invalid_argument("Failed to cast string " + values[i]);
    }
    out[i] = v;
  }
}

template <typename ValueType>
void SequenceFactory<ValueType>::addTextChild(const XMLText* text) {
  std::string t(text->Value());

  readSequence(t, values_, size_);
}

template class SequenceFactory<bool>;
template class SequenceFactory<int>;
template class SequenceFactory<unsigned int>;
template class SequenceFactory<double>;
template class SequenceFactory<float>;

template void readSequence<vector_t>(const std::string&, vector_t&, int);
template void readSequence<vector3_t>(const std::string&, vector3_t&, int);
}  // namespace parser
}  // namespace manipulation
}  // namespace hpp
