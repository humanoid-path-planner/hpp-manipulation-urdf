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

#define BOOST_TEST_MODULE String 
#include <boost/test/included/unit_test.hpp>

#include "hpp/manipulation/srdf/tools.hh"

using namespace hpp::manipulation::srdf::String;

BOOST_AUTO_TEST_CASE (srdfparser)
{
  char s[] = "0 2323 -55 0.251";
  std::vector <char*> val = split (s, " ");
  for (size_t i = 0; i < val.size(); i++)
    std::cout << val[i] << std::endl;
  for (size_t i = 0; i < val.size(); i++)
    delete val[i];
}
