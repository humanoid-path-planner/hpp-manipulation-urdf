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

namespace hpp {
  namespace manipulation {
    namespace srdf {
/**

 \mainpage

 \section intro_sec Basic use

 The library contains parser for SRDF describing robot \ref Gripper "grippers",
 object \ref Handle "handles", object and environment
 \ref Contact "contact surfaces".

 In order to load HRP2 from a pair of URDF and SRDF
 files, one can do:
 \code
   #include <hpp/model/device.hh>
   #include <hpp/manipulation/srdf/util.hh>

   int main (int argc, char** argv) {
     DevicePtr_t robot = Device::create ("hrp-2");
     loadRobotModel (robot, "freeflyer", "hrp2_14_description", "hrp2_14", "", "");
   }

 \endcode

 \section srdf_syntac SRDF syntax

 \subsection Handle
    \include handle.xml

 \subsection Gripper
    \include gripper.xml

 \subsection Contact
    \include contact.xml

 \section extend_sec Extend the parser

 To extend the parser, you must write a class that inherits from
 parser::ObjectFactory. Some factories such as parser::SequenceFactory might be
 useful. You also have to declare the new factory to the parser:
 \code
 #include <hpp/manipulation/parser/parser.hh>

 // See ObjectFactory documentation for more details.
 class YourFactory : public hpp::manipulation::parser::ObjectFactory {
   YourFactory (ObjectFactory* parent, const XMLElement* element) :
         ObjectFactory (parent, element)
   {}
 };

 int main (int argc, char** argv) {
   // Parameter false tells the constructor not to include default factories.
   hpp::manipulation::parser::Parser p (false);
   p.addObjectFactory ("tagname", hpp::manipulation::parser::create <YourFactory>);
 }
 \endcode
 \see parser::ObjectFactory

**/
    }
  }
}
