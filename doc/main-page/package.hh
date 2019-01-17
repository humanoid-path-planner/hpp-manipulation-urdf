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

/**

 \mainpage
 \anchor hpp_manipulation_urdf_documentation

 \section intro_sec Basic use

 The library contains parser for SRDF describing robot \ref Gripper "grippers",
 object \ref Handle "handles", object and environment
 \ref Contact "contact surfaces".

 In order to load HRP2 from a pair of URDF and SRDF
 files, one can do:
 \code
   #include <hpp/pinocchio/urdf/util.hh>

   #include <hpp/manipulation/device.hh>
   #include <hpp/manipulation/srdf/util.hh>

   int main (int argc, char** argv) {
     using hpp::manipulation::DevicePtr_t;
     using hpp::manipulation::Device;
     using hpp::manipulation::srdf::loadRobotModel;
     DevicePtr_t robot = Device::create ("hrp2");
     pinocchio::urdf::loadUrdfModel (robot, "freeflyer", "hrp2",
       "hrp2_14_description", "hrp2_14", "", "");
     loadModelFromFile (robot, "hrp2",
       "hrp2_14_description", "hrp2_14", "");
   }
 \endcode

 \section hpp_manipulation_urdf_srdf_syntax SRDF syntax

 \subsection Handle
    \include handle.xml

 \subsection Gripper
    \include gripper.xml

 \subsection Contact
    \include contact.xml

 \section hpp_manipulation_urdf_extend_sec Extend the parser

 To extend the parser, you must write a class that inherits from
 parser::ObjectFactory. Some \ref factories "factories" such as parser::SequenceFactory might be
 useful. You also have to declare the new factory to the parser:
 \code
 #include <hpp/manipulation/parser/parser.hh>

 // See ObjectFactory documentation for more details.
 // This factory parses something like:
 // <tagname>
 //   <position>0 0 0 1 0 0 0</position>
 //   <link name="linkname"/>
 // </tagname>
 class YourFactory : public hpp::manipulation::parser::ObjectFactory {
 public:
   YourFactory (ObjectFactory* parent, const XMLElement* element) :
         ObjectFactory (parent, element)
   {}

   void YourFactory::finishTags ()
   {
     ObjectFactory* o (NULL);
     if (!getChildOfType ("position", o)) {
       // There is more than one tag <position>
       // o is a pointer to the first one.
     }
     PositionFactory* pf = o->as <PositionFactory> ();
     Transform3f position = pf->position ();

     if (!getChildOfType ("link", o)) {
       // There is more than one tag <link>
       // o is a pointer to the first one.
     }
     std::string linkName = root ()->prependPrefix (o->name ());

     /// We have now all the information to build the handle.
     if (!root ()->device ()) {
       hppDout (error, "Device not found");
       return;
     }
     root ()->device ()->yourfunction (linkName, position);
   }

 };

 int main (int argc, char** argv) {
   // Parameter false tells the constructor not to include default factories.
   hpp::manipulation::parser::Parser p (false);
   p.addObjectFactory ("tagname", hpp::manipulation::parser::create <YourFactory>);
 }
 \endcode
 \see parser::ObjectFactory

**/
