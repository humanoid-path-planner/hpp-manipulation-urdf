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
   #include <hpp/manipulation/device.hh>
   #include <hpp/manipulation/srdf/util.hh>
   #include <hpp/pinocchio/urdf/util.hh>

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
 parser::ObjectFactory. Some \ref factories "factories" such as
parser::SequenceFactory might be useful. You also have to declare the new
factory to the parser: \code #include <hpp/manipulation/parser/parser.hh>

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
   p.addObjectFactory ("tagname", hpp::manipulation::parser::create
<YourFactory>);
 }
 \endcode
 \see parser::ObjectFactory

**/
