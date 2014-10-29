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

#include <stdexcept>

#include <hpp/util/debug.hh>

#include "hpp/manipulation/srdf/parser.hh"

namespace hpp {
  namespace manipulation {
    namespace srdf {
      Parser::Parser ()
      {}

      void Parser::parseFile (const char* filename)
      {
        doc_.LoadFile (filename);
        if (doc_.Error ()) {
          doc_.PrintError ();
          return;
        }
        const XMLElement* el = doc_.FirstChildElement ();
        root_ = ObjectFactory::create (NULL, NULL);
        while (el != NULL) {
          parseElement (el, NULL);
          el = el->NextSiblingElement ();
        }
        for (ObjectFactoryList::iterator it = objectFactories_.begin ();
            it != objectFactories_.end (); it++)
          (*it)->finishFile ();
      }

      void Parser::addObjectFactory (const std::string& tagname, FactoryType factory)
      {
        ObjectFactoryInsertRet ret = objFactoryMap_.insert (ObjectFactoryPair (tagname, factory));
        if (!ret.second)
          throw std::logic_error ("This tagname already exist");
      }

      void Parser::parseElement (const XMLElement* element, ObjectFactory* parent)
      {
        if (element == NULL)
          return;

        ObjectFactory* o = NULL;
        /// Look for this element in the map
        ObjectFactoryMap::const_iterator it = objFactoryMap_.find (element->Name ());
        if (it != objFactoryMap_.end ()) {
          o = it->second (parent, element);
        } else {
          o = DefaultFactory::create (parent, element);
        }
        objectFactories_.push_back (o);
        o->init ();
        const XMLAttribute* attr = element->FirstAttribute (); 
        while (attr != NULL) {
          o->setAttribute (attr);
          attr = attr->Next ();
        }
        o->finishAttributes ();

        /// Loop over is child tags
        if (o != NULL) {
          const XMLNode* el = element->FirstChild ();
          while (el != NULL) {
            if (el->ToElement () != NULL) {
              parseElement (el->ToElement (), o);
            } else if (el->ToUnknown () != NULL) {
              hppDout (warning, "Unknown Node in XML file: " << el->Value ());
            } else if (el->ToText () != NULL) {
              o->addTextChild (el->ToText ());
            } else if (el->ToComment () != NULL) {
            }

            el = el->NextSibling ();
          }
          o->finishTags ();
        } else {
          const XMLElement* el = element->FirstChildElement ();
          while (el != NULL) {
            parseElement (el, o);
            el = el->NextSiblingElement ();
          }
        }
      }

      std::ostream& Parser::print (std::ostream& os) const
      {
        return os << "Parser with " << objectFactories_.size () << " object." << std::endl;
      }
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp
