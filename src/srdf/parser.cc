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
#include <resource_retriever/retriever.h>

#include "hpp/manipulation/srdf/parser.hh"

namespace hpp {
  namespace manipulation {
    namespace srdf {
      Parser::Parser ()
      {}

      Parser::~Parser ()
      {
        for (ObjectFactoryList::iterator it = objectFactories_.begin ();
            it != objectFactories_.end (); it++)
          delete *it;
      }

      void Parser::parse (const std::string& semanticResourceName,
		     model::DevicePtr_t robot)
      {
        device_ = robot;

	resource_retriever::Retriever resourceRetriever;

	resource_retriever::MemoryResource semanticResource =
	  resourceRetriever.get(semanticResourceName);
	char* semanticDescription = new char[semanticResource.size + 1];
	for (unsigned i = 0; i < semanticResource.size; ++i)
          semanticDescription[i] = semanticResource.data.get()[i];
        semanticDescription[semanticResource.size] = '\0';

        loadString (semanticDescription);
        parse ();
        delete semanticDescription;
      }

      void Parser::parseFile (const char* filename)
      {
        loadFile (filename);
        parse ();
      }

      void Parser::loadFile (const char* filename)
      {
        doc_.LoadFile (filename);
        if (doc_.Error ()) {
          doc_.PrintError ();
          return;
        }
      }

      void Parser::loadString (const char* xmlstring)
      {
        doc_.Parse (xmlstring);
        if (doc_.Error ()) {
          doc_.PrintError ();
        }
      }

      void Parser::parse ()
      {
        const XMLElement* el = doc_.FirstChildElement ();
        root_ = RootFactory (device_);
        while (el != NULL) {
          parseElement (el, &root_);
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
          o = create <DefaultFactory> (parent, element);
          hppDout (warning, "I have no factory for tag " << o->tagName ());
        }
        objectFactories_.push_back (o);
        if (!o->init ()) return;
        const XMLAttribute* attr = element->FirstAttribute (); 
        while (attr != NULL) {
          o->setAttribute (attr);
          attr = attr->Next ();
        }
        if (!o->finishAttributes ()) return;

        /// Loop over is child tags
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
      }

      std::ostream& Parser::print (std::ostream& os) const
      {
        os << "Parser with " << objectFactories_.size () << " object." << std::endl;
        os << root_;
        return os;
      }

      std::ostream& operator<< (std::ostream& os, const Parser& p)
      {
        return p.print (os);
      }

      std::ostream& operator<< (std::ostream& os, const ObjectFactory& o)
      {
        return o.print (os);
      }

      ObjectFactory::ObjectFactory (ObjectFactory* parent, const XMLElement* element)
        : parent_ (parent), root_ (NULL), element_ (element), id_ (-1)
      {
        if (parent_ == NULL) {
          root_ = dynamic_cast <RootFactory*> (this);
          if (root_ == NULL)
            throw std::logic_error ("ObjectFactory with no parent must be RootFactory");
        }
        else {
          root_ = parent_->root ();
          if (element_ != NULL)
            parent_->addChild (this);
        }
      }

      ObjectFactory::ObjectFactory (RootFactory* root) :
        parent_ (NULL), root_ (root), element_ (NULL), id_ (-1)
      {}

      bool ObjectFactory::init ()
      {
        return true;
      }

      bool ObjectFactory::finishAttributes ()
      {
        return true;
      }

      void ObjectFactory::finishTags ()
      {}

      void ObjectFactory::finishFile ()
      {}

      void ObjectFactory::addTextChild (const XMLText* /* text */)
      {}

      std::string ObjectFactory::tagName () const
      {
        if (element_ != NULL)
          return element_->Name ();
        return "No element";
      }

      std::string ObjectFactory::name () const
      {
        return name_;
      }

      void ObjectFactory::name (const std::string& n)
      {
        name_ = n;
      }

      void ObjectFactory::name (const char* n)
      {
        name (std::string (n));
      }

      ObjectFactory* ObjectFactory::parent ()
      {
        return parent_;
      }

      RootFactory* ObjectFactory::root ()
      {
        //if (parent_ == NULL)
        //return this;
        return root_;
      }

      bool ObjectFactory::hasParent () const
      {
        return parent_ != NULL;
      }

      const XMLElement* ObjectFactory::XMLelement ()
      {
        return element_;
      }

      void ObjectFactory::impl_setAttribute (const XMLAttribute* /* attr */)
      {}

      void ObjectFactory::addChild (ObjectFactory* child)
      {
        children_ [child->tagName ()].push_back (child);
      }

      std::list <ObjectFactory*> ObjectFactory::getChildrenOfType (std::string type)
      {
        return children_ [type];
      }

      bool ObjectFactory::getChildOfType (std::string type, ObjectFactory*& o)
      {
        ObjectFactoryList l = getChildrenOfType (type);
        if (l.empty ()) {
          throw std::invalid_argument ("Tag " + tagName () + " has several children of type " + type);
        }
        o = l.front ();
        if (l.size () != 1) {
          hppDout (warning, "Tag " << tagName () << " has several children of type " << type
              << ". All but the first will be ignored.");
          return false;
        }
        return true;
      }

      std::ostream& ObjectFactory::print (std::ostream& os) const
      {
        os << "ObjectFactory " << tagName () << " with name " << name () << std::endl;
        for (ChildrenMap::const_iterator itTagName = children_.begin ();
            itTagName != children_.end (); itTagName++)
          for (ObjectFactoryList::const_iterator itObj = itTagName->second.begin ();
              itObj != itTagName->second.end (); itObj++)
            os << **itObj;
        return os;
      }

      void ObjectFactory::setAttribute (const XMLAttribute* attr)
      {
        std::string n = std::string (attr->Name ());
        if (n == "name")
          name (attr->Value ());
        else if (n == "id") {
          unsigned int v;
          if (attr->QueryUnsignedValue (&v) != tinyxml2::XML_NO_ERROR) {
            hppDout (error, "Attribute ID " << attr->Value () << " is incorrect.");
          } else {
            id_ = (int)v;
          }
        }
        attrMap_ [n] = attr->Value ();
        impl_setAttribute (attr);
      }

      bool ObjectFactory::hasAttribute (const std::string& attr) const
      {
        return attrMap_.find (attr) != attrMap_.end ();
      }

      std::string ObjectFactory::getAttribute (const std::string& attr) const
      {
        AttributeMap::const_iterator it = attrMap_.find (attr);
        if (it == attrMap_.end ()) {
          hppDout (error, "Asking for attribute " << attr);
          return std::string ();
        }
        return it->second;
      }

      RootFactory::RootFactory (const model::DevicePtr_t dev) :
        ObjectFactory (this), device_ (dev)
      {}

      model::DevicePtr_t RootFactory::device () const
      {
        return device_;
      }
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp
