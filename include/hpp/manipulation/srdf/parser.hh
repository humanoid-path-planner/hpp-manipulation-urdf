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

#ifndef HPP_MANIPULATION_SRDF_PARSER_HH
# define HPP_MANIPULATION_SRDF_PARSER_HH

# include <map>
# include <list>
# include <string>
# include <iostream>
# include <tinyxml2.h>

# include <hpp/model/fwd.hh>

namespace hpp {
  namespace manipulation {
    namespace srdf {
      using tinyxml2::XMLElement;
      using tinyxml2::XMLDocument;
      using tinyxml2::XMLAttribute;
      using tinyxml2::XMLNode;
      using tinyxml2::XMLText;
      using tinyxml2::XMLUtil;

      class RootFactory;

      /// \brief Class that catch XML Parser events for a specific tag and build the corresponding
      /// Object.
      ///
      /// Derive this class if you wish to extend the Parser.
      /// \note The derived class must have the following construtor
      /// \code
      /// DerivedFactory (ObjectFactory* parent, const XMLElement* element) :
      ///       ObjectFactory (parent, element)
      /// {
      ///   /*
      ///    * Keep in mind that it might be more convenient
      ///    * to build objects in a event callback, when all the needed information
      ///    * are already parsed.
      ///    */
      /// }
      /// \endcode
      class ObjectFactory {
        public:
          typedef std::list <ObjectFactory*> ObjectFactoryList;

          ObjectFactory (ObjectFactory* parent = NULL, const XMLElement* element = NULL);

          /// Called when the object is created.
          /// \return True to continue parsing this tag, False otherwise.
          virtual bool init ();

          /// Called for each attribute.
          /// A few reserved name are automatocally catched. The reserved names are
          /// "name" and "id".
          /// "name" expects a string.
          /// "id" expects an unsigned integer and can be use to define pointers to
          /// elements.
          void setAttribute (const XMLAttribute* attr);

          /// Called when all the attributes have been processed.
          /// \return True to continue parsing this tag, False otherwise.
          virtual bool finishAttributes ();

          /// Add Text child.
          virtual void addTextChild (const XMLText* text);

          /// Called when all the child tags have been processed.
          virtual void finishTags ();

          /// Called when parsing is finished.
          virtual void finishFile ();

          /// Return tag name of the element is any.
          /// Returns "No element" otherwise.
          std::string tagName () const;

          /// Return the content of the attribute name, or an
          /// empty string.
          std::string name () const;

          /// Set the name.
          /// The default value is the value of the attribute "name"
          /// of the XML tag or an empty string if this does not exist.
          void name (const std::string& n);

          /// See name(const std::string&)
          void name (const char* n);

          bool hasAttribute (const std::string& attr) const;

          std::string getAttribute (const std::string& attr) const;

          /// Cast this class to any child class.
          template <typename T> T* as ()
          {
            return static_cast <T*> (this);
          }

          ObjectFactoryList getChildrenOfType (std::string type);

          bool getChildOfType (std::string type, ObjectFactory*& o);

        protected:
          ObjectFactory (RootFactory* root);

          ObjectFactory* parent ();

          RootFactory* root ();

          bool hasParent () const;

          const XMLElement* XMLelement ();

          virtual void impl_setAttribute (const XMLAttribute* /* attr */);

          void addChild (ObjectFactory* child);

          virtual std::ostream& print (std::ostream& os) const;

        private:
          ObjectFactory* parent_;
          RootFactory* root_;
          typedef std::map <std::string, ObjectFactoryList > ChildrenMap;
          ChildrenMap children_;

          const XMLElement* element_;

          typedef std::map <std::string, std::string> AttributeMap;
          AttributeMap attrMap_;
          std::string name_;
          int id_;

          friend std::ostream& operator<< (std::ostream&, const ObjectFactory&);
      };

      /// Represent a XML document.
      class RootFactory : public ObjectFactory {
        public:
          RootFactory (const model::DevicePtr_t dev = model::DevicePtr_t ());

          model::DevicePtr_t device () const;

        private:
          model::DevicePtr_t device_;
      };

      /// To add a ObjectFactory to the Parser, use:
      /// Parser::addObjectFactory (TagName, create <ObjectFactory>)
      template <typename T>
      ObjectFactory* create (ObjectFactory* parent = NULL, const XMLElement* element = NULL)
      {
        return new T (parent, element);
      }

      /// \brief Parse an XML document
      ///
      /// This class uses the tinyXML library and derived classes of ObjectFactory
      /// to build object from an XML document.
      /// To extend its capabilities, see ObjectFactory.
      class Parser {
        public:
          typedef ObjectFactory* (*FactoryType) (ObjectFactory*, const XMLElement*);
          typedef ObjectFactory DefaultFactory;

          Parser ();

          ~Parser ();

          void parse (const std::string& semanticResourceName,
              model::DevicePtr_t robot);

          void addObjectFactory (const std::string& tagname, FactoryType factory);

          void parseFile (const char* filename);

        private:
          XMLDocument doc_;
          RootFactory root_;
          model::DevicePtr_t device_;

          void loadFile (const char* filename);

          void loadString (const char* xmlstring);

          void parse ();

          void parseElement (const XMLElement* element, ObjectFactory* parent);

          typedef std::map <std::string, FactoryType> ObjectFactoryMap;
          typedef std::pair <std::string, FactoryType> ObjectFactoryPair;
          typedef std::pair <ObjectFactoryMap::iterator, bool> ObjectFactoryInsertRet;
          ObjectFactoryMap objFactoryMap_;

          typedef std::list <ObjectFactory*> ObjectFactoryList;
          ObjectFactoryList objectFactories_;

          std::ostream& print (std::ostream&) const;
          friend std::ostream& operator<< (std::ostream&, const Parser&);
      };

      std::ostream& operator<< (std::ostream&, const ObjectFactory&);
      std::ostream& operator<< (std::ostream&, const Parser&);
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp

#endif //  HPP_MANIPULATION_SRDF_PARSER_HH
