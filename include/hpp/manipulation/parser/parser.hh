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

#ifndef HPP_MANIPULATION_PARSER_HH
# define HPP_MANIPULATION_PARSER_HH

# include <map>
# include <list>
# include <string>
# include <iostream>
# include <tinyxml.h>

# include <hpp/manipulation/fwd.hh>

namespace hpp {
  namespace manipulation {
    namespace parser {
      typedef TiXmlElement XMLElement;
      typedef TiXmlDocument XMLDocument;
      typedef TiXmlAttribute XMLAttribute;
      typedef TiXmlNode XMLNode;
      typedef TiXmlText XMLText;

      class RootFactory;

      /// \defgroup factories
      ///
      /// \brief    Classes used to build object from XML documents.
      ///
      /// See section \ref hpp_manipulation_urdf_extend_sec for more information
      /// about how to extend the parser with factories.

      /// \addtogroup factories
      /// \{

      /// \brief Class that catch XML Parser events for a specific tag and build the corresponding
      /// Object.
      ///
      /// Derive this class if you wish to extend the Parser.
      /// The event callbacks are called in the following order:
      /// \li ObjectFactory::init after having created the object.
      /// \li ObjectFactory::setAttribute for each attribute of the tag.
      /// \li ObjectFactory::finishAttributes after having processed every attribute.
      /// \li ObjectFactory::addTextChild when a child is a text element.
      /// \li ObjectFactory::finishTags when all the children have been parsed.
      /// \li ObjectFactory::finishFile when the file has been fully parsed.
      ///
      /// \note The derived class must have the following construtor
      /// \code
      /// DerivedFactory (ObjectFactory* parent, const XMLElement* element) :
      ///       ObjectFactory (parent, element)
      /// {
      ///   /*
      ///    * Keep in mind that it might be more convenient
      ///    * to build objects in an event callback, when some information
      ///    * has already been parsed.
      ///    */
      /// }
      /// \endcode
      class ObjectFactory {
        public:
          typedef std::vector <ObjectFactory*> ObjectFactoryList;

          ObjectFactory (ObjectFactory* parent = NULL, const XMLElement* element = NULL);

          virtual ~ObjectFactory ()
          {}

          /// \name Events
          /// \{

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

          /// Add Text child.
          virtual void addTextChild (const XMLText* text);

          /// Called when all the attributes have been processed.
          /// \return True to continue parsing this tag, False otherwise.
          virtual bool finishAttributes ();

          /// Called when all the child tags have been processed.
          virtual void finishTags ();

          /// Called when parsing is finished.
          virtual void finishFile ();

          /// \}

          /// \name Accessors
          /// \{

          /// Return tag name of the element is any.
          /// Returns "No element" otherwise.
          std::string tagName () const;

          /// Return the content of the attribute name, or an
          /// empty string.
          std::string name () const;

          /// Check if an attribute was set.
          bool hasAttribute (const std::string& attr) const;

          /// Return a given attributes.
          std::string getAttribute (const std::string& attr) const;

          /// Get a list of ObjectFactory whose tag name is type.
          ObjectFactoryList getChildrenOfType (std::string type);

          /// Get the ObjectFactory whose tag name is type.
          /// \param[out] o Set to the first element of the requested type.
          /// \return true if there was only element of the requested type. false if there are more than one.
          /// \throws std::invalid_argument if no ObjectFactory of the requested type exists.
          bool getChildOfType (std::string type, ObjectFactory*& o);

          /// \}

          /// Set the name.
          /// The default value is the value of the attribute "name"
          /// of the XML tag or an empty string if this does not exist.
          void name (const std::string& n);

          /// See name(const std::string&)
          void name (const char* n);

          /// Cast this class to any child class.
          template <typename T> T* as ()
          {
            if (!dynamic_cast <T*> (this)) {
              std::ostringstream oss;
              oss << "Unexpected tag: " << this->tagName ();
              throw std::invalid_argument (oss.str ().c_str ());
            }
            return static_cast <T*> (this);
          }

        protected:
          ObjectFactory (RootFactory* root);

          ObjectFactory* parent ();

          RootFactory* root ();

          bool hasParent () const;

          const XMLElement* XMLelement ();

          virtual void impl_setAttribute (const XMLAttribute* attr);

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
	virtual ~RootFactory () {}
          RootFactory (const DevicePtr_t dev = DevicePtr_t ());

          DevicePtr_t device () const;

          inline std::string prependPrefix (const std::string& in) const
          {
            if (prefix_.empty ()) return in;
            return prefix_ + in;
          }

          inline std::string removePrefix (const std::string& in) const
          {
            if (prefix_.empty ()) return in;
            assert (in.compare (0, prefix_.size (), prefix_) == 0);
            return in.substr (prefix_.size ());
          }

          void prefix (const std::string& prefix)
          {
            if (prefix.empty ()) return;
            prefix_ = prefix + "/";
          }

        private:
          DevicePtr_t device_;
          std::string prefix_;
      };

      /// \}

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
          typedef ObjectFactory::ObjectFactoryList ObjectFactoryList;

          /// Constructor
          /// \param fillWithDefaultFactories Set to true if you want to insert the default
          ///        factories. If set to false, the created instance will have no factories.
          /// \param defaultFactory The factory used when a tag is not know.
          Parser (bool fillWithDefaultFactories = true, FactoryType defaultFactory = create <ObjectFactory>);

          ~Parser ();

          void addObjectFactory (const std::string& tagname, FactoryType factory);

          void parseString (const std::string& xmlString, DevicePtr_t robot);

          void parseFile (const std::string& filename, DevicePtr_t robot);

          const ObjectFactoryList& objectFactories () const
          {
            return objectFactories_;
          }

          /// Set the prefix of all joints
          void prefix (const std::string& prefix)
          {
            prefix_ = prefix;
          }

        private:
          XMLDocument doc_;
          RootFactory* root_;
          DevicePtr_t device_;

          void loadFile (const char* filename);

          void loadString (const char* xmlstring);

          void parse ();

          void parseElement (const XMLElement* element, ObjectFactory* parent);

          typedef std::map <std::string, FactoryType> ObjectFactoryMap;
          typedef std::pair <std::string, FactoryType> ObjectFactoryPair;
          typedef std::pair <ObjectFactoryMap::iterator, bool> ObjectFactoryInsertRet;
          ObjectFactoryMap objFactoryMap_;
          FactoryType defaultFactory_;

          ObjectFactoryList objectFactories_;

          std::string prefix_;

          std::ostream& print (std::ostream&) const;
          friend std::ostream& operator<< (std::ostream&, const Parser&);
      };

      std::ostream& operator<< (std::ostream&, const ObjectFactory&);
      std::ostream& operator<< (std::ostream&, const Parser&);
    } // namespace parser
  } // namespace manipulation
} // namespace hpp

#endif //  HPP_MANIPULATION_PARSER_HH
