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

namespace hpp {
  namespace manipulation {
    namespace srdf {
      using tinyxml2::XMLElement;
      using tinyxml2::XMLDocument;
      using tinyxml2::XMLAttribute;
      using tinyxml2::XMLNode;
      using tinyxml2::XMLText;
      using tinyxml2::XMLUtil;

      class Parser;

      class ObjectFactory {
        public:
          static ObjectFactory* create (ObjectFactory* parent = NULL, const XMLElement* element = NULL)
          {
            return new ObjectFactory (parent, element);
          }

          /// Called when the object is created.
          /// It should create a new object;
          virtual void init () 
          {}

          /// Called for each attribute
          virtual void setAttribute (const XMLAttribute* /* attr */)
          {}

          /// Called when all the attributes have been processed.
          virtual void finishAttributes ()
          {};

          /// Called when all the child tags have been processed.
          virtual void finishTags ()
          {}

          /// Called when parsing is finished.
          virtual void finishFile ()
          {}

          /// Add Text child.
          virtual void addTextChild (const XMLText* /* text */)
          {}

        protected:
          ObjectFactory (ObjectFactory* parent, const XMLElement* element)
            : parent_ (parent), root_ (NULL), element_ (element)
          {
            if (parent_ == NULL)
              root_ = NULL;
            else
              root_ = parent_->root ();
          }

          ObjectFactory* parent ()
          {
            return parent_;
          }

          ObjectFactory* root ()
          {
            if (parent_ == NULL)
              return this;
            return root_;
          }

          bool hasParent () const
          {
            return parent_ != NULL;
          }

          const XMLElement* XMLelement ()
          {
            return element_;
          }

        private:
          ObjectFactory* parent_;
          ObjectFactory* root_;

          const XMLElement* element_;
      };

      class Parser {
        public:
          typedef ObjectFactory* (*FactoryType) (ObjectFactory*, const XMLElement*);
          typedef ObjectFactory DefaultFactory;

          Parser ();
          
          void parseFile (const char* filename);

          void addObjectFactory (const std::string& tagname, FactoryType factory);

        private:
          XMLDocument doc_;
          ObjectFactory* root_;

          void parseElement (const XMLElement* element, ObjectFactory* parent);

          typedef std::map <std::string, FactoryType> ObjectFactoryMap;
          typedef std::pair <std::string, FactoryType> ObjectFactoryPair;
          typedef std::pair <ObjectFactoryMap::iterator, bool> ObjectFactoryInsertRet;
          ObjectFactoryMap objFactoryMap_;

          typedef std::list <ObjectFactory*> ObjectFactoryList;
          ObjectFactoryList objectFactories_;

          virtual std::ostream& print (std::ostream&) const;
          friend std::ostream& operator<< (std::ostream&, const Parser&);
      };

      std::ostream& operator<< (std::ostream& os, const Parser& p)
      {
        return p.print (os);
      }
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp

#endif //  HPP_MANIPULATION_SRDF_PARSER_HH
