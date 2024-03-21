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

#include <hpp/util/debug.hh>
#include <stdexcept>
// #include <resource_retriever/retriever.h>

#include <pinocchio/parsers/utils.hpp>  // ::pinocchio::retrieveResourcePath
#include <pinocchio/utils/file-explorer.hpp>  // ::pinocchio::rosPaths

#include "hpp/manipulation/parser/parser.hh"
#include "hpp/manipulation/srdf/factories.hh"

namespace hpp {
namespace manipulation {
namespace parser {
Parser::Parser(bool fillWithDefaultFactories, FactoryType defaultFactory)
    : root_(NULL), defaultFactory_(defaultFactory) {
  if (fillWithDefaultFactories) {
    addObjectFactory("robot", create<srdf::RobotFactory>);
    addObjectFactory("handle", create<srdf::HandleFactory>);
    addObjectFactory("gripper", create<srdf::GripperFactory>);
    addObjectFactory("position", create<srdf::PositionFactory>);
    addObjectFactory("mask", create<SequenceFactory<bool> >);
    addObjectFactory("mask_complement", create<SequenceFactory<bool> >);
    addObjectFactory("contact", create<srdf::ContactFactory>);
    addObjectFactory("point", create<srdf::ContactFactory::PointFactory>);
    addObjectFactory("triangle", create<srdf::ContactFactory::TriangleFactory>);
    addObjectFactory("shape", create<srdf::ContactFactory::ShapeFactory>);

    /// This removes warnings
    addObjectFactory("link", create<ObjectFactory>);
    addObjectFactory("disable_collisions", create<IgnoreTagFactory>);
    addObjectFactory("material", create<IgnoreTagFactory>);
    addObjectFactory("texture", create<IgnoreTagFactory>);
  }
}

Parser::~Parser() {
  for (ObjectFactoryList::iterator it = objectFactories_.begin();
       it != objectFactories_.end(); ++it)
    delete *it;
  if (root_ != NULL) delete root_;
}

void Parser::parseString(const std::string& xmlString, DevicePtr_t robot) {
  device_ = robot;

  loadString(xmlString.c_str());
  try {
    parse();
  } catch (const std::exception& exc) {
    std::ostringstream oss;
    oss << "in XML string, " << exc.what();
    throw std::runtime_error(oss.str().c_str());
  }
}

void Parser::parseFile(const std::string& filename, DevicePtr_t robot) {
  device_ = robot;

  std::string fn =
      ::pinocchio::retrieveResourcePath(filename, ::pinocchio::rosPaths());
  loadFile(fn.c_str());
  try {
    parse();
  } catch (const std::exception& exc) {
    std::ostringstream oss;
    oss << "in " << filename << ", " << exc.what();
    throw std::runtime_error(oss.str().c_str());
  }
}

void Parser::loadFile(const char* filename) {
  doc_.LoadFile(filename);
  if (doc_.Error()) {
    std::cerr << doc_.ErrorStr() << std::endl;
    return;
  }
}

void Parser::loadString(const char* xmlstring) {
  doc_.Parse(xmlstring);
  if (doc_.Error()) {
    std::cerr << doc_.ErrorStr() << std::endl;
  }
}

void Parser::parse() {
  const XMLElement* el = doc_.RootElement();
  root_ = new RootFactory(device_);
  root_->prefix(prefix_);
  while (el != NULL) {
    parseElement(el, root_);
    el = el->NextSiblingElement();
  }
  for (ObjectFactoryList::iterator it = objectFactories_.begin();
       it != objectFactories_.end(); ++it)
    (*it)->finishFile();
}

void Parser::addObjectFactory(const std::string& tagname, FactoryType factory) {
  ObjectFactoryInsertRet ret =
      objFactoryMap_.insert(ObjectFactoryPair(tagname, factory));
  if (!ret.second) throw std::logic_error("This tagname already exist");
}

void Parser::parseElement(const XMLElement* element, ObjectFactory* parent) {
  if (element == NULL) return;

  ObjectFactory* o = NULL;
  /// Look for this element in the map
  ObjectFactoryMap::const_iterator it = objFactoryMap_.find(element->Value());
  if (it != objFactoryMap_.end()) {
    o = it->second(parent, element);
  } else {
    o = defaultFactory_(parent, element);
    hppDout(warning, "I have no factory for tag " << o->tagName());
  }
  objectFactories_.push_back(o);
  if (!o->init()) return;
  const XMLAttribute* attr = element->FirstAttribute();
  while (attr != NULL) {
    o->setAttribute(attr);
    attr = attr->Next();
  }
  if (!o->finishAttributes()) return;

  /// Loop over is child tags
  const XMLNode* el = element->FirstChild();
  while (el != NULL) {
    if (el->ToElement() != NULL) {
      parseElement(el->ToElement(), o);
    } else if (el->ToUnknown() != NULL) {
      hppDout(warning, "Unknown Node in XML file: " << el->Value());
    } else if (el->ToText() != NULL) {
      o->addTextChild(el->ToText());
    } else if (el->ToComment() != NULL) {
    }
    el = el->NextSibling();
  }
  o->finishTags();
}

std::ostream& Parser::print(std::ostream& os) const {
  os << "Parser with " << objectFactories_.size() << " object." << std::endl;
  if (root_ != NULL) os << *root_;
  return os;
}

std::ostream& operator<<(std::ostream& os, const Parser& p) {
  return p.print(os);
}

std::ostream& operator<<(std::ostream& os, const ObjectFactory& o) {
  return o.print(os);
}

ObjectFactory::ObjectFactory(ObjectFactory* parent, const XMLElement* element)
    : parent_(parent), root_(NULL), element_(element), id_(-1) {
  if (parent_ == NULL) {
    root_ = dynamic_cast<RootFactory*>(this);
    if (root_ == NULL)
      throw std::logic_error(
          "ObjectFactory with no parent must be RootFactory");
  } else {
    root_ = parent_->root();
    if (element_ != NULL) parent_->addChild(this);
  }
}

ObjectFactory::ObjectFactory(RootFactory* root)
    : parent_(NULL), root_(root), element_(NULL), id_(-1) {}

bool ObjectFactory::init() { return true; }

bool ObjectFactory::finishAttributes() { return true; }

void ObjectFactory::finishTags() {}

void ObjectFactory::finishFile() {}

void ObjectFactory::addTextChild(const XMLText* /* text */) {}

std::string ObjectFactory::tagName() const {
  if (element_ != NULL) return element_->Value();
  return "No element";
}

std::string ObjectFactory::name() const { return name_; }

void ObjectFactory::name(const std::string& n) { name_ = n; }

void ObjectFactory::name(const char* n) { name(std::string(n)); }

ObjectFactory* ObjectFactory::parent() { return parent_; }

RootFactory* ObjectFactory::root() {
  // if (parent_ == NULL)
  // return this;
  return root_;
}

bool ObjectFactory::hasParent() const { return parent_ != NULL; }

const XMLElement* ObjectFactory::XMLelement() { return element_; }

void ObjectFactory::impl_setAttribute(const XMLAttribute* /* attr */) {}

void ObjectFactory::addChild(ObjectFactory* child) {
  children_[child->tagName()].push_back(child);
}

ObjectFactory::ObjectFactoryList ObjectFactory::getChildrenOfType(
    std::string type) {
  return children_[type];
}

bool ObjectFactory::getChildOfType(std::string type, ObjectFactory*& o) {
  ObjectFactoryList l = getChildrenOfType(type);
  if (l.empty()) {
    throw std::invalid_argument("Tag " + tagName() + " has no child of type " +
                                type);
  }
  o = l.front();
  if (l.size() != 1) {
    hppDout(warning, "Tag " << tagName() << " has several children of type "
                            << type << ". All but the first will be ignored.");
    return false;
  }
  return true;
}

std::ostream& ObjectFactory::print(std::ostream& os) const {
  os << "ObjectFactory " << tagName() << " with name " << name() << std::endl;
  for (ChildrenMap::const_iterator itTagName = children_.begin();
       itTagName != children_.end(); ++itTagName)
    for (ObjectFactoryList::const_iterator itObj = itTagName->second.begin();
         itObj != itTagName->second.end(); ++itObj)
      os << **itObj;
  return os;
}

void ObjectFactory::setAttribute(const XMLAttribute* attr) {
  std::string n = std::string(attr->Name());
  if (n == "name")
    name(attr->Value());
  else if (n == "id") {
    int v;
    if (attr->QueryIntValue(&v) != tinyxml2::XML_SUCCESS) {
      hppDout(error, "Attribute ID " << attr->Value() << " is incorrect.");
    } else {
      id_ = (int)v;
    }
  }
  attrMap_[n] = attr->Value();
  impl_setAttribute(attr);
}

bool ObjectFactory::hasAttribute(const std::string& attr) const {
  return attrMap_.find(attr) != attrMap_.end();
}

std::string ObjectFactory::getAttribute(const std::string& attr) const {
  AttributeMap::const_iterator it = attrMap_.find(attr);
  if (it == attrMap_.end()) {
    hppDout(error, "Asking for attribute " << attr);
    return std::string();
  }
  return it->second;
}

RootFactory::RootFactory(const DevicePtr_t dev)
    : ObjectFactory(this), device_(dev), prefix_("") {}

DevicePtr_t RootFactory::device() const { return device_; }
}  // namespace parser
}  // namespace manipulation
}  // namespace hpp
