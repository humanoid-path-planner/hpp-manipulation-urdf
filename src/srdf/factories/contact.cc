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

#include "hpp/manipulation/srdf/factories/contact.hh"

#include <hpp/manipulation/device.hh>
#include <hpp/pinocchio/body.hh>
#include <hpp/pinocchio/collision-object.hh>
#include <hpp/pinocchio/joint.hh>
#include <hpp/util/debug.hh>
#include <hpp/util/pointer.hh>
#include <pinocchio/multibody/geometry.hpp>
#include <pinocchio/multibody/model.hpp>

namespace hpp {
namespace manipulation {
namespace srdf {
void ContactFactory::finishTags() {
  DevicePtr_t device = HPP_DYNAMIC_PTR_CAST(Device, root()->device());
  if (!device) {
    hppDout(error, "Failed to create contacts");
    return;
  }
  const pinocchio::Model& model = device->model();
  const pinocchio::GeomModel& geomModel = device->geomModel();

  /// Get the link
  ObjectFactory* o = NULL;
  getChildOfType("link", o);
  linkName_ = root()->prependPrefix(o->name());

  /// Build joint
  if (!model.existBodyName(linkName_))
    throw std::invalid_argument("Link " + linkName_ +
                                " not found. Cannot create contact");
  const ::pinocchio::Frame& linkFrame =
      model.frames[model.getFrameId(linkName_)];
  assert(linkFrame.type == ::pinocchio::BODY);
  JointPtr_t joint(Joint::create(device, linkFrame.parent));

  Transform3s M;
  M.setIdentity();
  if (o->hasAttribute("index")) {
    throw std::invalid_argument("attribute index is unsupported yet");
    // If there is an index, we consider the position are given relatively
    // to the "index"th collision object,
    objectName_ = linkName_ + "_" + o->getAttribute("index");
    /// In this case, coordinates are expressed in the body frame.
    pinocchio::GeomIndex id = geomModel.getGeometryId(objectName_);
    if (id < geomModel.geometryObjects.size()) {
      hppDout(error,
              "Geometry " << objectName_ << " not found in link " << linkName_);
    } else {
      M = geomModel.geometryObjects[id].placement;
    }
  } else {
    // If there is no index, the position are relative to the link.
    M = linkFrame.placement;
  }

  getChildOfType("point", o);
  PointFactory* pts = o->as<PointFactory>();

  /// First build the sequence of points
  const PointFactory::OutType& v = pts->values();
  if (v.size() % 3 != 0)
    throw std::length_error("Point sequence size should be a multiple of 3.");
  std::vector<vector3_t> points;
  for (size_t i = 0; i < v.size(); i += 3)
    points.push_back(M.act(vector3_t(v[i], v[i + 1], v[i + 2])));

  try {
    /// Construct shapes
    getChildOfType("shape", o);
    ShapeFactory* shapes = o->as<ShapeFactory>();

    const ShapeFactory::OutType& shapeIdxs = shapes->values();
    std::size_t i_shape = 0;
    while (i_shape < shapeIdxs.size()) {
      if (i_shape + shapeIdxs[i_shape] > shapeIdxs.size())
        throw std::out_of_range(
            "shape should be a sequence of unsigned "
            "integer: N iPoints_1 ... iPoints_N M iPoints_1 ... iPoints_M");
      Shape_t shape(shapeIdxs[i_shape]);
      for (size_t i = 1; i < shapeIdxs[i_shape] + 1; ++i)
        shape[i - 1] = points[shapeIdxs[i_shape + i]];
      i_shape += shapeIdxs[i_shape] + 1;
      shapes_.push_back(JointAndShape_t(joint, shape));
    }
  } catch (const std::invalid_argument& e) {
    /// Backward compatibility
    try {
      getChildOfType("triangle", o);
    } catch (const std::invalid_argument& eTri) {
      throw std::invalid_argument(e);
    }
    TriangleFactory* tri = o->as<TriangleFactory>();
    /// Group points by 3 to form triangle
    const TriangleFactory::OutType& indexes = tri->values();
    if (indexes.size() % 3 != 0)
      throw std::length_error(
          "Triangle sequence size should be a multiple of 3.");
    if (*std::max_element(indexes.begin(), indexes.end()) >= points.size())
      throw std::out_of_range(
          "triangle should be a sequence of unsigned integer lower than the "
          "number of points.");
    for (size_t i_tri = 0; i_tri < indexes.size(); i_tri += 3) {
      /// For each of the point indexes
      shapes_.push_back(JointAndShape_t(
          joint, {points[indexes[i_tri]], points[indexes[i_tri + 1]],
                  points[indexes[i_tri + 2]]}));
    }
    hppDout(info,
            "Triangles are deprecated and will be removed."
            " Use tag shape instead.");
  }

  device->jointAndShapes.add(root()->prependPrefix(name()), shapes_);
}
}  // namespace srdf
}  // namespace manipulation
}  // namespace hpp
