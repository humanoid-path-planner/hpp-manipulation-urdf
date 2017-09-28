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

#include "hpp/manipulation/srdf/util.hh"

#include <hpp/util/exception-factory.hh>

#include <hpp/pinocchio/urdf/util.hh>
#include <hpp/manipulation/device.hh>

#include "hpp/manipulation/parser/parser.hh"
#include "hpp/manipulation/srdf/factories/position.hh"

namespace hpp {
  namespace manipulation {
    namespace srdf {
      void loadModelFromFile (const DevicePtr_t& robot,
          const std::string& prefix,
          const std::string& package,
          const std::string& modelName,
          const std::string& srdfSuffix)
      {
        std::string srdfPath = "package://" + package + "/srdf/"
          + modelName + srdfSuffix + ".srdf";

        parser::Parser p;

        p.prefix(prefix);
        p.parseFile (srdfPath, robot);
        hppDout (notice, "Finished parsing semantic informations.");
      }

      void loadModelFromXML (const DevicePtr_t& robot,
          const std::string& prefix,
          const std::string& srdfString)
      {
        parser::Parser p;
        p.prefix(prefix);
        p.parseString (srdfString, robot);
        hppDout (notice, "Finished parsing semantic informations.");
      }

      void loadEnvironmentModel (const DevicePtr_t& robot,
          const std::string& package,
          const std::string& modelName,
          const std::string& urdfSuffix,
          const std::string& srdfSuffix)
      {
        hpp::pinocchio::urdf::loadUrdfModel (robot, "anchor", package, modelName + urdfSuffix);

        std::string srdfPath = "package://" + package + "/srdf/"
          + modelName + srdfSuffix + ".srdf";

        // Build robot model from URDF.
        parser::Parser handleParser;
        // For backward compatibility
        handleParser.addObjectFactory ("local_position", parser::create <PositionFactory>);

        handleParser.parseFile (srdfPath, robot);
        hppDout (notice, "Finished parsing environment contacts.");
      }

      void loadObjectModel (const DevicePtr_t& robot,
          const JointIndex& baseJoint,
          const std::string& prefix,
          const std::string& rootJointType,
          const std::string& package,
          const std::string& modelName,
          const std::string& urdfSuffix,
          const std::string& srdfSuffix)
      {
        if (robot->has<FrameIndexes_t> (prefix))
          HPP_THROW(std::invalid_argument, "A robot named " << prefix << " already exists");
        hpp::pinocchio::urdf::loadRobotModel (robot, baseJoint, prefix,
            rootJointType, package, modelName, urdfSuffix, srdfSuffix);

        std::string srdfPath = "package://" + package + "/srdf/"
          + modelName + srdfSuffix + ".srdf";

        // Build robot model from URDF.
        parser::Parser handleParser;
        // For backward compatibility
        handleParser.addObjectFactory ("local_position", parser::create <PositionFactory>);

        handleParser.prefix (prefix);
        handleParser.parseFile (srdfPath, robot);
        hppDout (notice, "Finished parsing handles.");
      }

      void loadHumanoidModel (const DevicePtr_t& robot,
          const JointIndex& baseJoint,
          const std::string& prefix,
          const std::string& rootJointType,
          const std::string& package,
          const std::string& modelName,
          const std::string& urdfSuffix,
          const std::string& srdfSuffix)
      {
        if (robot->has<FrameIndexes_t> (prefix))
          HPP_THROW(std::invalid_argument, "A robot named " << prefix << " already exists");
        hpp::pinocchio::urdf::loadHumanoidModel (robot, baseJoint, prefix,
            rootJointType, package, modelName, urdfSuffix, srdfSuffix);

        std::string srdfPath = "package://" + package + "/srdf/"
          + modelName + srdfSuffix + ".srdf";

        // Build robot model from URDF.
        parser::Parser gripperParser;
        // For backward compatibility
        gripperParser.addObjectFactory ("handle_position_in_joint", parser::create <PositionFactory>);

        gripperParser.prefix (prefix);
        gripperParser.parseFile (srdfPath, robot);
        hppDout (notice, "Finished parsing grippers.");
      }

      void loadRobotModel (const DevicePtr_t& robot,
          const JointIndex& baseJoint,
          const std::string& prefix,
          const std::string& rootJointType,
          const std::string& package,
          const std::string& modelName,
          const std::string& urdfSuffix,
          const std::string& srdfSuffix)
      {
        if (robot->has<FrameIndexes_t> (prefix))
          HPP_THROW(std::invalid_argument, "A robot named " << prefix << " already exists");
        hpp::pinocchio::urdf::loadRobotModel (robot, baseJoint, prefix,
            rootJointType, package, modelName, urdfSuffix, srdfSuffix);

        std::string srdfPath = "package://" + package + "/srdf/"
          + modelName + srdfSuffix + ".srdf";

        // Build robot model from URDF.
        parser::Parser gripperParser;
        // For backward compatibility
        gripperParser.addObjectFactory ("handle_position_in_joint", parser::create <PositionFactory>);

        gripperParser.prefix (prefix);
        gripperParser.parseFile (srdfPath, robot);
        hppDout (notice, "Finished parsing grippers.");
      }

      void addRobotSRDFModel (const DevicePtr_t& robot,
          const std::string& prefix,
          const std::string& package,
          const std::string& modelName,
          const std::string& srdfSuffix)
      {
        std::string srdfPath = "package://" + package + "/srdf/"
          + modelName + srdfSuffix + ".srdf";

        // Build robot model from URDF.
        parser::Parser gripperParser;
        // For backward compatibility
        gripperParser.addObjectFactory ("handle_position_in_joint",
            parser::create <PositionFactory>);

        gripperParser.prefix (prefix);
        gripperParser.parseFile (srdfPath, robot);
        hppDout (notice, "Finished parsing grippers.");
      }
    } // namespace srdf
  } // namespace manipulation
} // namespace hpp
