/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 * Copyright 2016 Geoffrey Hunter <gbmhunter@gmail.com>
 * Copyright (C) 2024 Open Source Robotics Foundation
 * Copyright (C) 2024 Benjamin Perseghetti, Rudis Laboratories
 * Copyright (C) 2024 Pedro Roque, DCS, KTH, Sweden
 * Copyright (C) 2025 Gregorio Marchesini, DCS, KTH, Sweden
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#include "ClohesseyWiltshireModel.hh"

#include <chrono>
#include <memory>
#include <mutex>
#include <optional>
#include <string>

#include <gz/msgs/actuators.pb.h>

#include <gz/common/Console.hh>
#include <gz/common/Profiler.hh>

#include <gz/plugin/Register.hh>
#include <gz/transport/Node.hh>

#include <gz/math/Helpers.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/Utility.hh>

#include <sdf/sdf.hh>

#include "gz/sim/components/Actuators.hh"
#include "gz/sim/components/ExternalWorldWrenchCmd.hh"
#include "gz/sim/components/Pose.hh"
#include "gz/sim/components/Inertial.hh"
#include "gz/sim/components/LinearVelocity.hh"
#include "gz/sim/Entity.hh"
#include "gz/sim/EntityComponentManager.hh"
#include "gz/sim/Link.hh"
#include "gz/sim/Model.hh"
#include "gz/sim/System.hh"
#include "gz/sim/Util.hh"

using namespace gz;
using namespace sim;
using namespace systems;

class gz::sim::systems::ClohesseyWiltshireModelPrivate
{

  /// \brief Apply link forces based on the relative state.
  public: void UpdateForces(EntityComponentManager &_ecm);

  /// \brief Link Entity
  public: Entity linkEntity;

  /// \brief Link name
  public: std::string linkName;

  /// \brief Mean motion of the orbit of the target spacecraft
  public: double meanMotion = 0.0;

  /// \brief Model interface
  public: Model model{kNullEntity};

  /// \brief Simulation time tracker
  public: double simTime = 0.01;

  /// \brief Gazebo communication node.
  public: transport::Node node;
};

//////////////////////////////////////////////////
ClohesseyWiltshireModel::ClohesseyWiltshireModel()
  : dataPtr(std::make_unique<ClohesseyWiltshireModelPrivate>())
{
}

//////////////////////////////////////////////////
void ClohesseyWiltshireModel::Configure(const Entity &_entity,
                                        const std::shared_ptr<const sdf::Element> &_sdf,
                                        EntityComponentManager &_ecm,
                                        EventManager &/*_eventMgr*/)
{

  /// save the model entity.
  this->dataPtr->model = Model(_entity);

  if (!this->dataPtr->model.Valid(_ecm))
  {
    gzerr << "ClohesseyWiltshireModel plugin should be attached to a model. Make sure the plugin is attached under a <model> tag in the SDF file." 
          << "entity. Failed to initialize." << std::endl;
    return;
  }
  
  //  sdf element tags under the <plugin> tag. Applied to read parameters of the plugin
  auto sdfClone = _sdf->Clone();
  
  // Check for the link_name parameter
  if (sdfClone->HasElement("link_name"))
  {
    this->dataPtr->linkName = sdfClone->Get<std::string>("link_name");
  }

  if (this->dataPtr->linkName.empty())
  {
    gzerr << "ClohesseyWiltshireModel found an empty link_name parameter. "
           << "Failed to initialize.";
    return;
  }

  // Check for the mean_motion parameter
  if (sdfClone->HasElement("mean_motion"))
  {
    this->dataPtr->meanMotion =
        sdfClone->GetElement("mean_motion")->Get<double>();
  }
  else
  {
    gzerr << "Please specify mean motion of the target spacecraft.\n";
    return;
  }

  // Look for the link entity over which the force should be applied
  if (this->dataPtr->linkEntity == kNullEntity)
  {
    this->dataPtr->linkEntity =
        this->dataPtr->model.LinkByName(_ecm, this->dataPtr->linkName);
  }

  if ( this->dataPtr->linkEntity == kNullEntity)
  {
    gzerr << "Failed to find link entity. "
          << "Failed to initialize." << std::endl;
    return;
  }

  // Make sure the inertial component of the link exists for force computation.
  const auto *inertialComp = _ecm.Component<components::Inertial>(this->dataPtr->linkEntity);
  if (!inertialComp)
  {
    gzerr << "Inertial component not found for link [" << this->dataPtr->linkName << "]\n";
    return;
  }

  // Access the mass
  double mass = inertialComp->Data().MassMatrix().Mass();

  gzdbg << "Mass of link [" << this->dataPtr->linkName << "]: " << mass << " kg\n";

  // Check that the world pose is available and add if not available
  if (!_ecm.Component<components::WorldPose>(this->dataPtr->linkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity, components::WorldPose());
  }

  // Check that the world linear velocity is available and add if not available
  if (!_ecm.Component<components::WorldLinearVelocity>(this->dataPtr->linkEntity))
  {
    _ecm.CreateComponent(this->dataPtr->linkEntity, components::WorldLinearVelocity());
  }
}

//////////////////////////////////////////////////
void ClohesseyWiltshireModel::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ClohesseyWiltshireModel::PreUpdate");


  if (_info.dt < std::chrono::steady_clock::duration::zero())
  {
    gzwarn << "Detected jump back in time ["
        << std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
        << "s]. System may not work properly." << std::endl;
  }

  // Nothing left to do if paused.
  if (_info.paused)
    return;

  this->dataPtr->simTime = std::chrono::duration<double>(_info.simTime).count();
  this->dataPtr->UpdateForces(_ecm);
}

//////////////////////////////////////////////////
void ClohesseyWiltshireModelPrivate::UpdateForces(
    EntityComponentManager &_ecm)
{
  GZ_PROFILE("ClohesseyWiltshireModelPrivate::UpdateForces");
  
  
  // Apply force to the link
  Link link(this->linkEntity);
  const auto worldPose = link.WorldPose(_ecm);
  const auto linearVel = link.WorldLinearVelocity(_ecm);
  
  if (!worldPose)
  {
    gzerr << "World pose not available for link [" << this->linkName
           << "]. Cannot apply forces." << std::endl;
    return;
  }
  if (!linearVel)
  {
    gzerr << "Linear velocity not available for link [" << this->linkName
           << "]. Cannot apply forces." << std::endl;
    return;
  }

  const double mass = link.WorldInertial(_ecm)->MassMatrix().Mass();

  double vx = linearVel->X();
  double vy = linearVel->Y();

  double x = worldPose->Pos().X();
  // double y = worldPose->Pos().Y();
  double z = worldPose->Pos().Z();
  double n = this->meanMotion;

  double ax = 3 * n * n * x + 2 * n * vy;
  double ay = -2 * n * vx;
  double az = -n * n * z;

  double fx = mass * ax;
  double fy = mass * ay;
  double fz = mass * az;

  link.AddWorldForce(_ecm, math::Vector3d(fx, fy, fz));
}

GZ_ADD_PLUGIN(ClohesseyWiltshireModel,
                    System,
                    ClohesseyWiltshireModel::ISystemConfigure,
                    ClohesseyWiltshireModel::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(ClohesseyWiltshireModel,
                          "gz::sim::systems::ClohesseyWiltshireModel")

// TODO(CH3): Deprecated, remove on version 8
GZ_ADD_PLUGIN_ALIAS(ClohesseyWiltshireModel,
                          "ignition::gazebo::systems::ClohesseyWiltshireModel")
