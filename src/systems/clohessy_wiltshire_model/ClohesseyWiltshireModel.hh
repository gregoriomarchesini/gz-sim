/*
 * Copyright (C) 2019 Open Source Robotics Foundation
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
#ifndef GZ_SIM_SYSTEMS_CLOESSEYWILTSHIREMODEL_HH_
#define GZ_SIM_SYSTEMS_CLOESSEYWILTSHIREMODEL_HH_

#include <gz/sim/System.hh>
#include <memory>

namespace gz
{
namespace sim
{
// Inline bracket to help doxygen filtering.
inline namespace GZ_SIM_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class ClosesseyWiltshireModelPrivate;

  /// \brief This plugin implements the Cloessey-Wiltshire Relative Dynamics
  /// acceleration model. For this model, the world frame origin (0,0,0) represents the 
  /// position of a target spacecraft (for example the International Space 
  /// Station) and the model applies an acceleration to the entity it is applied to 
  /// equal to the relative acceleration that the entity would experience if it was orbiting 
  /// relative to the target spacecraft. The model is accurate for relative spacecraft dynamics
  /// happening over circular orbits. The accleration model linear and the equations can be found in
  /// "Spacecraft formation flying: Dynamics, control and navigation", Kyle Alfriend, Srinivas Rao Vadali, Pini Gurfil, Jonathan How, Louis Breger
  /// Chapter 5 - equations 5.4,5.5,5.6.
  ///
  /// The explicit model is here provided for convenience
  /// 
  /// ax  =    2 n vy + 3 n^2 x
  /// ay  =  - 2 n vx
  /// az  =  - n^2 z
  /// where :
  /// ax -> accleration in the world x-axis
  /// ay -> acceleration in the world y-axis
  /// az -> acceleration in the world z-axis
  /// vx -> velocity in the world x-axis
  /// vy -> velocity in the world y-axis
  /// vz -> velocity in the world z-axis
  /// x  -> position in the world x-axis
  /// y  -> position in the world y-axis
  /// z  -> position in the world z-axis
  /// n  -> represents the mean motion of the orbit of the target spacraft 
  ///       in circular orbit. The mean motion can be computed as n = sqrt(mu/r^3)
  ///       where "mu" is the specific gravitational constant of the planet around which the relative
  ///       motion is happening and "r" is the orbital radius. You can find a list of mu parameters for each planet directly on 
  ///       wikipedia at : https://en.wikipedia.org/wiki/Standard_gravitational_parameter 
  ///
  /// N.B. In terms of orbital reference the y coordinate is in the direction of the
  /// velocity vector of the target spacraft as it moves along its orbit. The z direction is the normal 
  /// to the orbital plane and the x direction is the cross product of the previous two (the vector pointing
  /// outward in the radial direction of the target spaceraft).
  /// Below follow the minimum necessary parameters needed by the plugin:

  /// \param link_name Name of the link over which the acceleration should be applied.
  /// \param mean_motion Mean motion of the target spacecraft.


  class ClosesseyWiltshireModel
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: ClosesseyWiltshireModel();

    /// \brief Destructor
    public: ~ClosesseyWiltshireModel() override = default;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    // Documentation inherited
    public: void PreUpdate(
                const gz::sim::UpdateInfo &_info,
                gz::sim::EntityComponentManager &_ecm) override;

    /// \brief Private data pointer
    private: std::unique_ptr<ClosesseyWiltshireModelPrivate> dataPtr;
  };
  }
}
}
}

#endif
