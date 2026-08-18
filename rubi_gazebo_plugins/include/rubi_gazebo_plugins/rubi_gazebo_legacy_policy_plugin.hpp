#pragma once

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/sdf.hh>

#include <memory>

namespace rubi_gazebo_plugins {

class RubiGazeboLegacyPolicyPlugin final : public gazebo::ModelPlugin {
 public:
  RubiGazeboLegacyPolicyPlugin();
  ~RubiGazeboLegacyPolicyPlugin() override;
  void Load(gazebo::physics::ModelPtr model, sdf::ElementPtr sdf) override;

 private:
  class Impl;
  std::unique_ptr<Impl> impl_;
};

}  // namespace rubi_gazebo_plugins
