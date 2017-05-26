/*
 * Copyright (C) 2017 chapulina
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

#include <ignition/math/Pose3.hh>
#include <ignition/math/Rand.hh>
#include <gazebo/physics/physics.hh>
#include "SetStaticPose.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(SetStaticPose)

/////////////////////////////////////////////////
void SetStaticPose::Load(physics::ModelPtr _model, sdf::ElementPtr /*_sdf*/)
{
  this->model = _model;

  this->connections.push_back(event::Events::ConnectWorldUpdateEnd(
      std::bind(&SetStaticPose::OnUpdate, this)));
}

/////////////////////////////////////////////
void SetStaticPose::OnUpdate()
{
  // Get the desired pose, here giving a random offset
  ignition::math::Pose3d pose = this->model->GetWorldPose().Ign();

  pose += ignition::math::Pose3d(ignition::math::Rand::DblUniform(-0.01, 0.01),
                                 ignition::math::Rand::DblUniform(-0.01, 0.01),
                                 ignition::math::Rand::DblUniform(-0.01, 0.01),
                                 ignition::math::Rand::DblUniform(-0.01, 0.01),
                                 ignition::math::Rand::DblUniform(-0.01, 0.01),
                                 ignition::math::Rand::DblUniform(-0.01, 0.01));

  // Don't let it go far under the gound
  pose.Pos().Z() = pose.Pos().Z() < 0.5 ? 0.5 : pose.Pos().Z();

  this->model->SetWorldPose(pose);
}
