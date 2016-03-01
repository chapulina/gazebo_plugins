/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <gazebo/common/common.hh>
#include <gazebo/rendering/rendering.hh>
#include "FloatingModelNames.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(FloatingModelNames)

/////////////////////////////////////////////////
FloatingModelNames::FloatingModelNames()
  : GUIPlugin()
{
  // Position and resize this widget
  this->move(10, 10);
  this->resize(0, 0);

  // Request list of models on startup
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();

  this->requestMsg = msgs::CreateRequest("scene_info");

  this->requestPub = this->node->Advertise<msgs::Request>("~/request");
  this->requestPub->Publish(*this->requestMsg);

  this->responseSub = this->node->Subscribe("~/response",
      &FloatingModelNames::OnResponse, this);

  // Listen to event indicating a new model has been inserted
  this->connections.push_back(gui::Events::ConnectModelUpdate(
      std::bind(&FloatingModelNames::OnModelUpdate, this,
      std::placeholders::_1)));

  // Callback called on pre-render
  this->connections.push_back(event::Events::ConnectPreRender(
      std::bind(&FloatingModelNames::Update, this)));
}

/////////////////////////////////////////////////
FloatingModelNames::~FloatingModelNames()
{
  this->models.clear();
  this->connections.clear();
  this->requestPub.reset();
  this->responseSub.reset();
  this->node.reset();
}

/////////////////////////////////////////////////
void FloatingModelNames::OnResponse(ConstResponsePtr &_msg)
{
  // Check it is the response to our request
  if (!this->requestMsg || _msg->id() != this->requestMsg->id())
    return;

  // Verify message type
  msgs::Scene sceneMsg;
  if (_msg->has_type() && _msg->type() != sceneMsg.GetTypeName())
    return;

  // Get scene data
  sceneMsg.ParseFromString(_msg->serialized_data());

  // Fill vector with models in the scene
  std::lock_guard<std::mutex> lock(this->mutex);
  for (int i = 0; i < sceneMsg.model_size(); ++i)
  {
    if (this->models.find(sceneMsg.model(i).name()) == this->models.end())
      this->models[sceneMsg.model(i).name()] = false;
  }
}

/////////////////////////////////////////////////
void FloatingModelNames::OnModelUpdate(const msgs::Model &_msg)
{
  // Add new model to the list and it will be processed on Update
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->models.find(_msg.name()) == this->models.end())
    this->models[_msg.name()] = false;;
}

/////////////////////////////////////////////
void FloatingModelNames::Update()
{
  // Get scene pointer
  auto scene = rendering::get_scene();
  if (!scene)
    return;

  // Go through all models
  std::lock_guard<std::mutex> lock(this->mutex);
  for (auto &model : this->models)
  {
    // Skip if model has already been processed
    if (model.second)
      continue;

    std::string modelName = model.first;

    auto vis = scene->GetVisual(modelName);

    // Visual is not in the scene yet
    if (!vis)
      continue;

    std::string textName = modelName + "__TEXT__";
    auto modelBox = vis->GetBoundingBox().Ign();

    // Create text
    auto text = new rendering::MovableText;
    text->Load(textName,
        modelName, "Arial", 0.1, common::Color::Blue);
    text->SetBaseline(modelBox.Max().Z() + 0.1);
    text->SetShowOnTop(true);

    // Attach modelName to the visual's node
    auto textNode = vis->GetSceneNode()->createChildSceneNode(
        textName + "__NODE__");
    textNode->attachObject(text);
    textNode->setInheritScale(false);

    // Model has been processed
    model.second = true;
  }
}
