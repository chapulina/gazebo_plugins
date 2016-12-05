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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
#include <Winsock2.h>
#endif

#include <sstream>
#include <ignition/math/Vector3.hh>
#include <gazebo/rendering/Scene.hh>
#include <gazebo/rendering/UserCamera.hh>
#include <gazebo/gui/MouseEventHandler.hh>
#include "DominoesPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(DominoesPlugin)

/////////////////////////////////////////////////
DominoesPlugin::DominoesPlugin()
  : GUIPlugin()
{
  // Set the frame background and foreground colors
  this->setStyleSheet(
      "QFrame {"
        "background-color : rgba(255, 255, 255, 255);"
        "color : black;"
        "font-size: 12px;"
      "}");

  // Size
  this->heightSpin = new QDoubleSpinBox();
  this->widthSpin = new QDoubleSpinBox();
  this->depthSpin = new QDoubleSpinBox();

  // Density
  this->densitySpin = new QDoubleSpinBox();

  // Distance
  this->distanceSpin = new QDoubleSpinBox();

  // Color
  auto colorButton = new QPushButton();
  auto colorDialog = new QColorDialog();
  this->connect(colorButton, SIGNAL(clicked()), colorDialog, SLOT(open()));

  // Layout
  auto frameLayout = new QGridLayout();
  frameLayout->addWidget(new QLabel(tr("Width")), 0, 0);
  frameLayout->addWidget(this->widthSpin, 0, 1);
  frameLayout->addWidget(new QLabel(tr("Height")), 1, 0);
  frameLayout->addWidget(this->heightSpin, 1, 1);
  frameLayout->addWidget(new QLabel(tr("Depth")), 2, 0);
  frameLayout->addWidget(this->depthSpin, 2, 1);
  frameLayout->addWidget(new QLabel(tr("Distance")), 3, 0);
  frameLayout->addWidget(this->depthSpin, 3, 1);
  frameLayout->addWidget(new QLabel(tr("Density")), 4, 0);
  frameLayout->addWidget(this->depthSpin, 4, 1);

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(0, 0, 0, 0);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  gui::MouseEventHandler::Instance()->AddMoveFilter("dominoes",
      std::bind(&DominoesPlugin::OnMouseMove, this, std::placeholders::_1));

  this->camera = gui::get_active_camera();
}

/////////////////////////////////////////////////
DominoesPlugin::~DominoesPlugin()
{
}

/////////////////////////////////////////////////
void DominoesPlugin::Load(sdf::ElementPtr _elem)
{
}

/////////////////////////////////////////////////
bool DominoesPlugin::OnMouseMove(const common::MouseEvent &_event)
{
  // Skip if not dragging and pressing Ctrl
  if (!_event.Dragging() || !_event.Control())
    return false;

  // Check intersection
  ignition::math::Vector3d intersection;
  if (!this->camera->GetScene()->FirstContact(
       this->camera, _event.Pos(), intersection))
  {
    return false;
  }

std::cout << intersection << std::endl;

  return true;

}
