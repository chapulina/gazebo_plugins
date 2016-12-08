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
#ifndef DOMINOES_PLUGIN_HH_
#define DOMINOES_PLUGIN_HH_

#include <string>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/rendering/RenderTypes.hh>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

namespace gazebo
{
  /// \brief
  class GAZEBO_VISIBLE DominoesPlugin : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: DominoesPlugin();

    /// \brief Destructor
    public: virtual ~DominoesPlugin();

    // Documentation inherited
    public: void Load(sdf::ElementPtr _elem);

    /// \brief
    private: bool OnMouseMove(const common::MouseEvent &_event);

    /// \brief Node used to establish communication with gzserver.
    private: transport::NodePtr node;
    private: transport::PublisherPtr factoryPub;

    private: QDoubleSpinBox *heightSpin;
    private: QDoubleSpinBox *widthSpin;
    private: QDoubleSpinBox *depthSpin;
    private: QDoubleSpinBox *densitySpin;
    private: QDoubleSpinBox *distanceSpin;
    private: QColorDialog *colorDialog;
    private: rendering::UserCameraPtr camera;
  };
}

#endif
