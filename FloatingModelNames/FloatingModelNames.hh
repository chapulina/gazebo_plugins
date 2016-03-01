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

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>

#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

namespace gazebo
{
    class GAZEBO_VISIBLE FloatingModelNames : public GUIPlugin
    {
      Q_OBJECT

      /// \brief Constructor
      public: FloatingModelNames();

      /// \brief Destructor
      public: virtual ~FloatingModelNames();

      /// \brief Callback for response messages.
      /// \param[in] _msg Incoming message.
      private: void OnResponse(ConstResponsePtr &_msg);

      /// \brief Callback for pre-render event.
      private: void Update();

      /// \brief Callback for model update event.
      /// \param[in] _msg Incoming message.
      private: void OnModelUpdate(const msgs::Model &_msg);

      /// \brief Transport node used for communication.
      private: transport::NodePtr node;

      /// \brief Publish request messages.
      private: transport::PublisherPtr requestPub;

      /// \brief Subscribe to response messages.
      private: transport::SubscriberPtr responseSub;

      /// \brief Message used to make requests.
      private: msgs::Request *requestMsg;

      /// \brief Map with names of all models in the scene and a flag indicating
      /// whether they've been processed or not.
      private: std::map<std::string, bool> models;

      /// \brief All the event connections.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Mutex to protect variables
      private: std::mutex mutex;
    };
}
