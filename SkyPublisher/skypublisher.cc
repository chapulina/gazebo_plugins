#include <gazebo/gazebo_client.hh>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>

#include <iostream>

/////////////////////////////////////////////////
int main(int _argc, char **_argv)
{
  // Load gazebo
  gazebo::client::setup(_argc, _argv);

  // Create our node for communication
  gazebo::transport::NodePtr node(new gazebo::transport::Node());
  node->Init();

  // Publish to a Gazebo topic
  auto skyPub = node->Advertise<gazebo::msgs::Sky>("~/sky");
  auto lightPub = node->Advertise<gazebo::msgs::Light>("~/light/modify");

  // Wait for a subscriber to connect
  skyPub->WaitForConnection();
  lightPub->WaitForConnection();

  // Publisher loop...replace with your own code.
  for (int i = 0; i < 24; ++i)
  {
    // Throttle Publication
    gazebo::common::Time::MSleep(500);

    // Sky
    gazebo::msgs::Sky skyMsg;
    skyMsg.set_time(i);

    // Light
    ignition::math::Pose3d pose(0, 0, -1, 3.14/12*i - 3.14, 0, 0);

    gazebo::msgs::Pose poseMsg;
    gazebo::msgs::Set(&poseMsg, pose);


    gazebo::msgs::Light lightMsg;
    lightMsg.set_name("sun");
    lightMsg.mutable_pose()->CopyFrom(poseMsg);

    skyPub->Publish(skyMsg);
    lightPub->Publish(lightMsg);
  }

  // Make sure to shut everything down.
  gazebo::client::shutdown();
}
