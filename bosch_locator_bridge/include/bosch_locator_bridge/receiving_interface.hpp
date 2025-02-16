// Copyright (c) 2021 - for information on the respective copyright owner
// see the NOTICE file and/or the repository https://github.com/boschglobal/locator_ros_bridge.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <ros/ros.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <fstream>
#include <deque>

#include <Poco/Net/SocketReactor.h>
#include <Poco/Net/StreamSocket.h>
#include <Poco/Net/SocketNotification.h>
#include <Poco/Net/NetException.h>
#include "transform.hpp"

/**
 * @brief The ReceivingInterface class is the base class for all receiving interfaces, such as
 * ClientControlModeInterface, etc.
 */
class ReceivingInterface : public Poco::Runnable
{
public:
  ReceivingInterface(const Poco::Net::IPAddress& hostadress, Poco::UInt16 port, ros::NodeHandle& nh);

  virtual ~ReceivingInterface();

  virtual void onReadEvent(const Poco::AutoPtr<Poco::Net::ReadableNotification>& notification);

  void run();

protected:
  /**
   * @brief Actual function to be overwritten by child to handle data, e.g., convert to ros messages and
   * publish
   * @param datagram_buffer The data received via the binary connection socket
   * @return amount of bytes successfully parsed and can be removed from the buffer (0 if not parsing failed)
   */
  virtual size_t tryToParseData(const std::vector<char>& datagram_buffer) = 0;

  void publishTransform(const geometry_msgs::PoseStamped& pose, const std::string& parent_frame,
                        const std::string child_frame);

  //! Publisher
  std::vector<ros::Publisher> publishers_;
  tf2_ros::TransformBroadcaster tf_broadcaster_;

  //! Node handle
  ros::NodeHandle nh_;

  // port definitions for the different interfaces. See Locator API documentation section 12.8
  static constexpr Poco::UInt16 BINARY_CLIENT_CONTROL_MODE_PORT{ 9004 };
  static constexpr Poco::UInt16 BINARY_CLIENT_MAP_MAP_PORT{ 9005 };
  static constexpr Poco::UInt16 BINARY_CLIENT_MAP_VISUALIZATION_PORT{ 9006 };
  static constexpr Poco::UInt16 BINARY_CLIENT_RECORDING_MAP_PORT{ 9007 };
  static constexpr Poco::UInt16 BINARY_CLIENT_RECORDING_VISUALIZATION_PORT{ 9008 };
  static constexpr Poco::UInt16 BINARY_CLIENT_LOCALIZATION_MAP_PORT{ 9009 };
  static constexpr Poco::UInt16 BINARY_CLIENT_LOCALIZATION_VISUALIZATION_PORT{ 9010 };
  static constexpr Poco::UInt16 BINARY_CLIENT_LOCALIZATION_POSE_PORT{ 9011 };
  static constexpr Poco::UInt16 BINARY_CLIENT_GLOBAL_ALIGN_VISUALIZATION_PORT{ 9012 };

private:
  Poco::Net::StreamSocket ccm_socket_;
  Poco::Net::SocketReactor reactor_;
  // TODO use a better suited data structure (a deque?)
  std::vector<char> datagram_buffer_;
};

class ClientControlModeInterface : public ReceivingInterface
{
public:
  ClientControlModeInterface(const Poco::Net::IPAddress& hostadress, ros::NodeHandle& nh);
  size_t tryToParseData(const std::vector<char>& datagram) override;
};

class ClientMapMapInterface : public ReceivingInterface
{
public:
  ClientMapMapInterface(const Poco::Net::IPAddress& hostadress, ros::NodeHandle& nh);
  size_t tryToParseData(const std::vector<char>& datagram) override;
};

class ClientMapVisualizationInterface : public ReceivingInterface
{
public:
  ClientMapVisualizationInterface(const Poco::Net::IPAddress& hostadress, ros::NodeHandle& nh);
  ~ClientMapVisualizationInterface();
  size_t tryToParseData(const std::vector<char>& datagram) override;

private:
  void postProcess();
  std::string output_path;
  std::vector<std::deque<Pose>> all_poses;
};

class ClientRecordingMapInterface : public ReceivingInterface
{
public:
  ClientRecordingMapInterface(const Poco::Net::IPAddress& hostadress, ros::NodeHandle& nh);
  size_t tryToParseData(const std::vector<char>& datagram) override;
};

class ClientRecordingVisualizationInterface : public ReceivingInterface
{
public:
  ClientRecordingVisualizationInterface(const Poco::Net::IPAddress& hostadress, ros::NodeHandle& nh);
  size_t tryToParseData(const std::vector<char>& datagram) override;
};

class ClientLocalizationMapInterface : public ReceivingInterface
{
public:
  ClientLocalizationMapInterface(const Poco::Net::IPAddress& hostadress, ros::NodeHandle& nh);
  size_t tryToParseData(const std::vector<char>& datagram) override;
};

class ClientLocalizationVisualizationInterface : public ReceivingInterface
{
public:
  ClientLocalizationVisualizationInterface(const Poco::Net::IPAddress& hostadress, ros::NodeHandle& nh);
  size_t tryToParseData(const std::vector<char>& datagram) override;
};

class ClientLocalizationPoseInterface : public ReceivingInterface
{
public:
  ClientLocalizationPoseInterface(const Poco::Net::IPAddress& hostadress, ros::NodeHandle& nh);
  size_t tryToParseData(const std::vector<char>& datagram) override;
};

class ClientGlobalAlignVisualizationInterface : public ReceivingInterface
{
public:
  ClientGlobalAlignVisualizationInterface(const Poco::Net::IPAddress& hostadress, ros::NodeHandle& nh);
  size_t tryToParseData(const std::vector<char>& datagram) override;
};
