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

#include "receiving_interface.hpp"

#include "rosmsgs_datagram_converter.hpp"

#include "bosch_locator_bridge/ClientControlMode.h"
#include "bosch_locator_bridge/ClientRecordingVisualization.h"
#include "bosch_locator_bridge/ClientMapVisualization.h"
#include "bosch_locator_bridge/ClientLocalizationVisualization.h"
#include "bosch_locator_bridge/ClientLocalizationPose.h"
#include "bosch_locator_bridge/ClientGlobalAlignVisualization.h"

#include <Poco/NObserver.h>

ReceivingInterface::ReceivingInterface(const Poco::Net::IPAddress& hostadress, Poco::UInt16 port, ros::NodeHandle& nh)
  : nh_(nh), ccm_socket_(Poco::Net::SocketAddress(hostadress, port))
{
  std::cout << "Interface IP address: " << hostadress << ":" << port << std::endl;
  reactor_.addEventHandler(ccm_socket_, Poco::NObserver<ReceivingInterface, Poco::Net::ReadableNotification>(
                                            *this, &ReceivingInterface::onReadEvent));
}

ReceivingInterface::~ReceivingInterface()
{
  reactor_.stop();
  ccm_socket_.shutdown();
}

void ReceivingInterface::onReadEvent(const Poco::AutoPtr<Poco::Net::ReadableNotification>& notification)
{
  try
  {
    // Create buffer with size of available data
    const int bytes_available = ccm_socket_.available();
    std::vector<char> msg(bytes_available);
    int received_bytes = ccm_socket_.receiveBytes(&(msg[0]), bytes_available);
    if (received_bytes == 0)
    {
      std::cout << "received msg of length 0... Connection closed? \n";
      printf("This address: %s, peer address: %s\n", ccm_socket_.address().toString().c_str(), ccm_socket_.peerAddress().toString().c_str());
      printf("Socket status: available: %d, receive timeout: %ld\n", (int)ccm_socket_.available(), ccm_socket_.getReceiveTimeout().totalMicroseconds());
    }
    else
    {
      datagram_buffer_.insert(datagram_buffer_.end(), msg.begin(), msg.end());
      const auto bytes_to_delete = tryToParseData(datagram_buffer_);
      datagram_buffer_.erase(datagram_buffer_.begin(), datagram_buffer_.begin() + bytes_to_delete);
    }
  }
  catch (const std::ios_base::failure& io_failure)
  {
    // catching this exception is actually no error: the datagram is just not yet completely transmitted could not be
    // parsed because of that. Will automatically retry after more data is available.
  }
  catch (...)
  {
    ROS_ERROR_STREAM("Caught exception in ReceivingInterface!");
  }
}

void ReceivingInterface::run()
{
  reactor_.run();
}

void ReceivingInterface::publishTransform(const geometry_msgs::PoseStamped& pose, const std::string& parent_frame,
                                          const std::string child_frame)
{
  geometry_msgs::TransformStamped transform;
  transform.header.stamp = pose.header.stamp;
  transform.header.frame_id = parent_frame;

  transform.transform.translation.x = pose.pose.position.x;
  transform.transform.translation.y = pose.pose.position.y;
  transform.transform.translation.z = pose.pose.position.z;
  transform.transform.rotation = pose.pose.orientation;

  transform.child_frame_id = child_frame;

  tf_broadcaster_.sendTransform(transform);
}

ClientControlModeInterface::ClientControlModeInterface(const Poco::Net::IPAddress& hostadress, ros::NodeHandle& nh)
  : ReceivingInterface(hostadress, BINARY_CLIENT_CONTROL_MODE_PORT, nh)
{
  // Setup publisher
  publishers_.push_back(nh.advertise<bosch_locator_bridge::ClientControlMode>("client_control_mode", 5, true));
}

size_t ClientControlModeInterface::tryToParseData(const std::vector<char>& datagram)
{
  // convert datagram to ros message
  bosch_locator_bridge::ClientControlMode client_control_mode;
  const auto parsed_bytes =
      RosMsgsDatagramConverter::convertClientControlMode2Message(datagram, ros::Time::now(), client_control_mode);
  if (parsed_bytes > 0)
  {
    // publish client control mode
    publishers_[0].publish(client_control_mode);
  }
  return parsed_bytes;
}

ClientMapMapInterface::ClientMapMapInterface(const Poco::Net::IPAddress& hostadress, ros::NodeHandle& nh)
  : ReceivingInterface(hostadress, BINARY_CLIENT_MAP_MAP_PORT, nh)
{
  // Setup publisher
  publishers_.push_back(nh.advertise<sensor_msgs::PointCloud2>("client_map_map", 5));
}

size_t ClientMapMapInterface::tryToParseData(const std::vector<char>& datagram)
{
  // convert datagram to ros message
  sensor_msgs::PointCloud2 map;
  const auto parsed_bytes = RosMsgsDatagramConverter::convertMapDatagram2Message(datagram, ros::Time::now(), map);
  if (parsed_bytes > 0)
  {
    // publish
    publishers_[0].publish(map);
  }
  return parsed_bytes;
}

ClientMapVisualizationInterface::ClientMapVisualizationInterface(const Poco::Net::IPAddress& hostadress,
                                                                 ros::NodeHandle& nh)
  : ReceivingInterface(hostadress, BINARY_CLIENT_MAP_VISUALIZATION_PORT, nh)
{
  // Setup publisher
  publishers_.push_back(nh.advertise<bosch_locator_bridge::ClientMapVisualization>("client_map_visualization", 5));
  publishers_.push_back(nh.advertise<geometry_msgs::PoseStamped>("client_map_visualization/pose", 5));
  publishers_.push_back(nh.advertise<sensor_msgs::PointCloud2>("client_map_visualization/scan", 5));
  publishers_.push_back(nh.advertise<geometry_msgs::PoseArray>("client_map_visualization/path_poses", 5));
  nh.getParam("trajectory_output_path", output_path);
}

void ClientMapVisualizationInterface::postProcess() {
  printf("Post process started.\n");
  for (size_t i = 0; i < all_poses.size() - 1; i++) {
    std::deque<Pose>& pose_que = all_poses[i], &next_que = all_poses[i + 1];
    std::deque<uint64_t>& now_que = time_stamps[i], &next_stamp_que = time_stamps[i + 1];
    if (pose_que.size() < 3) {
      if (pose_que.size() == 1)
        continue;
      Pose last_pose = next_que.front().inverse() * (pose_que.front() * pose_que.back());
      next_que.insert(next_que.begin() + 1, last_pose);
      next_stamp_que.insert(next_stamp_que.begin() + 1, now_que.back());
      pose_que.pop_back();
      now_que.pop_back();
    } else {
      const Pose next_base_inv = next_que.front().inverse();
      size_t inv_id = pose_que.size() - 1;
      double min_distance = (next_base_inv * (pose_que.front() * pose_que[inv_id])).norm2d();
      for (size_t j = inv_id - 1; j > 0; j--) {
        Pose abs_pose = pose_que.front() * pose_que[j];
        double distance = (next_base_inv * abs_pose).norm2d();
        if (distance < min_distance) {
          min_distance = distance;
          inv_id = j;
        }
      }
      for (size_t j = inv_id + 1; j < pose_que.size(); j++) {
        Pose new_pose = (next_base_inv * (pose_que.front() * pose_que[j]));
        pose_que[j] = new_pose;
      }
      next_que.insert(next_que.begin() + 1, pose_que.begin() + inv_id + 1, pose_que.end());
      next_stamp_que.insert(next_stamp_que.begin() + 1, now_que.begin() + inv_id + 1, now_que.end());
      size_t num_to_pop = pose_que.size() - inv_id;
      for (size_t j = 0; j < num_to_pop; j++) {
        pose_que.pop_back();
        now_que.pop_back();
      }
    }
  }
  printf("Post process completed.\n");
}


ClientMapVisualizationInterface::~ClientMapVisualizationInterface() {
  if (all_poses.size() < 3) {
    printf("Too view valid way points (%lu), exiting...\n", all_poses.size());
    return;
  }
  // postProcess();
  std::fstream file;
  file.open(output_path, std::ios::out);
  for (size_t id = 0; id < all_poses.size(); id++) {
    const std::deque<Pose>& pose_array = all_poses[id];
    const std::deque<uint64_t>& stamp_array = time_stamps[id];
    const Pose& base_pose = pose_array.front();
    file << stamp_array.front() << " " << base_pose.x << " " << base_pose.y << " " << base_pose.theta << " " << 1 << std::endl;
    // size_t max_size = pose_array.size() > 3 ? pose_array.size() - 2 : 0; 
    // for (size_t i = 1; i < pose_array.size(); i++) {
    //   Pose abs_pose = base_pose * pose_array[i];
    //   file << stamp_array[i] << " " << abs_pose.x << " " << abs_pose.y << " " << abs_pose.theta << " " << 0 << std::endl;
    // } 
  }
  printf("Trajectory output to path '%s'\n", output_path.c_str());
}

size_t ClientMapVisualizationInterface::tryToParseData(const std::vector<char>& datagram)
{
  // convert datagram to ros messages
  bosch_locator_bridge::ClientMapVisualization client_map_visualization;
  geometry_msgs::PoseStamped pose;
  sensor_msgs::PointCloud2 scan;
  geometry_msgs::PoseArray path_poses;

  const auto bytes_parsed = RosMsgsDatagramConverter::convertClientMapVisualizationDatagram2Message(
      datagram, client_map_visualization, pose, scan, path_poses);

  if (bytes_parsed > 0)
  {
    // publish
    publishTransform(pose, MAP_FRAME_ID, LASER_FRAME_ID);
    publishers_[0].publish(client_map_visualization);
    publishers_[1].publish(pose);
    publishers_[2].publish(scan);
    publishers_[3].publish(path_poses);
    while (all_poses.size() < path_poses.poses.size()) {
      all_poses.emplace_back();
      all_poses.back().emplace_back(Pose());
      time_stamps.emplace_back();
      time_stamps.back().push_back(path_poses.header.stamp.toNSec());
    }
    for (size_t i = 0; i < path_poses.poses.size(); i++) {
      const geometry_msgs::Pose& single_pose = path_poses.poses[i];
      Pose& base_pose = all_poses[i].front();
      base_pose.x = single_pose.position.x;
      base_pose.y = single_pose.position.y;
      base_pose.theta = 2.0 * atan2(single_pose.orientation.z, single_pose.orientation.w);
    }
    const Pose last_base_pose = all_poses.back().front();
    // size_t pose_id = all_poses.size() > 1 ? all_poses.size() - 2 : 0;
    Pose current_pose;
    current_pose.x = pose.pose.position.x;
    current_pose.y = pose.pose.position.y;
    current_pose.theta = 2.0 * atan2(pose.pose.orientation.z, pose.pose.orientation.w);
    const Pose relative = last_base_pose.inverse() * current_pose;
    all_poses.back().push_back(relative);
    time_stamps.back().push_back(pose.header.stamp.toNSec());
  }
  return bytes_parsed;
}

ClientRecordingMapInterface::ClientRecordingMapInterface(const Poco::Net::IPAddress& hostadress, ros::NodeHandle& nh)
  : ReceivingInterface(hostadress, BINARY_CLIENT_RECORDING_MAP_PORT, nh)
{
  // Setup publisher
  publishers_.push_back(nh.advertise<sensor_msgs::PointCloud2>("client_recording_map", 5));
}

size_t ClientRecordingMapInterface::tryToParseData(const std::vector<char>& datagram)
{
  // convert datagram to ros message
  sensor_msgs::PointCloud2 map;
  const auto parsed_bytes = RosMsgsDatagramConverter::convertMapDatagram2Message(datagram, ros::Time::now(), map);
  if (parsed_bytes > 0)
  {
    // publish
    publishers_[0].publish(map);
  }
  return parsed_bytes;
}

ClientRecordingVisualizationInterface::ClientRecordingVisualizationInterface(const Poco::Net::IPAddress& hostadress,
                                                                             ros::NodeHandle& nh)
  : ReceivingInterface(hostadress, BINARY_CLIENT_RECORDING_VISUALIZATION_PORT, nh)
{
  // Setup publisher
  publishers_.push_back(
      nh.advertise<bosch_locator_bridge::ClientRecordingVisualization>("client_recording_visualization", 5));
  publishers_.push_back(nh.advertise<geometry_msgs::PoseStamped>("client_recording_visualization/pose", 5));
  publishers_.push_back(nh.advertise<sensor_msgs::PointCloud2>("client_recording_visualization/scan", 5));
  publishers_.push_back(nh.advertise<geometry_msgs::PoseArray>("client_recording_visualization/path_poses", 5));
}

size_t ClientRecordingVisualizationInterface::tryToParseData(const std::vector<char>& datagram)
{
  // convert datagram to ros messages
  bosch_locator_bridge::ClientRecordingVisualization client_recording_visualization;
  geometry_msgs::PoseStamped pose;
  sensor_msgs::PointCloud2 scan;
  geometry_msgs::PoseArray path_poses;

  const auto parsed_bytes = RosMsgsDatagramConverter::convertClientRecordingVisualizationDatagram2Message(
      datagram, client_recording_visualization, pose, scan, path_poses);

  if (parsed_bytes > 0)
  {
    // publish
    publishTransform(pose, MAP_FRAME_ID, LASER_FRAME_ID);
    publishers_[0].publish(client_recording_visualization);
    publishers_[1].publish(pose);
    publishers_[2].publish(scan);
    publishers_[3].publish(path_poses);
  }
  return parsed_bytes;
}

ClientLocalizationMapInterface::ClientLocalizationMapInterface(const Poco::Net::IPAddress& hostadress,
                                                               ros::NodeHandle& nh)
  : ReceivingInterface(hostadress, BINARY_CLIENT_LOCALIZATION_MAP_PORT, nh)
{
  // Setup publisher
  // enable latching, since this is usually only published once
  publishers_.push_back(nh.advertise<sensor_msgs::PointCloud2>("client_localization_map", 5, true));
}

size_t ClientLocalizationMapInterface::tryToParseData(const std::vector<char>& datagram)
{
  // convert datagram to ros message
  sensor_msgs::PointCloud2 map;
  const auto bytes_parsed = RosMsgsDatagramConverter::convertMapDatagram2Message(datagram, ros::Time::now(), map);
  if (bytes_parsed > 0)
  {
    // publish
    publishers_[0].publish(map);
  }
  return bytes_parsed;
}

ClientLocalizationVisualizationInterface::ClientLocalizationVisualizationInterface(
    const Poco::Net::IPAddress& hostadress, ros::NodeHandle& nh)
  : ReceivingInterface(hostadress, BINARY_CLIENT_LOCALIZATION_VISUALIZATION_PORT, nh)
{
  // Setup publisher
  publishers_.push_back(
      nh.advertise<bosch_locator_bridge::ClientLocalizationVisualization>("client_localization_visualization", 5));
  publishers_.push_back(nh.advertise<geometry_msgs::PoseStamped>("client_localization_visualization/pose", 5));
  publishers_.push_back(nh.advertise<sensor_msgs::PointCloud2>("client_localization_visualization/scan", 5));
}

size_t ClientLocalizationVisualizationInterface::tryToParseData(const std::vector<char>& datagram)
{
  // convert datagram to ros messages
  bosch_locator_bridge::ClientLocalizationVisualization client_localization_visualization;
  geometry_msgs::PoseStamped pose;
  sensor_msgs::PointCloud2 scan;

  const auto bytes_parsed = RosMsgsDatagramConverter::convertClientLocalizationVisualizationDatagram2Message(
      datagram, client_localization_visualization, pose, scan);

  if (bytes_parsed > 0)
  {
    // publish
    publishers_[0].publish(client_localization_visualization);
    publishers_[1].publish(pose);
    publishers_[2].publish(scan);
  }
  return bytes_parsed;
}

ClientLocalizationPoseInterface::ClientLocalizationPoseInterface(const Poco::Net::IPAddress& hostadress,
                                                                 ros::NodeHandle& nh)
  : ReceivingInterface(hostadress, BINARY_CLIENT_LOCALIZATION_POSE_PORT, nh)
{
  // Setup publisher
  publishers_.push_back(nh.advertise<bosch_locator_bridge::ClientLocalizationPose>("client_localization_pose", 5));
  publishers_.push_back(nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("client_localization_pose/pose", 5));
  publishers_.push_back(nh.advertise<geometry_msgs::PoseStamped>("client_localization_pose/lidar_odo_pose", 5));
  nh.getParam("loc_output_path", loc_out_path);
}

ClientLocalizationPoseInterface::~ClientLocalizationPoseInterface() {
  if (time_stamps.size() < 10) {
    printf("Too few valid way points (%lu), exiting...\n", time_stamps.size());
    return;
  }
  std::fstream file;
  file.open(loc_out_path, std::ios::out);
  for (size_t id = 0; id < time_stamps.size(); id++) {
    const Pose& pose = all_poses[id];
    const uint64_t& v = time_stamps[id];
    file << v << " " << pose.x << " " << pose.y << " " << pose.theta << std::endl;
  }
  printf("Trajectory output to path '%s'\n", loc_out_path.c_str());
}

size_t ClientLocalizationPoseInterface::tryToParseData(const std::vector<char>& datagram)
{
  // convert datagram to ros messages
  bosch_locator_bridge::ClientLocalizationPose client_localization_pose;
  geometry_msgs::PoseStamped pose;
  geometry_msgs::PoseWithCovarianceStamped poseWithCov;
  geometry_msgs::PoseStamped lidar_odo_pose;

  double covariance[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

  const auto bytes_parsed = RosMsgsDatagramConverter::convertClientLocalizationPoseDatagram2Message(
      datagram, client_localization_pose, pose, covariance, lidar_odo_pose);

  poseWithCov.pose.pose = pose.pose;
  poseWithCov.header = pose.header;
  // assign right triangle of 3x3 matrix to 6x6 matrix
  poseWithCov.pose.covariance[0] = covariance[0];
  poseWithCov.pose.covariance[1] = covariance[1];
  poseWithCov.pose.covariance[5] = covariance[2];
  poseWithCov.pose.covariance[7] = covariance[3];
  poseWithCov.pose.covariance[11] = covariance[4];
  poseWithCov.pose.covariance[35] = covariance[5];
  printf("Current pose status: %d\n", client_localization_pose.state);
  if (bytes_parsed > 0)
  {
    // publish
    publishTransform(pose, MAP_FRAME_ID, LASER_FRAME_ID);
    publishers_[0].publish(client_localization_pose);
    publishers_[1].publish(poseWithCov);
    publishers_[2].publish(lidar_odo_pose);
    if (client_localization_pose.state < 0 && initialized == false)
      return bytes_parsed;
    if (initialized == false) {     // pose during intialization state will not be recorded
      initialized = true;
    }
    Pose current_pose;
    current_pose.x = pose.pose.position.x;
    current_pose.y = pose.pose.position.y;
    current_pose.theta = 2.0 * atan2(pose.pose.orientation.z, pose.pose.orientation.w);
    time_stamps.push_back(poseWithCov.header.stamp.toNSec());
    all_poses.push_back(current_pose);
  }
  return bytes_parsed;
}

ClientGlobalAlignVisualizationInterface::ClientGlobalAlignVisualizationInterface(const Poco::Net::IPAddress& hostadress,
                                                                                 ros::NodeHandle& nh)
  : ReceivingInterface(hostadress, BINARY_CLIENT_GLOBAL_ALIGN_VISUALIZATION_PORT, nh)
{
  // Setup publisher
  publishers_.push_back(
      nh.advertise<bosch_locator_bridge::ClientGlobalAlignVisualization>("client_global_align_visualization", 5));
  publishers_.push_back(nh.advertise<geometry_msgs::PoseArray>("client_global_align_visualization/poses", 5));
  publishers_.push_back(nh.advertise<geometry_msgs::PoseArray>("client_global_align_visualization/landmarks/poses", 5));
}

size_t ClientGlobalAlignVisualizationInterface::tryToParseData(const std::vector<char>& datagram)
{
  // convert datagram to ros messages
  bosch_locator_bridge::ClientGlobalAlignVisualization client_global_align_visualization;
  geometry_msgs::PoseArray poses;
  geometry_msgs::PoseArray landmark_poses;

  const auto bytes_parsed = RosMsgsDatagramConverter::convertClientGlobalAlignVisualizationDatagram2Message(
      datagram, client_global_align_visualization, poses, landmark_poses);

  if (bytes_parsed > 0)
  {
    // publish
    publishers_[0].publish(client_global_align_visualization);
    publishers_[1].publish(poses);
    publishers_[2].publish(landmark_poses);
  }
  return bytes_parsed;
}
