// Copyright 2019 Zhushi Tech, Inc.
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

#ifndef CAMERA_PYLON__CAMERA_PYLON_HPP_
#define CAMERA_PYLON__CAMERA_PYLON_HPP_

#include <deque>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

#include <pylon/PylonIncludes.h>

namespace camera_pylon
{

using std_srvs::srv::Trigger;
using sensor_msgs::msg::PointCloud2;

using namespace Pylon;

class CameraPylon : public rclcpp::Node
{
public:
  explicit CameraPylon(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  virtual ~CameraPylon();

  /**
   * @brief Set the power's state: on or off.
   *
   * @param f true to power on camera.
   * @return int 0 if success.
   */
  // int _power(bool f);
  // void _Init();
  // void _InitializeParameters();
  // void _UpdateParameters();
  // void _Sub(std_msgs::msg::String::UniquePtr ptr);  // TODO(imp)
  // void _Srv(
  //   const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
  //   std::shared_ptr<std_srvs::srv::Trigger::Response> response);  // TODO(imp)

private:
  CInstantCamera cam;
  // class _Impl;
  // std::unique_ptr<_Impl> _impl;

  // const char * _pubName = "~/pub";  // TODO(imp)
  // rclcpp::Publisher<std_msgs::msg::String>::SharedPtr _pub;

  // const char * _subName = "~/sub";  // TODO(imp)
  // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _sub;

  const char * _srv_start_name = "~/start";  // TODO(imp)
  rclcpp::Service<Trigger>::SharedPtr _srv_start;

  const char * _srv_stop_name = "~/stop";  // TODO(imp)
  rclcpp::Service<Trigger>::SharedPtr _srv_stop;

  /**
   * @brief Publisher name.
   *
   */
  const char * _pub_name = "~/line";

  /**
   * @brief Shared pointer to publisher.
   *
   */
  rclcpp::Publisher<PointCloud2>::SharedPtr _pub;

  /**
   * @brief Number of co-workers.
   *
   */
  int _workers;

  /**
   * @brief Mutex to protect image queue.
   *
   */
  std::mutex _images_mut;

  /**
   * @brief Condition variable for image queue.
   *
   */
  std::condition_variable _images_con;

  /**
   * @brief Double end queue for images.
   *
   */
  std::deque<CGrabResultPtr> _images;

  /**
   * @brief Mutex to protect result queue.
   *
   */
  std::mutex _futures_mut;

  /**
   * @brief Condition variable for result queue.
   *
   */
  std::condition_variable _futures_con;

  /**
   * @brief Double end queue for results.
   *
   */
  std::deque<std::future<PointCloud2::UniquePtr>> _futures;

  /**
   * @brief Threads for workers and the manager.
   *
   */
  std::vector<std::thread> _threads;

  // std::thread _init;
  /**
   * @brief ROS parameter callback handle.
   *
   */
  // OnSetParametersCallbackHandle::SharedPtr _handle;
};

}  // namespace camera_pylon

#endif  // CAMERA_PYLON__CAMERA_PYLON_HPP_
