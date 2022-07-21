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

#include "camera_pylon/camera_pylon.hpp"

#include <memory>

namespace camera_pylon
{

class CImageEventPrinter : public CImageEventHandler
{
public:

  virtual void OnImageGrabbed(CInstantCamera & /*camera*/, const CGrabResultPtr & ptrGrabResult)
  {
    // std::cout << "OnImageGrabbed event for device " << camera.GetDeviceInfo().GetModelName() << std::endl;

    // Image grabbed successfully?
    if (ptrGrabResult->GrabSucceeded())
    {
      // std::cout << "SizeX: " << ptrGrabResult->GetWidth() << std::endl;
      // std::cout << "SizeY: " << ptrGrabResult->GetHeight() << std::endl;
      // const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
      // std::cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << std::endl;
      // std::cout << std::endl;
    } else {
      // std::cout << "Error: " << std::hex << ptrGrabResult->GetErrorCode() << std::dec << " " << ptrGrabResult->GetErrorDescription() << std::endl;
    }
  }
};

/**
 * @brief Extract extra 'worker' parameter from ROS node options.
 *
 * @param options Encapsulation of options for node initialization.
 * @return int Number of workers.
 */
int workers(const rclcpp::NodeOptions & options)
{
  for (const auto & p : options.parameter_overrides()) {
    if (p.get_name() == "workers") {
      return p.as_int();
    }
  }
  // Default
  return 1;
}

/**
 * @brief Extract serial number parameter from ROS node options.
 *
 * @param options Encapsulation of options for node initialization.
 * @return string Serial number.
 */
std::string serial_number(const rclcpp::NodeOptions & options)
{
  for (const auto & p : options.parameter_overrides()) {
    if (p.get_name() == "serial") {
      return p.as_string();
    }
  }
  // Defualt
  return std::string();
}

CameraPylon::CameraPylon(const rclcpp::NodeOptions & options)
: Node("camera_pylon_node", options)
{
  // Get node options
  _workers = workers(options);
  auto sn = serial_number(options);

  // for (int i = 0; i < _workers; ++i) {
  //   _threads.push_back(std::thread(&LaserLineCenter::_worker, this));
  // }
  // _threads.push_back(std::thread(&LaserLineCenter::_manager, this));

  // Initialize cameras
  PylonInitialize();
  CTlFactory& TlFactory = CTlFactory::GetInstance();
  CDeviceInfo di;
  di.SetSerialNumber(sn.c_str());
  di.SetDeviceClass(BaslerUsbDeviceClass);
  cam.Attach(TlFactory.CreateDevice(di));

  _pub = this->create_publisher<PointCloud2>(_pub_name, rclcpp::SensorDataQoS());

  _srv_start = this->create_service<Trigger>(
    _srv_start_name,
    [this](Trigger::Request::ConstSharedPtr /*request*/,
    Trigger::Response::SharedPtr /*response*/)
    {
      if (!cam.IsGrabbing()) {
        cam.StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera);
      }
    }
  );

  _srv_stop = this->create_service<Trigger>(
    _srv_stop_name,
    [this](Trigger::Request::ConstSharedPtr /*request*/,
    Trigger::Response::SharedPtr /*response*/) {
      if (cam.IsGrabbing()) {
        cam.StopGrabbing();
      }
    }
  );

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

CameraPylon::~CameraPylon()
{
  cam.Attach(NULL);
  PylonTerminate();
  // _init.join();

  // _srv.reset();
  // _sub.reset();
  // _impl.reset();
  // _pub.reset();

  RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
}

// int CameraPylon::_power(bool f)
// {
//   // if (f) {
//   //   if (_timer->is_canceled()) {
//   //     _timer->reset();
//   //   }
//   // } else {
//   //   if (_timer->is_canceled() == false) {
//   //     _timer->cancel();
//   //   }
//   // }

//   return 0;
// }

// void CameraPylon::_Init()
// {
//   try {
//     _InitializeParameters();

//     _UpdateParameters();

//     _pub = this->create_publisher<std_msgs::msg::String>(_pubName, 10);

//     _impl = std::make_unique<_Impl>(this);

//     _sub = this->create_subscription<std_msgs::msg::String>(
//       _subName,
//       10,
//       std::bind(&CameraPylon::_Sub, this, std::placeholders::_1));

//     _srv = this->create_service<std_srvs::srv::Trigger>(
//       _srvName,
//       std::bind(&CameraPylon::_Srv, this, std::placeholders::_1, std::placeholders::_2));

//     RCLCPP_INFO(this->get_logger(), "Initialized successfully");
//   } catch (const std::exception & e) {
//     RCLCPP_ERROR(this->get_logger(), "Exception in initializer: %s", e.what());
//     rclcpp::shutdown();
//   } catch (...) {
//     RCLCPP_ERROR(this->get_logger(), "Exception in initializer: unknown");
//     rclcpp::shutdown();
//   }
// }

// void CameraPylon::_Sub(std_msgs::msg::String::UniquePtr /*ptr*/)
// {
//   try {
//   } catch (const std::exception & e) {
//     RCLCPP_ERROR(this->get_logger(), "Exception in subscription: %s", e.what());
//   } catch (...) {
//     RCLCPP_ERROR(this->get_logger(), "Exception in subscription: unknown");
//   }
// }

// void CameraPylon::_Srv(
//   const std::shared_ptr<std_srvs::srv::Trigger::Request>/*request*/,
//   std::shared_ptr<std_srvs::srv::Trigger::Response>/*response*/)
// {
//   try {
//   } catch (const std::exception & e) {
//     RCLCPP_ERROR(this->get_logger(), "Exception in service: %s", e.what());
//   } catch (...) {
//     RCLCPP_ERROR(this->get_logger(), "Exception in service: unknown");
//   }
// }

// void CameraPylon::_InitializeParameters()
// {
//   // this->declare_parameter("");
// }

// void CameraPylon::_UpdateParameters()
// {
//   // this->get_parameter("", );
// }

}  // namespace camera_pylon

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(camera_pylon::CameraPylon)
