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

#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "opencv2/opencv.hpp"

namespace camera_pylon
{

using namespace Pylon;
using std_srvs::srv::Trigger;
using sensor_msgs::msg::PointCloud2;

/**
 * @brief A map between ksize and normalized scalar for sobel.
 *
 */
std::map<int, double> SCALAR {
  {1, 1.},
  {3, 1. / 4.},
  {5, 1. / 48.},
  {7, 1. / 640.},
  {-1, 1. / 16.}
};

/**
 * @brief The algorithm to extract laser line center row by row.
 *
 * For more details of the algorithm, refer to the README.md.
 * @param img The input opencv image.
 * @param buf The buffer to use.
 * @return PointCloud2::UniquePtr Point cloud message to publish.
 */
const std::vector<float> & center(
  const cv::Mat & img,
  int ksize = 5,
  int threshold = 35,
  double width_min = 1.,
  double width_max = 30.)
{
  thread_local cv::Mat buf;

  thread_local std::vector<float> pnts;

  if (img.empty()) {return pnts;}

  pnts.resize(img.rows);

  cv::Sobel(img, buf, CV_16S, 1, 0, ksize, SCALAR[ksize]);

  for (decltype(img.rows) r = 0; r < img.rows; ++r) {
    auto pRow = buf.ptr<short>(r);  // NOLINT
    auto minmax = std::minmax_element(pRow, pRow + img.cols);

    auto minEle = minmax.first;
    auto maxEle = minmax.second;

    auto minVal = *minEle;
    auto maxVal = *maxEle;

    auto minPos = minEle - pRow;
    auto minP = minPos == 0 ? pRow[minPos + 1] : pRow[minPos - 1];
    auto minN = minPos == img.cols - 1 ? pRow[minPos - 1] : pRow[minPos + 1];

    auto maxPos = maxEle - pRow;
    auto maxP = maxPos == 0 ? pRow[maxPos + 1] : pRow[maxPos - 1];
    auto maxN = maxPos == img.cols - 1 ? pRow[maxPos - 1] : pRow[maxPos + 1];

    auto a1 = maxP + maxN - maxVal * 2;
    auto b1 = maxP - maxN;
    auto s1 = (a1 < 0 ? 0.5 * b1 / a1 : 0.5 * b1);

    auto a2 = minP + minN - minVal * 2;
    auto b2 = minP - minN;
    auto s2 = (a2 > 0 ? 0.5 * b2 / a2 : 0.5 * b2);

    auto c = (maxPos + minPos + s1 + s2) / 2.;
    auto width = minPos + s2 - maxPos - s1;
    // std::cout << width << " " << minPos << " " << s2 << " " << maxPos << " " << s1 << "\n";
    if (
      maxVal >= threshold &&
      minVal <= -threshold &&
      width >= width_min &&
      width <= width_max &&
      maxPos > 0 &&
      minPos < img.cols - 1)
    {
      pnts[r] = c;
    } else {
      pnts[r] = -1.;
    }
  }

  return pnts;
}

/**
 * @brief Construct ROS point cloud message from vector of floats.
 *
 * @param pnts A sequence of floats as points' row coordinate.
 * @return PointCloud2::UniquePtr Point cloud message to publish.
 */
PointCloud2::UniquePtr to_pc2(const std::vector<float> & pnts)
{
  auto ptr = std::make_unique<PointCloud2>();
  if (pnts.empty()) {return ptr;}

  auto num = pnts.size();

  ptr->height = 1;
  ptr->width = num;

  ptr->fields.resize(1);

  ptr->fields[0].name = "u";
  ptr->fields[0].offset = 0;
  ptr->fields[0].datatype = 7;
  ptr->fields[0].count = 1;

  ptr->is_bigendian = false;
  ptr->point_step = 4;
  ptr->row_step = num * 4;

  ptr->data.resize(num * 4);

  ptr->is_dense = true;

  memcpy(ptr->data.data(), pnts.data(), num * 4);

  return ptr;
}

/**
 * @brief Given an image, a point cloud is returned.
 *
 * @param ptr ROS image
 * @param buf Buffer for sobel
 * @param pm Zipped input parameters
 * @return PointCloud2::UniquePtr ROS point cloud
 */
PointCloud2::UniquePtr execute(CGrabResultPtr ptr)
{
  if (ptr) {
    cv::Mat img(ptr->GetHeight(), ptr->GetWidth(), CV_8UC1, ptr->GetBuffer());
    auto pnts = center(img);
    auto line = to_pc2(pnts);
    line->header.frame_id = std::to_string(ptr->GetImageNumber());
    return line;
  } else {
    auto line = std::make_unique<PointCloud2>();
    line->header.frame_id = "-1";
    return line;
  }
}

class CImageEventPrinter : public CImageEventHandler
{
public:
  explicit CImageEventPrinter(CameraPylon * ptr)
  : _ptr(ptr) {}

  virtual void OnImageGrabbed(CInstantCamera & /*camera*/, const CGrabResultPtr & ptrGrabResult)
  {
    // Image grabbed successfully?
    if (ptrGrabResult->GrabSucceeded()) {
      _ptr->_push_back_image(ptrGrabResult);
      // std::cout << "SizeX: " << ptrGrabResult->GetWidth() << std::endl;
      // std::cout << "SizeY: " << ptrGrabResult->GetHeight() << std::endl;
      // const uint8_t* pImageBuffer = (uint8_t*) ptrGrabResult->GetBuffer();
      // std::cout << "Gray value of first pixel: " << (uint32_t) pImageBuffer[0] << std::endl;
      // std::cout << std::endl;
    } else {
    }
  }

private:
  CameraPylon * _ptr;
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

  for (int i = 0; i < _workers; ++i) {
    _threads.push_back(std::thread(&CameraPylon::_worker, this));
  }
  _threads.push_back(std::thread(&CameraPylon::_manager, this));

  // Initialize cameras
  PylonInitialize();
  CTlFactory & TlFactory = CTlFactory::GetInstance();
  CDeviceInfo di;
  di.SetSerialNumber(sn.c_str());
  di.SetDeviceClass(BaslerUsbDeviceClass);
  cam.Attach(TlFactory.CreateDevice(di));
  cam.BslDefectPixelCorrectionMode.SetValue(BslDefectPixelCorrectionMode_Off);
  cam.RegisterImageEventHandler(
    new CImageEventPrinter(this),
    RegistrationMode_Append,
    Cleanup_Delete);
  _pub = this->create_publisher<PointCloud2>(_pub_name, rclcpp::SensorDataQoS());

  _srv_start = this->create_service<Trigger>(
    _srv_start_name,
    [this](
      const std::shared_ptr<Trigger::Request>/*request*/,
      std::shared_ptr<Trigger::Response> response)
    {
      response->success = true;
      if (!cam.IsGrabbing()) {
        cam.StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera);
      }
    }
  );

  _srv_stop = this->create_service<Trigger>(
    _srv_stop_name,
    [this](
      const std::shared_ptr<Trigger::Request>/*request*/,
      std::shared_ptr<Trigger::Response> response)
    {
      response->success = true;
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

void CameraPylon::_worker()
{
  // cv::Mat buf;
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lk(_images_mut);
    if (_images.empty() == false) {
      auto ptr = _images.front();
      _images.pop_front();
      std::promise<PointCloud2::UniquePtr> prom;
      _push_back_future(prom.get_future());
      lk.unlock();
      auto line = execute(std::move(ptr));
      // auto line = std::make_unique<PointCloud2>();
      // line->header.frame_id = std::to_string(ptr->GetImageNumber());
      prom.set_value(std::move(line));
    } else {
      _images_con.wait(lk);
    }
  }
}

void CameraPylon::_manager()
{
  while (rclcpp::ok()) {
    std::unique_lock<std::mutex> lk(_futures_mut);
    if (_futures.empty() == false) {
      auto f = std::move(_futures.front());
      _futures.pop_front();
      lk.unlock();
      auto ptr = f.get();
      // _pub->publish(std::move(ptr));
      RCLCPP_INFO(this->get_logger(), "Image: %s", ptr->header.frame_id.c_str());
    } else {
      _futures_con.wait(lk);
    }
  }
}

void CameraPylon::_push_back_image(const CGrabResultPtr & rhs)
{
  std::unique_lock<std::mutex> lk(_images_mut);
  _images.push_back(rhs);
  auto s = static_cast<int>(_images.size());
  if (s > _workers + 1) {
    _images.pop_front();
  }
  lk.unlock();
  _images_con.notify_all();
}

void CameraPylon::_push_back_future(std::future<PointCloud2::UniquePtr> fut)
{
  std::unique_lock<std::mutex> lk(_futures_mut);
  _futures.emplace_back(std::move(fut));
  lk.unlock();
  _futures_con.notify_one();
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
