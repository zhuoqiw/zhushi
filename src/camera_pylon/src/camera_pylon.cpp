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
int COUNT = 0;
int ID = 0;

using namespace Pylon;  // NOLINT
using std_srvs::srv::Trigger;
using sensor_msgs::msg::PointCloud2;

/**
 * @brief Basler time stamp, ticks per second.
 *
 */
constexpr int TICKS_PER_SEC = 1000000000;

/**
 * @brief Basler image buffer number.
 *
 */
constexpr int BUFFER_NUMBER = 10;

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
PointCloud2::UniquePtr execute(const CGrabResultPtr & ptr)
{
  if (ptr) {
    cv::Mat img(ptr->GetHeight(), ptr->GetWidth(), CV_8UC1, ptr->GetBuffer());
    auto pnts = center(img);
    auto line = to_pc2(pnts);
    line->header.frame_id = std::to_string(ptr->GetImageNumber());
    auto stamp = ptr->GetTimeStamp();
    line->header.stamp.sec = stamp / TICKS_PER_SEC;
    line->header.stamp.nanosec = stamp % TICKS_PER_SEC;
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
    } else {
      // RCLCPP_WARN(this->get_logger(), "Image broken");
    }
  }

private:
  CameraPylon * _ptr;
};

class CHardwareTriggerConfiguration : public CConfigurationEventHandler
{
public:
  /// Apply software trigger configuration.
  // static void ApplyConfiguration( GENAPI_NAMESPACE::INodeMap& nodemap )
  // {
  //   using namespace GENAPI_NAMESPACE;

  //   //Disable compression mode.
  //   CConfigurationHelper::DisableCompression( nodemap );

  //   //Disable GenDC streaming.
  //   CConfigurationHelper::DisableGenDC( nodemap );

  //   //Select image component.
  //   CConfigurationHelper::SelectRangeComponent( nodemap );

  //   // Disable all trigger types except the trigger type used for triggering the acquisition of
  //   // frames.
  //   {
      

  //     // Check the available camera trigger mode(s) to select the appropriate one: acquisition start trigger mode
  //     // (used by older cameras, i.e. for cameras supporting only the legacy image acquisition control mode;
  //     // do not confuse with acquisition start command) or frame start trigger mode
  //     // (used by newer cameras, i.e. for cameras using the standard image acquisition control mode;
  //     // equivalent to the acquisition start trigger mode in the legacy image acquisition control mode).
  //     String_t triggerName( "FrameStart" );
  //     if (!triggerSelector.CanSetValue( triggerName ))
  //     {
  //         triggerName = "AcquisitionStart";
  //         if (!triggerSelector.CanSetValue( triggerName ))
  //         {
  //             throw RUNTIME_EXCEPTION( "Could not select trigger. Neither FrameStart nor AcquisitionStart is available." );
  //         }
  //     }

  //     // Get all enumeration entries of trigger selector.
  //     StringList_t triggerSelectorEntries;
  //     triggerSelector.GetSettableValues( triggerSelectorEntries );

  //     // Turn trigger mode off for all trigger selector entries except for the frame trigger given by triggerName.
  //     for (StringList_t::const_iterator it = triggerSelectorEntries.begin(); it != triggerSelectorEntries.end(); ++it)
  //     {
  //         // Set trigger mode to off.
  //         triggerSelector.SetValue( *it );
  //         if (triggerName == *it)
  //         {
  //           // Activate trigger.
  //           triggerMode.SetValue( "On" );

  //           // The trigger source must be set to 'Software'.
  //           // CEnumParameter( nodemap, "TriggerSource" ).SetValue( "Software" );

  //           //// Alternative hardware trigger configuration:
  //           //// This configuration can be copied and modified to create a hardware trigger configuration.
  //           //// Remove setting the 'TriggerSource' to 'Software' (see above) and
  //           //// use the commented lines as a starting point.
  //           //// The camera user's manual contains more information about available configurations.
  //           //// The Basler pylon Viewer tool can be used to test the selected settings first.

  //           //// The trigger source must be set to the trigger input, e.g. 'Line1'.
            

  //           ////The trigger activation must be set to e.g. 'RisingEdge'.
            
  //         }
  //         else
  //         {
  //           triggerMode.TrySetValue( "Off" );
  //         }
  //     }
  //     // Finally select the frame trigger type (resp. acquisition start type
  //     // for older cameras). Issuing a software trigger will now trigger
  //     // the acquisition of a frame.
  //     triggerSelector.SetValue( triggerName );
  //   }


  //   //Set acquisition mode to "continuous"
  //   CEnumParameter( nodemap, "AcquisitionMode" ).SetValue( "Continuous" );
  // }

  //Set basic camera settings.
  virtual void OnOpened(CInstantCamera & cam)
  {
    GenApi_3_1_Basler_pylon::INodeMap & nodemap = cam.GetNodeMap();
    // Disable defect pixel correction
    // CEnumParameter(nodemap, "BslDefectPixelCorrectionMode").SetValue("Off");

    CEnumParameter(nodemap, "TriggerSelector").SetValue("FrameStart");
    CEnumParameter(nodemap, "TriggerMode").SetValue("On");
    CEnumParameter(nodemap, "TriggerSource").SetValue("Line4");
    CEnumParameter(nodemap, "TriggerActivation").SetValue("AnyEdge");
    // try
    // {
    //   ApplyConfiguration( camera.GetNodeMap() );
    // }
    // catch (const GenericException& e)
    // {
    //   throw RUNTIME_EXCEPTION( "Could not apply configuration. Pylon::GenericException caught in OnOpened method msg=%hs", e.what() );
    // }
    // catch (const std::exception& e)
    // {
    //   throw RUNTIME_EXCEPTION( "Could not apply configuration. std::exception caught in OnOpened method msg=%hs", e.what() );
    // }
    // catch (...)
    // {
    //   throw RUNTIME_EXCEPTION( "Could not apply configuration. Unknown exception caught in OnOpened method." );
    // }
  }
};

CameraPylon::CameraPylon(const rclcpp::NodeOptions & options)
: Node("camera_pylon_node", options)
{
  _workers = this->declare_parameter<int>("workers", 1);
  auto sn = this->declare_parameter<std::string>("serial", "");

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
  
  cam.RegisterImageEventHandler(
    new CImageEventPrinter(this),
    RegistrationMode_Append,
    Cleanup_Delete
  );
  cam.RegisterConfiguration(new CHardwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete);
  cam.Open();
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
      std::promise<PointCloud2::UniquePtr> prom;
      _push_back_future(prom.get_future());
      auto line = std::make_unique<PointCloud2>();
      line->header.frame_id = "-1";
      prom.set_value(std::move(line));
    }
  );

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

CameraPylon::~CameraPylon()
{
  cam.Attach(NULL);
  PylonTerminate();
  RCLCPP_INFO(this->get_logger(), "count: %i, id: %i", COUNT, ID);
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
      auto line = execute(ptr);
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
      _pub->publish(std::move(ptr));
      // RCLCPP_INFO(this->get_logger(), "Image: %s", ptr->header.frame_id.c_str());
    } else {
      _futures_con.wait(lk);
    }
  }
}

void CameraPylon::_push_back_image(const CGrabResultPtr & rhs)
{
  COUNT++;
  ID = rhs->GetImageNumber();
  std::unique_lock<std::mutex> lk(_images_mut);
  _images.push_back(rhs);
  auto s = static_cast<int>(_images.size());
  if (s >= BUFFER_NUMBER - 1) {
    _images.pop_front();
    RCLCPP_WARN(this->get_logger(), "Image skipped");
  }
  lk.unlock();
  _images_con.notify_all();
}

void CameraPylon::_push_back_future(std::future<PointCloud2::UniquePtr> fut)
{
  std::unique_lock<std::mutex> lk(_futures_mut);
  _futures.emplace_back(std::move(fut));
  lk.unlock();
  _futures_con.notify_all();
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
