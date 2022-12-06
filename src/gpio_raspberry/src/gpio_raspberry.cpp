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

#include "gpio_raspberry/gpio_raspberry.hpp"

#include <gpiod.h>

#include <vector>

namespace gpio_raspberry
{

using rcl_interfaces::msg::ParameterDescriptor;
using rcl_interfaces::msg::SetParametersResult;

using namespace std::chrono_literals;

GpioRaspberry::GpioRaspberry(const rclcpp::NodeOptions & options)
: Node("gpio_raspberry_node", options),
  _chip(gpiod_chip_open_by_name("gpiochip0"), gpiod_chip_close),
  _line_4(gpiod_chip_get_line(_chip.get(), 4), gpiod_line_release),
  _line_17(gpiod_chip_get_line(_chip.get(), 17), gpiod_line_release)
{
  // To enforce start with laser off
  this->declare_parameter("laser", false, ParameterDescriptor(), true);
  this->declare_parameter("trigger", false, ParameterDescriptor(), true);

  gpiod_line_request_output(_line_4.get(), "ros", 0);
  gpiod_line_request_output(_line_17.get(), "ros", 1);

  _handle = this->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      SetParametersResult result;
      result.successful = true;
      for (const auto & p : parameters) {
        if (p.get_name() == "laser") {
          this->_laser(p.as_bool());
        } else if (p.get_name() == "trigger") {
          this->_trigger(p.as_bool());
        }
      }
      return result;
    });

  _timer = this->create_wall_timer(16ms, [this]() {_callback();});
  _timer->cancel();

  RCLCPP_INFO(this->get_logger(), "Initialized successfully");
}

GpioRaspberry::~GpioRaspberry()
{
  try {
    RCLCPP_INFO(this->get_logger(), "Destroyed successfully");
  } catch (const std::exception & e) {
    RCLCPP_FATAL(this->get_logger(), "Exception in destructor: %s", e.what());
  } catch (...) {
    RCLCPP_FATAL(this->get_logger(), "Exception in destructor: unknown");
  }
}

void GpioRaspberry::_laser(bool f)
{
  gpiod_line_set_value(_line_4.get(), f);
}

void GpioRaspberry::_trigger(bool f)
{
  // static bool b = false;
  // gpiod_line_set_value(_line_17.get(), b);
  // b = !b;
  if (f) {
    if (_timer->is_canceled()) {
      _timer->reset();
    }
  } else {
    if (_timer->is_canceled() == false) {
      _timer->cancel();
    }
  }
}

void GpioRaspberry::_callback()
{
  static bool f = true;
  gpiod_line_set_value(_line_17.get(), f);
  f = !f;
}

}  // namespace gpio_raspberry

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(gpio_raspberry::GpioRaspberry)
