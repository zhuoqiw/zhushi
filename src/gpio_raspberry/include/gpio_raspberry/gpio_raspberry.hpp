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

#ifndef GPIO_RASPBERRY__GPIO_RASPBERRY_HPP_
#define GPIO_RASPBERRY__GPIO_RASPBERRY_HPP_

#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"

struct gpiod_chip;
struct gpiod_line;

namespace gpio_raspberry
{

/**
 * @brief Control GPIO, and laser by using libgpiod.
 *
 */
class GpioRaspberry : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Gpio Raspberry object.
   * Open gpiochip0.
   * Get line 4 for laser.
   * Get line 17 for trigger.
   * Set laser off.
   * Set led on.
   *
   * @param options Encapsulation of options for node initialization.
   */
  explicit GpioRaspberry(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  /**
   * @brief Destroy the Gpio Raspberry object.
   *
   * Release inner implementation.
   * Throw no exception.
   */
  virtual ~GpioRaspberry();

private:
  /**
   * @brief Set the laser's state: on or off.
   *
   * @param f true to power on laser.
   * @return int 0 if success.
   */
  int _laser(bool f);

  void _callback();

  /**
   * @brief The handle to gpio chip.
   *
   */
  std::unique_ptr<gpiod_chip, void (*)(gpiod_chip *)> _chip;

  /**
   * @brief The handle to gpio line 26.
   *
   */
  std::unique_ptr<gpiod_line, void (*)(gpiod_line *)> _line_4;

  /**
   * @brief The handle to gpio line 22.
   *
   */
  std::unique_ptr<gpiod_line, void (*)(gpiod_line *)> _line_17;

  /**
   * @brief ROS parameter callback handle.
   *
   */
  OnSetParametersCallbackHandle::SharedPtr _handle;

  rclcpp::TimerBase::SharedPtr _timer;
};

}  // namespace gpio_raspberry

#endif  // GPIO_RASPBERRY__GPIO_RASPBERRY_HPP_
