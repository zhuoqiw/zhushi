#include "motor/motor.hpp"

extern "C"
{
    #include "motor_tmcl.h"
}

namespace motor
{

Motor::Motor(const rclcpp::NodeOptions& options) : Node("motor_node", options)
{
    _InitializeParameters();

    _UpdateParameters();

    //TODO
    MotorCreateProcess();

    _srvScan = this->create_service<std_srvs::srv::Trigger>(_srvScanName, [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try
        {
            response->success = false;
            response->message = "Fail: motor scan";

            _UpdateParameters();

            auto status =  MotorCtlScan(_speed);
            if(status >= 0)
            {
                response->success = true;
                response->message = "Success: motor scan";
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service scan: %s", e.what());
        }
        catch(...)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service scan: unknown");
        }
    });

    _srvZero = this->create_service<std_srvs::srv::Trigger>(_srvZeroName, [](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try
        {
            response->success = false;
            response->message = "Fail: motor zero";

            auto status = MotorCtlZero();
            if(status >= 0)
            {
                response->success = true;
                response->message = "Success: motor zero";
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service zero: %s", e.what());
        }
        catch(...)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service zero: unknown");
        }
    });

    _srvCenter = this->create_service<std_srvs::srv::Trigger>(_srvCenterName, [](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try
        {
            response->success = false;
            response->message = "Fail: motor center";

            auto status = MotorCtlCenter();
            if(status >= 0)
            {
                response->success = true;
                response->message = "Success: motor center";
            }
        }
        catch(const std::exception& e)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service center: %s", e.what());
        }
        catch(...)
        {
            RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service center: unknown");
        }
    });

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motor initialized successfully");
}

Motor::~Motor()
{
    MotorDestroyProcess();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motor destroyed successfully");
}

void Motor::_InitializeParameters()
{
    rclcpp::Node::declare_parameter("speed", _speed);
}

void Motor::_UpdateParameters()
{
    this->get_parameter("speed", _speed);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(motor::Motor)
