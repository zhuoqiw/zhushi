#include "motor_encoder/motor_encoder.hpp"

#include "motor_tmcl.hpp"

namespace motor_encoder
{

using std_srvs::srv::Trigger;

void ReachCallback(int /*type*/)
{

}

MotorEncoder::MotorEncoder(const rclcpp::NodeOptions& options) : Node("motor_encoder_node", options)
{
    _InitializeParameters();
    _UpdateParameters();

    _motorTmcl = std::make_unique<motor_tmcl::MotorTmcl>(ReachCallback,_pos_init,_direction_flag);
    
    _srvScan = this->create_service<Trigger>(_srvScanName, [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try
        {
            response->success = false;
            response->message = "Fail: motor_encoder scan";

            _UpdateParameters();

            _motorTmcl->MotorCtlScan(_speed);

            response->success = true;
            response->message = "Success: motor_encoder scan";
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

    _srvZero = this->create_service<Trigger>(_srvZeroName, [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try
        {
            response->success = false;
            response->message = "Fail: motor_encoder zero";

            _motorTmcl->MotorCtlZero();

            response->success = true;
            response->message = "Success: motor_encoder zero";
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
    
    _srvCenter = this->create_service<Trigger>(_srvCenterName, [this](const std::shared_ptr<std_srvs::srv::Trigger::Request>, std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        try
        {
            response->success = false;
            response->message = "Fail: motor_encoder center";

            _motorTmcl->MotorCtlCenter();

            response->success = true;
            response->message = "Success: motor_encoder center";
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
    
    // _srvLocation = this->create_service<shared_interfaces::srv::TriggerNew>(_srvLocationName, [this](const std::shared_ptr<shared_interfaces::srv::TriggerNew::Request> request, std::shared_ptr<shared_interfaces::srv::TriggerNew::Response> response)
    // {
    //     try
    //     {
    //         bool res;
            
    //         response->success = false;
    //         response->message = "Fail: motor_encoder Location";
    //         res = _motorTmcl->MotorCtl(request->move_position);
    //         if(res == true)
    //         {
    //             response->success = true;
    //             response->message = "Success: motor_encoder Location";    
    //         }
    //     }
    //     catch(const std::exception& e)
    //     {
    //         RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service center: %s", e.what());
    //     }
    //     catch(...)
    //     {
    //         RCLCPP_WARN(rclcpp::get_logger("rclcpp"), "Exception in service center: unknown");
    //     }
    // });

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motor_encoder initialized successfully");
}

MotorEncoder::~MotorEncoder()
{
    _srvCenter.reset();
    _srvZero.reset();
    _srvScan.reset();
    
    _motorTmcl.reset();

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "motor_encoder destroyed successfully");
}

void MotorEncoder::_InitializeParameters()
{
    this->declare_parameter("speed", _speed);
    this->declare_parameter("pos_init", _pos_init);
    this->declare_parameter("direction_flag", _direction_flag);
}

void MotorEncoder::_UpdateParameters()
{
    this->get_parameter("speed", _speed);
    this->get_parameter("pos_init", _pos_init);
    this->get_parameter("direction_flag", _direction_flag);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(motor_encoder::MotorEncoder)

