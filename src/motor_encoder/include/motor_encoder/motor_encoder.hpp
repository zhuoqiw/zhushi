#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
// #include "shared_interfaces/srv/trigger_new.hpp"

namespace motor_tmcl
{
	class MotorTmcl;
}

namespace motor_encoder
{

class MotorEncoder : public rclcpp::Node
{
public:
    explicit MotorEncoder(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~MotorEncoder();

private:
    void _InitializeParameters();
    void _UpdateParameters();

private:
	int _speed = 1600;
    int _pos_init = 1100;
    int _direction_flag = -1;

    std::unique_ptr<motor_tmcl::MotorTmcl> _motorTmcl;

    const char* _srvScanName = "~/scan";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvScan;

    const char* _srvZeroName = "~/zero";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvZero;

    const char* _srvCenterName = "~/center";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvCenter;
    
    // const char* _srvLocationName = "~/Location";
    // rclcpp::Service<shared_interfaces::srv::TriggerNew>::SharedPtr _srvLocation;
    
};

}

