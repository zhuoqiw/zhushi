#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace motor
{

class Motor : public rclcpp::Node
{
public:
    explicit Motor(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());
    ~Motor();

private:
    void _InitializeParameters();
    void _UpdateParameters();

private:
    int _speed = 3200;

    const char* _srvScanName = "~/scan";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvScan;

    const char* _srvZeroName = "~/zero";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvZero;

    const char* _srvCenterName = "~/center";
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr _srvCenter;
};

}
