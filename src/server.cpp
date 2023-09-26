#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/devinfo.h"


class Params : public vehicle_interfaces::GenericParams
{
public:
    std::string devInfoServerDirPath = "devinfoserver/devices";

private:
    void _getParams()
    {
        this->get_parameter("devInfoServerDirPath", this->devInfoServerDirPath);
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName)
    {
        this->declare_parameter<std::string>("devInfoServerDirPath", this->devInfoServerDirPath);
        this->_getParams();
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("devinfoserver_params_node");
    auto server = std::make_shared<vehicle_interfaces::DevInfoServer>(params->nodeName, params->devInfoService, params->devInfoServerDirPath);
    rclcpp::spin(server);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}
