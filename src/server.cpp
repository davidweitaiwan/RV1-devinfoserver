#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/devinfo.h"


class Params : public vehicle_interfaces::GenericParams
{
public:
    std::string devInfoDirPath = "grounddetect_rgb_0_node";

private:
    void _getParams()
    {
        this->get_parameter("devInfoDirPath", this->devInfoDirPath);
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName)
    {
        this->declare_parameter<std::string>("devInfoDirPath", this->devInfoDirPath);
        this->_getParams();
    }
};


int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("devinfoserver_params_node");
    auto server = std::make_shared<vehicle_interfaces::DevInfoServer>(params->nodeName, params->devInfoService, params->devInfoDirPath);
    rclcpp::spin(server);
    std::cerr << "Spin Exit" << std::endl;
    rclcpp::shutdown();
}
