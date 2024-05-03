#include <random>
#include <thread>
#include <iostream>
#include <sstream>

#include <chrono>

#include <rclcpp/rclcpp.hpp>
#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/devinfo.h"

static std::atomic<bool> __global_exit_flag = false;

class Params : public vehicle_interfaces::GenericParams
{
public:
    std::string serviceName = "devinfo_0";

private:
    void _getParams()
    {
        this->get_parameter("serviceName", this->serviceName);
    }

public:
    Params(std::string nodeName) : vehicle_interfaces::GenericParams(nodeName)
    {
        this->declare_parameter<std::string>("serviceName", this->serviceName);
        this->_getParams();
    }
};

enum DevInfoContentEnum { NODENAME, HOSTNAME, MAC, IPV4, IPV6 };

std::string GenDevInfoContent(DevInfoContentEnum type)
{
    std::random_device rd_;
    std::mt19937 gen_{rd_()};
    std::string ret = "";
    std::stringstream ss;

    std::uniform_int_distribution<> uniInt26Distrib{0, 25};
    std::uniform_int_distribution<> uniInt255Distrib{1, 255};
    std::uniform_int_distribution<> uniInt65535Distrib{1, 255};

    switch (type)
    {
    case DevInfoContentEnum::NODENAME:
        ret += "/V0/";
        for (int i = 0; i < 8; i++)
            ret += uniInt26Distrib(gen_) + 97;
        ret += "_" + std::to_string(uniInt26Distrib(gen_)) + "_node";
        break;
    case DevInfoContentEnum::HOSTNAME:
        ret += "v0-";
        for (int i = 0; i < 8; i++)
            ret += uniInt26Distrib(gen_) + 97;
        ret += "-" + std::to_string(uniInt26Distrib(gen_));
        break;
    case DevInfoContentEnum::MAC:
        for (int i = 0; i < 5; i++)
            ss << std::setfill('0') << std::setw(2) << std::hex << uniInt255Distrib(gen_) << ":";
        ss << std::setfill('0') << std::setw(2) << std::hex << uniInt255Distrib(gen_);
        ret += ss.str();
        break;
    case DevInfoContentEnum::IPV4:
        for (int i = 0; i < 3; i++)
            ret += std::to_string(uniInt255Distrib(gen_)) + ".";
        ret += std::to_string(uniInt255Distrib(gen_));
        break;
    case DevInfoContentEnum::IPV6:
        for (int i = 0; i < 7; i++)
            ss << std::hex << uniInt65535Distrib(gen_) << ":";
        ss << std::hex << uniInt65535Distrib(gen_);
        ret += ss.str();
        break;
    default:
        break;
    }
    return ret;
}

std::shared_ptr<rclcpp::Node> GenTmpNode(std::string prefix = "tmp_")
{
    auto ts = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    return rclcpp::Node::make_shared(prefix + std::to_string(ts) + "_node");
}

vehicle_interfaces::ReasonResult<bool> SendRegister(std::string serviceName, vehicle_interfaces::srv::DevInfoReg::Request::SharedPtr request)
{
    auto node = GenTmpNode("devinfocontrol_");
    auto client = node->create_client<vehicle_interfaces::srv::DevInfoReg>(serviceName);
    auto result = client->async_send_request(request);
#if ROS_DISTRO == 0
    if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
    if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
    {
        auto res = result.get();
        return { res->response, "" };
    }
    return { false, "Request failed." };
}

vehicle_interfaces::ReasonResult<bool> SendRequest(std::string serviceName, vehicle_interfaces::srv::DevInfoReq::Request::SharedPtr req, std::vector<vehicle_interfaces::msg::DevInfo>& devQue)
{
    auto node = GenTmpNode("devinfocontrol_");
    auto client = node->create_client<vehicle_interfaces::srv::DevInfoReq>(serviceName);
    auto result = client->async_send_request(req);
#if ROS_DISTRO == 0
    if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
    if (rclcpp::spin_until_future_complete(node, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
    {
        auto res = result.get();
        if (res->response && res->dev_info_vec.size() > 0)
            devQue = res->dev_info_vec;
        return { res->response, "" };
    }
    return { false, "Request failed." };
}

void PrintHelp()
{
    printf("/** \n\
 * Register: \n\
 *   g <node_name> <hostname> <ipv4> <mac> \n\
 *   gm <node_name> <hostname> <ipv4> <mac> \n\
 * \n\
 * Request: \n\
 *   q <node_name> <hostname> <ipv4> <mac> \n\
 *   q all \n\
 * \n\
 * Ignore item: '-' \n\
 * \n\
 * Quit: \n\
 *   !\n\
 * Help: \n\
 *   ?\n\
 */\n");
}

int main(int argc, char* argv[])
{
    // ctrl-c handler
    signal(SIGINT, 
        [](int)
        {
            __global_exit_flag = true;
        });

    rclcpp::init(argc, argv);
    auto params = std::make_shared<Params>("devinfocontrol_params_node");
    std::cout << "Service name: " << params->serviceName << std::endl;
    PrintHelp();

    while (!__global_exit_flag)
    {
        std::this_thread::sleep_for(100ms);
        printf(">");
        std::string inputStr;
        std::getline(std::cin, inputStr);
        
        if (inputStr.size() < 1)
            continue;
        auto inputStrVec = vehicle_interfaces::split(inputStr, ", ");
        if (inputStrVec.size() == 1 && inputStrVec[0] == "!")
            __global_exit_flag = true;
        else if (inputStrVec.size() == 1 && inputStrVec[0] == "?")
            PrintHelp();

        if (inputStrVec[0] == "g" || inputStrVec[0] == "gm")
        {
            auto req = std::make_shared<vehicle_interfaces::srv::DevInfoReg::Request>();

            if (inputStrVec.size() >= 2)
                req->dev_info.node_name = inputStrVec[1] == "-" ? GenDevInfoContent(NODENAME) : inputStrVec[1];
            else
                req->dev_info.node_name = GenDevInfoContent(NODENAME);

            if (inputStrVec.size() >= 3)
                req->dev_info.hostname = inputStrVec[2] == "-" ? GenDevInfoContent(HOSTNAME) : inputStrVec[2];
            else
                req->dev_info.hostname = GenDevInfoContent(HOSTNAME);

            if (inputStrVec.size() >= 4)
                req->dev_info.ipv4_addr = inputStrVec[3] == "-" ? GenDevInfoContent(IPV4) : inputStrVec[3];
            else
                req->dev_info.ipv4_addr = GenDevInfoContent(IPV4);

            if (inputStrVec.size() >= 5)
                req->dev_info.mac_addr = inputStrVec[4] == "-" ? GenDevInfoContent(MAC) : inputStrVec[4];
            else
                req->dev_info.mac_addr = GenDevInfoContent(MAC);

            if (inputStrVec[0] == "gm")
                req->dev_info.multi_node = true;

            std::cout << "Reg: " << req->dev_info.node_name << "[" << req->dev_info.hostname << "]\t" << req->dev_info.ipv4_addr << "/" << req->dev_info.mac_addr << std::endl;
            auto res = SendRegister(params->serviceName + "_Reg", req);
            std::cout << std::boolalpha << "Request: " << res.result << " Reason: " << res.reason << std::endl;
        }
        else if (inputStrVec[0] == "q")
        {
            auto req = std::make_shared<vehicle_interfaces::srv::DevInfoReq::Request>();

            if (inputStrVec.size() > 1)
                req->dev_info.node_name = inputStrVec[1] == "-" ? "" : inputStrVec[1];
            if (inputStrVec.size() > 2)
                req->dev_info.hostname = inputStrVec[2] == "-" ? "" : inputStrVec[2];
            if (inputStrVec.size() > 3)
                req->dev_info.ipv4_addr = inputStrVec[3] == "-" ? "" : inputStrVec[3];
            if (inputStrVec.size() > 4)
                req->dev_info.mac_addr = inputStrVec[4] == "-" ? "" : inputStrVec[4];

            std::vector<vehicle_interfaces::msg::DevInfo> ret;
            auto res = SendRequest(params->serviceName + "_Req", req, ret);
            std::cout << std::boolalpha << "Request: " << res.result << " Reason: " << res.reason << std::endl;
            if (res.result)
            {
                std::cout << std::left << std::setw(5) << "NO." << std::setw(28) << "NODE" << std::setw(20) << "HOST" << std::setw(16) << "IPv4" << "MAC" << std::endl;
                std::cout << "--------------------------------------------------------------------------------------" << std::endl;
                for (int i = 0; i < ret.size(); i++)
                {
                    std::cout << std::left << std::setw(5) << i << std::setw(28) << ret[i].node_name << std::setw(20) << ret[i].hostname << std::setw(16) << ret[i].ipv4_addr << ret[i].mac_addr << std::endl;
                }
            }
        }
    }

    rclcpp::shutdown();
}
