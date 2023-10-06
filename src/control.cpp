#include <random>
#include <thread>
#include <iostream>
#include <sstream>

#include <rclcpp/rclcpp.hpp>
#include "vehicle_interfaces/utils.h"
#include "vehicle_interfaces/params.h"
#include "vehicle_interfaces/devinfo.h"

class DevInfoControlNode : public rclcpp::Node
{
private:
    std::shared_ptr<rclcpp::Node> regClientNode_;
    rclcpp::Client<vehicle_interfaces::srv::DevInfoReg>::SharedPtr regClient_;

    std::shared_ptr<rclcpp::Node> reqClientNode_;
    rclcpp::Client<vehicle_interfaces::srv::DevInfoReq>::SharedPtr reqClient_;

public:
    DevInfoControlNode(const std::string& nodeName, const std::string& devInfoServiceName) : rclcpp::Node(nodeName)
    {
        this->regClientNode_ = rclcpp::Node::make_shared(nodeName + "_devinforeg_client");
        this->regClient_ = this->regClientNode_->create_client<vehicle_interfaces::srv::DevInfoReg>(devInfoServiceName + "_Reg");

        this->reqClientNode_ = rclcpp::Node::make_shared(nodeName + "_devinforeq_client");
        this->reqClient_ = this->reqClientNode_->create_client<vehicle_interfaces::srv::DevInfoReq>(devInfoServiceName + "_Req");

        RCLCPP_INFO(this->get_logger(), "[DevInfoControlNode] Constructed");

        bool stopF = false;
        vehicle_interfaces::ConnToService(this->regClient_, stopF, 1000ms, -1);
        vehicle_interfaces::ConnToService(this->reqClient_, stopF, 1000ms, -1);
    }

    bool regDevInfo(const vehicle_interfaces::msg::DevInfo& devInfo)
    {
        auto request = std::make_shared<vehicle_interfaces::srv::DevInfoReg::Request>();
        request->dev_info = devInfo;
        auto result = this->regClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->regClientNode_, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto res = result.get();
            RCLCPP_INFO(this->get_logger(), "[DevInfoControlNode::regDevInfo] Response: %d", res->response);
            return res->response;
        }
        RCLCPP_INFO(this->get_logger(), "[DevInfoControlNode::regDevInfo] Request failed.");
        return false;
    }

    bool reqDevInfo(const vehicle_interfaces::msg::DevInfo& reqDevInfo, std::vector<vehicle_interfaces::msg::DevInfo>& devInfoVec)
    {
        auto request = std::make_shared<vehicle_interfaces::srv::DevInfoReq::Request>();
        request->dev_info = reqDevInfo;
        auto result = this->reqClient_->async_send_request(request);
#if ROS_DISTRO == 0
        if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 500ms) == rclcpp::executor::FutureReturnCode::SUCCESS)
#else
        if (rclcpp::spin_until_future_complete(this->reqClientNode_, result, 500ms) == rclcpp::FutureReturnCode::SUCCESS)
#endif
        {
            auto res = result.get();
            RCLCPP_INFO(this->get_logger(), "[DevInfoControlNode::reqDevInfo] Response: %d, size: %ld", res->response, res->dev_info_vec.size());
            if (res->response)
            {
                devInfoVec = res->dev_info_vec;
            }
            return res->response;
        }
        RCLCPP_INFO(this->get_logger(), "[DevInfoControlNode::reqDevInfo] Request failed.");
        return false;
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

void SpinExecutor(rclcpp::executors::SingleThreadedExecutor* exec, bool& stopF)
{
    std::this_thread::sleep_for(1s);
    printf("[SpinExecutor] Spin start.\n");
    stopF = false;
    exec->spin();
    printf("[SpinExecutor] Spin ended.\n");
    stopF = true;
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto control = std::make_shared<DevInfoControlNode>("devinfocontrol_0_node", "devinfo_0");
    rclcpp::executors::SingleThreadedExecutor* exec = new rclcpp::executors::SingleThreadedExecutor();
    exec->add_node(control);
    bool stopF = true;
    auto th = std::thread(SpinExecutor, exec, std::ref(stopF));

    while (stopF)
        std::this_thread::sleep_for(500ms);

    printf("/** \n\
* Register \n\
* g <node_name> <hostname> <ipv4> <mac> \n\
* \n\
* Request \n\
* q <node_name> <hostname> <ipv4> <mac> \n\
* \n\
* Ignore item: '-' \n\
*/\n");

    while (!stopF)
    {
        printf(">");
        std::string inputStr;
        std::getline(std::cin, inputStr);
        
        if (inputStr.size() < 1)
            continue;
        auto inputStrVec = vehicle_interfaces::split(inputStr, ", ");

        /**
         * Register
         * g <node_name> <hostname> <ipv4> <mac>
         * 
         * Request
         * q <node_name> <hostname> <ipv4> <mac>
         */

        vehicle_interfaces::msg::DevInfo msg;

        if (inputStrVec[0] == "g")
        {
            if (inputStrVec.size() >= 2)
                msg.node_name = inputStrVec[1] == "-" ? GenDevInfoContent(NODENAME) : inputStrVec[1];
            else
                msg.node_name = GenDevInfoContent(NODENAME);

            if (inputStrVec.size() >= 3)
                msg.hostname = inputStrVec[2] == "-" ? GenDevInfoContent(HOSTNAME) : inputStrVec[2];
            else
                msg.hostname = GenDevInfoContent(HOSTNAME);

            if (inputStrVec.size() >= 4)
                msg.ipv4_addr = inputStrVec[3] == "-" ? GenDevInfoContent(IPV4) : inputStrVec[3];
            else
                msg.ipv4_addr = GenDevInfoContent(IPV4);

            if (inputStrVec.size() >= 5)
                msg.mac_addr = inputStrVec[4] == "-" ? GenDevInfoContent(MAC) : inputStrVec[4];
            else
                msg.mac_addr = GenDevInfoContent(MAC);
            
            if (control->regDevInfo(msg))
            {
                printf("Reg: %s[%s]\t%s/%s\n", 
                        msg.node_name.c_str(), 
                        msg.hostname.c_str(), 
                        msg.ipv4_addr.c_str(), 
                        msg.mac_addr.c_str());
            }
        }
        else if (inputStrVec[0] == "q")
        {
            if (inputStrVec.size() > 1)
                msg.node_name = inputStrVec[1] == "-" ? "" : inputStrVec[1];
            if (inputStrVec.size() > 2)
                msg.hostname = inputStrVec[2] == "-" ? "" : inputStrVec[2];
            if (inputStrVec.size() > 3)
                msg.ipv4_addr = inputStrVec[3] == "-" ? "" : inputStrVec[3];
            if (inputStrVec.size() > 4)
                msg.mac_addr = inputStrVec[4] == "-" ? "" : inputStrVec[4];

            std::vector<vehicle_interfaces::msg::DevInfo> ret;
            if (control->reqDevInfo(msg, ret))
            {
                printf("Req:\n");
                for (const auto& i : ret)
                {
                    printf("\t%s[%s]\t%s/%s\n", 
                            i.node_name.c_str(), 
                            i.hostname.c_str(), 
                            i.ipv4_addr.c_str(), 
                            i.mac_addr.c_str());
                }
            }
        }

        std::this_thread::sleep_for(500ms);
    }

    th.join();
    rclcpp::shutdown();
}
