// RS485 硬件通信桥：打通 ROS 2 异步环境与半双工串口底层
// 解决问题：Linux 多线程踩踏、串口收发半双工乱序丢包、编码器物理映射

#include <vector>
#include <string>
#include <chrono>
#include <algorithm>
#include <mutex>
#include <memory>
#include <boost/asio.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class InspireHand {
private:
    boost::asio::io_context io_;
    boost::asio::serial_port serial_;

public:
    InspireHand() : io_(), serial_(io_) {}

    // 初始化硬件接口，固定 115200 波特率
    bool port_connect(const std::string & port_name, int baudrate = 115200) {
        try {
            serial_.open(port_name);
            serial_.set_option(boost::asio::serial_port_base::baud_rate(baudrate));
            serial_.set_option(boost::asio::serial_port_base::character_size(8));
            serial_.set_option(boost::asio::serial_port_base::stop_bits(boost::asio::serial_port_base::stop_bits::one));
            serial_.set_option(boost::asio::serial_port_base::flow_control(boost::asio::serial_port_base::flow_control::none));
            serial_.set_option(boost::asio::serial_port_base::parity(boost::asio::serial_port_base::parity::none));
            return true;
        } catch(const boost::system::system_error &e) {
            std::cerr << "串口初始化失败: " << e.what() << std::endl;
            return false;
        }
    }

    bool write_angles(const std::vector<int> & t_angle) {
        if(t_angle.size() < 6 || !serial_.is_open()) return false;
        
        std::vector<uint8_t> frame;
        frame.push_back(0xEB); frame.push_back(0x90);
        frame.push_back(0x01); frame.push_back(0x0F);
        frame.push_back(0x12); frame.push_back(0x10); frame.push_back(0x04);

        for(int i = 0; i < 6; i++) {
            frame.push_back(static_cast<uint8_t>(t_angle[i] & 0xFF));
            frame.push_back(static_cast<uint8_t>((t_angle[i] >> 8) & 0xFF));
        }
        
        uint8_t checksum = 0;
        for (size_t j = 2; j < frame.size(); j++) checksum += frame[j];
        frame.push_back(checksum);

        try {
            boost::asio::write(serial_, boost::asio::buffer(frame));
            // 避坑：写完指令必须把硬件回传的 9 字节 ACK 收干净
            // 否则半双工信道内会产生数据堆积，污染下一帧查询
            std::vector<uint8_t> w_ack(9);
            boost::asio::read(serial_, boost::asio::buffer(w_ack));
            return true;
        } catch(const boost::system::system_error &e) {
            return false;
        }
    }

    bool read_angles(std::vector<int> & a_angle) {
        uint8_t q_frame[] = {0xEB, 0x90, 0x01, 0x04, 0x11, 0x28, 0x04, 0x0C, 0x4E};
        try {
            boost::asio::write(serial_, boost::asio::buffer(q_frame, sizeof(q_frame)));

            // 核心实现：字节流滑动窗口状态机
            // 动态寻址 0x90 0xEB，免疫硬件抖动导致的错位断帧
            uint8_t b = 0;
            while(true) {
                boost::asio::read(serial_, boost::asio::buffer(&b, 1));
                if(b == 0x90) {
                    boost::asio::read(serial_, boost::asio::buffer(&b, 1));
                    if(b == 0xEB) break; 
                }
            }

            std::vector<uint8_t> body(18);
            boost::asio::read(serial_, boost::asio::buffer(body));

            a_angle.resize(6, 0);
            for(int i = 0; i < 6; i++) {
                int low = body[5 + 2*i];
                int high = body[5 + 2*i + 1];
                a_angle[i] = static_cast<int16_t>((high << 8) | low);
            }
            return true;
        } catch(...) {
            return false;
        }
    }
};

class InspireHandNodeBridge : public rclcpp::Node {
private:
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    InspireHand driver;
    
    std::vector<int> cmd_angles;
    std::mutex mutex_; // 异步回调锁，防止非实时系统下 ROS 2 踩踏发包

public:
    InspireHandNodeBridge() : Node("inspire_bridge_node") {
        this->declare_parameter<std::string>("port", "/dev/ttyUSB0");
        this->declare_parameter<int>("baudrate", 115200);
        std::string port = this->get_parameter("port").as_string();
        int baudrate = this->get_parameter("baudrate").as_int();

        if (!driver.port_connect(port, baudrate)) throw std::runtime_error("Port error");
        
        publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("/inspire_hand_jointstates", 10);
        joint_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/inspire_hand_command", 10, std::bind(&InspireHandNodeBridge::cmdcallback, this, std::placeholders::_1));
            
        timer_ = create_wall_timer(20ms, std::bind(&InspireHandNodeBridge::control_callback, this));
        cmd_angles = {1740, 1740, 1740, 1740, 1350, 1800}; // 出厂安全停靠位置
    }

private:
    void cmdcallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg) {
        if(msg->data.size() < 6) return;
        std::lock_guard<std::mutex> lock(mutex_);
        
        // 软件弧度(Rad) 到 物理电缸伺服数值(0-2000) 的等比例映射与截断防御
        for(int i = 0; i < 4; i++) {
            double q = std::clamp(msg->data[i], 0.0, 1.466);
            cmd_angles[i] = static_cast<int>(1740.0 - (q / 1.466) * (1740.0 - 900.0));
        }
        double q_thumb_flex = std::clamp(msg->data[4], 0.0, 0.436);
        cmd_angles[4] = static_cast<int>(1350.0 - (q_thumb_flex / 0.436) * (1350.0 - 1100.0));
        
        double q_thumb_swivel = std::clamp(msg->data[5], 0.0, 2.094);
        cmd_angles[5] = static_cast<int>(1800.0 - (q_thumb_swivel / 2.094) * (1800.0 - 600.0));
    }

    void control_callback() {
        std::vector<int> t_angles;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            t_angles = cmd_angles;
        }
        driver.write_angles(t_angles);

        std::vector<int> actual_angles;
        if(driver.read_angles(actual_angles)) {
            auto state_msg = sensor_msgs::msg::JointState();
            state_msg.header.stamp = this->now();
            state_msg.name = {
                "pinky_joint", "ring_joint", "middle_joint",
                "index_joint", "thumb_flex_joint", "thumb_swivel_joint"
            };
            state_msg.position.resize(6);
            
            for(int i = 0; i < 4; i++) {
                state_msg.position[i] = (1740.0 - actual_angles[i]) / (1740.0 - 900.0) * 1.466;
            }
            state_msg.position[4] = (1350.0 - actual_angles[4]) / (1350.0 - 1100.0) * 0.436;
            state_msg.position[5] = (1800.0 - actual_angles[5]) / (1800.0 - 600.0) * 2.094;
            
            publisher_->publish(state_msg);
        }
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<InspireHandNodeBridge>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
