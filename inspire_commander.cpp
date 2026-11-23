#include <vector>
#include <chrono>
#include <Eigen/Dense>
#include <cmath>
#include <Eigen/Geometry>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <algorithm>
using namespace std::chrono_literals;

class InspireHand_commander : public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    double step_time;
    bool is_initialized;
    
    Eigen::Vector2d q_thumb;
    Eigen::Matrix<double,3,2> J_thumb;
    Eigen::Vector3d J_index;
    
    Eigen::Vector3d P_thumb_tip;
    Eigen::Vector3d P_index_tip;
    
    Eigen::Vector3d actual_angles;
    Eigen::Vector3d current_target_q; 
    
public:
    InspireHand_commander() : Node("inspire_commander"), step_time(0.0) {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("inspire_hand_command", 10);
        joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/inspire_hand_jointstates", 10, std::bind(&InspireHand_commander::joint_state_callback, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(10ms, std::bind(&InspireHand_commander::timer_callback, this));

        is_initialized = false;
        actual_angles = Eigen::Vector3d::Zero();
        current_target_q = Eigen::Vector3d::Zero();

        RCLCPP_INFO(this->get_logger(), " Inspire RH56F1 Sim-to-Real Commander is ready!");
    }

private:
    // =========================================================================
    // 异步状态回调 (Producer): 快进快出，只更新本地状态缓存
    // =========================================================================
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        int j = 0;
        for (size_t i = 0; i < msg->name.size(); ++i) {
            if (msg->name[i] == "index_joint") { 
                actual_angles.x() = msg->position[i];
                j++;
            }
            if (msg->name[i] == "thumb_flex_joint") {
                actual_angles.z() = msg->position[i];
                j++;
            }
            if (msg->name[i] == "thumb_swivel_joint") {
                actual_angles.y() = msg->position[i];
                j++;
            }
            // 确保集齐 3 个关键关节数据后，再解锁控制器
            if(j == 3 && !is_initialized){
                is_initialized = true;
            }
        }
    }

    // =========================================================================
    // 因时实机大拇指正运动学与雅可比 (处理机械连杆耦合比例)
    // =========================================================================
    void compute_thumb_jacobian(double a1, double a2) {
        using namespace Eigen;
        Matrix<double,3,4> P_thumb = Matrix<double,3,4>::Zero();
        
        Isometry3d T_base = Isometry3d::Identity();
        T_base.translation() = Vector3d(0.01836, -0.027241, 0.070511);
        Isometry3d T0 = T_base * AngleAxisd(a1, Vector3d(8.7274E-05, 0, 1));
        P_thumb.col(0) = T0.translation();

        Isometry3d T_1 = Isometry3d::Identity();
        T_1.linear() = (AngleAxisd(-1.5708, Vector3d::UnitZ()) * AngleAxisd(0, Vector3d::UnitY()) * AngleAxisd(1.5707, Vector3d::UnitX())).toRotationMatrix();
        T_1.translation() = Vector3d(0.005034, -0.01349, -0.0049121);
        Isometry3d T1 = T_1 * AngleAxisd(a2, Vector3d(0, 0, 1));
        P_thumb.col(1) = (T0 * T1).translation();

        Isometry3d T_2 = Isometry3d::Identity();
        T_2.translation() = Vector3d(0.033907, 0.034601, -0.0029656);
        Isometry3d T2 = T_2 * AngleAxisd(a2, Vector3d(0, 0, 1));
        P_thumb.col(2) = (T0 * T1 * T2).translation();

        // 连杆耦合传动：关节3受到关节2 0.8962 倍的机械耦合驱动！
        Isometry3d T_3 = Isometry3d::Identity();
        T_3.translation() = Vector3d(0.01441, 0.014648, 0.0013);
        Isometry3d T3 = T_3 * AngleAxisd(0.8962 * a2, Vector3d(0, 0, 1));
        P_thumb.col(3) = (T0 * T1 * T2 * T3).translation();

        Isometry3d T_tip = Isometry3d::Identity();
        T_tip.linear() = (AngleAxisd(-3.1286, Vector3d::UnitZ()) * AngleAxisd(-1.5641, Vector3d::UnitY()) * AngleAxisd(1.8172, Vector3d::UnitX())).toRotationMatrix();
        T_tip.translation() = Vector3d(0.016676, 0.025314, 0.0067577);
        P_thumb_tip = (T0 * T1 * T2 * T3 * T_tip).translation();

        // 几何雅可比 (遵循连杆叠加原理)
        Vector3d J0 = (T0.linear() * Vector3d(8.7274E-05, 0, 1 )).cross(P_thumb_tip - P_thumb.col(0));
        Vector3d J1 = ((T0 * T1).linear() * Vector3d(0, 0, 1)).cross(P_thumb_tip - P_thumb.col(1));
        Vector3d J2 = ((T0 * T1 * T2).linear() * Vector3d(0, 0, 1)).cross(P_thumb_tip - P_thumb.col(2));
        Vector3d J3 = ((T0 * T1 * T2 * T3).linear() * Vector3d(0, 0, 1)).cross(P_thumb_tip - P_thumb.col(3));
        
        J_thumb << J0, J1 + J2 + 0.8962 * J3; // 融合耦合偏导数
    }

    // =========================================================================
    // 因时实机食指正运动学与雅可比
    // =========================================================================
    void compute_index_jacobian(double b1) {
        using namespace Eigen;
        Matrix<double,3,2> P_index = Matrix<double,3,2>::Zero();
        
        Isometry3d T_base = Isometry3d::Identity();
        T_base.linear() = (AngleAxisd(0, Vector3d::UnitZ()) * AngleAxisd(0, Vector3d::UnitY()) * AngleAxisd(1.6319, Vector3d::UnitX())).toRotationMatrix();
        T_base.translation() = Vector3d(0.00070753, -0.034377, 0.13554);
        Isometry3d T0 = T_base * AngleAxisd(b1, Vector3d(0, 0, -1));
        P_index.col(0) = T0.translation();

        // 连杆耦合传动：远端指节受到近端 1.1545 倍的机械耦合！
        Isometry3d T_1 = Isometry3d::Identity();
        T_1.translation() = Vector3d(0.0031056, 0.032854, -0.001);
        Isometry3d T1 = T_1 * AngleAxisd(1.1545 * b1, Vector3d(0, 0, -1));
        P_index.col(1) = (T0 * T1).translation();

        Isometry3d T_2 = Isometry3d::Identity();
        T_2.translation() = Vector3d(0.0049188, 0.039631, -0.006);
        T_2.linear() = (AngleAxisd(-1.8461, Vector3d::UnitZ()) * AngleAxisd(1.5073, Vector3d::UnitY()) * AngleAxisd(-2.1925, Vector3d::UnitX())).toRotationMatrix();

        P_index_tip = (T0 * T1 * T_2).translation();
        
        Eigen::Vector3d J0 = (T_base.linear() * (Vector3d(0,0,-1))).cross(P_index_tip - P_index.col(0));
        Eigen::Vector3d J1 = ((T0 * T1).linear() * Vector3d(0, 0, -1)).cross(P_index_tip - P_index.col(1));
        
        J_index = J0 + 1.1545 * J1;
    }

    // =========================================================================
    // 核心控制循环 (Consumer): 100Hz (10ms) 硬实时阻抗闭环
    // =========================================================================
    void timer_callback() {
        if(!is_initialized) return;
        
        compute_thumb_jacobian(current_target_q.y(), current_target_q.z());
        compute_index_jacobian(current_target_q.x());
        
        // 目标接触点微调
        Eigen::Vector3d Error = P_index_tip - P_thumb_tip;
        Error.z() -= 0.004;
        Error.x() += 0.007;
        
        double K_P = 1.0;
        double lamda_squ = 0.01;
        
        // 构建双指相对雅可比 (Relative Jacobian) 实现统一笛卡尔控制
        Eigen::Matrix<double,3,3> J_rel;
        J_rel << J_index, -J_thumb;
        
        // DLS 阻尼最小二乘求解伪逆
        Eigen::Matrix3d J_dagger = J_rel.transpose() * (J_rel * J_rel.transpose() + Eigen::Matrix3d::Identity() * lamda_squ).inverse();
        Eigen::Vector3d q_dot = J_dagger * (-K_P * Error);
        
        double dt = 0.01; // 100Hz 对应 dt = 0.01

        if (current_target_q == Eigen::Vector3d::Zero()) {
            current_target_q = actual_angles; // 锁定初始起跑线，防止真机抽搐
        }

        // 速度积分与单步步长硬限幅
        Eigen::Vector3d step = q_dot * dt;
        double max_step = 0.01;
        step = step.cwiseMax(-max_step).cwiseMin(max_step);

        //  Pre-Grasp 时序状态机 避开手指自干涉 
        if (current_target_q.y() < 1.2) {
            step.x() = 0.0;  // 食指强制静止
            step.z() = 0.0;  // 大拇指弯曲强制静止
            
            // 强行接管 y 的速度: 0.015 rad/周期 ≈ 0.75 rad/s，强行外旋大拇指避障
            step.y() = 0.005; 
            
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                "[Phase 1]  强行外旋大拇指中... 当前 Swivel_y = %.2f / 1.20", current_target_q.y());
        } else {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, 
                "[Phase 2]  相对雅可比闭环接管，指尖收敛中...");
        }

        // 纯指令积分累加，克服实机摩擦死区
        current_target_q += step;

        // 物理限位保护
        current_target_q.x() = std::clamp(current_target_q.x(), 0.0, 1.466); 
        current_target_q.y() = std::clamp(current_target_q.y(), 0.0, 2.094); 
        current_target_q.z() = std::clamp(current_target_q.z(), 0.0, 0.436);

        // 下发控制指令
        auto message = std_msgs::msg::Float64MultiArray();
        message.data.resize(6, 0.0);
        message.data[3] = current_target_q.x();
        message.data[4] = current_target_q.z();
        message.data[5] = current_target_q.y();

        publisher_->publish(message);
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<InspireHand_commander>());
    rclcpp::shutdown();
    return 0;
}
