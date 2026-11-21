// 核心控制节点：基于解析几何雅可比与虚拟弹簧的双指对捏
// 解决问题：高维矩阵连乘、DLS防奇异点死锁、指令积分抗真机摩擦

#include <vector>
#include <memory>
#include <cmath>
#include <chrono>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::chrono_literals;

class AllegroCommander: public rclcpp::Node {
private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    
    // 当前角度缓存
    double a0, a1, a2, a3;
    double b0, b1, b2, b3;
    
    // 控制与状态变量
    Eigen::Vector4d cmd_t, cmd_i;
    Eigen::Vector3d P_thumb, P_index;
    Eigen::Vector3d P_thumb_target, P_index_target;
    Eigen::Vector3d V_thumb, V_index;
    Eigen::Vector4d q_thumb_dot, q_index_dot;
    
    Eigen::Matrix<double,3,4> J_thumb;
    Eigen::Matrix<double,3,4> J_index;
    
    double K_p;
    double step_time;
    double dt;
    double T_total;
    double index_start_y, thumb_start_y;
    bool if_run;

public:
    AllegroCommander() : Node("thumb_index_catch"), step_time(0.0), K_p(3.0), if_run(true) {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/allegro_hand_position_controller/commands", 10);
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10, std::bind(&AllegroCommander::joint_state_callback, this, std::placeholders::_1));
        
        // 锁定 50Hz 控制周期
        timer_ = this->create_wall_timer(20ms, std::bind(&AllegroCommander::timer_callback, this));

        a0 = 0.0; a1 = 0.0; a2 = 0.0; a3 = 0.0;
        b0 = 0.0; b1 = 0.0; b2 = 0.0; b3 = 0.0;
        T_total = 3.0; // 拟人化收拢时间设定为3秒
        
        RCLCPP_INFO(this->get_logger(), "Sim-to-Real Controller started! Waiting for joint states...");
    }

private:
    // 异步回调只做一件事：从总线抓取最新角度，不做任何耗时计算
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr joint_state){
        if(joint_state->velocity.size() < 8) return;
        for(size_t i = 0; i < joint_state->name.size(); i++){
            if(joint_state->name[i] == "ah_joint00") a0 = joint_state->position[i];
            if(joint_state->name[i] == "ah_joint01") a1 = joint_state->position[i];
            if(joint_state->name[i] == "ah_joint02") a2 = joint_state->position[i];
            if(joint_state->name[i] == "ah_joint03") a3 = joint_state->position[i];
            
            if(joint_state->name[i] == "ah_joint10") b0 = joint_state->position[i];
            if(joint_state->name[i] == "ah_joint11") b1 = joint_state->position[i];
            if(joint_state->name[i] == "ah_joint12") b2 = joint_state->position[i];
            if(joint_state->name[i] == "ah_joint13") b3 = joint_state->position[i];
        }
    }

    // 手推正运动学与几何雅可比 (大拇指)
    void compute_fk_thumb_xyz(double q0, double q1, double q2, double q3){
        using namespace Eigen;
        Matrix<double,3,4> P = Matrix<double,3,4>::Zero();
        
        // 注意：提取轴向时必须用前一级的绝对位姿 (T_pre)，否则旋转方向全错
        Isometry3d T_base = Isometry3d::Identity();
        T_base.linear() = (AngleAxisd(-1.570796, Vector3d::UnitZ()) * 
                           AngleAxisd(-1.65806, Vector3d::UnitY()) * 
                           AngleAxisd(0, Vector3d::UnitX())).toRotationMatrix();
        T_base.translation() = Vector3d(-0.0182, 0.019333, -0.045987);
        Isometry3d T0 = T_base * AngleAxisd(q0, Vector3d(-1, 0, 0));
        P.col(0) = T_base.translation();

        Isometry3d T_link01 = Isometry3d::Identity();
        T_link01.translation() = Vector3d(-0.027, 0.005, 0.0399);
        Isometry3d T1_pre = T0 * T_link01; 
        Isometry3d T1 = T1_pre * AngleAxisd(q1, Vector3d(0, 0, 1));
        P.col(1) = T1_pre.translation();

        Isometry3d T_link02 = Isometry3d::Identity();
        T_link02.translation() = Vector3d(0, 0, 0.0177);
        Isometry3d T2_pre = T1 * T_link02;
        Isometry3d T2 = T2_pre * AngleAxisd(q2, Vector3d(0, 1, 0));
        P.col(2) = T2_pre.translation();

        Isometry3d T_link03 = Isometry3d::Identity();
        T_link03.translation() = Vector3d(0, 0, 0.0514);
        Isometry3d T3_pre = T2 * T_link03;
        Isometry3d T3 = T3_pre * AngleAxisd(q3, Vector3d(0, 1, 0));
        P.col(3) = T3_pre.translation();

        Isometry3d T_link04 = Isometry3d::Identity();
        T_link04.translation() = Vector3d(0, 0, 0.0267);
        P_thumb = (T3 * T_link04).translation();
        
        // 解析几何雅可比 (w x r)
        Vector3d J0 = (T_base.linear() * Vector3d(-1,0,0)).cross(P_thumb - P.col(0));
        Vector3d J1 = (T1_pre.linear() * Vector3d(0,0,1)).cross(P_thumb - P.col(1));
        Vector3d J2 = (T2_pre.linear() * Vector3d(0,1,0)).cross(P_thumb - P.col(2));
        Vector3d J3 = (T3_pre.linear() * Vector3d(0,1,0)).cross(P_thumb - P.col(3));

        J_thumb << J0, J1, J2, J3;
    }

    // 手推正运动学与几何雅可比 (食指)
    void compute_fk_index_xyz(double q0, double q1, double q2, double q3){
        using namespace Eigen;
        Matrix<double,3,4> P = Matrix<double,3,4>::Zero();
        
        Isometry3d T_base = Isometry3d::Identity();
        T_base.linear() = (AngleAxisd(0, Vector3d::UnitZ()) * 
                           AngleAxisd(0, Vector3d::UnitY()) * 
                           AngleAxisd(-0.087266, Vector3d::UnitX())).toRotationMatrix();
        T_base.translation() = Vector3d(0, 0.0435, 0);
        Isometry3d T0 = T_base * AngleAxisd(q0, Vector3d(0, 0, 1));
        P.col(0) = T_base.translation();

        Isometry3d T_link01 = Isometry3d::Identity();
        T_link01.translation() = Vector3d(0, 0, 0.0164);
        Isometry3d T1_pre = T0 * T_link01;
        Isometry3d T1 = T1_pre * AngleAxisd(q1, Vector3d(0, 1, 0));
        P.col(1) = T1_pre.translation();

        Isometry3d T_link02 = Isometry3d::Identity();
        T_link02.translation() = Vector3d(0, 0, 0.054);
        Isometry3d T2_pre = T1 * T_link02;
        Isometry3d T2 = T2_pre * AngleAxisd(q2, Vector3d(0, 1, 0));
        P.col(2) = T2_pre.translation();

        Isometry3d T_link03 = Isometry3d::Identity();
        T_link03.translation() = Vector3d(0, 0, 0.0384);
        Isometry3d T3_pre = T2 * T_link03;
        Isometry3d T3 = T3_pre * AngleAxisd(q3, Vector3d(0, 1, 0));
        P.col(3) = T3_pre.translation();

        Isometry3d T_link04 = Isometry3d::Identity();
        T_link04.translation() = Vector3d(0, 0, 0.0267);
        P_index = (T3 * T_link04).translation();
        
        Vector3d J0 = (T_base.linear() * Vector3d(0,0,1)).cross(P_index - P.col(0));
        Vector3d J1 = (T1_pre.linear() * Vector3d(0,1,0)).cross(P_index - P.col(1));
        Vector3d J2 = (T2_pre.linear() * Vector3d(0,1,0)).cross(P_index - P.col(2));
        Vector3d J3 = (T3_pre.linear() * Vector3d(0,1,0)).cross(P_index - P.col(3));

        J_index << J0, J1, J2, J3;
    }

    // 生成余弦平滑轨迹 (Smooth-Step)
    void target(){
        double progress = 0.0;
        if (step_time <= T_total) {
            progress = 0.5 * (1.0 - std::cos(M_PI * step_time / T_total));
        } else {
            progress = 1.0; 
        }   
        
        // 目标接触点：前伸85mm，高度25mm，食指正前方偏内侧25mm
        double center_x = 0.085;  
        double center_y = 0.025;
        double center_z = 0.025;  

        double current_index_y = index_start_y + progress * (center_y - index_start_y);
        double current_thumb_y = thumb_start_y + progress * (center_y - thumb_start_y);

        this->P_index_target = Eigen::Vector3d(center_x, current_index_y, center_z);
        this->P_thumb_target = Eigen::Vector3d(center_x, current_thumb_y, center_z);
    }

    // 50Hz 核心控制循环
    void timer_callback(){
        using namespace Eigen;
        dt = 0.02;

        compute_fk_index_xyz(b0, b1, b2, b3);
        compute_fk_thumb_xyz(a0, a1, a2, a3);

        // 软启动：只有算完真正的 FK 后，才能抓取准确的起跑线
        if(if_run){
            index_start_y = P_index.y();
            thumb_start_y = P_thumb.y();
            cmd_t = Vector4d(a0, a1, a2, a3);
            cmd_i = Vector4d(b0, b1, b2, b3);
            if_run = false;
            return;
        }

        target();

        // 虚拟弹簧产生拉力速度
        Vector3d error_thumb = P_thumb_target - P_thumb;
        Vector3d error_index = P_index_target - P_index;
        V_thumb = K_p * error_thumb;
        V_index = K_p * error_index;
        
        // DLS 阻尼最小二乘伪逆 (防奇异点)
        Matrix3d I3 = Matrix3d::Identity();
        double lambda = 0.01;
        Matrix<double,4,3> J_dagger_thumb = J_thumb.transpose() * (J_thumb * J_thumb.transpose() + I3 * lambda).inverse();
        Matrix<double,4,3> J_dagger_index = J_index.transpose() * (J_index * J_index.transpose() + I3 * lambda).inverse();
        
        q_thumb_dot = J_dagger_thumb * V_thumb;
        q_index_dot = J_dagger_index * V_index;
        
        // 纯指令积分器 (cmd += v*dt)，强行克服真机静摩擦和稳态死区
        cmd_t += q_thumb_dot * dt;
        cmd_i += q_index_dot * dt;

        // 硬限幅，防止输出打穿物理机械极限
        for(int k=0; k<4; ++k){
            cmd_t(k) = std::clamp(cmd_t(k), -0.2, 1.6);
            cmd_i(k) = std::clamp(cmd_i(k), -0.2, 1.6);
        }

        auto message = std_msgs::msg::Float64MultiArray();
        message.data.resize(16, 0.0);
        message.data[0] = cmd_t.x();
        message.data[1] = cmd_t.y();
        message.data[2] = cmd_t.z();
        message.data[3] = cmd_t.w();
        message.data[4] = cmd_i.x();
        message.data[5] = cmd_i.y();
        message.data[6] = cmd_i.z();
        message.data[7] = cmd_i.w();
        publisher_->publish(message);
        
        RCLCPP_INFO(this->get_logger(),"thumb: %5.2f | %5.2f | %5.2f", P_thumb.x(), P_thumb.y(), P_thumb.z());
        RCLCPP_INFO(this->get_logger(),"index: %5.2f | %5.2f | %5.2f", P_index.x(), P_index.y(), P_index.z());
        step_time += dt;
    }
};

int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    std::shared_ptr<AllegroCommander> node_ = std::make_shared<AllegroCommander>();
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;
}
