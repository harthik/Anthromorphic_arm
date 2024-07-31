#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::placeholders::_1;

class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher()
        : Node("minimal_publisher"), q(VectorXd::Zero(3)), t(0.0), dt(0.01), a1(10.0), a2(10.0), accumulated_q_move(VectorXd::Zero(3)), elapsed_time(0.0) {
        publisher_ = this->create_publisher<std_msgs::msg::Int32MultiArray>("joint_angles", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MinimalPublisher::timer_callback, this));
        q(0) = 0.0;
        q(1) = M_PI/4;
        q(2) = -M_PI/4;
    }

private:
    void timer_callback() {
        VectorXd qdot = compute_joint_angle_update(a1, a2, q, t); // for calc
        VectorXd q_move = qdot * dt * 180 / M_PI;  // Not for calc
        accumulated_q_move += q_move;  // Not for calc
        q += qdot * dt;  // for calc
        t += dt;  // for calc
        elapsed_time += dt;  // Not for calc

        // Check if the condition to stop the publisher is met
        VectorXd xe = direct_kinematics_SCARA(a1, a2, q);
        double norm_xe = xe.norm();
        if (norm_xe < 9) {
            RCLCPP_INFO(this->get_logger(), "Stopping publisher: norm(xe) = %f", norm_xe);
            rclcpp::shutdown();
            return;
        }

        // Publish the accumulated updates every 0.5 seconds
        if (elapsed_time >= 0.5) {
            std_msgs::msg::Int32MultiArray msg;
            msg.data = std::vector<int>(accumulated_q_move.data(), accumulated_q_move.data() + accumulated_q_move.size());
            publisher_->publish(msg);
            RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", vector_to_string(msg.data).c_str());

            // Reset the accumulator and elapsed time
            accumulated_q_move.setZero();
            elapsed_time = 0.0;
        }
    }

    VectorXd direct_kinematics_SCARA(double a1, double a2, const VectorXd &q) {
        VectorXd xe(3);
        xe(0) = cos(q(0)) * (a1 * cos(q(1)) + a2 * cos(q(1) + q(2)));
        xe(1) = sin(q(0)) * (a1 * cos(q(1)) + a2 * cos(q(1) + q(2)));
        xe(2) = a1 * sin(q(1)) + a2 * sin(q(1)+q(2));
        //RCLCPP_INFO(this->get_logger(), "q1 %f, q2 %f, q3 %f", q(0), q(1), q(2));
        //RCLCPP_INFO(this->get_logger(), "x1 %f, x2 %f, x3 %f", xe(0), xe(1), xe(2));
        return xe;
    }

    MatrixXd analytical_jacobian(const VectorXd &q) {
        MatrixXd J_A(3, 3);
        J_A(0, 0) = 5.0000 * sin(q(1) - q(0) + q(2)) - 5.0000 * sin(q(0) + q(1) + q(2)) - 5.0000 * sin(q(0) - q(1)) - 5.0000 * sin(q(0) + q(1));
        J_A(0, 1) = 5.0000 * sin(q(0) - q(1)) - 5.0000 * sin(q(0) + q(1) + q(2)) - 5.0000 * sin(q(1) - q(0) + q(2)) - 5.0000 * sin(q(0) + q(1));
        J_A(0, 2) = -5.0000 * sin(q(0) + q(1) + q(2)) - 5.0000 * sin(q(1) - q(0) + q(2));
        J_A(1, 0) = 5.0000 * cos(q(0) - q(1)) + 5.0000 * cos(q(0) + q(1) + q(2)) + 5.0000 * cos(q(1) - q(0) + q(2)) + 5.0000 * cos(q(0) + q(1));
        J_A(1, 1) = 5.0000 * cos(q(0) + q(1) + q(2)) - 5.0000 * cos(q(0) - q(1)) - 5.0000 * cos(q(1) - q(0) + q(2));
        J_A(1, 2) = 5.0000 * cos(q(0) + q(1) + q(2)) - 5.0000 * cos(q(1) - q(0) + q(2));
        J_A(2, 0) = 0;
        J_A(2, 1) = 10 * cos(q(1) + q(2)) + 10 * cos(q(1));
        J_A(2, 2) = 10 * cos(q(1) + q(2));
        return J_A;
    }

    VectorXd compute_joint_angle_update(double a1, double a2, const VectorXd &q, double t) {
        VectorXd pd(3);
        pd(0) = 17 - 0.5 * t;
        pd(1) = 0;
        pd(2) = 7;

        VectorXd pd_dot(3);
        pd_dot(0) = 0.5;
        pd_dot(1) = 0;
        pd_dot(2) = 0;

        MatrixXd K = MatrixXd::Identity(3, 3) * 8;
        VectorXd x_e = direct_kinematics_SCARA(a1, a2, q);
        MatrixXd J_A = analytical_jacobian(q);
        VectorXd e = pd - x_e;
        VectorXd qdot = J_A.inverse() * (pd_dot + K * e);
        return qdot;
    }

    std::string vector_to_string(const std::vector<int>& vec) {
        std::ostringstream oss;
        for (const auto& val : vec) {
            oss << val << " ";
        }
        return oss.str();
    }

    rclcpp::Publisher<std_msgs::msg::Int32MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    VectorXd q;
    double t;
    double dt;
    int a1;
    int a2;
    VectorXd accumulated_q_move;
    double elapsed_time;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}
