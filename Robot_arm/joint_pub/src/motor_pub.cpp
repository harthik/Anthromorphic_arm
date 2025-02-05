#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>  // Change to Float64MultiArray
#include <eigen3/Eigen/Dense>
#include <cmath>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::placeholders::_1;

class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher()
        : Node("minimal_publisher"), q(VectorXd::Zero(3)), t(0.0), dt(0.01), accumulated_q_move(VectorXd::Zero(3)), elapsed_time(0.0) {
        // Change to Float64MultiArray publisher
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MinimalPublisher::timer_callback, this));
        q(0) = 0.0;
        q(1) = M_PI/4;
        q(2) = -M_PI/2;
    }

private:
    void timer_callback() {
        VectorXd qdot = compute_joint_angle_update(q, t); // for calc
        VectorXd q_move = qdot * dt;  // Not for calc
        accumulated_q_move += q_move;  // Not for calc
        q += qdot * dt;  // for calc
        t += dt;  // for calc
        elapsed_time += dt;  // Not for calcs

        // Check if the condition to stop the publisher is met
        VectorXd xe = direct_kinematics_SCARA(q);
        VectorXd pd(3);
        pd(0) = 0.14;
        pd(1) = 0.14;
        pd(2) = 0.05;
        VectorXd diff = xe - pd;
        double norm_xe = diff.norm();
        if (norm_xe < 0.001) {
            RCLCPP_INFO(this->get_logger(), "Stopping publisher: norm(xe) = %f", norm_xe);
            rclcpp::shutdown();
            return;
        }

        // Publish the accumulated updates every 0.5 seconds this needs to be updated when publishing to stepper motors
        if (elapsed_time >= 0.05) {
            // Create the Float64MultiArray message
            std_msgs::msg::Float64MultiArray msg;
            msg.data = std::vector<double>(q.data(), q.data() + q.size()); // publish accumilaed_q_move for physical stepper motors
            publisher_->publish(msg);

            // std::string joint_angles_str = vector_to_string(msg.data);  // Convert the vector to a string
            // RCLCPP_INFO(this->get_logger(), "Publishing joint angles: %s", joint_angles_str.c_str());

            // Reset the accumulator and elapsed time
            accumulated_q_move.setZero();
            elapsed_time = 0.0;
        }
    }

    VectorXd direct_kinematics_SCARA(const VectorXd &q) {
        VectorXd xe(3);
        xe(0) = 0.1 * (cos(q(1)) + cos(q(1) + q(2))) * cos(q(0));
        xe(1) = 0.1 * (cos(q(1)) + cos(q(1) + q(2))) * sin(q(0));
        xe(2) = 0.1 * sin(q(1)) + 0.1 * sin(q(1) + q(2)) + 0.05;
        return xe;
    }

    MatrixXd analytical_jacobian(const VectorXd &q) {
        MatrixXd J_A(3, 3);
        J_A(0, 0) = -0.1 * (cos(q(1)) + cos(q(1) + q(2))) * sin(q(0));
        J_A(0, 1) = -0.1 * (sin(q(1)) + sin(q(1) + q(2))) * cos(q(0));
        J_A(0, 2) = -0.1 * sin(q(1) + q(2)) * cos(q(0));
        J_A(1, 0) = 0.1 * (cos(q(1)) + cos(q(1) + q(2))) * cos(q(0));
        J_A(1, 1) = -0.1 * (sin(q(1)) + sin(q(1) + q(2))) * sin(q(0));
        J_A(1, 2) = -0.1 * sin(q(1) + q(2)) * sin(q(0));
        J_A(2, 0) = 0;
        J_A(2, 1) = 0.1 * (cos(q(1)) + cos(q(1) + q(2)));
        J_A(2, 2) = 0.1 * cos(q(1) + q(2));
        return J_A;
    }

    VectorXd compute_joint_angle_update(const VectorXd &q, double t) {
        VectorXd pd(3);
        pd(0) = 0.14;
        pd(1) = 0.14;
        pd(2) = 0.05;

        VectorXd pd_dot(3);
        pd_dot(0) = 0.05;
        pd_dot(1) = 0.05;
        pd_dot(2) = 0;

        MatrixXd K = MatrixXd::Identity(3, 3);
        VectorXd x_e = direct_kinematics_SCARA(q);
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

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    VectorXd q;
    double t;
    double dt;
    VectorXd accumulated_q_move;
    double elapsed_time;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}

