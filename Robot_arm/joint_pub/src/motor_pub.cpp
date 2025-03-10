#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <geometry_msgs/msg/point.hpp>  // For receiving position commands
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::placeholders::_1;

class MinimalPublisher : public rclcpp::Node {
public:
    MinimalPublisher()
        : Node("minimal_publisher"), q(VectorXd::Zero(3)), accumulated_q_move(VectorXd::Zero(3)) {
        
        // Publisher for joint commands
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/forward_position_controller/commands", 10);
        
        // Subscriber for desired position
        position_subscriber_ = this->create_subscription<geometry_msgs::msg::Point>(
            "desired_pos",
            10,
            std::bind(&MinimalPublisher::position_callback, this, _1));
        
        // Timer for control loop
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MinimalPublisher::timer_callback, this));
        
        // Transform listener setup
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // Initialize joint positions
        q(0) = 0.0;
        q(1) = M_PI/4;
        q(2) = -M_PI/2;
        
        // Initialize actual end effector position
        actual_xe = VectorXd::Zero(3);
        
        // Initialize desired position with default values
        pd = VectorXd::Zero(3);
        pd(0) = 0.135;
        pd(1) = 0.0;
        pd(2) = 0.1;

        // Initialize the last update time to now
        last_update_time_ = this->now();
        last_publish_time_ = this->now();

        RCLCPP_INFO(this->get_logger(), "Robot controller initialized with default target position: [%f, %f, %f]", 
                    pd(0), pd(1), pd(2));
        RCLCPP_INFO(this->get_logger(), "Listening for new position commands on topic: desired_pos");
    }

private:
    void position_callback(const geometry_msgs::msg::Point::SharedPtr msg) {
        // Check if the new position is different from the current desired position
        if (pd(0) != msg->x || pd(1) != msg->y || pd(2) != msg->z) {
            // Update the desired position when a new message is received
            pd(0) = msg->x;
            pd(1) = msg->y;
            pd(2) = msg->z;
            
            // Reset position reached flag and set target changed flag
            position_reached_ = false;
            
            RCLCPP_INFO(this->get_logger(), "Received new desired position: [%f, %f, %f]", pd(0), pd(1), pd(2));
        }
    }

    void timer_callback() {
        // Calculate actual elapsed time since last update
        rclcpp::Time current_time = this->now();
        double dt = (current_time - last_update_time_).seconds();
        last_update_time_ = current_time;

        try {
            geometry_msgs::msg::TransformStamped transform = 
                tf_buffer_->lookupTransform("base_link", "link_4", tf2::TimePointZero);
                
            actual_xe(0) = transform.transform.translation.x;
            actual_xe(1) = transform.transform.translation.y;
            actual_xe(2) = transform.transform.translation.z;
        } catch (const tf2::TransformException & ex) {
            RCLCPP_ERROR(this->get_logger(), "Could not transform: %s", ex.what());
            // Fall back to forward kinematics
            //actual_xe = direct_kinematics_SCARA(q);
        }

        // Only proceed if enough time has passed (to handle very fast callbacks)
        if (dt > 0.001) { // Minimum threshold to avoid numerical issues
            VectorXd qdot = compute_joint_angle_update(q, actual_xe);
            VectorXd q_move = qdot * dt;  // Use actual elapsed time
            accumulated_q_move += q_move;
            q += qdot * dt;
            
            // Check if the condition to stop the publisher is met
            VectorXd diff = actual_xe - pd;
            double norm_xe = diff.norm();
            if (norm_xe < 0.005) {
                RCLCPP_INFO(this->get_logger(), "Target position reached: norm(xe) = %f", norm_xe);
                position_reached_ = true;
                // Don't shutdown - wait for new position commands
            }
        }

        // Check if it's time to publish (every 50ms)
        double time_since_publish = (current_time - last_publish_time_).seconds();
        if (time_since_publish >= 0.05 && position_reached_ == false) {
            // Create the Float64MultiArray message
            std_msgs::msg::Float64MultiArray msg;
            msg.data = std::vector<double>(q.data(), q.data() + q.size());
            publisher_->publish(msg);
            
            RCLCPP_INFO(this->get_logger(), "publishing joint angles: [%f, %f, %f]", 
                        q(0) * 180 / M_PI, q(1) * 180 / M_PI, q(2) * 180 / M_PI);
            
            // Reset the accumulator and update the last publish time
            accumulated_q_move.setZero();
            last_publish_time_ = current_time;
        }
    }

    MatrixXd analytical_jacobian(const VectorXd &q) {
        MatrixXd J_A(3, 3);
        J_A(0, 0) = -0.048 * sin(q(0) - q(1)) - 0.048 * sin(q(0) + q(1)) + 0.049 * sin(-q(0) + q(1) + q(2)) - 0.049 * sin(q(0) + q(1) + q(2));
        J_A(0, 1) = 0.048 * sin(q(0) - q(1)) - 0.048 * sin(q(0) + q(1)) - 0.049 * sin(-q(0) + q(1) + q(2)) - 0.049 * sin(q(0) + q(1) + q(2));
        J_A(0, 2) = -0.049 * sin(-q(0) + q(1) + q(2)) - 0.049 * sin(q(0) + q(1) + q(2));
        J_A(1, 0) = 0.048 * cos(q(0) - q(1)) + 0.048 * cos(q(0) + q(1)) + 0.049 * cos(-q(0) + q(1) + q(2)) + 0.049 * cos(q(0) + q(1) + q(2));
        J_A(1, 1) = -0.048 * cos(q(0) - q(1)) + 0.048 * cos(q(0) + q(1)) - 0.049 * cos(-q(0) + q(1) + q(2)) + 0.049 * cos(q(0) + q(1) + q(2));
        J_A(1, 2) = -0.049 * cos(-q(0) + q(1) + q(2)) + 0.049 * cos(q(0) + q(1) + q(2));
        J_A(2, 0) = 0;
        J_A(2, 1) = 0.096 * cos(q(1)) + 0.098 * cos(q(1) + q(2));
        J_A(2, 2) = 0.098 * cos(q(1) + q(2));
        
        return J_A;
    }

    VectorXd compute_joint_angle_update(const VectorXd &q, const VectorXd &actual_xe) {
        VectorXd pd_dot(3);
        pd_dot(0) = 0.0;
        pd_dot(1) = 0.0;
        pd_dot(2) = 0.0;

        MatrixXd K = MatrixXd::Identity(3, 3);
        MatrixXd J_A = analytical_jacobian(q);
        VectorXd e = pd - actual_xe;
        MatrixXd J_A_pseudo_inverse = J_A.completeOrthogonalDecomposition().pseudoInverse();
        VectorXd qdot = J_A_pseudo_inverse * (pd_dot + K * e);
        return qdot;
    }

    // Publisher for joint commands
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    
    // Subscriber for desired position
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr position_subscriber_;
    
    // Timer for control loop
    rclcpp::TimerBase::SharedPtr timer_;
    
    // Joint positions and accumulated movement
    VectorXd q;
    VectorXd accumulated_q_move;
    
    // Timing variables
    rclcpp::Time last_update_time_;
    rclcpp::Time last_publish_time_;
    
    // TF listener components
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    
    // Position variables
    VectorXd actual_xe;
    VectorXd pd;  // Desired position, now updated via subscription
    
    bool position_reached_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalPublisher>());
    rclcpp::shutdown();
    return 0;
}