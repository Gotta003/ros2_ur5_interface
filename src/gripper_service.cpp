#include "rclcpp/rclcpp.hpp"
#include <std_srvs/srv/trigger.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_tolerance.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <memory>
#include <vector>

class GripperController : public rclcpp::Node
{
public:
    GripperController()
        : Node("gripper_service"),
          start_config_({0.0, 0.0}),
          time_between_points_(0.1)
    {
        // Create services
        open_service_ = this->create_service<std_srvs::srv::Trigger>(
            "open_gripper", std::bind(&GripperController::open, this, std::placeholders::_1, std::placeholders::_2));
        close_service_ = this->create_service<std_srvs::srv::Trigger>(
            "close_gripper", std::bind(&GripperController::close, this, std::placeholders::_1, std::placeholders::_2));
        neutral_service_ = this->create_service<std_srvs::srv::Trigger>(
            "neutral_gripper", std::bind(&GripperController::neutral, this, std::placeholders::_1, std::placeholders::_2));

        // Subscribe to the joint states topic
        subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&GripperController::joint_state_callback, this, std::placeholders::_1));

        // Initialize action client
        action_client_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            this, "/gripper_controller/follow_joint_trajectory");

        RCLCPP_INFO(this->get_logger(), "Gripper controller node initialized.");
    }

private:
    std::vector<double> start_config_;
    double time_between_points_;

    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr open_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr close_service_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr neutral_service_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr subscription_;
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr action_client_;

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        if (msg->position.size() > 7) // Ensure valid indices
        {
            start_config_ = {msg->position[5], msg->position[7]};
        }
    }

    void send_action(const std::vector<double> &end_config)
    {
        if (!action_client_->wait_for_action_server(std::chrono::seconds(5)))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting.");
            return;
        }

        // Create the JointTrajectory message
        trajectory_msgs::msg::JointTrajectory traj_msg;
        traj_msg.header.stamp = rclcpp::Time(0);
        traj_msg.joint_names = {"soft_robotics_gripper_left_finger_joint1", "soft_robotics_gripper_right_finger_joint1"};

        double T = 3 * time_between_points_;

        for (int i = 0; i <= 3; i++)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            double t = i * time_between_points_;
            for (size_t j = 0; j < start_config_.size(); j++)
            {
                double interpolated_position = start_config_[j] + (t / T) * (end_config[j] - start_config_[j]);
                point.positions.push_back(interpolated_position);
            }
            point.time_from_start = rclcpp::Duration::from_seconds(t);
            traj_msg.points.push_back(point);
        }

        // Create a goal message
        auto goal_msg = control_msgs::action::FollowJointTrajectory::Goal();
        goal_msg.trajectory = traj_msg;
        goal_msg.goal_time_tolerance = rclcpp::Duration::from_seconds(0.5);

        RCLCPP_INFO(this->get_logger(), "Sending gripper goal.");

        auto send_goal_options = rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SendGoalOptions();
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    void open(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
              std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Request to open the gripper received.");
        send_action({0.3, 0.3});
        response->success = true;
    }

    void close(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
               std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Request to close the gripper received.");
        send_action({-0.3, -0.3});
        response->success = true;
    }

    void neutral(const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
                 std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Request to move the gripper to neutral position received.");
        send_action({0.0, 0.0});
        response->success = true;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
