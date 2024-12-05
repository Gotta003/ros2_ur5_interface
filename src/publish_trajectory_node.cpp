#include <rclcpp/rclcpp.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <control_msgs/msg/joint_tolerance.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <chrono>

using namespace std::chrono_literals;

class TrajectoryActionClient : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    TrajectoryActionClient() : Node("trajectory_action_client")
    {
        increment_ = 0.0; // Initialize the increment

        // Create an action client for the FollowJointTrajectory action
        action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/scaled_joint_trajectory_controller/follow_joint_trajectory");

        // Wait for the action server to be available
        if (!action_client_->wait_for_action_server(10s))
        {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        // Send a trajectory goal
        send_trajectory_goal();

    }

private:
    void send_trajectory_goal()
    {
        // Create the JointTrajectory message
        trajectory_msgs::msg::JointTrajectory traj_msg;
        // Set time to zero
        traj_msg.header.stamp = rclcpp::Time(0);
        traj_msg.joint_names = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"};
        
        for (int i = 0; i < 10; i++)
        {    
            // Create a joint trajectory point
            trajectory_msgs::msg::JointTrajectoryPoint point;
            point.positions = {-1.60 + increment_, -1.72, -2.20, -0.81, 1.60, -0.03}; // Example positions for each joint
            // point.velocities = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Example velocities for each joint
            // point.accelerations = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Example accelerations for each joint
            increment_ += 0.1; // Increment the position for the next trajectory

            // Set the time from the start for the point (e.g., 2 seconds)
            point.time_from_start.sec = i * 2.0;
            traj_msg.points.push_back(point);
        }
        
        // Create a goal message for the action
        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory = traj_msg;

        goal_msg.goal_time_tolerance.nanosec = 500000000;
        for (int i = 0; i < traj_msg.joint_names.size(); i++)
        {
            control_msgs::msg::JointTolerance joint_tolerance;
            joint_tolerance.name = traj_msg.joint_names[i];
            joint_tolerance.position = 0.1;
            joint_tolerance.velocity = 0.1;
            goal_msg.goal_tolerance.push_back(joint_tolerance);
        }

        RCLCPP_INFO(this->get_logger(), "Sending trajectory goal");

        // Send the goal to the action server
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            [this](const GoalHandleFollowJointTrajectory::SharedPtr& goal_handle) {
                if (!goal_handle)
                {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by the server");
                }
                else
                {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by the server, waiting for result");
                }
            };

        send_goal_options.result_callback =
            [this](const GoalHandleFollowJointTrajectory::WrappedResult &result) {
                switch (result.code)
                {
                case rclcpp_action::ResultCode::SUCCEEDED:
                    RCLCPP_INFO(this->get_logger(), "Goal succeeded");
                    break;
                case rclcpp_action::ResultCode::ABORTED:
                    RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                    break;
                case rclcpp_action::ResultCode::CANCELED:
                    RCLCPP_WARN(this->get_logger(), "Goal was canceled");
                    break;
                default:
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    break;
                }
            };

        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr action_client_;
    rclcpp::TimerBase::SharedPtr timer_;
    double increment_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryActionClient>());
    rclcpp::shutdown();
    return 0;
}
