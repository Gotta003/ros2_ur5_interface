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

        time_between_points_ = 0.5; // Time between points in seconds

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
        
        // Interpolate between start and middle positions
        std::vector<double> start_config = {-1.60, -1.72, -2.20, -0.81, 1.60, 0.0};
        std::vector<double> end_config = {-1.41, -0.96, -1.8, -1.96, -1.60, 0.0}; 
                                          // -80.72, -70.44, -72.04, -127.13, -91.40, 0.0};

        // Total interpolation time (10 points * time_between_points_)
        double T = 10 * time_between_points_;

        double middle_time;
        for (int i = 0; i < 10 + 1; i++)
        {    
            // Create a joint trajectory point
            trajectory_msgs::msg::JointTrajectoryPoint point;

            // Calculate time elapsed
            double t = i * time_between_points_;

            // Interpolate each joint's position
            for (size_t j = 0; j < start_config.size(); j++) 
            {
                double interpolated_position = start_config[j] + (t / T) * (end_config[j] - start_config[j]);
                point.positions.push_back(interpolated_position);
            }

            // Set the time from the start for the point
            point.time_from_start = rclcpp::Duration::from_seconds(i * time_between_points_);
            // Add the point to the trajectory
            traj_msg.points.push_back(point);

            middle_time = i * time_between_points_;
        }

        middle_time += time_between_points_;

        // Interpolate between start and middle positions
        start_config = {-1.41, -0.96, -1.8, -1.96, -1.60, 0.0};
        end_config = {-1.41, -0.96, -1.8, -1.96, -1.60, 0.0}; 

        // Total interpolation time (10 points * time_between_points_)
        T = 3 * time_between_points_;

        double start_time = middle_time;
        for (int i = 0; i < 3 + 1; i++)
        {    
            // Create a joint trajectory point
            trajectory_msgs::msg::JointTrajectoryPoint point;

            // Calculate time elapsed
            double t = i * time_between_points_;

            // Interpolate each joint's position
            for (size_t j = 0; j < start_config.size(); j++) 
            {
                double interpolated_position = start_config[j] + (t / T) * (end_config[j] - start_config[j]);
                point.positions.push_back(interpolated_position);
            }

            // Set the time from the start for the point
            point.time_from_start = rclcpp::Duration::from_seconds(i * time_between_points_ + start_time);
            // Add the point to the trajectory
            traj_msg.points.push_back(point);

            middle_time = i * time_between_points_;
        }
        
        middle_time += time_between_points_;

        // Interpolate between start and middle positions
        start_config = {-1.41, -0.96, -1.8, -1.96, -1.60, 0.0}; 
        end_config = {-1.60, -1.72, -2.20, -0.81, 1.60, 0.0}; 

        // Total interpolation time (10 points * time_between_points_)
        T = 10 * time_between_points_;

        start_time += middle_time;
        for (int i = 0; i < 10 + 1; i++)
        {    
            // Create a joint trajectory point
            trajectory_msgs::msg::JointTrajectoryPoint point;

            // Calculate time elapsed
            double t = i * time_between_points_;

            // Interpolate each joint's position
            for (size_t j = 0; j < start_config.size(); j++) 
            {
                double interpolated_position = start_config[j] + (t / T) * (end_config[j] - start_config[j]);
                point.positions.push_back(interpolated_position);
            }

            // Set the time from the start for the point
            point.time_from_start = rclcpp::Duration::from_seconds(i * time_between_points_ + start_time);
            // Add the point to the trajectory
            traj_msg.points.push_back(point);

            middle_time = i * time_between_points_;
        }

        
        // Create a goal message for the action
        auto goal_msg = FollowJointTrajectory::Goal();
        goal_msg.trajectory = traj_msg;

        goal_msg.goal_time_tolerance.nanosec = 500000000;
        for (int i = 0; i < traj_msg.joint_names.size(); i++)
        {
            control_msgs::msg::JointTolerance joint_tolerance;
            joint_tolerance.name = traj_msg.joint_names[i];
            joint_tolerance.position = 1.0;
            joint_tolerance.velocity = 1.0;
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
    double time_between_points_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TrajectoryActionClient>());
    rclcpp::shutdown();
    return 0;
}
