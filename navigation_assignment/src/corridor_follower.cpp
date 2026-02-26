#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "group17_assignment_1/action/follow_corridor.hpp"

// node class for manually moving the robot in the corridor

class CorridorFollower : public rclcpp::Node
{
public:
    CorridorFollower() : Node("corridor_follower")
    {
        // publisher to send velocity commands   
        vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10);

        // lidar subscriber to monitor obstacles
        lidar_subscriber_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10,
            std::bind(&CorridorFollower::lidar_callback, this, std::placeholders::_1));

        // action server to follow corridor
        corridor_action_server_ = rclcpp_action::create_server<group17_assignment_1::action::FollowCorridor>(
            this, "follow_corridor",
            std::bind(&CorridorFollower::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CorridorFollower::handle_cancel, this, std::placeholders::_1),
            std::bind(&CorridorFollower::handle_accepted, this, std::placeholders::_1)
        );

        prev_time = this->now();
        RCLCPP_INFO(this->get_logger(), "Corridor Manual Move Node has been started.");
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscriber_;
    rclcpp_action::Server<group17_assignment_1::action::FollowCorridor>::SharedPtr corridor_action_server_;

    sensor_msgs::msg::LaserScan::SharedPtr last_scan_;
    geometry_msgs::msg::PoseStamped entry_pose_;

    bool end_of_corridor_ = false;
    bool in_corridor_ = false;
    bool first_error_ = true;

    float wall_distance_error_ = 0.0;
    float prev_error = 0.0;

    float kp = 0.2;
    float kd = 0.3;

    rclcpp::Time prev_time;
    float dt = 0.1;


    // Action server setup
    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID & uuid,
                    std::shared_ptr<const group17_assignment_1::action::FollowCorridor::Goal> goal)
    {
        RCLCPP_INFO(this->get_logger(), "Received corridor follow goal request");
        in_corridor_ = true;
        entry_pose_ = goal->initial_pose;
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<group17_assignment_1::action::FollowCorridor>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel corridor follow goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<group17_assignment_1::action::FollowCorridor>> goal_handle)
    {
        std::thread{std::bind(&CorridorFollower::execute, this, std::placeholders::_1), goal_handle}.detach();
    }


    // Main execution function for the corridor following
    void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<group17_assignment_1::action::FollowCorridor>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing corridor follow goal...");

        auto feedback = std::make_shared<group17_assignment_1::action::FollowCorridor::Feedback>();
        auto result = std::make_shared<group17_assignment_1::action::FollowCorridor::Result>();

        // Main loop to move in the corridor
        rclcpp::Rate loop_rate(10);

        while (rclcpp::ok()) {

            // Check if goal is canceled
            if (goal_handle->is_canceling()) {
                RCLCPP_INFO(this->get_logger(), "Corridor follow goal canceled");
                goal_handle->canceled(result);
                return;
            }

            // Check for end of corridor
            if (end_of_corridor_) {
                in_corridor_ = false;
                RCLCPP_INFO(this->get_logger(), "End of corridor detected, stopping execution.");
                goal_handle->succeed(result);
                return;
            }

            // Move in corridor
            move_in_corridor();

            // Publish feedback (I use wall distance error as feedback)
            feedback->distance_error = wall_distance_error_; 
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }

        // Goal succeeded
        RCLCPP_INFO(this->get_logger(), "Corridor follow goal succeeded");
        goal_handle->succeed(result);
    }


    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_ = msg;

        // i need to compute the wall distances and error only when in corridor
        if (in_corridor_)
        { 
            detect_walls_and_error(last_scan_);

            // Update time
            dt = (this->now() - prev_time).seconds();
            prev_time = this->now();
        } 
    }

    void detect_walls_and_error(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // initialize minimum distances (casually)
        float min_left = msg->ranges[0];
        float min_right = msg->ranges[0];

        int total_ranges = msg->ranges.size();

        // I check the minimum distance for half of the LIDAR ranges
        for (int i = 0; i < total_ranges/2; ++i) {
            if (msg->ranges[i] < min_left) {
                min_left = msg->ranges[i];
            }
        }
        for (int i = total_ranges/2; i < total_ranges; ++i) {
            if (msg->ranges[i] < min_right) {
                min_right = msg->ranges[i];
            }
        }

        // debug info
        // RCLCPP_INFO(this->get_logger(), "Min Left: %.2f -- Min Right: %.2f", min_left, min_right);

        // I define an error based on the wall distances
        wall_distance_error_ = min_left - min_right;
        
        RCLCPP_INFO(this->get_logger(), "Wall Distance Error: %.2f", wall_distance_error_);

        // condition to detect end of corridor (end when one side distance is too high)
        if(wall_distance_error_ > 0.1) {
            end_of_corridor_ = true;
        }
    }

    // Function to move the robot in the corridor based on wall distance error
    void move_in_corridor()
    {
        // fixed linear velocity
        auto vel_msg = geometry_msgs::msg::Twist();
        vel_msg.linear.x = 0.3;
        vel_msg.linear.y = 0.0;
        vel_msg.linear.z = 0.0;

        // angular velocity based on the error (PD controller)
        float curr_error = wall_distance_error_;
        float derivative = 0.0;

        if (first_error_) {
            derivative = 0.0;
            prev_error = curr_error;
            first_error_ = false;
        } else {
            derivative = (curr_error - prev_error) / dt;
        }

        float angular_vel_z = kp * curr_error + kd * derivative;

        prev_error = curr_error;

        vel_msg.angular.x = 0.0;
        vel_msg.angular.y = 0.0;
        vel_msg.angular.z = angular_vel_z;

        RCLCPP_INFO(this->get_logger(), "Publishing velocity command: linear_x=%.2f angular_z=%.2f",
                    vel_msg.linear.x, vel_msg.angular.z);     
        
        vel_publisher_->publish(vel_msg);
    }
};


int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CorridorFollower>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}