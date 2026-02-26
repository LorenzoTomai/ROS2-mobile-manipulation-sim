#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "group17_assignment_1/srv/corridor_detection.hpp"
#include "group17_assignment_1/action/follow_corridor.hpp"
#include "group17_assignment_1/srv/table_detection.hpp"

// Node for managing navigation goals and corridor following

class ManagerNode : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigateToPose = rclcpp_action::ClientGoalHandle<NavigateToPose>;
    using FollowCorridor = group17_assignment_1::action::FollowCorridor;
    using GoalHandleCorridor = rclcpp_action::ClientGoalHandle<FollowCorridor>;

    ManagerNode()
    : Node("manager_node")
    {
        RCLCPP_INFO(this->get_logger(), "ManagerNode has been started.");

        // Subscriber to navigation goal poses
        nav_goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/nav_goal_pose", 10,
            std::bind(&ManagerNode::nav_goal_callback, this, std::placeholders::_1)
        );

        // Subscriber to the robot's pose (used for checking corridor entry)
        robot_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/amcl_pose", 10,
            std::bind(&ManagerNode::robot_pose_callback, this, std::placeholders::_1)
        );

        // Action client for Nav2 navigation
        nav2_client_ = rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");

        // Service client for corridor detection
        corridor_detection_client_ = this->create_client<group17_assignment_1::srv::CorridorDetection>(
            "trigger_corridor_detection"
        );

        // Action client for corridor following
        corridor_action_client_ = rclcpp_action::create_client<group17_assignment_1::action::FollowCorridor>(
            this, "follow_corridor"
        );

        // Service client for table detection
        table_detection_client_ = this->create_client<group17_assignment_1::srv::TableDetection>(
            "trigger_table_detection"
        );

        // Timer to check if the Table Detection service is available
        service_timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            [this]() {
                if (!table_detection_client_->service_is_ready()) {
                    RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                        "Table detection service not available...");
                } else {
                    service_timer_->cancel(); // Service is ready, stop the timer
                    RCLCPP_INFO(this->get_logger(), "Table detection service ready.");
                }
            }
        );
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr nav_goal_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr robot_pose_sub_;
    rclcpp_action::Client<NavigateToPose>::SharedPtr nav2_client_;
    rclcpp::Client<group17_assignment_1::srv::CorridorDetection>::SharedPtr corridor_detection_client_;
    rclcpp_action::Client<group17_assignment_1::action::FollowCorridor>::SharedPtr corridor_action_client_;
    rclcpp::Client<group17_assignment_1::srv::TableDetection>::SharedPtr table_detection_client_;
    rclcpp::TimerBase::SharedPtr service_timer_;

    // Variables to save the last goal pose if navigation is interrupted for corridor entry
    geometry_msgs::msg::PoseStamped final_goal_pose_;
    geometry_msgs::msg::PoseStamped robot_pose_;
    geometry_msgs::msg::PoseStamped corridor_entry_pose_;

    bool goal_reached = false;
    bool tables_detected_ = false;
    bool goal_in_progress_ = false;
    bool corridor_active_ = false;
    bool robot_pose_received_ = false;
    bool corridor_trigger_received_ = false;
    bool canceling_for_corridor_ = false;  // Used to distinguish cancellation reason

    // NAVIGATION CALLBACKS

    void nav_goal_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        // Prevent multiple calls if a goal is already in progress or has been reached
        if (goal_in_progress_) {
            RCLCPP_WARN(this->get_logger(), "Goal already in progress, ignored.");
            return;
        }

        if (goal_reached) {
            RCLCPP_INFO(this->get_logger(), "Goal already reached, ignoring new goals.");
            return;
        }

        if (!nav2_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available.");
            return;
        }

        // Do not accept new goals while robot is navigating a corridor
        if (corridor_active_) {
            RCLCPP_WARN(this->get_logger(), "Robot is in the corridor, new goal ignored.");
            return;
        }

        NavigateToPose::Goal goal_msg;
        goal_msg.pose = *msg;

        // Save the goal globally for possible corridor navigation interruption
        final_goal_pose_ = *msg;

        // Attempt to call corridor detection service
        if (corridor_detection_client_->service_is_ready()) {
            trigger_corridor_detection();
        } else {
            RCLCPP_WARN(this->get_logger(), "Corridor detection service not available, proceeding without it.");
            corridor_trigger_received_ = false;
        }

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

        // Callback when the goal is accepted or rejected
        options.goal_response_callback = [this](std::shared_ptr<GoalHandleNavigateToPose> goal_handle){
            if (!goal_handle) {
                RCLCPP_ERROR(get_logger(), "Goal rejected by Nav2.");
            } else {
                RCLCPP_INFO(get_logger(), "Goal accepted by Nav2.");
                goal_in_progress_ = true;
            }
        };

        // Feedback callback while navigating
        options.feedback_callback = [](GoalHandleNavigateToPose::SharedPtr,
                                       const std::shared_ptr<const NavigateToPose::Feedback> feedback){
            RCLCPP_INFO(rclcpp::get_logger("manager_node"), "Feedback: %.2f", feedback->distance_remaining);
        };

        // Callback when the navigation goal finishes
        options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result){
            goal_in_progress_ = false;

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Goal reached successfully.");
                goal_reached = true;
                RCLCPP_INFO(this->get_logger(), "Calling table detection after Nav2 goal.");
                call_table_detection();

            } else if (rclcpp_action::ResultCode::CANCELED == result.code) {
                // Distinguish cancellation for corridor navigation vs other reasons
                if (canceling_for_corridor_) {
                    canceling_for_corridor_ = false;
                    RCLCPP_INFO(this->get_logger(), "Goal canceled for corridor navigation.");
                } else {
                    RCLCPP_WARN(this->get_logger(), "Goal canceled (unknown reason).");
                }
            } else {
                RCLCPP_ERROR(this->get_logger(), "Goal failed.");
            }
        };

        nav2_client_->async_send_goal(goal_msg, options);
    }

    // Callback to update the robot's current pose and check for corridor entry
    void robot_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        robot_pose_.header = msg->header;
        robot_pose_.pose = msg->pose.pose;
        robot_pose_received_ = true;

        // Check if robot is close to corridor entry to start corridor navigation
        if (corridor_trigger_received_ && !corridor_active_) {
            double dx = robot_pose_.pose.position.x - corridor_entry_pose_.pose.position.x;
            double dy = robot_pose_.pose.position.y - corridor_entry_pose_.pose.position.y;
            double distance = std::sqrt(dx * dx + dy * dy);

            if (distance < 0.5) {  // Threshold to start corridor navigation
                RCLCPP_INFO(this->get_logger(), "Robot is near corridor entry, starting corridor navigation.");
                start_corridor_navigation(corridor_entry_pose_);
            }
        }
    }

    // CORRIDOR NAVIGATION FUNCTIONS

    // Trigger corridor detection service
    void trigger_corridor_detection()
    {
        if (corridor_trigger_received_) { return; }

        if (!corridor_detection_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Corridor detection service not available.");
            return;
        }

        auto req = std::make_shared<group17_assignment_1::srv::CorridorDetection::Request>();

        auto future = corridor_detection_client_->async_send_request(req,
            [this](rclcpp::Client<group17_assignment_1::srv::CorridorDetection>::SharedFuture future) {
                auto resp = future.get();

                if (!resp->success) {
                    RCLCPP_ERROR(this->get_logger(), "Corridor detection failed.");
                    return;
                }

                RCLCPP_INFO(this->get_logger(), "Corridor detection succeeded.");
                corridor_entry_pose_ = resp->corridor_pose;
                corridor_trigger_received_ = true;
            }
        );
    }

    // Start corridor navigation
    void start_corridor_navigation(const geometry_msgs::msg::PoseStamped& msg)
    {
        if (!corridor_action_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "FollowCorridor action server not available.");
            return;
        }

        // Stop Nav2 stack if a goal is in progress (the final goal is already saved)
        if (goal_in_progress_) {
            RCLCPP_INFO(this->get_logger(), "Cancelling current Nav2 goal for corridor entry...");
            canceling_for_corridor_ = true;
            nav2_client_->async_cancel_all_goals();
        }

        group17_assignment_1::action::FollowCorridor::Goal goal;
        goal.initial_pose = msg;

        rclcpp_action::Client<FollowCorridor>::SendGoalOptions options;
        options.goal_response_callback = [this](std::shared_ptr<GoalHandleCorridor> goal_handle){
            if (!goal_handle) {
                RCLCPP_ERROR(get_logger(), "Corridor goal rejected.");
            } else {
                RCLCPP_INFO(get_logger(), "Corridor goal accepted.");
            }
        };

        options.feedback_callback = [](GoalHandleCorridor::SharedPtr,
                                    const std::shared_ptr<const FollowCorridor::Feedback> feedback){
            // Feedback processing
        };

        options.result_callback = [this](const GoalHandleCorridor::WrappedResult & result){
            corridor_active_ = false;

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Corridor goal reached successfully.");
                nav_goal_restart(final_goal_pose_);
            } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                RCLCPP_WARN(this->get_logger(), "Corridor goal canceled.");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Corridor goal failed.");
            }
        };

        corridor_active_ = true;
        corridor_action_client_->async_send_goal(goal, options);
    }

    // Restart the navigation goal after corridor completion
    void nav_goal_restart(const geometry_msgs::msg::PoseStamped& msg)
    {
        if (!nav2_client_->wait_for_action_server(std::chrono::seconds(5))) {
            RCLCPP_ERROR(this->get_logger(), "Nav2 action server not available.");
            return;
        }

        NavigateToPose::Goal goal;
        goal.pose = msg;

        rclcpp_action::Client<NavigateToPose>::SendGoalOptions options;

        options.goal_response_callback = [this](std::shared_ptr<GoalHandleNavigateToPose> goal_handle){
            if (!goal_handle) {
                RCLCPP_ERROR(get_logger(), "Nav2 goal rejected (restart).");
            } else {
                RCLCPP_INFO(get_logger(), "Nav2 goal accepted (restart).");
                goal_in_progress_ = true;
            }
        };

        options.feedback_callback = [](GoalHandleNavigateToPose::SharedPtr,
                                       const std::shared_ptr<const NavigateToPose::Feedback> feedback){
            RCLCPP_INFO(rclcpp::get_logger("manager_node"), "Feedback (restart): %.2f", feedback->distance_remaining);
        };

        options.result_callback = [this](const GoalHandleNavigateToPose::WrappedResult & result){
            goal_in_progress_ = false;

            if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                RCLCPP_INFO(this->get_logger(), "Goal reached successfully (after corridor).");
                goal_reached = true;
                RCLCPP_INFO(this->get_logger(), "Calling table detection after Nav2 restart.");
                call_table_detection();
            } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                RCLCPP_WARN(this->get_logger(), "Goal canceled (restart).");
            } else {
                RCLCPP_ERROR(this->get_logger(), "Goal failed (restart).");
            }
        };

        nav2_client_->async_send_goal(goal, options);
        corridor_active_ = false;
        RCLCPP_INFO(this->get_logger(), "Resuming previous navigation goal after corridor.");
    }

    // TABLE DETECTION FUNCTIONS

    void call_table_detection()
    {
        if (tables_detected_) return;

        // Wait for the table detection service to be ready
        if (!table_detection_client_->wait_for_service(std::chrono::seconds(5))) {
            RCLCPP_WARN(this->get_logger(), "Table detection service not available after 5s.");
            return;
        }

        auto req = std::make_shared<group17_assignment_1::srv::TableDetection::Request>();

        auto future = table_detection_client_->async_send_request(req,
            [this](rclcpp::Client<group17_assignment_1::srv::TableDetection>::SharedFuture future) {
                auto resp = future.get();

                RCLCPP_INFO(this->get_logger(), "TableDetection success: %d, number of tables: %zu",
                                                resp->success, resp->tables.poses.size());

                if (!resp->success || resp->tables.poses.empty()) {
                    RCLCPP_WARN(this->get_logger(), "No tables detected or service failed.");
                    return;
                }

                // Log detected tables
                for (size_t i = 0; i < resp->tables.poses.size(); i++) {
                    RCLCPP_INFO(this->get_logger(),
                        "Table %zu → map: x=%.2f y=%.2f",
                        i, resp->tables.poses[i].position.x, resp->tables.poses[i].position.y
                    );
                }

                tables_detected_ = true;
            }
        );
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ManagerNode>());
    rclcpp::shutdown();
    return 0;
}