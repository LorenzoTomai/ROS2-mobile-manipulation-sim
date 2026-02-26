#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <nav2_msgs/srv/manage_lifecycle_nodes.hpp>

// Node class to setup Nav2 by activating localization and navigation nodes
class Nav2Setup : public rclcpp::Node {
public:
    Nav2Setup() : Node("nav2_setup_node") {
        // Create service clients to manage the lifecycle of localization and navigation nodes
        loc_client_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
            "/lifecycle_manager_localization/manage_nodes");
        nav_client_ = this->create_client<nav2_msgs::srv::ManageLifecycleNodes>(
            "/lifecycle_manager_navigation/manage_nodes");

        // Publisher for the initial pose to AMCL
        initialpose_pub_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 10);

        // Run the setup sequence
        run_sequence();
    }

private:
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr loc_client_;  // Client for localization lifecycle service
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr nav_client_;  // Client for navigation lifecycle service
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initialpose_pub_;  // Publisher for the initial robot pose

    // Sequence to setup Nav2: activate localization, publish initial pose, then activate navigation
    void run_sequence() {
        RCLCPP_INFO(this->get_logger(), "Waiting for Nav2 lifecycle services...");

        // Wait until both lifecycle services are available
        loc_client_->wait_for_service();
        nav_client_->wait_for_service();

        auto req = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        req->command = 0; // STARTUP command

        // 1. Activate the localization node (synchronously)
        RCLCPP_INFO(this->get_logger(), "Activating localization (waiting for response)...");
        auto loc_result_future = loc_client_->async_send_request(req);

        // Wait synchronously for the service response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), loc_result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to activate localization!");
            return;
        }

        // 2. Publish the initial pose (AMCL should now be ready)
        RCLCPP_INFO(this->get_logger(), "Publishing initial pose...");
        geometry_msgs::msg::PoseWithCovarianceStamped pose;
        pose.header.frame_id = "map";  // Reference frame for AMCL
        pose.header.stamp = now();
        pose.pose.pose.position.x = 0.0;
        pose.pose.pose.position.y = 0.0;
        pose.pose.pose.orientation.z = 0.0;
        pose.pose.pose.orientation.w = 1.0;

        // Covariance matrix representing uncertainty in the initial pose
        pose.pose.covariance = {
            0.25, 0, 0, 0, 0, 0,
            0, 0.25, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0,
            0, 0, 0, 0, 0, 0.068
        };

        // Publish the initial pose to ensure AMCL receives it
        initialpose_pub_->publish(pose);
        RCLCPP_INFO(this->get_logger(), "Initial pose published.");

        // 3. Activate the navigation node (synchronously)
        RCLCPP_INFO(this->get_logger(), "Activating navigation (waiting for response)...");
        auto nav_result_future = nav_client_->async_send_request(req);

        // Wait synchronously for the service response
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), nav_result_future) !=
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to activate navigation!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Nav2 setup complete!");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Nav2Setup>());
    rclcpp::shutdown();
    return 0;
}
