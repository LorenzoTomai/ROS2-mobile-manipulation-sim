    #include <memory>
    #include <string>

    #include "rclcpp/rclcpp.hpp"
    #include "geometry_msgs/msg/pose_array.hpp"
    #include "geometry_msgs/msg/pose_stamped.hpp"

    // node class for processing AprilTag poses and publishing navigation goals 
    // (it also add a visualization marker for RViz)

    class AprilTagPoseProcessor : public rclcpp::Node
    {
    public:
        AprilTagPoseProcessor() : Node("apriltag_pose_processor")
        {
            // Subscribe to the PoseArray of AprilTag poses
            apriltag_pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
                "/tag_pose", 10,
                std::bind(&AprilTagPoseProcessor::apriltag_pose_callback, this, std::placeholders::_1));

            RCLCPP_INFO(this->get_logger(), "AprilTag Pose Processor Node started, waiting for tag poses...");

            // Publisher for navigation goal as PoseStamped
            goal_pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                "/nav_goal_pose", 10);
        }

    private:
        rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr apriltag_pose_subscription_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;

        // distance from apriltag midpoint and desired goal pose
        double offset = 0.5; 

        void apriltag_pose_callback(geometry_msgs::msg::PoseArray::SharedPtr msg)
        {
            if (msg->poses.size() < 2)
                return;

            // Determine goal pose based on detected tags
            choose_goal_pose(msg);
        }

        // Choose a goal pose between two AprilTags and publish it
        void choose_goal_pose(geometry_msgs::msg::PoseArray::SharedPtr msg)
        {
            const auto& pose_1 = msg->poses[0].position;
            const auto& pose_2 = msg->poses[1].position;

            geometry_msgs::msg::PoseStamped goal_pose_stamped;
            goal_pose_stamped.header.stamp = this->now();
            goal_pose_stamped.header.frame_id = "map";

            // Compute midpoint between two tag positions
            double mid_x = (pose_1.x + pose_2.x) / 2.0;
            double mid_y = (pose_1.y + pose_2.y) / 2.0;

            // Compute vector from first to second tag
            double dx = pose_2.x - pose_1.x;
            double dy = pose_2.y - pose_1.y;

            double norm = std::sqrt(dx*dx + dy*dy);
            dx /= norm;
            dy /= norm;

            // Compute perpendicular vector to define an offset
            double px = -dy;
            double py = dx;

            // Set goal position with offset from midpoint
            goal_pose_stamped.pose.position.x = mid_x + px * offset;
            goal_pose_stamped.pose.position.y = mid_y + py * offset;
            goal_pose_stamped.pose.position.z = 0.0;

            // Neutral orientation (facing forward)
            goal_pose_stamped.pose.orientation.x = 0.0;
            goal_pose_stamped.pose.orientation.y = 0.0;
            goal_pose_stamped.pose.orientation.z = 0.0;
            goal_pose_stamped.pose.orientation.w = 1.0;

            goal_pose_publisher_->publish(goal_pose_stamped);

            RCLCPP_INFO(this->get_logger(), "Published goal pose: [%.2f, %.2f]",
                        goal_pose_stamped.pose.position.x,
                        goal_pose_stamped.pose.position.y);
        }
    };


    int main(int argc, char * argv[])
    {
        rclcpp::init(argc, argv);
        rclcpp::spin(std::make_shared<AprilTagPoseProcessor>());  
        rclcpp::shutdown();
        return 0;
    }
