#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "apriltag_msgs/msg/april_tag_detection_array.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_array.hpp"

class ApriltagListener : public rclcpp::Node
{
public:
    ApriltagListener() : Node("apriltag_listener")
    {
        // Create a TF buffer and listener to handle transforms
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to AprilTag detections
        apriltag_subscription_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
              "/apriltag/detections", 10,
            std::bind(&ApriltagListener::apriltag_callback, this, std::placeholders::_1));

        // Publisher of the poses
        tag_pose_array_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
              "/tag_poses", 10);

        RCLCPP_INFO(this->get_logger(), "Node started, waiting for tag detections...");
    }

private:
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr tag_pose_array_publisher_;

    void apriltag_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        if(msg->detections.empty()) 
        {
            // RCLCPP_INFO(this->get_logger(), "No tags detected.");
            return;
        }
        geometry_msgs::msg::PoseArray tag_pose_array;
        tag_pose_array.header.frame_id = "base_link";       // The poses will be taken with respect to base_link
        tag_pose_array.header.stamp = this->get_clock()->now();

        tag_pose_array.poses.resize(2);     // Fix the space for two poses (tag 1 and tag 10)

        for (const auto& detection : msg->detections)
        {
            int tag_id = detection.id;

            // Filter only tags with ID 1 and 10
            if(tag_id != 1 && tag_id != 10)
                continue;

            std::string tag_frame = "tag36h11:" + std::to_string(tag_id);

            geometry_msgs::msg::TransformStamped tag_tf;
            try {
                // Lookup the transform from the tag frame to the robot's base_link
                tag_tf = tf_buffer_->lookupTransform("base_link", tag_frame, tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "TF not found for tag %d: %s", tag_id, ex.what());
                continue;
            }

            // Convert the transform to a PoseStamped message
            geometry_msgs::msg::PoseStamped tag_pose;
            tag_pose.header = tag_tf.header;
            tag_pose.pose.position.x = tag_tf.transform.translation.x;
            tag_pose.pose.position.y = tag_tf.transform.translation.y;
            tag_pose.pose.position.z = tag_tf.transform.translation.z;
            tag_pose.pose.orientation = tag_tf.transform.rotation;

            // Add the pose to the PoseArray in the correct index
            if(tag_id == 1)
                tag_pose_array.poses[0] = tag_pose.pose;
            else if(tag_id == 10)
                tag_pose_array.poses[1] = tag_pose.pose;   
        }

        tag_pose_array_publisher_->publish(tag_pose_array);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApriltagListener>());  // Run the node
    rclcpp::shutdown();
    return 0;
}
