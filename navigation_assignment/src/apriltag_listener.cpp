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

// node class for listening to AprilTag detections and publishing their poses

class ApriltagListener : public rclcpp::Node
{
public:
    ApriltagListener() : Node("apriltag_listener")
    {
        // Initialize TF2 buffer and listener for transform lookups
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to the AprilTag detections topic
        apriltag_subscription_ = this->create_subscription<apriltag_msgs::msg::AprilTagDetectionArray>(
              "/apriltag/detections", 10,
            std::bind(&ApriltagListener::apriltag_callback, this, std::placeholders::_1));

        // Publisher to output the PoseArray of detected tags
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
              "/tag_pose", 10);

        RCLCPP_INFO(this->get_logger(), "Node started, waiting for tag detections...");
    }

private:

    // class members
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    rclcpp::Subscription<apriltag_msgs::msg::AprilTagDetectionArray>::SharedPtr apriltag_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_publisher_;

    void apriltag_callback(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        // Wait for 1 second to ensure TF2 buffer is populated with transforms
        rclcpp::sleep_for(std::chrono::seconds(1));

        // Process detections and publish poses
        detect_and_publish_tag_pose(msg);
    }

    // Function to process tag detections and publish their poses as a PoseArray
    void detect_and_publish_tag_pose(const apriltag_msgs::msg::AprilTagDetectionArray::SharedPtr msg)
    {
        if (msg->detections.empty())
        {
            RCLCPP_INFO(this->get_logger(), "No tags detected.");
            return;
        }

        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header = msg->header;  // Keep same timestamp and frame as detections

        for (const auto& detection : msg->detections)
        {
            int tag_id = detection.id;
            std::string tag_frame = detection.family + ":" + std::to_string(tag_id);  // Construct tag frame name

            geometry_msgs::msg::TransformStamped tag_tf;
            try {
                // Lookup transform from 'map' frame to the tag frame
                tag_tf = tf_buffer_->lookupTransform("map", tag_frame, tf2::TimePointZero);
            } catch (const tf2::TransformException & ex) {
                RCLCPP_WARN(this->get_logger(), "TF not found: %s", ex.what());
                continue;  // Skip this detection if TF not available
            }

            // Construct PoseStamped message from the transform
            geometry_msgs::msg::PoseStamped tag_pose;
            tag_pose.header = tag_tf.header;
            tag_pose.pose.position.x = tag_tf.transform.translation.x;
            tag_pose.pose.position.y = tag_tf.transform.translation.y;
            tag_pose.pose.position.z = tag_tf.transform.translation.z;
            tag_pose.pose.orientation = tag_tf.transform.rotation;

            // Add the pose to the PoseArray
            pose_array.poses.push_back(tag_pose.pose);
        }
        pose_publisher_->publish(pose_array);
    }
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ApriltagListener>());
    rclcpp::shutdown();
    return 0;
}
