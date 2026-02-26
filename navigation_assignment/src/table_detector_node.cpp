#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point32.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "std_srvs/srv/trigger.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "group17_assignment_1/srv/table_detection.hpp"

// Node for detecting tables using laser scans

class TableDetectorNode : public rclcpp::Node
{
public:
    TableDetectorNode() : Node("table_detector_node")
    {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher for detected table positions
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseArray>("table_positions", 10);

        // Subscriber to laser scan topic
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 1, std::bind(&TableDetectorNode::scan_callback, this, std::placeholders::_1));

        // Service to trigger table detection manually
        service_ = this->create_service<group17_assignment_1::srv::TableDetection>(
            "trigger_table_detection",
            std::bind(&TableDetectorNode::trigger_callback, this, std::placeholders::_1, std::placeholders::_2));

        // Initialize variables
        previous_point.x = 0.0f;
        previous_point.y = 0.0f;
        previous_point.z = 0.0f;

        // Initialize the last detected table poses array to avoid null access
        last_detected_pose_array_ = std::make_shared<geometry_msgs::msg::PoseArray>();
    }

private:

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Service<group17_assignment_1::srv::TableDetection>::SharedPtr service_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::vector<geometry_msgs::msg::Pose> current_table_poses_vector_;
    std::shared_ptr<geometry_msgs::msg::PoseArray> last_detected_pose_array_;
    std::vector<std::vector<geometry_msgs::msg::Point32>> all_clusters;
    std::vector<geometry_msgs::msg::Point32> current_cluster;
    geometry_msgs::msg::Point32 previous_point;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg_;

    double jump_threshold = 0.25; // threshold to separate clusters
    bool first_point_of_scan = true;
    double var_threshold_ = 0.0025; // variance threshold to identify circular clusters

    // Service callback: triggers table detection
    void trigger_callback(
        const std::shared_ptr<group17_assignment_1::srv::TableDetection::Request> request,
        std::shared_ptr<group17_assignment_1::srv::TableDetection::Response> response)
    {
        (void)request;

        RCLCPP_INFO(this->get_logger(), "Trigger request received.");

        // Run clustering analysis on last laser scan
        cluster_analysis(last_scan_msg_);

        // Return the latest table poses
        if (last_detected_pose_array_) {
            response->tables = *last_detected_pose_array_;
            response->success = !last_detected_pose_array_->poses.empty();
        } else {
            response->success = false;
        }

        response->message = response->success ? "Tables found." : "No tables detected.";
    }

    // LaserScan callback: store last received scan
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_msg_ = msg;
    }

    // Analyze clusters of laser scan points to detect tables
    void cluster_analysis(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        // Reset all clusters for new scan
        all_clusters.clear();
        current_cluster.clear();
        current_table_poses_vector_.clear();

        double current_angle = msg->angle_min;

        // Iterate through all ranges in the scan
        for (const float &range : msg->ranges) {
            // Ignore invalid points
            if (!std::isfinite(range) || range < msg->range_min || range > msg->range_max) {
                if (!current_cluster.empty()) {
                    all_clusters.push_back(current_cluster);
                    current_cluster.clear();
                }
                first_point_of_scan = true; // next point will start a new cluster
            } else {
                // Convert polar to Cartesian coordinates
                geometry_msgs::msg::Point32 current_point;
                current_point.x = range * cos(current_angle);
                current_point.y = range * sin(current_angle);
                current_point.z = 0.0f;

                if (first_point_of_scan) {
                    current_cluster.push_back(current_point);
                    first_point_of_scan = false;
                } else {
                    // Compute distance to previous point
                    double distance_to_previous = std::hypot(
                        static_cast<double>(current_point.x - previous_point.x),
                        static_cast<double>(current_point.y - previous_point.y));

                    if (distance_to_previous > jump_threshold) {
                        // End of current cluster
                        if (!current_cluster.empty()) {
                            all_clusters.push_back(current_cluster);
                        }
                        // Start new cluster
                        current_cluster.clear();
                        current_cluster.push_back(current_point);
                    } else {
                        // Continue current cluster
                        current_cluster.push_back(current_point);
                    }
                }
                // Update previous point
                previous_point = current_point;
            }

            current_angle += msg->angle_increment;
        }

        // Add last cluster if not empty
        if (!current_cluster.empty()) {
            all_clusters.push_back(current_cluster);
            current_cluster.clear();
        }

        // Analyze clusters to detect tables
        for (const auto &cluster : all_clusters) {

            if (is_cluster_circular(cluster)) {
                // Compute centroid of the cluster
                geometry_msgs::msg::Pose table_pose;
                float cx = 0.0f, cy = 0.0f;
                for (const auto &p : cluster) {
                    cx += p.x;
                    cy += p.y;
                }
                cx /= cluster.size();
                cy /= cluster.size();

                /*// Check if the table is within the "detection" radius to eliminate distant false positives
                double dist_sq = cx * cx + cy * cy;
                if (dist_sq > detection_radius_ * detection_radius_) {
                    continue;
                }*/

                table_pose.position.x = cx;
                table_pose.position.y = cy;
                table_pose.position.z = 0.0f;
                table_pose.orientation.w = 1.0; // No rotation

                try{
                    // Transform table pose to "odom" frame
                    geometry_msgs::msg::PoseStamped table_pose_stamped;
                    table_pose_stamped.header = msg->header;
                    table_pose_stamped.pose = table_pose;

                    auto table_pose_in_map = tf_buffer_->transform(table_pose_stamped,"odom", tf2::durationFromSec(0.5));

                    current_table_poses_vector_.push_back(table_pose_in_map.pose);

                }catch (const tf2::TransformException &ex) {
                    RCLCPP_WARN(this->get_logger(), "Transform failed: %s", ex.what());
                    continue;
                }
            }
        }

        // Publish detected table poses
        auto pose_array_msg = std::make_shared<geometry_msgs::msg::PoseArray>();
        pose_array_msg->header.stamp = this->now();
        pose_array_msg->header.frame_id = "map";
        pose_array_msg->poses = current_table_poses_vector_;

        // Update last detected table poses for service response
        last_detected_pose_array_ = pose_array_msg;

        publisher_->publish(*pose_array_msg);
        RCLCPP_INFO(this->get_logger(), "Published %zu tables.", current_table_poses_vector_.size());
    }

    // Check if a cluster is approximately circular
    bool is_cluster_circular(const std::vector<geometry_msgs::msg::Point32>& cluster)
    {
        if (cluster.size() < 2)
            return false;

        // Compute centroid
        float cx = 0.0f, cy = 0.0f;
        for (const auto &p : cluster) {
            cx += p.x;
            cy += p.y;
        }
        cx /= cluster.size();
        cy /= cluster.size();

        // Compute mean radius
        double mean_r = 0.0;
        std::vector<double> radii;
        radii.reserve(cluster.size());

        for (const auto &p : cluster) {
            double r = std::hypot(p.x - cx, p.y - cy);
            radii.push_back(r);
            mean_r += r;
        }
        mean_r /= cluster.size();

        // Compute radial variance
        double var_r = 0.0;
        for (double r : radii)
            var_r += (r - mean_r) * (r - mean_r);
        var_r /= cluster.size();

        // Low radial variance indicates circular shape (likely a table)
        return (var_r < var_threshold_); 
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TableDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
