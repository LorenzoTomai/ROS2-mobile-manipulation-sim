#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp" 
#include "group17_assignment_1/srv/corridor_detection.hpp"

struct Point2D {
    double x;
    double y;
    int index; // original index in the scan array for reference
};

// node class for detecting corridors based on laser scan data

class CorridorDetector : public rclcpp::Node
{
public:
    CorridorDetector() : Node("corridor_detector_node")
    {
        // 1. Publisher
        publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("corridor_position", 10);

        // 2. Subscriber
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 1, std::bind(&CorridorDetector::scan_callback, this, std::placeholders::_1));

        // 3. Service
        service_ = this->create_service<group17_assignment_1::srv::CorridorDetection>(
            "trigger_corridor_detection",
            std::bind(&CorridorDetector::trigger_callback, this, std::placeholders::_1, std::placeholders::_2));
    }
private:

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Service<group17_assignment_1::srv::CorridorDetection>::SharedPtr service_;
    sensor_msgs::msg::LaserScan::SharedPtr last_scan_msg_;
    geometry_msgs::msg::PoseStamped corridor_position_;
        
    float jump_threshold = 0.20; // threshold for discontinuity


    void trigger_callback(
        const std::shared_ptr<group17_assignment_1::srv::CorridorDetection::Request> request,
        std::shared_ptr<group17_assignment_1::srv::CorridorDetection::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Trigger request received.");

        if (last_scan_msg_ == nullptr) {
            RCLCPP_WARN(this->get_logger(), "No scan data available.");
            response->success = false;
            return;
        }

        scan_analysis(last_scan_msg_);

        // Return the last available result
        response->corridor_pose = corridor_position_;
        response->success = true;

    }
    
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        last_scan_msg_ = msg;

        //scan_analysis(msg); Remove comment for testing
    }

    void scan_analysis(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        corridor_position_.header.frame_id = "odom";
        corridor_position_.header.stamp = msg->header.stamp;

        std::vector<float> discontinuity_angles;
        std::vector<float> discontinuity_ranges;
        std::vector<Point2D> points;

        for (int i = 0; i < msg->ranges.size() - 1; ++i) {
            float range = msg->ranges[i];
            float angle = msg->angle_min + i * msg->angle_increment;
            if (range < msg->range_min || range > msg->range_max || !std::isfinite(range)) continue;

            float next_range = msg->ranges[i+1];
            if (!std::isfinite(next_range)) continue;

            if (fabs(next_range - range) > jump_threshold) {
                discontinuity_angles.push_back(angle);
                discontinuity_ranges.push_back(range);
            }

            Point2D p;
            p.x = range * cos(angle);
            p.y = range * sin(angle);
            p.index = i;
            points.push_back(p);
        }

        if(discontinuity_ranges.size() < 2){
            RCLCPP_INFO(this->get_logger(), "Corridor not detected.");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Corridor detected with %zu discontinuities.", discontinuity_ranges.size());
        // ---- Retrieve points with smallest range ----

        std::vector<int> indices(discontinuity_ranges.size());
        std::iota(indices.begin(), indices.end(), 0);

        std::sort(indices.begin(), indices.end(),
                [&](int a, int b) {
                    return discontinuity_ranges[a] < discontinuity_ranges[b];
                });
        
        
        std::vector<float> best_angles;
        std::vector<float> best_ranges;

        for (int i = 0; i < 8 && i < indices.size(); ++i) {
            int idx = indices[i];
            best_angles.push_back(discontinuity_angles[idx]);
            best_ranges.push_back(discontinuity_ranges[idx]);
        }

        float cx = 0.0f;
        float cy = 0.0f;

        for (int i = 0; i < best_angles.size(); ++i) {
            float x = best_ranges[i] * std::cos(best_angles[i]);
            float y = best_ranges[i] * std::sin(best_angles[i]);
            cx += x;
            cy += y;
        }
     
        cx = cx/best_angles.size(); 
        cy = cy/best_angles.size();

        // Sliding Window to find corners
        int k = 20; // Window size
        std::vector<Point2D> angle_points;
        
        for (size_t i = k; i < points.size() - k; ++i) {
            
            Point2D p_prev = points[i - k];
            Point2D p_curr = points[i];
            Point2D p_next = points[i + k];

            // Vector 1: From previous to current
            double v1_x = p_curr.x - p_prev.x;
            double v1_y = p_curr.y - p_prev.y;

            // Vector 2: From current to next
            double v2_x = p_next.x - p_curr.x;
            double v2_y = p_next.y - p_curr.y;

            // Calculate absolute angles of vectors
            double angle1 = atan2(v1_y, v1_x);
            double angle2 = atan2(v2_y, v2_x);

            // Calculate difference (the "turn" angle)
            double angle_diff = angle2 - angle1;

            // Normalize angle between -PI and PI
            while (angle_diff <= -M_PI) angle_diff += 2 * M_PI;
            while (angle_diff > M_PI) angle_diff -= 2 * M_PI;

            // Convert to degrees for easier logic
            double deg_diff = angle_diff * (180.0 / M_PI);

            // Corner detection (around 90 degrees)
            if (std::abs(deg_diff) > 80.0 && std::abs(deg_diff) < 100.0) {
                angle_points.push_back(p_curr);

                // Skip ahead to avoid multiple detections of the same corner
                i += k; 
            }
        }

        // Find the two corners closest to the corridor centroid
        double dist1 = std::hypot(cx - angle_points[0].x,
                                  cy - angle_points[0].y);
        double dist2 = std::hypot(cx - angle_points[1].x,
                                  cy - angle_points[1].y);  
        Point2D corner1 = angle_points[0];
        Point2D corner2 = angle_points[1];                        
        for(size_t i = 2; i < angle_points.size(); ++i){      
            double dist = std::hypot(cx - angle_points[i].x,
                                      cy - angle_points[i].y);
            if(dist < dist1){
                dist1 = dist;
                corner1 = angle_points[i];
            }else if(dist < dist2){
                dist2 = dist;
                corner2 = angle_points[i];
            }          
        }

        RCLCPP_INFO(this->get_logger(), "Angles detected at (%.2f, %.2f) and (%.2f, %.2f).", 
                    corner1.x, corner1.y, corner2.x, corner2.y);
        RCLCPP_INFO(this->get_logger(), "Preliminary centroid at (%.2f, %.2f).", 
                    cx, cy);

        // The two closest corners are corner1 and corner2 and represent the corridor entrance
        // Give more weight to these two points to calculate the final centroid
        cx = (cx + 3*corner1.x + 3*corner2.x)/ 7.0;
        cy = (cy + 3*corner1.y + 3*corner2.y) / 7.0;

        corridor_position_.pose.position.x = cx;
        corridor_position_.pose.position.y = cy;

        // The coordinates are in the laser frame, no need to transform them to odom because the detection is done 
        // with the robot stationary at the starting point

        RCLCPP_INFO(this->get_logger(), "Corridor detected at position (%.2f, %.2f).", 
                    corridor_position_.pose.position.x, corridor_position_.pose.position.y);
        publisher_->publish(corridor_position_);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CorridorDetector>());
    rclcpp::shutdown();
    return 0;
}