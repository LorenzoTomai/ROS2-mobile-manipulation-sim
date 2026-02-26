#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <std_msgs/msg/bool.hpp>

class ColorDetectorNode : public rclcpp::Node
{
public:
    ColorDetectorNode() : Node("color_detector_node")
    {
        // Create TF buffer and listener to handle transforms between frames
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Subscribe to the RGB camera images
        image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/rgb_camera/image", 10,
            std::bind(&ColorDetectorNode::image_callback, this, std::placeholders::_1)
        );

        // Subscribe to detected AprilTag poses
        tag_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/tag_poses", 10,
            std::bind(&ColorDetectorNode::tag_callback, this, std::placeholders::_1)
        );

        // Subscribe to camera info to get intrinsics (fx, fy, cx, cy)
        camera_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/rgb_camera/camera_info", 10,
            std::bind(&ColorDetectorNode::camera_info_callback, this, std::placeholders::_1)
        );

        // Subscribe to a "simulation_done" topic to print final summary
        simulation_done_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "simulation_done",
            10,
            [this](const std_msgs::msg::Bool::SharedPtr msg){
                if(msg->data){
                    RCLCPP_INFO(this->get_logger(), "Simulation done received. Printing final summary...");
                    print_final_summary();
                }
            }
        );

        RCLCPP_INFO(this->get_logger(), "ColorDetectorNode started");
    }

private:
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr tag_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr simulation_done_sub_;

    // Latest tag poses and flags
    geometry_msgs::msg::PoseArray latest_tags_;
    bool tags_received_ = false;
    bool camera_info_received_ = false;

    // Store last detected color of each cube (key: tag_id)
    std::map<int, std::string> detected_colors_;

    // Camera intrinsics
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;

    // TF buffer and listener for frame transforms
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // Callback to store latest AprilTag poses
    void tag_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        latest_tags_ = *msg;
        tags_received_ = true;
    }

    // Callback to store camera intrinsics
    void camera_info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr msg)
    {
        if(msg->k.size() >= 6) {
            fx = msg->k[0];
            fy = msg->k[4];
            cx = msg->k[2];
            cy = msg->k[5];
            camera_info_received_ = true;
        }
    }

    // Print final color detection summary
    void print_final_summary()
    {
        RCLCPP_INFO(this->get_logger(), "=== FINAL COLOR DETECTION SUMMARY ===");
        for (const auto &entry : detected_colors_)
        {
            int tag_id = entry.first;
            const std::string &color = entry.second;

            std::string color_text;
            if(color == "RED") color_text = "\033[31mRED\033[0m";  // Red text in terminal
            else if(color == "BLUE") color_text = "\033[34mBLUE\033[0m";  // Blue text
            else color_text = "UNKNOWN";

            RCLCPP_INFO(this->get_logger(), "Cube (tag %d): %s", tag_id, color_text.c_str());
        }
        RCLCPP_INFO(this->get_logger(), "===================================");
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        // Ensure that both AprilTag detections and camera intrinsics have been received before processing the image
        if (!tags_received_ || !camera_info_received_)
            return;

        // Convert ROS image message to OpenCV BGR image
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        cv::Mat image = cv_ptr->image;
        if (image.empty())
            return;

        // Iterate over all detected AprilTag poses
        for (size_t i = 0; i < latest_tags_.poses.size(); ++i)
        {
            const auto &tag_pose = latest_tags_.poses[i];

            // Ignore invalid or non-visible tags
            if (tag_pose.position.z <= 0.01)
                continue;

            // Manually associate index to tag ID (project-specific assumption)
            int tag_id = (i == 0) ? 1 : 10;

            // If the color for this tag has already been detected and locked,
            // skip further processing to avoid redundant computation
            if (detected_colors_.count(tag_id) &&
                detected_colors_[tag_id] != "UNKNOWN")
            {
                continue;
            }

            // Transform pose from base_link to camera frame
            geometry_msgs::msg::PoseStamped pose_base;
            pose_base.header.frame_id = "base_link";
            pose_base.header.stamp = this->now();
            pose_base.pose = tag_pose;

            geometry_msgs::msg::PoseStamped pose_camera;
            try
            {
                pose_camera = tf_buffer_->transform(
                    pose_base,
                    "external_camera/link/rgb_camera"
                );
            }
            catch (const tf2::TransformException &ex)
            {
                // If TF transform fails, skip this tag for the current frame
                RCLCPP_WARN(
                    this->get_logger(),
                    "TF error for tag %d: %s",
                    tag_id,
                    ex.what()
                );
                continue;
            }

            // Project 3D point into 2D image coordinates
            double X = pose_camera.pose.position.x;
            double Y = pose_camera.pose.position.y;
            double Z = pose_camera.pose.position.z;
            if (Z <= 0.01)
                continue;

            // Pinhole camera projection using intrinsic parameters
            int u = static_cast<int>(fx * X / Z + cx);
            int v = static_cast<int>(fy * Y / Z + cy);

            // A fixed-size ROI centered around the projected tag position
            int roi_size = 80;
            int x0 = std::max(0, u - roi_size / 2);
            int y0 = std::max(0, v - roi_size / 2);
            int w = std::min(roi_size, image.cols - x0);
            int h = std::min(roi_size, image.rows - y0);

            if (w <= 0 || h <= 0)
                continue;

            cv::Rect roi_rect(x0, y0, w, h);
            cv::Mat cube_roi = image(roi_rect);

            // HSV is used for better robustness against illumination changes
            cv::Mat hsv;
            cv::cvtColor(cube_roi, hsv, cv::COLOR_BGR2HSV);

            // Binary masks for red and blue colors
            cv::Mat mask_red1, mask_red2, mask_red, mask_blue;
            cv::inRange(hsv, cv::Scalar(0, 70, 50),
                        cv::Scalar(10, 255, 255), mask_red1);
            cv::inRange(hsv, cv::Scalar(170, 70, 50),
                        cv::Scalar(180, 255, 255), mask_red2);
            mask_red = mask_red1 | mask_red2;

            cv::inRange(hsv, cv::Scalar(100, 150, 50),
                        cv::Scalar(140, 255, 255), mask_blue);

            // Count number of pixels belonging to each color
            int red_pixels  = cv::countNonZero(mask_red);
            int blue_pixels = cv::countNonZero(mask_blue);

            // Decide color based on dominant pixel count
            std::string color_detected = "UNKNOWN";
            if (red_pixels > blue_pixels && red_pixels > 20)
                color_detected = "RED";
            else if (blue_pixels > red_pixels && blue_pixels > 20)
                color_detected = "BLUE";

            // Once a color is found, it is stored and no longer recomputed
            if (color_detected != "UNKNOWN")
            {
                detected_colors_[tag_id] = color_detected;

                std::string color_text =
                    (color_detected == "RED")  ? "\033[31mRED\033[0m" :
                    (color_detected == "BLUE") ? "\033[34mBLUE\033[0m" :
                                                 "UNKNOWN";

                RCLCPP_INFO(
                    this->get_logger(),
                    "[LOCKED] Cube (tag %d) detected as %s",
                    tag_id,
                    color_text.c_str()
                );
            }
            else
            {
                // Color not yet determined → keep searching, but limit log frequency
                RCLCPP_INFO_THROTTLE(
                    this->get_logger(),
                    *this->get_clock(),
                    2000,
                    "[ColorDetector] Searching color for tag %d...",
                    tag_id
                );
            }
        }
    }

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ColorDetectorNode>());
    rclcpp::shutdown();
    return 0;
}
