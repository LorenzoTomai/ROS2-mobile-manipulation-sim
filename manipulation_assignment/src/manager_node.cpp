#include <chrono>
#include <memory>
#include <thread>

// ===================== ROS2 CORE =====================
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

// ===================== MESSAGES =====================
// AprilTag poses
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
// Gripper action
#include <control_msgs/action/gripper_command.hpp>
// End simulation
#include <std_msgs/msg/bool.hpp>

// ===================== MOVEIT =====================
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>

// ===================== TF =====================
#include <tf2/LinearMath/Quaternion.h>

/**
 * Main manager node:
 * - Handles perception (AprilTags)
 * - Controls robot motion with MoveIt
 * - Controls the gripper via action interface
 */
class ManagerNode : public rclcpp::Node
{
public:
  // Type aliases for the gripper action
  using GripperCommand    = control_msgs::action::GripperCommand;
  using GoalHandleGripper = rclcpp_action::ClientGoalHandle<GripperCommand>;

  explicit ManagerNode(const rclcpp::NodeOptions & options)
  : Node("manager_node", options)
  {
    RCLCPP_INFO(get_logger(), "ManagerNode has been started.");

    // Subscribe to detected AprilTag poses
    target_sub_ = create_subscription<geometry_msgs::msg::PoseArray>(
      "/tag_poses",
      10,
      std::bind(&ManagerNode::apriltag_poses_callback, this, std::placeholders::_1)
    );

    // Action client to control the gripper
    gripper_action_client_ =
      rclcpp_action::create_client<GripperCommand>(
        this,
        "/gripper_controller/gripper_cmd"
      );

    // Publisher used only for RViz visualization/debug
    debug_target_poses_pub_ =
      create_publisher<geometry_msgs::msg::PoseArray>(
        "debug_target_poses",
        10
      );
    // Red and blue tag sums
    red_tag_sum_ = geometry_msgs::msg::Pose();
    blue_tag_sum_ = geometry_msgs::msg::Pose();

    // Publisher simulation done
    simulation_done_pub_ = create_publisher<std_msgs::msg::Bool>("simulation_done", 10);
  }

  ~ManagerNode() = default;

  // Entry point called from main()
  void run(moveit::planning_interface::MoveGroupInterface & move_group)
  {
    main_logic(move_group);
  }

private:
  /* ===================== ROS INTERFACES ===================== */

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr target_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr    debug_target_poses_pub_;
  rclcpp_action::Client<GripperCommand>::SharedPtr               gripper_action_client_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr simulation_done_pub_;

  // MoveIt planning scene interface (collision handling)
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;

  /* ===================== CONSTANTS ===================== */
  const double GRIPPER_LENGTH_ = 0.15;      // gripper length
  const double GRASP_DEPTH_ = 0.04;         // depth to insert gripper into the cube (almost half cube height)
  const double APPROACH_OFFSET_ = 0.03;     // offset above the cube top surface
  const double PLACE_OFFSET_ = 0.10;        // offset above the place position

  /* ===================== POSES ===================== */

  // Detected AprilTag poses
  geometry_msgs::msg::PoseStamped red_tag_pose_;
  geometry_msgs::msg::PoseStamped blue_tag_pose_;

  // Pick and place poses
  geometry_msgs::msg::PoseStamped red_pick_pose_;
  geometry_msgs::msg::PoseStamped blue_pick_pose_;
  geometry_msgs::msg::PoseStamped red_goal_pose_;
  geometry_msgs::msg::PoseStamped blue_goal_pose_;

  // Debug pose array for RViz
  geometry_msgs::msg::PoseArray debug_poses;

  geometry_msgs::msg::Pose red_tag_sum_;
  geometry_msgs::msg::Pose blue_tag_sum_;

  /* ===================== JOINTS POSES ===================== */

  std::vector<double> home_joints_ = {0.0, -1.57, 0.0, -1.57, 0.0, 0.0};
  std::vector<double> ready_joints_ = {2.356, -1.57, -0.78, -2.0, 1.57, 0.0};

  /* ===================== PIPELINE STATE ===================== */

  // AprilTag averaging
  static constexpr int TAG_AVG_SAMPLES = 20;
  int tag_sample_count_ = 0;
  int tag_sample_red_ = 0;
  int tag_sample_blue_ = 0;

  // Used to avoid reprocessing AprilTag detections
  bool targets_received_{false};
  bool red_tag_received_{false};
  bool blue_tag_received_{false};

  enum class PickPlaceState {
    NONE,
    PRE_PICK_FAILURE,       // Approach failure
    DESCEND_FAILURE,        // Descend failure
    LIFT_FAILURE,           // Lift failure
    PRE_PLACE_FAILURE,      // Pre-place movement failure
    PLACE_DESCEND_FAILURE,  // Place descend failure
    LOST,                   // Lost target (probably dropped)
    MOVED,                  // Target was moved before pick
    SUCCESS
  };

  PickPlaceState ERROR_STATE_ = PickPlaceState::NONE;
  PickPlaceState red_state_ = PickPlaceState::NONE;
  PickPlaceState blue_state_ = PickPlaceState::NONE;

  /* ===================== APRILTAG CALLBACK ===================== */

  /**
   * Receives AprilTag poses.
   * Averaging over TAG_AVG_SAMPLES frames for stability.
   */

  void apriltag_poses_callback(
    const geometry_msgs::msg::PoseArray::SharedPtr msg)
  {
    if (targets_received_) return;
    
    if (msg->poses.size() < 2) return;  

    // Accumulated RED
    if (msg->poses[0].position.z != 0.0) { // (for sure a non_valid detection)
      red_tag_sum_.position.x += msg->poses[0].position.x;
      red_tag_sum_.position.y += msg->poses[0].position.y;
      red_tag_sum_.position.z += msg->poses[0].position.z;
      tag_sample_red_++;
    }

    // Accumulated BLUE
    if (msg->poses[1].position.z != 0.0) {
      blue_tag_sum_.position.x += msg->poses[1].position.x;
      blue_tag_sum_.position.y += msg->poses[1].position.y;
      blue_tag_sum_.position.z += msg->poses[1].position.z;
      tag_sample_blue_++;
    }

    tag_sample_count_++;

    RCLCPP_INFO( get_logger(),
      "Averaging AprilTags: %d / %d samples collected", tag_sample_count_, TAG_AVG_SAMPLES );

    if (tag_sample_count_ >= TAG_AVG_SAMPLES) {
      
      red_tag_pose_.header.frame_id = "base_link";
      blue_tag_pose_.header.frame_id = "base_link";
      
      // RED average
      if (tag_sample_red_ > 0) {
    
        red_tag_pose_.pose.position.x =
          red_tag_sum_.position.x / tag_sample_red_;
        red_tag_pose_.pose.position.y =
          red_tag_sum_.position.y / tag_sample_red_;
        red_tag_pose_.pose.position.z =
          red_tag_sum_.position.z / tag_sample_red_;
        
        red_tag_pose_.pose.orientation = msg->poses[0].orientation;

        red_tag_received_ = true;
      }

      // BLUE average
      if (tag_sample_blue_ > 0) {
        blue_tag_pose_.pose.position.x =
          blue_tag_sum_.position.x / tag_sample_blue_;
        blue_tag_pose_.pose.position.y =
          blue_tag_sum_.position.y / tag_sample_blue_;
        blue_tag_pose_.pose.position.z =
          blue_tag_sum_.position.z / tag_sample_blue_;

        blue_tag_pose_.pose.orientation = msg->poses[1].orientation;

        blue_tag_received_ = true;
      }

      // Correction offsets 
      red_tag_pose_.pose.position.x += 0.03;
      blue_tag_pose_.pose.position.x += 0.03;

      // Reset 
      tag_sample_count_ = 0;
      tag_sample_red_ = 0;
      tag_sample_blue_ = 0;
      
      // Reset sums in case of re-use
      red_tag_sum_ = geometry_msgs::msg::Pose();
      blue_tag_sum_ = geometry_msgs::msg::Pose();

      targets_received_ = true;   // to unlock main logic
        
      RCLCPP_INFO(get_logger(), "AprilTags averaged and locked");
    } 
  }

  /* ===================== MAIN LOGIC ===================== */

  /**
   * High-level task execution:
   *  - Go home
   *  - Wait for AprilTags
   *  - Compute pick/place poses
   *  - Pick & place red and blue objects
   */
  void main_logic(moveit::planning_interface::MoveGroupInterface & move_group)
  {
    // Allow controllers to fully start
    std::this_thread::sleep_for(std::chrono::seconds(10));

    // MoveIt planning parameters
    move_group.setMaxVelocityScalingFactor(0.3);    
    move_group.setMaxAccelerationScalingFactor(0.3);  
    move_group.setPlanningTime(10.0);
    move_group.setGoalPositionTolerance(0.005);
    move_group.setGoalOrientationTolerance(0.05);

    // Move robot to READY configuration (above the tables)
    if (!go_ready(move_group)) return;

    // Wait until AprilTag targets are received
    while (rclcpp::ok() && !targets_received_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    // Check that both tags were received
    if (!red_tag_received_ || !blue_tag_received_) {
      RCLCPP_ERROR(get_logger(), "AprilTag detection failed: missing tags");
      return;
    }

    // Compute pick and place poses (swap targets)
    define_pick_and_place_poses_and_orientations_();

    // Add collision objects to the planning scene
    add_collision_object(red_tag_pose_, "red_cube");
    add_collision_object(blue_pick_pose_, "blue_cube");
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Execute pick & place for red cube
    if (pick_and_place_(move_group, red_pick_pose_, red_goal_pose_, "red_cube")) {
      red_state_ = PickPlaceState::SUCCESS;

    } else {  
      RCLCPP_ERROR(get_logger(), "Pick & Place failed for RED cube");
      
      if(!handle_pick_place_failure(move_group, "red_cube")) {
        RCLCPP_INFO(get_logger(), "END THE SIMULATION: aborted due to fatal error.");
        return; // Stop the simulation
      }
      // Save the error state for future analysis 
      red_state_ = ERROR_STATE_;
    }

    // -- OPTIONAL: Re-detect AprilTags before handling the blue cube to consider possible shifts
    tag_sample_count_ = 0;
    red_tag_received_ = false;
    blue_tag_received_ = false;
    targets_received_ = false;

    // Return robot to READY to have a better view    
    if (!go_ready(move_group)) return;

    while (rclcpp::ok() && !targets_received_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    // Check that both tags were received
    if (!red_tag_received_) {
        RCLCPP_INFO(get_logger(), "Red tag not detected after re-detection.");
        red_state_ = PickPlaceState::LOST;  
        // If only the red tag is missing, proceed anyway with the blue cube handling
    }

    if (!blue_tag_received_) {
      RCLCPP_INFO(get_logger(), "blue tag not detected after re-detection.");
      RCLCPP_INFO(get_logger(), "END THE SIMULATION: blue cube missing after red cube moved."); 
      blue_state_ = PickPlaceState::MOVED;
      return;
    }

    // Remove previous collision objects
    planning_scene_interface_.removeCollisionObjects({"red_cube", "blue_cube"});
    rclcpp::sleep_for(std::chrono::milliseconds(500)); 

    define_pick_and_place_poses_and_orientations_(false);

    // Update collision object
    if (red_tag_received_)
      add_collision_object(red_pick_pose_, "red_cube");
    
    add_collision_object(blue_pick_pose_, "blue_cube");
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Execute pick & place for blue cube
    if (pick_and_place_(move_group, blue_pick_pose_, blue_goal_pose_, "blue_cube")) {
      blue_state_ = PickPlaceState::SUCCESS;

    } else {
      RCLCPP_ERROR(get_logger(), "Pick & Place failed for blue cube");
      if(!handle_pick_place_failure(move_group, "blue_cube")) {
        RCLCPP_INFO(get_logger(), "END THE SIMULATION: aborted due to fatal error.");
        return; // Stop the simulation
      }
      // save the error state for future analysis 
      blue_state_ = ERROR_STATE_;
    }

    // Return robot to HOME
    go_home(move_group);

    // Close the gripper
    close_gripper_fully();

    // Final check of cube positions
    final_check();

    // Simulation Done
    std_msgs::msg::Bool msg;
    msg.data = true;
    simulation_done_pub_->publish(msg);

    // Print final states
    RCLCPP_INFO(get_logger(), "Simulation completed. Published 'simulation_done'. \n ");
    if (red_state_ == PickPlaceState::SUCCESS) {
      RCLCPP_INFO(get_logger(), "Red cube: SUCCESS");
    } else {
      RCLCPP_INFO(get_logger(), "Red cube: FAILURE (code %d)", static_cast<int>(red_state_));
    }
    if (blue_state_ == PickPlaceState::SUCCESS) {
      RCLCPP_INFO(get_logger(), "blue cube: SUCCESS");
    } else {
      RCLCPP_INFO(get_logger(), "blue cube: FAILURE (code %d)", static_cast<int>(blue_state_));
    }
  }

  /* ===================== PICK & PLACE ===================== */

  /**
   * Executes a full pick & place sequence:
   *  - Approach object
   *  - Grasp
   *  - Attach collision object
   *  - Move to place pose
   *  - Release and detach
   */

  bool pick_and_place_(
      moveit::planning_interface::MoveGroupInterface & move_group,
      const geometry_msgs::msg::PoseStamped & pick_pose,
      const geometry_msgs::msg::PoseStamped & place_pose,
      const std::string & object_id)
  {

    auto print_pose = [](const std::string & label, const geometry_msgs::msg::PoseStamped & p) {
        RCLCPP_INFO(rclcpp::get_logger("pick_and_place"), "%s: x=%.3f y=%.3f z=%.3f",
                    label.c_str(),
                    p.pose.position.x,
                    p.pose.position.y,
                    p.pose.position.z);
    };

    // Pre-pick (above the object)
    geometry_msgs::msg::PoseStamped pre_pick = pick_pose; // Tag pose height
    pre_pick.pose.position.z += GRIPPER_LENGTH_ + APPROACH_OFFSET_;   

    RCLCPP_INFO(get_logger(), "[START] pick_and_place for '%s'", object_id.c_str());
    RCLCPP_INFO(get_logger(), "[STEP] Pre-pick above object");
    print_pose("Pre-pick target", pre_pick);
    if (!move_without_constraints(move_group, pre_pick)) {
        RCLCPP_ERROR(get_logger(), "Failed to move to pre-pick");
        ERROR_STATE_ = PickPlaceState::PRE_PICK_FAILURE;
        return false;
    }

    // Open gripper
    RCLCPP_INFO(get_logger(), "[STEP] Open gripper");
    open_gripper();

    // Descend to grasp height
    geometry_msgs::msg::PoseStamped pick = pick_pose;
    pick.pose.position.z += GRIPPER_LENGTH_ - GRASP_DEPTH_;   
    RCLCPP_INFO(get_logger(), "[STEP] Descend to pick");
    print_pose("Pick target", pick);

    if (!move_linear(move_group, pick)) {
      move_group.setMaxVelocityScalingFactor(0.1);   // Try to reduce velocity    
      move_group.setMaxAccelerationScalingFactor(0.1);
      if (!move_with_constraints(move_group, pick)) {
        RCLCPP_ERROR(get_logger(), "Failed to move to pick pose");
        ERROR_STATE_ = PickPlaceState::DESCEND_FAILURE;
        return false;
      }
    }

    // Close gripper
    //rclcpp::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(get_logger(), "[STEP] Close gripper");
    close_gripper();
    //rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Attach object
    RCLCPP_INFO(get_logger(), "[STEP] Attach object '%s'", object_id.c_str());
    attach_object(object_id);

    // Lift object
    RCLCPP_INFO(get_logger(), "[STEP] Lift object back to pre-pick");
    if (!move_linear(move_group, pre_pick)) {
      move_group.setMaxVelocityScalingFactor(0.1);    
      move_group.setMaxAccelerationScalingFactor(0.1);
      if (!move_with_constraints(move_group, pre_pick)) {
        RCLCPP_ERROR(get_logger(), "Failed to lift object");
        ERROR_STATE_ = PickPlaceState::LIFT_FAILURE;
        return false;    
      }
    }

    // Pre-place (above target)
    geometry_msgs::msg::PoseStamped pre_place = place_pose;
    pre_place.pose.position.z += GRIPPER_LENGTH_ + PLACE_OFFSET_;
    RCLCPP_INFO(get_logger(), "[STEP] Move to pre-place above target");
    print_pose("Pre-place target", pre_place);
    if (!move_without_constraints(move_group, pre_place)) {
      RCLCPP_ERROR(get_logger(), "Failed to move to pre-place");
      ERROR_STATE_ = PickPlaceState::PRE_PLACE_FAILURE;
      return false;
    }

    // Descend to place
    geometry_msgs::msg::PoseStamped place = place_pose;   // Tag pose height
    place.pose.position.z += GRIPPER_LENGTH_ - GRASP_DEPTH_;
    RCLCPP_INFO(get_logger(), "[STEP] Descend to place");
    print_pose("Place target", place);
    if (!move_linear(move_group, place)) {
      move_group.setMaxVelocityScalingFactor(0.1);    
      move_group.setMaxAccelerationScalingFactor(0.1);
      if (!move_with_constraints(move_group, place)) {
        RCLCPP_ERROR(get_logger(), "Failed to descend to place pose");
        ERROR_STATE_ = PickPlaceState::PLACE_DESCEND_FAILURE;
        return false;
      }
    }

    // Open gripper to release
    //rclcpp::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(get_logger(), "[STEP] Release object");
    open_gripper();
    //rclcpp::sleep_for(std::chrono::milliseconds(500));

    // Detach object
    RCLCPP_INFO(get_logger(), "[STEP] Detach object '%s'", object_id.c_str());
    detach_object(object_id);

    // Lift
    RCLCPP_INFO(get_logger(), "[STEP] Lift to pre-place");
    if (!move_linear(move_group, pre_place)) {
      move_group.setMaxVelocityScalingFactor(0.1);    
      move_group.setMaxAccelerationScalingFactor(0.1);
      if (!move_with_constraints(move_group, pre_place)) {
        RCLCPP_ERROR(get_logger(), "Failed to lift after placing");
        ERROR_STATE_ = PickPlaceState::LIFT_FAILURE;
        return false;
      }
    }

    RCLCPP_INFO(get_logger(), "[DONE] pick_and_place completed for '%s'", object_id.c_str());
    return true;
  }

  /* ===================== MOTION PLANNING ===================== */

  /**
   * Plans and executes a Cartesian pose goal
   * without additional constraints.
   */
  bool move_without_constraints(
    moveit::planning_interface::MoveGroupInterface & move_group,
    geometry_msgs::msg::PoseStamped target_pose)
  {
    move_group.clearPoseTargets();
    moveit::planning_interface::MoveGroupInterface::Plan plan;

    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp    = now();

    // Publish pose for RViz debugging
    debug_poses.header = target_pose.header;
    debug_poses.poses.clear();
    debug_poses.poses.push_back(target_pose.pose);
    debug_target_poses_pub_->publish(debug_poses);

    move_group.setPoseTarget(target_pose);

    if (move_group.plan(plan) ==
        moveit::core::MoveItErrorCode::SUCCESS)
    {
      move_group.execute(plan);
      return true;
    }

    return false;
  }

  bool move_with_constraints(
    moveit::planning_interface::MoveGroupInterface & move_group,
    geometry_msgs::msg::PoseStamped target_pose)
  {
    move_group.clearPoseTargets();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp    = now();

    // Publish pose for RViz debugging
    debug_poses.header = target_pose.header;
    debug_poses.poses.clear();
    debug_poses.poses.push_back(target_pose.pose);
    debug_target_poses_pub_->publish(debug_poses);

    geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();

    // Set path constraints
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "tool0";
    ocm.header.frame_id = "base_link";
    ocm.orientation = current_pose.pose.orientation;
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 3.14;   // Effectively no constraint on z axis
    ocm.weight = 1.0; 
      
    moveit_msgs::msg::Constraints constraints;
    constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(constraints);    

    move_group.setPlanningTime(10.0);  
      
    move_group.setPoseTarget(target_pose);
      
    if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
      move_group.execute(my_plan);
      move_group.clearPathConstraints(); // Clear path constraints after use
      move_group.setPlanningTime(10.0);  // Initial planning time
      return true;
    }
    
    move_group.clearPathConstraints();  // Clear path constraints after use
    move_group.setPlanningTime(10.0);   // Initial planning time
    return false;
  }


  bool move_linear(
  moveit::planning_interface::MoveGroupInterface & move_group,
  geometry_msgs::msg::PoseStamped target_pose)
  {
    move_group.clearPoseTargets();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group.setPlanningPipelineId("pilz_industrial_motion_planner");
    move_group.setPlannerId("LIN"); 
    
    target_pose.header.frame_id = "base_link";
    target_pose.header.stamp    = now();

    // RViz debug
    debug_poses.header = target_pose.header;
    debug_poses.poses.clear();
    debug_poses.poses.push_back(target_pose.pose);
    debug_target_poses_pub_->publish(debug_poses);

    move_group.setPoseTarget(target_pose);
    
    // In the vertical displacement, reduce speed for safety (to avoid hitting the cube)
    move_group.setMaxVelocityScalingFactor(0.2); 
    move_group.setMaxAccelerationScalingFactor(0.1);

    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success) {
      move_group.execute(my_plan);
    } else {
      RCLCPP_ERROR(get_logger(), "PILZ LIN planning failed (Goal unreachable via straight line?)");
    }

    // Restore OMPL planner
    move_group.setPlanningPipelineId("ompl");
    move_group.setPlannerId("RRTConnectkConfigDefault"); 
    
    // Restore standard velocity and acceleration
    move_group.setMaxVelocityScalingFactor(0.3);
    move_group.setMaxAccelerationScalingFactor(0.3);

    return success;
  }


  /* ===================== JOINTS MOTION ===================== */

  /**
   * Moves the robot to a predefined joint configuration.
   */
  bool go_home(moveit::planning_interface::MoveGroupInterface & move_group)
  {
    return move_joints(move_group, home_joints_);
  }

  bool go_ready(moveit::planning_interface::MoveGroupInterface & move_group)
  {
    return move_joints(move_group, ready_joints_);
  }

  bool move_joints(moveit::planning_interface::MoveGroupInterface & move_group, std::vector<double> joints_vector)
  {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    move_group.setJointValueTarget(joints_vector);

    if (move_group.plan(plan) ==
        moveit::core::MoveItErrorCode::SUCCESS)
    {
      move_group.execute(plan);
      return true;
    }

    return false;
  }



  /* ===================== PICK & PLACE SETUP ===================== */

  /**
   * Defines pick and place poses and sets
   * the gripper orientation pointing down.
   */
  void define_pick_and_place_poses_and_orientations_(bool goal_update = true)
  {
    // Pick poses correspond to tag poses
    red_pick_pose_ = red_tag_pose_;
    blue_pick_pose_ = blue_tag_pose_;

    // Swap targets for placing
    if(goal_update == true){
    red_goal_pose_ = blue_pick_pose_;
    red_goal_pose_.pose.position.x -= 0.12;
    red_goal_pose_.pose.position.y -= 0.12;
    blue_goal_pose_ = red_pick_pose_;
    blue_goal_pose_.pose.position.x -= 0.10;
    blue_goal_pose_.pose.position.y += 0.10;
    }

    // Gripper pointing downward
    tf2::Quaternion q;
    q.setRPY(M_PI, 0.0, 0.0);
    q.normalize();

    geometry_msgs::msg::Quaternion orientation;
    orientation.x = q.x();
    orientation.y = q.y();
    orientation.z = q.z();
    orientation.w = q.w();

    red_pick_pose_.pose.orientation = orientation;
    blue_pick_pose_.pose.orientation = orientation;
    red_goal_pose_.pose.orientation = orientation;
    blue_goal_pose_.pose.orientation = orientation;
  }

  /* ===================== GRIPPER CONTROL ===================== */

  /**
   * Sends a generic gripper command via action.
   */
  bool handle_gripper(double position, double effort)
  {
    if (!gripper_action_client_->wait_for_action_server(std::chrono::seconds(2))) {
      return false;
    }

    GripperCommand::Goal goal;
    goal.command.position   = position;
    goal.command.max_effort = effort;

    auto goal_handle_future = gripper_action_client_->async_send_goal(goal);
    if (goal_handle_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
      return false;

    auto goal_handle = goal_handle_future.get();
    if (!goal_handle) return false;

    auto result_future = gripper_action_client_->async_get_result(goal_handle);
    if (result_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready)
      return false;

    return result_future.get().code == rclcpp_action::ResultCode::SUCCEEDED;
  }

  void open_gripper()  { handle_gripper(0.0, 40.0); }
  void close_gripper() { handle_gripper(0.3, 80.0); }
  void close_gripper_fully() { handle_gripper(0.8, 40.0); }

  /* ===================== COLLISION HANDLING ===================== */

  // Attach a collision object to the robot end-effector
  void attach_object(const std::string & object_id)
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.link_name = "tool0";
    aco.object.id = object_id;
    aco.object.operation = moveit_msgs::msg::CollisionObject::ADD;
    aco.touch_links = {
      "tool0",
      "robotiq_85_left_finger_tip_link",
      "robotiq_85_right_finger_tip_link",
      "robotiq_85_left_inner_knuckle_link",
      "robotiq_85_right_inner_knuckle_link"
    };
    planning_scene_interface_.applyAttachedCollisionObject(aco);
  }

  // Detach a collision object from the robot
  void detach_object(const std::string & object_id)
  {
    moveit_msgs::msg::AttachedCollisionObject aco;
    aco.object.id = object_id;
    aco.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
    planning_scene_interface_.applyAttachedCollisionObject(aco);
  }

  // Add a box-shaped collision object to the scene
  void add_collision_object(
    const geometry_msgs::msg::PoseStamped & pose,
    const std::string & object_id)
  {
    moveit_msgs::msg::CollisionObject obj;
    obj.header.frame_id = "base_link";
    obj.id = object_id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {0.06, 0.06, 0.1};

    geometry_msgs::msg::Pose box_pose = pose.pose;
    box_pose.position.z -= 0.05; // center of the cube

    obj.primitives.push_back(primitive);
    obj.primitive_poses.push_back(box_pose);
    obj.operation = obj.ADD;

    planning_scene_interface_.applyCollisionObject(obj);
  }

  bool handle_pick_place_failure(moveit::planning_interface::MoveGroupInterface & move_group, 
                               std::string object_id)
  {
    RCLCPP_INFO(get_logger(), "Handling pick & place failure ...");

    // NONE, PRE_PICK_FAILURE, DESCEND_FAILURE, LIFT_FAILURE, PRE_PLACE_FAILURE, 
    // PLACE_DESCEND_FAILURE, LOST, MOVED, SUCCESS

    bool can_continue = true; // Default: the simulation can go on
  
    switch (ERROR_STATE_)
    {    
      case PickPlaceState::PRE_PICK_FAILURE:
      case PickPlaceState::DESCEND_FAILURE:
      // Failed to reach pre-pick pose, nothing to do
      go_ready(move_group);
      break;

      case PickPlaceState::LIFT_FAILURE:
      // Failed to lift with the object (CRITICAL), need to release it safely
      {      
        detach_object(object_id);
        open_gripper();
          
        // Move vertically up to avoid collisions
        geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
        current_pose.pose.position.z += 0.10;

        // Slow down for safety
        move_group.setMaxVelocityScalingFactor(0.1);    
        move_group.setMaxAccelerationScalingFactor(0.1);

        move_with_constraints(move_group, current_pose); 

        // Restore normal speed
        move_group.setMaxVelocityScalingFactor(0.3);    
        move_group.setMaxAccelerationScalingFactor(0.3);
      }
      break;

      case PickPlaceState::PRE_PLACE_FAILURE:
      // Railed to reach pre-place pose, i should pose the object back to the pick position, 
      // then release it safely
      {
        geometry_msgs::msg::PoseStamped return_place_pose;
        if (object_id == "red_cube") {
          return_place_pose = red_pick_pose_;
        } else {
          return_place_pose = blue_pick_pose_;
        }
        return_place_pose.pose.position.z += GRIPPER_LENGTH_ - GRASP_DEPTH_;

        // Slow down for safety
        move_group.setMaxVelocityScalingFactor(0.1);    
        move_group.setMaxAccelerationScalingFactor(0.1);

        move_without_constraints(move_group, return_place_pose);
        
        detach_object(object_id);
        open_gripper();
          
        // Move vertically up to avoid collisions
        geometry_msgs::msg::PoseStamped current_pose = move_group.getCurrentPose();
        current_pose.pose.position.z += 0.10;

        // Slow down for safety
        move_group.setMaxVelocityScalingFactor(0.1);    
        move_group.setMaxAccelerationScalingFactor(0.1);

        move_with_constraints(move_group, current_pose); 

        // Restore normal speed
        move_group.setMaxVelocityScalingFactor(0.3);    
        move_group.setMaxAccelerationScalingFactor(0.3);
      }
      break;

      case PickPlaceState::PLACE_DESCEND_FAILURE:
      // Failed to descend to place, i should release the object at pre-place height
      {
        // To complicated... it is request the user intervention
        RCLCPP_WARN(get_logger(), "PLACE_DESCEND_FAILURE: manual intervention required to safely release the object.");

        can_continue = false;   // Simulation cannot continue
      }
      break;

      default: // LOST, MOVED, SUCCESS are used only for state reporting, no action needed
      break;
    }

    ERROR_STATE_ = PickPlaceState::NONE; // Reset error state after handling

    return can_continue;
  }

  /* ===================== FINAL CHECK ===================== */

  void final_check() {

    tag_sample_count_ = 0;
    red_tag_received_ = false;
    blue_tag_received_ = false;
    targets_received_ = false;

    while (rclcpp::ok() && !targets_received_) {
      std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }

    //Check that both tags were received
    if (!red_tag_received_) {
        RCLCPP_INFO(get_logger(), "Red tag not detected during final check.");
        red_state_ = PickPlaceState::LOST;  
    }

    if (!blue_tag_received_) {
        RCLCPP_INFO(get_logger(), "blue tag not detected during final check.");
        blue_state_ = PickPlaceState::LOST;  
    }

    // Define tolerance for position check
    const double TOLERANCE = 0.05;

    // Red cube check
    if(red_tag_received_){

      double red_dist = std::sqrt(
          std::pow(red_tag_pose_.pose.position.x - red_goal_pose_.pose.position.x, 2) +
          std::pow(red_tag_pose_.pose.position.y - red_goal_pose_.pose.position.y, 2) +
          std::pow(red_tag_pose_.pose.position.z - red_goal_pose_.pose.position.z, 2)
      );

      if (red_dist <= TOLERANCE) {
          RCLCPP_INFO(rclcpp::get_logger("final_check"), "RED Cube: OK - Position reached (Error: %.3f m)", red_dist);
      } else {
          RCLCPP_INFO(rclcpp::get_logger("final_check"), "RED Cube: FAIL - Out of position! (Error: %.3f m)", red_dist);
          red_state_ = PickPlaceState::MOVED;
      }

    } 

    // Blue cube check
    if(blue_tag_received_){
      double blue_dist = std::sqrt(
          std::pow(blue_tag_pose_.pose.position.x - blue_goal_pose_.pose.position.x, 2) +
          std::pow(blue_tag_pose_.pose.position.y - blue_goal_pose_.pose.position.y, 2) +
          std::pow(blue_tag_pose_.pose.position.z - blue_goal_pose_.pose.position.z, 2)
      );

      if (blue_dist <= TOLERANCE) {
          RCLCPP_INFO(rclcpp::get_logger("final_check"), "blue Cube: OK - Position reached (Error: %.3f m)", blue_dist);
      } else {
          RCLCPP_INFO(rclcpp::get_logger("final_check"), "blue Cube: FAIL - Out of position! (Error: %.3f m)", blue_dist);
          blue_state_ = PickPlaceState::MOVED;
      }
    }
  }
};



/* ===================== MAIN ===================== */

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  options.automatically_declare_parameters_from_overrides(true);

  // Create ROS2 node
  auto node = std::make_shared<ManagerNode>(options);

  // MoveIt interface for the robot arm
  moveit::planning_interface::MoveGroupInterface move_group(node, "ir_arm");

  // Spin ROS callbacks in a separate thread
  std::thread spin_thread([&node]() {
    rclcpp::spin(node);
  });

  // Run pick & place logic
  node->run(move_group);

  spin_thread.join();
  rclcpp::shutdown();
  return 0;
}