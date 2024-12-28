#include <maze_runner.hpp>
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <utils.hpp>
#include <deque>
#include <sstream>
// needed for the listener
#include <tf2/exceptions.h>
//including the topic aruco_marker
using namespace std::chrono_literals;

void MazeRunner::goalpush_timer(){
    if (got_all_parts == 1)
    {
        if(goal_reach == 1){
            for (int i = 0; i < static_cast<int>(battery_color_.size()); i++) {
                std::string color ;
                    if(battery_color_[i] == 0){color = "red";}
                else if(battery_color_[i] == 1){color = "green";}
                else if(battery_color_[i] == 2){color = "blue";}
                else if(battery_color_[i] == 3){color = "orange";}
                else if(battery_color_[i] == 4){color = "purple";}
                if(wp_list[curr_goal_pos] == color){
                wp = i;
                }
            }
            RCLCPP_INFO_STREAM(this->get_logger(), "WayPoint " << curr_goal_pos << " reached");
            std::this_thread::sleep_for(std::chrono::seconds(5));
            goal_reach = 0;
            send_goal();
            curr_goal_pos += 1;
            RCLCPP_INFO_STREAM(this->get_logger(), "Going to WayPoint " << curr_goal_pos);
            if(curr_goal_pos == 6){
                RCLCPP_INFO_STREAM(this->get_logger(), "All waypoints reached!");
                goal_timer_->cancel();
                rclcpp::shutdown();
            goal_points_reached += 1;
        }
    }
    }
}

void MazeRunner::aruco_pos(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg) {
    if (!msg->poses.empty()) {
        // RCLCPP_WARN(this->get_logger(), "Waypoints created");
        // Assuming poses is a vector of poses, access the first pose in the vector
        const auto& marker_id_ = msg->marker_ids[0];
        marker_id = marker_id_;
        RCLCPP_INFO_STREAM(this->get_logger(), "Aruco ID: " << marker_id);
        std::string aruco_type;
        if(marker_id == 0){
            // aruco_type = "aruco_0.";wp1_color = "green";  wp2_color = "red";wp3_color = "orange";wp4_color = "purple";wp5_color = "blue";
            // wp_aruco0 = {"green", "red", "orange", "purple", "blue"};
            // wp_list = {"green", "red", "orange", "purple", "blue"};
            wp_list = {a0_wp1_color, a0_wp2_color, a0_wp3_color, a0_wp4_color, a0_wp5_color};
            // aruco0_list = {1,0,3,4,2};
        }
        else if(marker_id == 1){
            // aruco_type = "aruco_1.";wp1_color = "blue";  wp2_color = "green";wp3_color = "orange"; wp4_color = "red"; wp5_color = "purple";
            // wp_aruco1 = {"blue", "green", "orange", "red", "purple"};
            // wp_list = {"blue", "green", "orange", "red", "purple"};
            wp_list = {a1_wp1_color, a1_wp2_color, a1_wp3_color, a1_wp4_color, a1_wp5_color};
            // aruco1_list = {2,1,3,0,4};
        }
        got_aruco =1;
        aruco_subscription_.reset();
        got_all_parts = 1;
        print_data_vector();
        // send_goal();
    } else {
        RCLCPP_WARN(this->get_logger(), "ArucoMarkers message is empty");
    }
}

void MazeRunner::print_data_vector(){
    std::string color ;
    std::cout << "Data Vector: [ " << std::endl;
    for (int i = 0; i < static_cast<int>(battery_color_.size()); i++) {
             if(battery_color_[i] == 0){color = "red";}
        else if(battery_color_[i] == 1){color = "green";}
        else if(battery_color_[i] == 2){color = "blue";}
        else if(battery_color_[i] == 3){color = "orange";}
        else if(battery_color_[i] == 4){color = "purple";}
        std::cout << color << " battery detected at X,Y = ["
                << battery_x_pos_[i] << ", " << battery_y_pos_[i] << "]" << std::endl;
    }
}

void MazeRunner::battery1_pos(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if(camera2_got_data == 0){
    RCLCPP_INFO_STREAM(this->get_logger(), "Entered Battery1Pos()");
        if (!msg->part_poses.empty()) {
        const auto& battery_positon = msg->part_poses[0];

        geometry_msgs::msg::PoseStamped target_pose;
        geometry_msgs::msg::PoseStamped current_pose;
        geometry_msgs::msg::TransformStamped map_pose;

        target_pose.header.frame_id = "camera1_frame";
        target_pose.pose = msg->part_poses[0].pose;

        try{
            map_pose = tf_buffer_->lookupTransform("map", target_pose.header.frame_id, this->get_clock()->now());
            tf2::doTransform(target_pose, current_pose, map_pose);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what()); //Print exception which was caught
        }

        bool isPartAdded = false;

        for (int i = 0; i < static_cast<int>(battery_color_.size()); i++) {
            if (battery_color_[i] == battery_positon.part.color) {
                isPartAdded = true;
                break;
            }
        }
        if (!isPartAdded) {
            battery_color_.push_back(msg->part_poses[0].part.color);
            battery_type_.push_back(msg->part_poses[0].part.type);
            battery_x_pos_.push_back(current_pose.pose.position.x);
            battery_y_pos_.push_back(current_pose.pose.position.y);
            camera1_subscription_.reset();
            camera1_got_data = 1;
        }

    } else {
        //RCLCPP_WARN(this->get_logger(), "BatteryPos message is empty or part_poses is empty");
    }
    }
}

void MazeRunner::battery2_pos(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if(camera2_got_data == 0){
    RCLCPP_INFO_STREAM(this->get_logger(), "Entered Battery2Pos()");
        if (!msg->part_poses.empty()) {
        const auto& battery_positon = msg->part_poses[0];

        geometry_msgs::msg::PoseStamped target_pose;
        geometry_msgs::msg::PoseStamped current_pose;
        geometry_msgs::msg::TransformStamped map_pose;

        target_pose.header.frame_id = "camera2_frame";
        target_pose.pose = msg->part_poses[0].pose;

        try{
            map_pose = tf_buffer_->lookupTransform("map", target_pose.header.frame_id, this->get_clock()->now());
            tf2::doTransform(target_pose, current_pose, map_pose);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what()); //Print exception which was caught
        }
        
        bool isPartAdded = false;

        for (int i = 0; i < static_cast<int>(battery_color_.size()); i++) {
            if (battery_color_[i] == battery_positon.part.color) {
                isPartAdded = true;
                break;
            }
        }
        if (!isPartAdded) {
            battery_color_.push_back(msg->part_poses[0].part.color);
            battery_type_.push_back(msg->part_poses[0].part.type);
            battery_x_pos_.push_back(current_pose.pose.position.x);
            battery_y_pos_.push_back(current_pose.pose.position.y);
            camera2_subscription_.reset();
            camera2_got_data = 1;
        }

    } else {
        //RCLCPP_WARN(this->get_logger(), "BatteryPos message is empty or part_poses is empty");
    }
    }
}

void MazeRunner::battery3_pos(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if(camera3_got_data == 0){
    RCLCPP_INFO_STREAM(this->get_logger(), "Entered Battery3Pos()");

        if (!msg->part_poses.empty()) {
        const auto& battery_positon = msg->part_poses[0];

        geometry_msgs::msg::PoseStamped target_pose;
        geometry_msgs::msg::PoseStamped current_pose;
        geometry_msgs::msg::TransformStamped map_pose;

        target_pose.header.frame_id = "camera3_frame";
        target_pose.pose = msg->part_poses[0].pose;

        try{
            map_pose = tf_buffer_->lookupTransform("map", target_pose.header.frame_id, this->get_clock()->now());
            tf2::doTransform(target_pose, current_pose, map_pose);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what()); //Print exception which was caught
        }

        bool isPartAdded = false;

        for (int i = 0; i < static_cast<int>(battery_color_.size()); i++) {
            if (battery_color_[i] == battery_positon.part.color) {
                isPartAdded = true;
                break;
            }
        }
        if (!isPartAdded) {
            battery_color_.push_back(msg->part_poses[0].part.color);
            battery_type_.push_back(msg->part_poses[0].part.type);
            battery_x_pos_.push_back(current_pose.pose.position.x);
            battery_y_pos_.push_back(current_pose.pose.position.y);
            camera3_subscription_.reset();
            camera3_got_data = 1;
        }

    } else {
        //RCLCPP_WARN(this->get_logger(), "BatteryPos message is empty or part_poses is empty");
    }
    }
}



void MazeRunner::battery4_pos(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if(camera4_got_data == 0){
    RCLCPP_INFO_STREAM(this->get_logger(), "Entered Battery4Pos()");
        if (!msg->part_poses.empty()) {
        const auto& battery_positon = msg->part_poses[0];

        geometry_msgs::msg::PoseStamped target_pose;
        geometry_msgs::msg::PoseStamped current_pose;
        geometry_msgs::msg::TransformStamped map_pose;

        target_pose.header.frame_id = "camera4_frame";
        target_pose.pose = msg->part_poses[0].pose;

        try{
            map_pose = tf_buffer_->lookupTransform("map", target_pose.header.frame_id, this->get_clock()->now());
            tf2::doTransform(target_pose, current_pose, map_pose);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what()); //Print exception which was caught
        }

        bool isPartAdded = false;

        for (int i = 0; i < static_cast<int>(battery_color_.size()); i++) {
            if (battery_color_[i] == battery_positon.part.color) {
                isPartAdded = true;
                break;
            }
        }
        if (!isPartAdded) {
            battery_color_.push_back(msg->part_poses[0].part.color);
            battery_type_.push_back(msg->part_poses[0].part.type);
            battery_x_pos_.push_back(current_pose.pose.position.x);
            battery_y_pos_.push_back(current_pose.pose.position.y);
            camera4_subscription_.reset();
            camera4_got_data = 1;
        }

    } else {
        //RCLCPP_WARN(this->get_logger(), "BatteryPos message is empty or part_poses is empty");
    }
    }
}


void MazeRunner::battery5_pos(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg) {
    if(camera5_got_data == 0){
    RCLCPP_INFO_STREAM(this->get_logger(), "Entered Battery5Pos()");
        if (!msg->part_poses.empty()) {
        const auto& battery_positon = msg->part_poses[0];

        geometry_msgs::msg::PoseStamped target_pose;
        geometry_msgs::msg::PoseStamped current_pose;
        geometry_msgs::msg::TransformStamped map_pose;

        target_pose.header.frame_id = "camera5_frame";
        target_pose.pose = msg->part_poses[0].pose;

        try{
            map_pose = tf_buffer_->lookupTransform("map", target_pose.header.frame_id, this->get_clock()->now());
            tf2::doTransform(target_pose, current_pose, map_pose);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Failure %s\n", ex.what()); //Print exception which was caught
        }

        bool isPartAdded = false;

        for (int i = 0; i < static_cast<int>(battery_color_.size()); i++) {
            if (battery_color_[i] == battery_positon.part.color) {
                isPartAdded = true;
                break;
            }
        }
        if (!isPartAdded) {
            battery_color_.push_back(msg->part_poses[0].part.color);
            battery_type_.push_back(msg->part_poses[0].part.type);
            battery_x_pos_.push_back(current_pose.pose.position.x);
            battery_y_pos_.push_back(current_pose.pose.position.y);
            camera5_subscription_.reset();
            camera5_got_data = 1;
        }

    } else {
        //RCLCPP_WARN(this->get_logger(), "BatteryPos message is empty or part_poses is empty");
    }
    }
}



void MazeRunner::odom_sub(const nav_msgs::msg::Odometry::SharedPtr msg) {
    if(turtlebot_init_pos_set == 0){
    turtlebot_initial_pose_ = *msg;
    turtlebot_init_pos_set = 1;
    MazeRunner::set_initial_pose();
    std::this_thread::sleep_for(std::chrono::seconds(5));
    }

}

//===============================================
void MazeRunner::set_initial_pose() {
  auto message = geometry_msgs::msg::PoseWithCovarianceStamped();
  message.header.frame_id = "map";
  message.pose.pose.position.x = turtlebot_initial_pose_.pose.pose.position.x;
  message.pose.pose.position.y = turtlebot_initial_pose_.pose.pose.position.y;
  message.pose.pose.orientation.x = turtlebot_initial_pose_.pose.pose.orientation.x;
  message.pose.pose.orientation.y = turtlebot_initial_pose_.pose.pose.orientation.y;
  message.pose.pose.orientation.z = turtlebot_initial_pose_.pose.pose.orientation.z;
  message.pose.pose.orientation.w = turtlebot_initial_pose_.pose.pose.orientation.w;
  initial_pose_pub_->publish(message);
}

//===============================================
void MazeRunner::send_goal() {     
  using namespace std::placeholders;
    if (!this->client_->wait_for_action_server()) {
        RCLCPP_ERROR(this->get_logger(),
                    "Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = NavigateToPose::Goal();
    // std::vector<int> waypoints = {1,2,3,4,0};
    goal_msg.pose.header.frame_id = "map";
    goal_msg.pose.pose.position.x = battery_x_pos_[wp];
    goal_msg.pose.pose.position.y = battery_y_pos_[wp];
    goal_msg.pose.pose.position.z = 0.0;
    goal_msg.pose.pose.orientation.x = 0.0;
    goal_msg.pose.pose.orientation.y = 0.0;
    goal_msg.pose.pose.orientation.z = 0.0;
    goal_msg.pose.pose.orientation.w = 1.0;

    RCLCPP_INFO(this->get_logger(), "Sending goal");
    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&MazeRunner::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&MazeRunner::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&MazeRunner::result_callback, this, _1);
    client_->async_send_goal(goal_msg, send_goal_options);

}

//===============================================
void MazeRunner::goal_response_callback(
    std::shared_future<GoalHandleNavigation::SharedPtr> future) {
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    rclcpp::shutdown();
  } else {
    RCLCPP_INFO(this->get_logger(),
                "Goal accepted by server, waiting for result");
  }
}

//===============================================
void MazeRunner::feedback_callback(
    GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback) {
    // RCLCPP_INFO(this->get_logger(), "Robot is driving towards the goal");
}

//===============================================
void MazeRunner::result_callback(
    const GoalHandleNavigation::WrappedResult& result) {
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
    goal_reach = 1;
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
}

//===============================================
int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MazeRunner>("maze_runner");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}