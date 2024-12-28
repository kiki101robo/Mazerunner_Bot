#pragma once
#include <rclcpp/rclcpp.hpp>
#include "rclcpp_action/rclcpp_action.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "nav2_msgs/action/navigate_through_poses.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <geometry_msgs/msg/pose.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <cmath>
#include <tf2_ros/static_transform_broadcaster.h>
#include <utils.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "mage_msgs/msg/advanced_logical_camera_image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "mage_msgs/msg/sensors.hpp"
#include "rosgraph_msgs/msg/clock.hpp"
#include <cmath>
#include "action_msgs/msg/goal_status.hpp"
// for static broadcaster
#include "tf2_ros/static_transform_broadcaster.h"
// for dynamic broadcaster
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

using namespace std::chrono_literals;
/**
 * @class MazeRunner
 * @brief A class that represents a broadcaster demo.
 * 
 * This class inherits from rclcpp::Node and provides functionality for broadcasting transforms and handling subscriptions.
 */

class MazeRunner : public rclcpp::Node
{
public:
    using NavigateToPose = nav2_msgs::action::NavigateToPose;
    using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<NavigateToPose>;
  
    /**
     * @brief Constructor for MazeRunner.
     * @param node_name The name of the node.
     */
    void load_parameters(const std::string& parameter_name) {};

    MazeRunner(std::string node_name) : Node(node_name)
    {
        // parameter to decide whether to execute the broadcaster or not
        RCLCPP_INFO(this->get_logger(), "Maze Runner node started");

        std::vector<std::string> names = {"wp1", "wp2", "wp3", "wp4", "wp5"};

        this->declare_parameter("aruco_0.wp1.color", "green");
        this->declare_parameter("aruco_0.wp2.color", "red");
        this->declare_parameter("aruco_0.wp3.color", "orange");
        this->declare_parameter("aruco_0.wp4.color", "purple");
        this->declare_parameter("aruco_0.wp5.color", "blue");
        this->declare_parameter("aruco_1.wp1.color", "blue");
        this->declare_parameter("aruco_1.wp2.color","green");
        this->declare_parameter("aruco_1.wp3.color","orange");
        this->declare_parameter("aruco_1.wp4.color","red");
        this->declare_parameter("aruco_1.wp5.color","purple");
        this->declare_parameter("aruco_0.wp1.type","battery");
        this->declare_parameter("aruco_0.wp2.type","battery");
        this->declare_parameter("aruco_0.wp3.type","battery");
        this->declare_parameter("aruco_0.wp4.type","battery");
        this->declare_parameter("aruco_0.wp5.type","battery");
        this->declare_parameter("aruco_1.wp1.type","battery");
        this->declare_parameter("aruco_1.wp2.type","battery");
        this->declare_parameter("aruco_1.wp3.type","battery");
        this->declare_parameter("aruco_1.wp4.type","battery");
        this->declare_parameter("aruco_1.wp5.type","battery");

        a0_wp1_color = this->get_parameter("aruco_0.wp1.color").as_string();
        a0_wp2_color = this->get_parameter("aruco_0.wp2.color").as_string();
        a0_wp3_color = this->get_parameter("aruco_0.wp3.color").as_string();
        a0_wp4_color = this->get_parameter("aruco_0.wp4.color").as_string();
        a0_wp5_color = this->get_parameter("aruco_0.wp5.color").as_string();
        a1_wp1_color = this->get_parameter("aruco_1.wp1.color").as_string();
        a1_wp2_color = this->get_parameter("aruco_1.wp2.color").as_string();
        a1_wp3_color = this->get_parameter("aruco_1.wp3.color").as_string();
        a1_wp4_color = this->get_parameter("aruco_1.wp4.color").as_string();
        a1_wp5_color = this->get_parameter("aruco_1.wp5.color").as_string();
        a0_wp1_type = this->get_parameter("aruco_0.wp1.type").as_string();
        a0_wp2_type = this->get_parameter("aruco_0.wp2.type").as_string();
        a0_wp3_type = this->get_parameter("aruco_0.wp3.type").as_string();
        a0_wp4_type = this->get_parameter("aruco_0.wp4.type").as_string();
        a0_wp5_type = this->get_parameter("aruco_0.wp5.type").as_string();
        a1_wp1_type = this->get_parameter("aruco_1.wp1.type").as_string();
        a1_wp2_type = this->get_parameter("aruco_1.wp2.type").as_string();
        a1_wp3_type = this->get_parameter("aruco_1.wp3.type").as_string();
        a1_wp4_type = this->get_parameter("aruco_1.wp4.type").as_string();
        a1_wp5_type = this->get_parameter("aruco_1.wp5.type").as_string();

        RCLCPP_INFO(this->get_logger(), "Parameters loaded");
        // RCLCPP_INFO(this->get_logger(), "aruco_0.wp1.color: %s", a0_wp1_color.c_str());

    //     for (const std::string& name : names){
    //         this->declare_parameter("aruco_0." + name + ".type");
    //         this->declare_parameter("aruco_0." + name + ".color");
    //         std::string aruco_0_type = this->get_parameter("aruco_0." + name + ".type").as_string();
    //         std::string aruco_0_color = this->get_parameter("aruco_0." + name + ".color").as_string();
            
    //         this->declare_parameter("aruco_1." + name + ".type");
    //         this->declare_parameter("aruco_1." + name + ".color");
    //         std::string aruco_1_type = this->get_parameter("aruco_1." + name + ".type").as_string();
    //         std::string aruco_1_color = this->get_parameter("aruco_1." + name + ".color").as_string();
    // }

        // initialize the transform broadcaster
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        // Load a buffer of transforms
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        //tf_buffer_->setUsingDedicatedThread(true);
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        // Create a utils object to use the utility functions
        utils_ptr_ = std::make_shared<Utils>();
        // // Create a publisher for the Aruco marker
        // rclcpp::Publisher<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_publisher_;
        // aruco_publisher_ = this->create_publisher<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 10);
        rclcpp::QoS qos(10);     qos.reliability(RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT);
        // logical_subscription_= this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/advanced_logical_camera/image", 
        // qos, std::bind(&MazeRunner::battery_pos, this, std::placeholders::_1));

        camera1_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera1/image",
        qos,std::bind(&MazeRunner::battery1_pos, this, std::placeholders::_1));

        camera2_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera2/image",
        qos,std::bind(&MazeRunner::battery2_pos, this, std::placeholders::_1));

        camera3_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera3/image",
        qos,std::bind(&MazeRunner::battery3_pos, this, std::placeholders::_1));

        camera4_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera4/image",
        qos,std::bind(&MazeRunner::battery4_pos, this, std::placeholders::_1));

        camera5_subscription_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>("/mage/camera5/image",
        qos,std::bind(&MazeRunner::battery5_pos, this, std::placeholders::_1));

        client_ =
        rclcpp_action::create_client<NavigateToPose>(this, "navigate_to_pose");
        // initialize the publisher
        initial_pose_pub_ =
            this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "initialpose", 10);

        // set the initial pose for navigation
        // set_initial_pose();
        // pause for 5 seconds
        // std::this_thread::sleep_for(std::chrono::seconds(5));
        // send the goal
        // send_goal();

        aruco_subscription_= this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>("aruco_markers", 
        10, std::bind(&MazeRunner::aruco_pos, this, std::placeholders::_1));

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&MazeRunner::odom_sub, this, std::placeholders::_1));

        goal_timer_ = this->create_wall_timer(std::chrono::milliseconds(1000), std::bind(&MazeRunner::goalpush_timer, this));
    }

private:

    rclcpp::TimerBase::SharedPtr goal_timer_;

    void goalpush_timer();
    // /*!< Boolean parameter to whether or not start the broadcaster */
    // bool param_broadcast_;
    /*!< Buffer that stores several seconds of transforms for easy lookup by the listener. */
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    /*!< Transform listener object */
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    /*!< Broadcaster object */
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    //!< Utils object to access utility functions/
    std::shared_ptr<Utils> utils_ptr_;
    //!< Wall timer object for the broadcaster/
    rclcpp::TimerBase::SharedPtr broadcast_timer_;
    //!< Wall timer object for the static broadcaster/
    rclcpp::TimerBase::SharedPtr static_broadcast_timer_;

    //flags and intermediate variables
    double marker_id;

    std::string wp1_color;
    std::string wp2_color;
    std::string wp3_color;
    std::string wp4_color;
    std::string wp5_color;


    // Parameter attributes for a0
    std::string a0_wp1_type;
    std::string a0_wp2_type;
    std::string a0_wp3_type;
    std::string a0_wp4_type;
    std::string a0_wp5_type;
    std::string a0_wp1_color;
    std::string a0_wp2_color;
    std::string a0_wp3_color;
    std::string a0_wp4_color;
    std::string a0_wp5_color;

    // Parameter attributes for a1
    std::string a1_wp1_type;
    std::string a1_wp2_type;
    std::string a1_wp3_type;
    std::string a1_wp4_type;
    std::string a1_wp5_type;
    std::string a1_wp1_color;
    std::string a1_wp2_color;
    std::string a1_wp3_color;
    std::string a1_wp4_color;
    std::string a1_wp5_color;

    //Vectors to store battery location and type
    std::vector<int> battery_color_;   

    std::vector<float> battery_type_; 

    std::vector<float> battery_x_pos_;

    std::vector<float> battery_y_pos_;

    std::vector<std::string> wp_aruco0;

    std::vector<std::string> wp_aruco1; 

    std::vector<std::string> wp_list; 

    std::vector<int> aruco0_list;

    std::vector<int> aruco1_list;    

    int camera1_got_data = 0;

    int camera2_got_data = 0;

    int camera3_got_data = 0;

    int camera4_got_data = 0;

    int camera5_got_data = 0;

    int turtlebot_init_pos_set = 0;

    int wp = 0;

    int got_aruco = 0;

    int got_all_parts = 0;

    int goal_reach = 1;

    int curr_goal_pos = 0;
    
    int goal_points_reached = 0;

    nav_msgs::msg::Odometry turtlebot_initial_pose_;
    
    rclcpp::TimerBase::SharedPtr velocity_timer_;

    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;
    
    /*!< Subscriber to the camera1 */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera1_subscription_;
    /**
     * @brief Callback function for the camera1
     *
     * @param msg
     */
    void battery1_pos(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /*!< Subscriber to the camera2 topic */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera2_subscription_;
    /**
     * @brief Callback function for the camera2 topic
     *
     * @param msg
     */
    void battery2_pos(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    /*!< Subscriber to the camear3 topic */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera3_subscription_;
    /**
     * @brief Callback function for the camera3 topic
     *
     * @param msg
     */
    void battery3_pos(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    /*!< Subscriber to the camera4 topic */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera4_subscription_;
    /**
     * @brief Callback function for the camera4 topic
     *
     * @param msg
     */
    void battery4_pos(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    /*!< Subscriber to the camera5 topic */
    rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr camera5_subscription_;
    /**
     * @brief Callback function for the camera5 topic
     *
     * @param msg
     */
    void battery5_pos(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);
    /**
     * @brief Method to broadcast the transform of the turtlebot
     *
     */
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr aruco_subscription_;
    /**
     * @brief Callback function for the aruco_markers topic
     *
     * @param msg
     */
    void aruco_pos(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg); 

    /*!< Subscriber to the camera5 topic */
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;
    /**
     * @brief Callback function for the camera5 topic
     *
     * @param msg
     */
    void odom_sub(const nav_msgs::msg::Odometry::SharedPtr msg);

    // void move_robot();
    /**
     * @brief Method to broadcast the transform of the turtlebot
     *
     */
    void print_data_vector();

    /**
     * @brief Publisher to the topic /initialpose
     *
     */
    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
        initial_pose_pub_;
    /**
     * @brief Action client for the action server navigate_to_pose
     *
     */
    rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
    //   rclcpp::TimerBase::SharedPtr timer_;
    /**
     * @brief Response from the server after sending the goal
     */
    void goal_response_callback(
        std::shared_future<GoalHandleNavigation::SharedPtr> future);
    /**
     * @brief Feedback received while the robot is driving towards the goal
     *
     * @param feedback
     */
    void feedback_callback(
        GoalHandleNavigation::SharedPtr,
        const std::shared_ptr<const NavigateToPose::Feedback> feedback);
    /**
     * @brief Result after the action has completed
     *
     * @param result
     */
    void result_callback(const GoalHandleNavigation::WrappedResult& result);
    /**
     * @brief Method to build and send a goal using the client
     *
     */
    void send_goal();
    /**
     * @brief Method to set the initial pose for the robot
     *
     */
    void set_initial_pose();
};