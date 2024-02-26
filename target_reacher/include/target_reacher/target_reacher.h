#pragma once
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <string>
#include "bot_controller/bot_controller.h"
#include "ros2_aruco_interfaces/msg/aruco_markers.hpp"
#include "tf2_ros/static_transform_broadcaster.h"

/**
 * @brief This is the class TargetReacher.
 * This class contains all the methods required to complete the task.
 *
 */
class TargetReacher : public rclcpp::Node
{
public:
    /**
     * @brief Construct a new Target Reacher object
     *
     * @param bot_controller shared pointer to BotController class object
     */
    TargetReacher(std::shared_ptr<BotController> const &bot_controller) : Node("target_reacher")
    {
        m_bot_controller = bot_controller;

        // Declare parameters
        auto aruco_target_x = this->declare_parameter<double>("aruco_target.x");
        auto aruco_target_y = this->declare_parameter<double>("aruco_target.y");

        this->declare_parameter<std::string>("final_destination.frame_id");
        this->declare_parameter<double>("final_destination.aruco_0.x");
        this->declare_parameter<double>("final_destination.aruco_0.y");
        this->declare_parameter<double>("final_destination.aruco_1.x");
        this->declare_parameter<double>("final_destination.aruco_1.y");
        this->declare_parameter<double>("final_destination.aruco_2.x");
        this->declare_parameter<double>("final_destination.aruco_2.y");
        this->declare_parameter<double>("final_destination.aruco_3.x");
        this->declare_parameter<double>("final_destination.aruco_3.y");

        // Set Goal1 for aruco target
        m_bot_controller->set_goal(aruco_target_x, aruco_target_y);

        m_tf_buffer =
            std::make_unique<tf2_ros::Buffer>(this->get_clock());

        // goal_reached subscriber callback
        m_subscriber_goal_reached = this->create_subscription<std_msgs::msg::Bool>(
            "/goal_reached", 10,
            std::bind(&TargetReacher::goal_reached_callback, this, std::placeholders::_1));

        m_rotation_publisher = this->create_publisher<geometry_msgs::msg::Twist>(
            "/robot1/cmd_vel", 10);

        // aruco_markers subscriber callback
        m_subscriber_aruco_markers = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
            "/aruco_markers", 10,
            std::bind(&TargetReacher::aruco_markers_callback, this, std::placeholders::_1));

        // Initialize the static transform broadcaster
        m_tf_broadcaster =
            std::make_unique<tf2_ros::StaticTransformBroadcaster>(this);

        // Initialize the transform listener
        m_tf_listener =
            std::make_unique<tf2_ros::TransformListener>(*m_tf_buffer);
    }

private:
    // attributes
    /**
     * @brief Shared pointer to class BotController
     * 
     */
    std::shared_ptr<BotController> m_bot_controller;

    /**
     * @brief Shared pointer to subscriber of topic /goal_reached
     * 
     */
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr m_subscriber_goal_reached;

    /**
     * @brief Shared pointer to subscriber of topic /aruco_markers
     * 
     */
    rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr m_subscriber_aruco_markers;

    /**
     * @brief shared pointer to publisher to topic /robot1/cmd_vel
     * 
     */
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr m_rotation_publisher;

    /**
     * @brief Unique pointer to static transform broadcaster
     * 
     */
    std::unique_ptr<tf2_ros::StaticTransformBroadcaster> m_tf_broadcaster{nullptr};

     /**
     * @brief Unique pointer to transform listener
     */
    std::unique_ptr<tf2_ros::TransformListener> m_tf_listener{nullptr};

    /**
     * @brief Unique pointer to buffer
     * 
     */
    std::unique_ptr<tf2_ros::Buffer> m_tf_buffer;

    /**
     * @brief Aruco detector flag
     * 
     */
    bool m_is_aruco_marker_detected{false};

    // methods
    /**
     * @brief This is the callback to the subscriber of the topic /goal_reached.
     * The angular velocity is published on /robot1/cmd_vel topic if the data 'true' is published on /goal_reached
     * and aruco detector flag is not true.
     * 
     * @param goal_reached shared pointer to the message of type std_msgs::msg::Bool published on /goal_reached
     */
    void goal_reached_callback(const std::shared_ptr<std_msgs::msg::Bool> goal_reached);

    /**
     * @brief This is the callback to the subscriber of the topic /aruco_markers.
     * Once the data is published on topic /aruco_markers, aruco detector flag is set true so that the robot stops rotating.
     * Furtehr processing is done to compute and reach the final goal.
     * @param msg shared pointer to the message of type ros2_aruco_interfaces::msg::ArucoMarkers published on /aruco_markers.
     */
    void aruco_markers_callback(const std::shared_ptr<ros2_aruco_interfaces::msg::ArucoMarkers> msg);

    /**
     * @brief Get the final destination using parameters in final_params.yaml file.
     * As per the detected marker id, retrive the coordinates of final goal in the frame specified by the parameter frame_id.
     * @param marker_id int representing detected marker id
     * @param x *double for populating by retrived x coordinate of goal.
     * @param y *double for populating by retrived y coordinate of goal.
     */
    void get_final_destination_from_parameters(const int marker_id, double *x, double *y);

    /**
     * @brief This function creates a frame final_destination and broadcast it as a child of given frame.
     * 
     * @param given_frame string representing name of the frame specified in parameters file.
     * @param retrived_x double which is a retrived x coordinate of final goal in given frame.
     * @param retrived_y double which is a retrived y coordinate of final goal in given frame.
     */
    void broadcast_frame_final_destination(const std::string &given_frame, const double retrived_x, const double retrived_y);

    /**
     * @brief This function computes goal coordinates in odom frame.
     * 
     * @param final_destination_x *double for populating x coordinate of final destination in odom frame.
     * @param final_destination_y *double for populating y coordinate of final destination in odom frame.
     */
    void compute_goal_in_odom_frame(double *final_destination_x, double *final_destination_y);
};