
#ifndef JOINT_SPACE_CONTROL
#define JOINT_SPACE_CONTROL

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <moveit/move_group_interface/move_group_interface.h>
#include <cmath>

/**
 * Solution for take-home assignment 9 
 * Applied Robotics
 */
class TelemanipControlNode : public rclcpp::Node
{
public:
    /**
     * Constructor
     */
    TelemanipControlNode();

    /**
     * Destructor
     */
    ~TelemanipControlNode() = default;

    /**
     * Sets up the move group. Must call before moveToNextPose()
     */
    void initializeMoveGroup();

    /**
     * Moves joints to the starting angles 
     */
    void moveToStart();
    
    /**
     * Main telemanipulation function. Publishes the twist 
     */
    void mainTelemanipulation();

private:

    /**
     * Wrapper function for transformLookup(). Will lookup transform between world and end_effector frame name
     */
    void lookupFrame();
    
    /**
     * Callback for the /joy topic. converts the joystick readings to a twist
     * If A is held, control the robot in orientation. If not, control in translation. If B is pressed, print the current ee-frame
     * @param msg the message on the /joy topic
     */
    void joystickCallback(const sensor_msgs::msg::Joy & msg);

    // subscriber for the joystick
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    // the previous value of the B button (prevents multiple prints per press)
    bool b_button_prev_ = false;

    // Frames
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    // twist message to publish
    geometry_msgs::msg::TwistStamped twist_msg_;

    // publisher for twist
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr twist_publisher_;

    // gains
    double lin_gain_ = 0.1; // 100 mm/s max velocity
    double ang_gain_ = 30*M_PI/180.0; // 10 deg/s max velocity

    // MoveIt
    moveit::planning_interface::MoveGroupInterface::Plan plan_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    std::vector<double> goal_angles_ = {0, 20*M_PI/180.0, 120*M_PI/180.0, 0, -50*M_PI/180.0, 0};
};

#endif // JOINT_SPACE_CONTROL