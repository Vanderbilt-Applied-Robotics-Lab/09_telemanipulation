#include <telemanip_control/telemanip_control_node.h>

TelemanipControlNode::TelemanipControlNode() : Node("telemanip_control_node")
{
    // setup subscriber for robot description
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy", 
        10,
        std::bind(&TelemanipControlNode::joystickCallback, this, std::placeholders::_1));

    // setup publisher for twist commands
    twist_publisher_ = this->create_publisher<geometry_msgs::msg::TwistStamped>("/servo_node/delta_twist_cmds", 10);

    // Setup frame
    twist_msg_.header.frame_id = "base_link";

    // setup transform buffer
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
}

void TelemanipControlNode::initializeMoveGroup()
{
    move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            shared_from_this(), "rv_3s_arm");
}

void TelemanipControlNode::moveToStart()
{
    // set movegroup to target joint angles
    move_group_->setJointValueTarget(goal_angles_);

    // if move plan succeeded, move robot
    if (move_group_->plan(plan_) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        move_group_->execute(plan_);
    }
    else // print error otherwise
    {
        RCLCPP_INFO(this->get_logger(), "Plan failed");
    }
}

void TelemanipControlNode::mainTelemanipulation()
{
    twist_msg_.header.stamp = this->get_clock()->now();

    twist_publisher_->publish(twist_msg_);
}

void TelemanipControlNode::joystickCallback(const sensor_msgs::msg::Joy & msg)
{
    if (msg.buttons[0] == 0)
    {
        twist_msg_.twist.linear.x = lin_gain_*msg.axes[0];
        twist_msg_.twist.linear.y = -lin_gain_*msg.axes[1];
        twist_msg_.twist.linear.z = lin_gain_*msg.axes[4];
        twist_msg_.twist.angular.x = 0;
        twist_msg_.twist.angular.y = 0;
        twist_msg_.twist.angular.z = 0; 
    }
    else
    {
        twist_msg_.twist.linear.x = 0;
        twist_msg_.twist.linear.y = 0;
        twist_msg_.twist.linear.z = 0;
        twist_msg_.twist.angular.x = ang_gain_*msg.axes[0];
        twist_msg_.twist.angular.y = -ang_gain_*msg.axes[1];
        twist_msg_.twist.angular.z = ang_gain_*msg.axes[4]; 
    }
    
    // only print once per press
    if ((msg.buttons[1] == 1) && !b_button_prev_)
        lookupFrame();

    b_button_prev_ = msg.buttons[1];
}

void TelemanipControlNode::lookupFrame()
{
    try
    {
        geometry_msgs::msg::TransformStamped transform = tf_buffer_->lookupTransform("world", "end_effector", tf2::TimePointZero);

        RCLCPP_INFO(this->get_logger(), "Position -- x: %0.3f m, y: %0.3f m, z: %0.3f", transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z);
        RCLCPP_INFO(this->get_logger(), "Orientation -- qw: %0.3f, qx: %0.3f, qy: %0.3f, qz: %0.3f", transform.transform.rotation.w, transform.transform.rotation.x, transform.transform.rotation.y, transform.transform.rotation.z);
    }
    catch (const tf2::TransformException & ex)
    {
        RCLCPP_ERROR(this->get_logger(), "lookup failed! Reason: %s", ex.what());
    }
}

int main(int argc, char** argv)
{
    // initialize the node
    rclcpp::init(argc, argv);
    
    // create instance of class
    auto node = std::make_shared<TelemanipControlNode>();

    node->initializeMoveGroup();
    node->moveToStart();
    
    // Set loop rate
    rclcpp::Rate rate = rclcpp::Rate(100); // Hz
    
    while (rclcpp::ok())
    {   
        rclcpp::spin_some(node);
        node->mainTelemanipulation();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}