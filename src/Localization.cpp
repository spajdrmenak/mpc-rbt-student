#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "Localization.hpp"

LocalizationNode::LocalizationNode() : 
    rclcpp::Node("localization_node"), 
    last_time_(this->get_clock()->now()) {

    // Odometry message initialization
    odometry_.header.frame_id = "map";
    odometry_.child_frame_id = "base_link";
    
    odometry_.pose.pose.position.x = 0.0;
    odometry_.pose.pose.position.y = 0.0;
    odometry_.pose.pose.position.z = 0.0;
    odometry_.pose.pose.orientation.w = 0.0;

    // Subscriber for joint_states
    joint_subscriber_ = create_subscription<sensor_msgs::msg::JointState>("joint_states", 1, std::bind(&LocalizationNode::jointCallback, this, std::placeholders::_1));
    
    // Publisher for odometry
    odometry_publisher_ = this->create_publisher<nav_msgs::msg::Odometry> ("cmd_vel",rclcpp::QoS(10));

    // tf_briadcaster 
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(get_logger(), "Localization node started.");
}

void LocalizationNode::jointCallback(const sensor_msgs::msg::JointState & msg) {
    
    // ********
    // * Help *
    // ********
    
    auto current_time = this->get_clock()->now();
    auto dt = (double)(current_time-last_time_).seconds(); 



    updateOdometry(msg.velocity[0], msg.velocity[1], dt);
    publishOdometry();
    publishTransform();

    last_time_ = current_time;
    
}

void LocalizationNode::updateOdometry(double left_wheel_vel, double right_wheel_vel, double dt) {

    // ********
    // * Help *
    // ********
    
    double linear = (left_wheel_vel + right_wheel_vel) * robot_config::WHEEL_RADIUS / 2.0;
    double angular = (left_wheel_vel - right_wheel_vel) * robot_config::WHEEL_RADIUS/robot_config::HALF_DISTANCE_BETWEEN_WHEELS;

    tf2::Quaternion tf_quat;
    tf2::fromMsg(odometry_.pose.pose.orientation, tf_quat);
    double roll, pitch, theta;
    tf2::Matrix3x3(tf_quat).getRPY(roll, pitch, theta);

    theta += angular*dt; //std::atan2(std::sin(theta), std::cos(theta));

    tf2::Quaternion q;
    q.setRPY(0, 0, theta);

    double delta_x = linear * dt * std::cos(theta);
    double delta_y = linear * dt * std::sin(theta);

    odometry_.pose.pose.position.x += delta_x;
    odometry_.pose.pose.position.y += delta_y;
    odometry_.pose.pose.orientation = tf2::toMsg(q);
    
}

void LocalizationNode::publishOdometry() {

    odometry_publisher_->publish(odometry_);
}

void LocalizationNode::publishTransform() {
    
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = this->now();
    t.header.frame_id = "map";
    t.child_frame_id = "base_link";

    t.transform.translation.x = odometry_.pose.pose.position.x;
    t.transform.translation.y = odometry_.pose.pose.position.y;
    t.transform.translation.z = odometry_.pose.pose.position.z;
    
    t.transform.rotation = odometry_.pose.pose.orientation;

    // ********
    // * Help *
    // ********
    tf_broadcaster_->sendTransform(t);
}
