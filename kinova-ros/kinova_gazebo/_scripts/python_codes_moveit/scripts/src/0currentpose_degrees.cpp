#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

bool receivedMessage = false;  // Flag to track if message has been received

void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg) {
    if (!receivedMessage) {
        ROS_INFO("Received joint state message with size: %lu", msg->name.size());

        // Print joint names and positions for debugging
        ROS_INFO(" ");
        ROS_INFO("In RADIANS");
        for (size_t i = 0; i < msg->name.size(); ++i) {
            ROS_INFO("Joint %s position: %.4f radians", msg->name[i].c_str(), msg->position[i]);

            receivedMessage = true;  // Set flag to true to indicate message processed
        }

        ROS_INFO(" ");
        ROS_INFO("In DEGREES");
        for (size_t i = 0; i < msg->name.size(); ++i) {
            
            double position_degrees = msg->position[i] * (180.0 / M_PI);
            ROS_INFO("Joint %s position: %.4f degrees", msg->name[i].c_str(), position_degrees);

            receivedMessage = true;  // Set flag to true to indicate message processed
        }
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "joint_state_listener");
    ros::NodeHandle nh;

    //for gazebo
    ros::Subscriber joint_state_sub = nh.subscribe("/joint_states", 10, jointStateCallback);

    //for real_robot
    // ros::Subscriber joint_state_sub = nh.subscribe("/kinova_arm_driver/out/joint_state", 10, jointStateCallback);

    while (!receivedMessage && ros::ok()) {
        ros::spinOnce();  // Process callbacks
        ros::Duration(0.1).sleep();  // Wait for callbacks and sleep for a short duration
    }

    return 0;
}
