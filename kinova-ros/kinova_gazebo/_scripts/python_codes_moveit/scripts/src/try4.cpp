#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "cartesian_path_slow");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    static const std::string PLANNING_GROUP = "arm"; // Change this to your planning group name

    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    // Set a slow speed scaling factor
    move_group.setMaxVelocityScalingFactor(0.1); // 10% of the maximum velocity

    // Define a Cartesian path
    std::vector<geometry_msgs::Pose> waypoints;
    geometry_msgs::Pose start_pose = move_group.getCurrentPose().pose;
    waypoints.push_back(start_pose);

    geometry_msgs::Pose target_pose = start_pose;
    target_pose.position.x += 0.1; // Move 0.1m in x direction
    waypoints.push_back(target_pose);

    // Plan the Cartesian path
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    move_group.setStartStateToCurrentState();
    move_group.setPoseTarget(target_pose);
    move_group.setPlanningTime(10.0);

    moveit_msgs::RobotTrajectory trajectory;
    double fraction = move_group.computeCartesianPath(waypoints, 0.01, 0.0, trajectory);

    // Slow down the trajectory by scaling the time duration of each waypoint
    for (auto& point : trajectory.joint_trajectory.points) {
        point.time_from_start *= 1; // Scale the time by 10x to slow down
    }

    // Execute the planned path
    my_plan.trajectory_ = trajectory;
    if (fraction == 1.0) {
        ROS_INFO("Planning successfully");
        move_group.execute(my_plan);
    } else {
        ROS_INFO("Planning failed");
    }

    ros::shutdown();
    return 0;
}
