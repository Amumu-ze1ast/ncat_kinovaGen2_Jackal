#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>


int main(int argc, char** argv)

{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  static const std::string PLANNING_GROUP = "arm"; 
  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);   
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;   
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);   


  // Getting Basic Information
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str()); 
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());  
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");   
  
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  ROS_INFO("");
  
  // Get the current pose of the end effector
  geometry_msgs::PoseStamped current_pose = move_group.getCurrentPose();
  
  ROS_INFO("");
  ROS_INFO("");
  ROS_INFO("Current Pose:");
  ROS_INFO("Position (x, y, z): x=%.4f, y=%.4f, z=%.4f",
             current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z);
  ROS_INFO("Orientation (qx, qy, qz, qw): x=%.4f, y=%.4f, z=%.4f, w=%.4f",
             current_pose.pose.orientation.x, current_pose.pose.orientation.y,
             current_pose.pose.orientation.z, current_pose.pose.orientation.w);


      ros::shutdown();
  return 0;
}