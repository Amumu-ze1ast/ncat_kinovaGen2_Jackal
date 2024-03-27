#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

int main(int argc, char** argv)

{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  // for gazebo
  // static const std::string PLANNING_GROUP = "arm";  

  // for real_robot
  static const std::string PLANNING_GROUP = "manipulator";  

  moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP); 

  move_group.setMaxVelocityScalingFactor(0.05); 

  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;   
  const robot_state::JointModelGroup* joint_model_group =
      move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);



  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  // Visualization
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //

  namespace rvt = rviz_visual_tools;
  moveit_visual_tools::MoveItVisualTools visual_tools("j2s6s300_end_effector");  
  visual_tools.deleteAllMarkers();

  visual_tools.loadRemoteControl();   

  Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
  text_pose.translation().z() = 1.75;
  visual_tools.publishText(text_pose, "MoveGroupInterface Demo", rvt::WHITE, rvt::XLARGE);

  visual_tools.trigger();  


  // Getting Basic Information
  ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group.getPlanningFrame().c_str()); 
  ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group.getEndEffectorLink().c_str());  
  ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  
  std::copy(move_group.getJointModelGroupNames().begin(), move_group.getJointModelGroupNames().end(),
            std::ostream_iterator<std::string>(std::cout, ", "));

  


  moveit::planning_interface::MoveGroupInterface::Plan my_plan;  
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);


  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //
  // Planning to a joint-space goal. 
  // ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ //  
  
  
  moveit::core::RobotStatePtr current_state = move_group.getCurrentState();   

  std::vector<double> joint_group_positions(6);   
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);


  std::vector<double> degree_arm;
  std::vector<double> degree_gripper;

  // double degree_arm1 = -1.478 * 180 / M_PI;
  // double degree_arm2 =  2.924 * 180 / M_PI;
  // double degree_arm3 =  1.002 * 180 / M_PI;
  // double degree_arm4 = -2.080 * 180 / M_PI;
  // double degree_arm5 =  1.446 * 180 / M_PI;
  // double degree_arm6 =  1.323 * 180 / M_PI;

  // double degree_gripper1 = 70;
  // double degree_gripper2 = 70;
  // double degree_gripper3 = 70;


///////////////////////////////////////////////////
  
  double degree_arm1 = 250.1;
  double degree_arm2 = 158.68;
  double degree_arm3 =  242.86;
  double degree_arm4 = 141.05;
  double degree_arm5 =  111.18;
  double degree_arm6 =  21.18;

///////////////////////////////////////////////////

  degree_arm.push_back(degree_arm1);
  degree_arm.push_back(degree_arm2);
  degree_arm.push_back(degree_arm3);
  degree_arm.push_back(degree_arm4);
  degree_arm.push_back(degree_arm5);
  degree_arm.push_back(degree_arm6);

  // degree_gripper.push_back(degree_gripper1);
  // degree_gripper.push_back(degree_gripper2);
  // degree_gripper.push_back(degree_gripper3);

  joint_group_positions[0] = degree_arm[0] * M_PI / 180;
  joint_group_positions[1] = degree_arm[1] * M_PI / 180; 
  joint_group_positions[2] = degree_arm[2] * M_PI / 180;
  joint_group_positions[3] = degree_arm[3] * M_PI / 180;
  joint_group_positions[4] = degree_arm[4] * M_PI / 180;
  joint_group_positions[5] = degree_arm[5] * M_PI / 180;        
  
  move_group.setJointValueTarget(joint_group_positions);

  success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

  // Visualize the plan in RViz
  visual_tools.deleteAllMarkers();
  visual_tools.publishText(text_pose, "Joint Space Goal", rvt::WHITE, rvt::XLARGE);
  visual_tools.publishTrajectoryLine(my_plan.trajectory_, joint_model_group);
  visual_tools.trigger();
  
  visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
  
  move_group.move();

    ros::shutdown();
  return 0;
}