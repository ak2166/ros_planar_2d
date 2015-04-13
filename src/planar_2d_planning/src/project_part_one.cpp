/**Author: Anthony A. Kolodzinski **/
#include <ros/ros.h>

//MoveIt! includes
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("arm_group");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  ROS_INFO_STREAM("" << group.getCurrentPose());
  std::vector<geometry_msgs::Pose> waypoints;

  geometry_msgs::Pose target_pose;
  target_pose.position.x = 0.2 - 0.2;
  target_pose.position.y = 0.08 - 0.2;
  waypoints.push_back(target_pose);

  target_pose.position.x = 0.3 - 0.2;
  target_pose.position.y = 0.02 - 0.2;
  waypoints.push_back(target_pose);

  target_pose.position.x = 0.38 - 0.2;
  target_pose.position.y = 0.32 - 0.2;
  waypoints.push_back(target_pose);
  
  target_pose.position.x = 0.02 - 0.2;
  target_pose.position.y = 0.38 - 0.2;
  waypoints.push_back(target_pose);
  
  target_pose.position.x = 0.38 - 0.2;
  target_pose.position.y = 0.38 - 0.2;
  waypoints.push_back(target_pose);
  
  group.setGoalTolerance(0.1);
  
  moveit_msgs::RobotTrajectory trajectory;
  double fraction = group.computeCartesianPath(waypoints,
					       0.001,  // eef_step
					       0.0,   // jump_threshold
					       trajectory);

  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
	   fraction * 100.0);
  /* Sleep to give Rviz time to visualize the plan. */
  group.move();
  sleep(15.0);

  
}
