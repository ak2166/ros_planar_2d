/**Author: Anthony A. Kolodzinski **/
#include <ros/ros.h>

//MoveIt! includes
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_node");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  moveit::planning_interface::MoveGroup group("arm_group");
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  ros::Publisher display_publisher = nh.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;
  geometry_msgs::Pose target_pose1;
  
  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "link3";
  ocm.header.frame_id = "link3";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 6.28318530718;
  ocm.absolute_y_axis_tolerance = 6.28318530718;
  ocm.absolute_z_axis_tolerance = 6.28318530718;
  ocm.weight = 1.0;

  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  group.setPathConstraints(test_constraints);

  //target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.10;
  target_pose1.position.y = 0.10;
  target_pose1.position.z = 0.05;
  //group.setPoseTarget(target_pose1);
  group.setPositionTarget(0.08, 0.08, 0.05, "link3");
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);

  
}
