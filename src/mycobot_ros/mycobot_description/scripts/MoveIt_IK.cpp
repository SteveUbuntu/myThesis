#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroupInterface move_group("firefighter");
  move_group.setPoseReferenceFrame("g_base");
  move_group.setPlanningTime(10.0);

  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = 1.0;
  target_pose.position.x = -0.02;
  target_pose.position.y = -0.04;
  target_pose.position.z = 0.29;

  move_group.setPoseTarget(target_pose, "joint6_flange");

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  bool success = (move_group.plan(my_plan) ==
                 moveit::planning_interface::MoveItErrorCode::SUCCESS);

  if (success)
  {
    move_group.execute(my_plan);
    ROS_INFO("Motion execution completed.");
  }
  else
  {
    ROS_ERROR("Motion planning failed!");
  }

  ros::waitForShutdown();
  return 0;
}