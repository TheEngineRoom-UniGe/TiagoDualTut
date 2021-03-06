/*
 * Copyright 2019 PAL Robotics SL. All Rights Reserved
 *
 * Unauthorized copying of this file, via any medium is strictly prohibited,
 * unless it was supplied under the terms of a license agreement or
 * nondisclosure agreement with PAL Robotics SL. In this case it may not be
 * copied or disclosed except in accordance with the terms of that agreement.
 */


/** \author Alessandro Di Fava. */

// ROS headers
#include <ros/ros.h>

// MoveIt! headers
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/transform_broadcaster.h>
#include <moveit_msgs/Constraints.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/kinematic_constraints/kinematic_constraint.h>


// Std C++ headers
#include <string>
#include <vector>
#include <map>

int main(int argc, char** argv)
{
 int lockTorso=0;
  ros::init(argc, argv, "plan_dual_arm_torso_ik");

  if ( argc < 8 )
  {
    ROS_INFO(" ");
    ROS_INFO("\tUsage:");
    ROS_INFO(" ");
    ROS_INFO("\trosrun tiago_moveit_tutorial plan_dual_arm_torso_ik [left|right] x y z  r p y");
    ROS_INFO(" ");
    ROS_INFO("\twhere the list of arguments specify the target pose of /arm_[left|right]_tool_link expressed in /base_footprint");
    ROS_INFO(" ");
    return EXIT_FAILURE;
  }
  else if (argc==9)
    lockTorso = atof(argv[8]);

  std::string arm_name = argv[1];

  auto constraints = moveit_msgs::Constraints(); 
  auto joint_constraint = moveit_msgs::JointConstraint();
  constraints.name = "fullrotation";
  joint_constraint.position = 1.22;
  joint_constraint.tolerance_above = 1.57;
  joint_constraint.tolerance_below = 1.57;
  joint_constraint.weight = 1;
  joint_constraint.joint_name = "arm_" + arm_name + "_6_joint";
  constraints.joint_constraints= {joint_constraint};

  
  geometry_msgs::PoseStamped goal_pose;
  goal_pose.header.frame_id = "base_footprint";
  goal_pose.pose.position.x = atof(argv[2]);
  goal_pose.pose.position.y = atof(argv[3]);
  goal_pose.pose.position.z = atof(argv[4]);
  goal_pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(atof(argv[5]), atof(argv[6]), atof(argv[7]));
  

  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  std::vector<std::string> torso_arm_joint_names;
  std::string moveit_group ="";
  //select group of joints
  if(lockTorso==1)
     moveit_group = "arm_" + arm_name;//execlude torso
  else
     moveit_group = "arm_" + arm_name + "_torso";
  moveit::planning_interface::MoveGroupInterface group_arm_torso(moveit_group);
  //choose your preferred planner
  group_arm_torso.setPlannerId("SBLkConfigDefault");
  group_arm_torso.setPoseReferenceFrame("base_footprint");
  group_arm_torso.setPoseTarget(goal_pose);
  group_arm_torso.setPathConstraints(constraints);

  ROS_INFO_STREAM("Planning to move " <<
                  group_arm_torso.getEndEffectorLink() << " to a target pose expressed in " <<
                  group_arm_torso.getPlanningFrame());

  group_arm_torso.setStartStateToCurrentState();
  group_arm_torso.setMaxVelocityScalingFactor(1.0);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  //set maximum time to find a plan
  group_arm_torso.setPlanningTime(20.0);
  bool success = bool(group_arm_torso.plan(my_plan));

  if ( !success )
    throw std::runtime_error("No plan found");

  ROS_INFO_STREAM("Plan found in " << my_plan.planning_time_ << " seconds");

  // Execute the plan
  ros::Time start = ros::Time::now();


  moveit::planning_interface::MoveItErrorCode e = group_arm_torso.move();
  if (!bool(e))
    throw std::runtime_error("Error executing plan");

  ROS_INFO_STREAM("Motion duration: " << (ros::Time::now() - start).toSec());

  spinner.stop();

  return EXIT_SUCCESS;
}
