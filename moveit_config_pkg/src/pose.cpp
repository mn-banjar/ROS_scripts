#include <ros/ros.h>
#include "std_msgs/String.h"
//MOVE IT
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


int main(int argc, char **argv)
{

ros::init(argc, argv, "pose");
ros::NodeHandle nh;


ros::AsyncSpinner spinner(1);
spinner.start();
sleep(2.0);


	//planning group that we would like to control

	moveit::planning_interface::MoveGroupInterface group("arm");
	//we can add or remove collision objects in our virtual world scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	//raw pointers are used to refer to the planning group for improved performance
	const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup("arm");
	group.setEndEffectorLink("arm3");
	group.setPoseReferenceFrame("base");
	group.setPlannerId("base");
	group.setNumPlanningAttempts(50);
	group.setPlanningTime(50.0);
	group.allowReplanning(true);
	group.setGoalJointTolerance(0.0001);
	group.setGoalPositionTolerance(0.0001);
	group.setGoalOrientationTolerance(0.001);
	//group.setNamedTarget("random");
	group.setRandomTarget();
	group.move();  // WORKS FINE :)
	sleep(5.0); //################

	//  CUSTOM PLANNING
	geometry_msgs::Pose target_pose1;
	//NOTE: THIS IS THE VALID POSE FROM RANDOM NODE
	   
	//target_pose1.orientation.w = -0.2607;
	//target_pose1.orientation.x = -0.4014;
	//target_pose1.orientation.y = 0.5875;
	//target_pose1.orientation.z =  -0.652;
	
	target_pose1.position.x =  -0.0425;
	target_pose1.position.y =  -0.0859;
	target_pose1.position.z =   0.0087;

	group.setStartStateToCurrentState();
	group.setPoseTarget(target_pose1,"arm3");

	moveit::planning_interface::MoveGroupInterface::Plan my_plan;
	moveit_msgs::MotionPlanRequest response;

	group.plan(my_plan);
	group.execute(my_plan);


sleep(5.0);
//ros::shutdown();
ros::waitForShutdown();


return 0;

}
