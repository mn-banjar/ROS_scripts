#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_group_interface_tutorial");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    spinner.start();



	//planning group that we would like to control

	moveit::planning_interface::MoveGroupInterface group("arm");
	//we can add or remove collision objects in our virtual world scene
	moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
	//raw pointers are used to refer to the planning group for improved performance
	//const robot_state::JointModelGroup* joint_model_group = group.getCurrentState()->getJointModelGroup("arm");
	group.setEndEffectorLink("arm3");
	group.setPoseReferenceFrame("base");
	group.setPlannerId("base");
	group.setNumPlanningAttempts(30);
	group.setPlanningTime(30.0);
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


    // Initialising and defining the planning group for move_base
    static const std::string PLANNING_GROUP = "gripper";
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const moveit::core::JointModelGroup *joint_model_group =
        move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

    // A pointer that references the current robot's state.
    // RobotState is the object that contains all the current position/velocity/acceleration data.
    moveit::core::RobotStatePtr current_state = move_group.getCurrentState();
    // Get the current set of joint values for the group.
    std::vector<double> joint_group_positions;
    current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
    // Modifing one of the joint positions
    joint_group_positions[0] = 1.0; // radians
    // Pass the desired joint positions to move_group as goal for planning
    move_group.setJointValueTarget(joint_group_positions);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan1;
    bool success = (move_group.plan(my_plan1) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    move_group.move();

    ros::shutdown();
    return 0;
}
