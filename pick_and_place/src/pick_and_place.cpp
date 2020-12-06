/* This code began life as a the MoveIt Pick and Place tutorial with the Panda robot arm
 */

// ROS
#include <ros/ros.h>
#include <ros/console.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "pick_and_place/GraspSrv.h"

static const std::string PLANNING_GROUP = "left_arm";
static const std::string GRIPPER_GROUP = "left_gripper";
bool use_grasp_generator = true;

void openGripper(trajectory_msgs::JointTrajectory &posture)
{

  // Add all joints of right gripper move group.

  posture.joint_names.resize(6);
  posture.joint_names[0] = "trina2_1/left_arm_finger_joint";
  posture.joint_names[1] = "trina2_1/left_arm_left_inner_finger_joint";
  posture.joint_names[2] = "trina2_1/left_arm_left_inner_knuckle_joint";
  posture.joint_names[3] = "trina2_1/left_arm_right_inner_knuckle_joint";
  posture.joint_names[4] = "trina2_1/left_arm_right_outer_knuckle_joint";
  posture.joint_names[5] = "trina2_1/left_arm_right_inner_finger_joint";

  // Set them as open.
  posture.points.resize(1);
  posture.points[0].positions.resize(6);
  posture.points[0].positions[0] = 0.00;
  posture.points[0].positions[1] = 0.00;
  posture.points[0].positions[2] = 0.00;
  posture.points[0].positions[3] = 0.00;
  posture.points[0].positions[4] = 0.00;
  posture.points[0].positions[5] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

void closedGripper(trajectory_msgs::JointTrajectory &posture)
{
  // Add all joints of the left gripper move group - make these generic later
  posture.joint_names.resize(6);
  posture.joint_names[0] = "trina2_1/left_arm_finger_joint";
  posture.joint_names[1] = "trina2_1/left_arm_left_inner_finger_joint";
  posture.joint_names[2] = "trina2_1/left_arm_left_inner_knuckle_joint";
  posture.joint_names[3] = "trina2_1/left_arm_right_inner_knuckle_joint";
  posture.joint_names[4] = "trina2_1/left_arm_right_outer_knuckle_joint";
  posture.joint_names[5] = "trina2_1/left_arm_right_inner_finger_joint";
  /* Set them as closed. */
  posture.points.resize(1);
  posture.points[0].positions.resize(6);
  posture.points[0].positions[0] = 0.8;
  posture.points[0].positions[1] = 0.8;
  posture.points[0].positions[2] = 0.00;
  posture.points[0].positions[3] = 0.00;
  posture.points[0].positions[4] = 0.00;
  posture.points[0].positions[5] = 0.00;
  posture.points[0].time_from_start = ros::Duration(0.5);
}

// Currently this routine creates one grasp to try.  We will need to adapt this for the optimal grasp of a
// cube to use in stacking/ manipulation.  We may create additional grasps later.
void pick(moveit::planning_interface::MoveGroupInterface &move_group, ros::NodeHandle *nh)
{
  // BEGIN_SUB_TUTORIAL pick1
  // Create a vector of grasps to be attempted, currently only creating single grasp.
  // This is essentially useful when using a grasp generator to generate and test multiple grasps.
  std::vector<moveit_msgs::Grasp> grasps;
  grasps.resize(1);

  if (use_grasp_generator)
  {
    ros::ServiceClient grasp_client = nh->serviceClient<pick_and_place::GraspSrv>("/trina2_1/trina_collab_grasp");
    pick_and_place::GraspSrv grasp_srv;
    grasp_srv.request.object = "demo_object";
    if(grasp_client.call(grasp_srv)) {
      grasps[0] = grasp_srv.response.grasp;
      std::cout << "Grasp returned: " << grasps[0] << std::endl;
    }
    else {
      ROS_ERROR("Failed to get grasp");
      return;
    }
  }
  else
  {
    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
    // of the cube). |br|
    // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
    // extra padding)
    grasps[0].grasp_pose.header.frame_id = "trina2_1/base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.415;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.5;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = "trina2_1/base_link";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = "trina2_1/base_link";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;
  }
  // Setting posture of eef before grasp
  // +++++++++++++++++++++++++++++++++++
  openGripper(grasps[0].pre_grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick2
  // Setting posture of eef during grasp
  // +++++++++++++++++++++++++++++++++++
  closedGripper(grasps[0].grasp_posture);
  // END_SUB_TUTORIAL

  // BEGIN_SUB_TUTORIAL pick3
  // Set support surface as table1.
  move_group.setSupportSurfaceName("table1");
  // Call pick to pick up the object using the grasps given
  move_group.pick("object", grasps);
  // END_SUB_TUTORIAL
}

// Again, tries a single placement location.  We can use this list to create "failures" in place actions.
void place(moveit::planning_interface::MoveGroupInterface &group)
{
  // BEGIN_SUB_TUTORIAL place
  // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
  // location in
  // verbose mode." This is a known issue and we are working on fixing it. |br|
  // Create a vector of placings to be attempted, currently only creating single place location.
  std::vector<moveit_msgs::PlaceLocation> place_location;
  place_location.resize(1);

  // Setting place location pose
  // +++++++++++++++++++++++++++
  place_location[0].place_pose.header.frame_id = "trina2_1/base_link";
  tf2::Quaternion orientation;
  orientation.setRPY(0, 0, M_PI / 2);
  place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

  /* While placing it is the exact location of the center of the object. */
  place_location[0].place_pose.pose.position.x = 0;
  place_location[0].place_pose.pose.position.y = 0.5;
  place_location[0].place_pose.pose.position.z = 0.5;

  // Setting pre-place approach
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].pre_place_approach.direction.header.frame_id = "trina2_1/base_link";
  /* Direction is set as negative z axis */
  place_location[0].pre_place_approach.direction.vector.z = -1.0;
  place_location[0].pre_place_approach.min_distance = 0.095;
  place_location[0].pre_place_approach.desired_distance = 0.115;

  // Setting post-grasp retreat
  // ++++++++++++++++++++++++++
  /* Defined with respect to frame_id */
  place_location[0].post_place_retreat.direction.header.frame_id = "trina2_1/base_link";
  /* Direction is set as negative y axis */
  place_location[0].post_place_retreat.direction.vector.y = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface as table2.
  group.setSupportSurfaceName("table2");
  // Call place to place the object using the place locations given.
  group.place("object", place_location);
  // END_SUB_TUTORIAL
}

// This routine (shamelessly stolen from the MoveIt tutorials) creates collision objects for the robot
// to interact with in RViz.  We will need to create "real" objects in Gazebo and figure out how to interact
// with those instead, but this works well for testing manipulation.
void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface)
{
  // BEGIN_SUB_TUTORIAL table1
  //
  // Creating Environment
  // ^^^^^^^^^^^^^^^^^^^^
  // Create vector to hold 3 collision objects.
  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.resize(3);

  // Add the first table where the cube will originally be kept.
  collision_objects[0].id = "table1";
  collision_objects[0].header.frame_id = "trina2_1/base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[0].primitives.resize(1);
  collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
  collision_objects[0].primitives[0].dimensions.resize(3);
  collision_objects[0].primitives[0].dimensions[0] = 0.2;
  collision_objects[0].primitives[0].dimensions[1] = 0.4;
  collision_objects[0].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[0].primitive_poses.resize(1);
  collision_objects[0].primitive_poses[0].position.x = 0.5;
  collision_objects[0].primitive_poses[0].position.y = 0;
  collision_objects[0].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[0].operation = collision_objects[0].ADD;

  // BEGIN_SUB_TUTORIAL table2
  // Add the second table where we will be placing the cube.
  collision_objects[1].id = "table2";
  collision_objects[1].header.frame_id = "trina2_1/base_link";

  /* Define the primitive and its dimensions. */
  collision_objects[1].primitives.resize(1);
  collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[1].primitives[0].dimensions.resize(3);
  collision_objects[1].primitives[0].dimensions[0] = 0.4;
  collision_objects[1].primitives[0].dimensions[1] = 0.2;
  collision_objects[1].primitives[0].dimensions[2] = 0.4;

  /* Define the pose of the table. */
  collision_objects[1].primitive_poses.resize(1);
  collision_objects[1].primitive_poses[0].position.x = 0;
  collision_objects[1].primitive_poses[0].position.y = 0.5;
  collision_objects[1].primitive_poses[0].position.z = 0.2;
  // END_SUB_TUTORIAL

  collision_objects[1].operation = collision_objects[1].ADD;

  // BEGIN_SUB_TUTORIAL object
  // Define the object that we will be manipulating
  collision_objects[2].header.frame_id = "trina2_1/base_link";
  collision_objects[2].id = "object";

  /* Define the primitive and its dimensions. */
  collision_objects[2].primitives.resize(1);
  collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
  collision_objects[2].primitives[0].dimensions.resize(3);
  collision_objects[2].primitives[0].dimensions[0] = 0.02;
  collision_objects[2].primitives[0].dimensions[1] = 0.02;
  collision_objects[2].primitives[0].dimensions[2] = 0.2;

  /* Define the pose of the object. */
  collision_objects[2].primitive_poses.resize(1);
  collision_objects[2].primitive_poses[0].position.x = 0.5;
  collision_objects[2].primitive_poses[0].position.y = 0;
  collision_objects[2].primitive_poses[0].position.z = 0.5;
  // END_SUB_TUTORIAL

  collision_objects[2].operation = collision_objects[2].ADD;

  planning_scene_interface.applyCollisionObjects(collision_objects);
}

// Demo function to try moving the arm manually
void test_moveit(moveit::planning_interface::MoveGroupInterface &group)
{
  moveit::core::RobotStatePtr current_state = group.getCurrentState();
  std::vector<double> joint_group_positions;
  const moveit::core::JointModelGroup *joint_model_group = group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
  current_state->copyJointGroupPositions(joint_model_group, joint_group_positions);
  moveit::planning_interface::MoveGroupInterface::Plan my_plan;
  joint_group_positions[0] = joint_group_positions[0] - 0.3; // radians
  group.setJointValueTarget(joint_group_positions);
  bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  ROS_INFO_NAMED("tutorial", "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
  if (success)
  {
    success = (group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  }
  ROS_INFO_NAMED("tutorial", "Executing plan (joint space goal) %s", success ? "" : "FAILED");
  std::vector<std::string> remembered = group.getNamedTargets();
  std::cout << remembered.front();
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trina_pick_place");
  ros::NodeHandle nh;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::WallDuration(1.0).sleep();
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  moveit::planning_interface::MoveGroupInterface arm_group(PLANNING_GROUP); // make generic later
  arm_group.setPlanningTime(45.0);

  // Diagnostics - remove later
  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug))
  {
    ros::console::notifyLoggerLevelsChanged();
  }
  // moveit::planning_interface::MoveGroupInterface grip_group(GRIPPER_GROUP);
  // grip_group.setPlanningTime(45.0);
  // ROS_INFO_NAMED("tutorial", "Planning frame: %s", arm_group.getPlanningFrame().c_str());
  // ROS_INFO_NAMED("tutorial", "End effector link: %s", arm_group.getEndEffectorLink().c_str());
  // ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
  // std::copy(arm_group.getJointModelGroupNames().begin(), arm_group.getJointModelGroupNames().end(),
  //         std::ostream_iterator<std::string>(std::cout, ", "));
  // ROS_INFO_NAMED("tutorial", "Joints in gripper:");
  // std::copy(grip_group.getJointNames().begin(), grip_group.getJointNames().end(),
  //         std::ostream_iterator<std::string>(std::cout, ", "));
  //test_moveit(arm_group);
  ros::WallDuration(1.0).sleep();

  // TODO: Create a service that can be called by a button click, then loop.
  addCollisionObjects(planning_scene_interface);

  // Wait a bit for ROS things to initialize
  ros::WallDuration(1.0).sleep();

  pick(arm_group, &nh);

  ros::WallDuration(1.0).sleep();

  place(arm_group);

  ros::waitForShutdown();
  return 0;
}