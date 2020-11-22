#include <ros/ros.h>
#include <ros/console.h>
// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Gazebo
#include <gazebo_msgs/GetModelState.h>
#include <gazebo_msgs/SetModelState.h>

#include "pick_and_place/PickSrv.h"

moveit::planning_interface::PlanningSceneInterface* planning_scene_interface;
boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> group; // make generic later
//ros::ServiceClient get_block_client;

void add_coll_object(std::string obj)
{
    moveit_msgs::CollisionObject object;

    // Add the bounding box for obj.

    object.id = obj;
    object.header.frame_id = "trina2_1/base_link";

    /* Define the primitive and its dimensions. Update with dimensions from Gazebo when available*/
    object.primitives.resize(1);
    object.primitives[0].type = object.primitives[0].BOX;
    object.primitives[0].dimensions.resize(3);
    if (obj == "unit_box") {//table
        object.primitives[0].dimensions[0] = 2.0;
        object.primitives[0].dimensions[1] = 1.0;
        object.primitives[0].dimensions[2] = 0.5;
    }
    else {
        object.primitives[0].dimensions[0] = 0.03;
        object.primitives[0].dimensions[1] = 0.03;
        object.primitives[0].dimensions[2] = 0.1;
    }

    gazebo_msgs::GetModelState block_state;
    block_state.request.model_name = obj;
    block_state.request.relative_entity_name = "trina2_1/base_link";
    // get_block_client.call(block_state);
    // bool result = block_state.response.success;
    // if (!result)
    //     ROS_WARN("service call to get_model_state failed!");
    // else
    //     ROS_INFO("Done");
    if (ros::service::call("/gazebo/get_model_state", block_state))
        ROS_INFO("Got model state");
    else
    {
        ROS_WARN("service call to get_model_state failed!");
        return;
    }

    /* Define the pose of the table. */
    object.primitive_poses.resize(1);
    object.primitive_poses[0] = block_state.response.pose;
    object.operation = object.ADD;
    planning_scene_interface->applyCollisionObject(object);
}

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
    // posture.points[0].positions[1] = -0.8;
    // posture.points[0].positions[2] = 0.00;
    // posture.points[0].positions[3] = 0.00;
    // posture.points[0].positions[4] = 0.00;
    // posture.points[0].positions[5] = -0.8;
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
    // posture.points[0].positions[1] = 0.8;
    // posture.points[0].positions[2] = 0.0;
    // posture.points[0].positions[3] = 0.0;
    // posture.points[0].positions[4] = 0.0;
    // posture.points[0].positions[5] = 0.8;
    posture.points[0].time_from_start = ros::Duration(0.5);
}

void pick_block(std::string pick_obj)
{
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting grasp pose
    // ++++++++++++++++++++++
    gazebo_msgs::GetModelState block_state;
    block_state.request.relative_entity_name = "trina2_1/base_link";
    block_state.request.model_name = pick_obj;
    if (ros::service::call("/gazebo/get_model_state", block_state))
        ROS_INFO("Done");
    else
    {
        ROS_WARN("service call to get_model_state failed!");
        return;
    }
    
    // get_block_client.call(block_state);
    // bool result = block_state.response.success;
    // if (!result)
    // {
    //     ROS_WARN("service call to get_model_state failed!");
    //     return;
    // }
    // else
    //     ROS_INFO("Done");

    geometry_msgs::Pose target_pose1 = block_state.response.pose;
    grasps[0].grasp_pose.header.frame_id = "trina2_1/base_link";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 4, -M_PI, -M_PI/2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position = block_state.response.pose.position;
    grasps[0].grasp_pose.pose.position.x -= 0.08;
    grasps[0].grasp_pose.pose.position.y -= 0.01;
    grasps[0].grasp_pose.pose.position.z += 0.08;

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

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasps[0].pre_grasp_posture);

    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasps[0].grasp_posture);

    // Set support surface as table1.
    group->setSupportSurfaceName("unit_box");
    // Call pick to pick up the object using the grasps given
    group->setGoalTolerance(0.05);
    group->setStartStateToCurrentState();
    group->pick(pick_obj, grasps);
}

bool trina_pick(pick_and_place::PickSrv::Request &req, pick_and_place::PickSrv::Response &res)
{
    std::string pick_obj = req.pick_obj;
    ROS_INFO("Pick requested");
    add_coll_object(pick_obj);
    pick_block(pick_obj);
    return true;
}

void place_block(std::string place_obj)
{
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
  place_location[0].place_pose.pose.position.x = 0.8;
  place_location[0].place_pose.pose.position.y = 0.51;
  place_location[0].place_pose.pose.position.z = 0.54;

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
  place_location[0].post_place_retreat.direction.vector.x = -1.0;
  place_location[0].post_place_retreat.min_distance = 0.1;
  place_location[0].post_place_retreat.desired_distance = 0.25;

  // Setting posture of eef after placing object
  // +++++++++++++++++++++++++++++++++++++++++++
  /* Similar to the pick case */
  openGripper(place_location[0].post_place_posture);

  // Set support surface 
  group->setSupportSurfaceName("unit_box");
  // Call place to place the object using the place locations given.
  group->place(place_obj, place_location);

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_to_block");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    group.reset(new moveit::planning_interface::MoveGroupInterface("left_arm"));
    group->setPlanningTime(45.0);
    planning_scene_interface = new moveit::planning_interface::PlanningSceneInterface();
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    // Make sure we have a clean planning scene before starting the node
    planning_scene_interface->removeCollisionObjects(planning_scene_interface->getKnownObjectNames());
    add_coll_object("unit_box");

    // gazebo_msgs::GetModelState block_state;
    // block_state.request.relative_entity_name = "trina2_1/base_link";
    // block_state.request.model_name = "unit_box_0";
    // if (ros::service::call("/gazebo/get_model_state", block_state))
    //     ROS_INFO("Done");
    // else
    // {
    //     ROS_WARN("service call to get_model_state failed!");
    //     return 0;
    // }

    // // // Move to pose

    // geometry_msgs::Pose target_pose1 = block_state.response.pose;
    // tf2::Quaternion orientation;
    // orientation.setRPY(-M_PI / 4, -M_PI, -M_PI/2);
    // target_pose1.orientation = tf2::toMsg(orientation);
    // target_pose1.position.z += 0.2;


    // group->setPoseTarget(target_pose1);
    // group->setGoalTolerance(0.1);
    // group->setStartStateToCurrentState();
    // bool success = (group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // ROS_INFO_NAMED("move", "Planning to pose goal %s", success ? "" : "FAILED");
    // if (success)
    // {
    //     success = (group->execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    // }
    // gazebo_msgs::SetModelState sphere_state;
    // ros::ServiceClient set_sphere_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    // sphere_state.request.model_state.pose = target_pose1;
    // sphere_state.request.model_state.reference_frame = "trina2_1/base_link";
    // sphere_state.request.model_state.model_name = "unit_sphere";
    // set_sphere_client.call(sphere_state);

    //ros::ServiceServer service = nh.advertiseService("trina_pick", trina_pick);
    std::string pick_obj = "unit_box_0";
    add_coll_object(pick_obj);
    pick_block(pick_obj);
    place_block(pick_obj);
    ros::waitForShutdown();
    return 0;
}