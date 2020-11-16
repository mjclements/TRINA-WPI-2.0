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
void test_moveit(moveit::planning_interface::MoveGroupInterface &group)
{

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "arm_to_block");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::WallDuration(1.0).sleep();

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("left_arm"); // make generic later
    group.setPlanningTime(45.0);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    moveit_msgs::CollisionObject table;

    // Add the bounding box for the table.

    table.id = "table1";
    table.header.frame_id = "trina2_1/base_link";

    /* Define the primitive and its dimensions. Update with dimensions from Gazebo when available*/
    table.primitives.resize(1);
    table.primitives[0].type = table.primitives[0].BOX;
    table.primitives[0].dimensions.resize(3);
    table.primitives[0].dimensions[0] = 2.0;
    table.primitives[0].dimensions[1] = 1.0;
    table.primitives[0].dimensions[2] = 0.5;

    gazebo_msgs::GetModelState block_state;
    block_state.request.model_name = "unit_box";
    block_state.request.relative_entity_name = "trina2_1/base_link";
    ros::ServiceClient get_block_client = nh.serviceClient<gazebo_msgs::GetModelState>("/gazebo/get_model_state");
    get_block_client.call(block_state);
    bool result = block_state.response.success;
    if (!result)
        ROS_WARN("service call to get_model_state failed!");
    else
        ROS_INFO("Done");
    
    /* Define the pose of the table. */
    table.primitive_poses.resize(1);
    table.primitive_poses[0] = block_state.response.pose;
    table.operation = table.ADD;
    planning_scene_interface.applyCollisionObject(table);
    block_state.request.model_name = "unit_box_2";
    get_block_client.call(block_state);
    result = block_state.response.success;
    if (!result)
        ROS_WARN("service call to get_model_state failed!");
    else
        ROS_INFO("Done");
    
    // Move to pose

    geometry_msgs::Pose target_pose1 = block_state.response.pose;
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, -M_PI, -M_PI / 2);
    target_pose1.orientation = tf2::toMsg(orientation);
    target_pose1.position.x -= 0.1;

    gazebo_msgs::SetModelState sphere_state;
    ros::ServiceClient set_sphere_client = nh.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    sphere_state.request.model_state.pose = target_pose1;
    sphere_state.request.model_state.reference_frame = "trina2_1/base_link";
    sphere_state.request.model_state.model_name = "unit_sphere";
    set_sphere_client.call(sphere_state);

    group.setPoseTarget(target_pose1);
    group.setGoalTolerance(0.1);
    group.setStartStateToCurrentState();
    bool success = (group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("move", "Planning to pose goal %s", success ? "" : "FAILED");
    if (success)
    {
        success = (group.execute(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    }


}