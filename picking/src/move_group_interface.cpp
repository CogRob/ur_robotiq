#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf/transform_listener.h>

#include <iostream>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iomanip>

bool foundobject;

bool moveToOrigin(moveit::planning_interface::MoveGroupInterface &move_group)
{
    // original position of joints
    std::map<std::string, double> originJoints = {{"shoulder_pan_joint", 0.588144},
                                                  {"shoulder_lift_joint", -2.32071},
                                                  {"elbow_joint", 1.2686},
                                                  {"wrist_1_joint", -0.646353},
                                                  {"wrist_2_joint", 1.75472},
                                                  {"wrist_3_joint", 0.958517}};
    move_group.setJointValueTarget(originJoints);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    move_group.move();
    return success;
}

void analyseObject(moveit::planning_interface::MoveGroupInterface &move_group, double x, double y, double z, double heightAbove = 0.2)
{
    std::string oldEffectorLink = move_group.getEndEffectorLink();

    geometry_msgs::Pose searchPoint;
    searchPoint.orientation.x = -0.5;
    searchPoint.orientation.y = -0.5;
    searchPoint.orientation.z = 0.5;
    searchPoint.orientation.w = 0.5;
    searchPoint.position.x = x;
    searchPoint.position.y = y;
    searchPoint.position.z = z + heightAbove;
    std::vector<geometry_msgs::Pose> waypoints;
    waypoints.push_back(searchPoint);

    geometry_msgs::Pose searchPoint2 = searchPoint;

    searchPoint2.orientation.x = 0.6;
    searchPoint2.orientation.y = 0.6;
    searchPoint2.orientation.z = -0.4;
    searchPoint2.orientation.w = -0.4;
    searchPoint2.position.x -= 0.02;
    waypoints.push_back(searchPoint2);

    searchPoint2.orientation.x = -0.4;
    searchPoint2.orientation.y = -0.4;
    searchPoint2.orientation.z = 0.6;
    searchPoint2.orientation.w = 0.6;
    searchPoint2.position.x += 0.4;
    waypoints.push_back(searchPoint2);

    searchPoint2.orientation.x = -0.5;
    searchPoint2.orientation.y = -0.5;
    searchPoint2.orientation.z = 0.5;
    searchPoint2.orientation.w = 0.5;
    searchPoint2.position.x -= 0.02;
    waypoints.push_back(searchPoint2);

    move_group.setMaxVelocityScalingFactor(0.2);

    move_group.setEndEffectorLink("camera_link");
    moveit_msgs::RobotTrajectory trajectory;
    const double jump_threshold = 0.0;
    const double eef_step = 0.01;
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);
    moveit::planning_interface::MoveGroupInterface::Plan goal_plan;
    goal_plan.trajectory_ = trajectory;
    move_group.execute(goal_plan);

    move_group.setEndEffectorLink(oldEffectorLink);
}

void printJointValue(moveit::planning_interface::MoveGroupInterface &move_group)
{
    std::cout << std::left << std::setfill(' ') << std::setw(20) << "Joint name" << '|' << std::right << std::setw(20) << "Joint values" << std::endl;

    for (std::vector<std::string>::const_iterator it = move_group.getJointNames().begin(); it != move_group.getJointNames().end(); ++it)
    {
        std::cout << std::left << std::setfill(' ') << std::setw(20) << *it << '|' << std::right << std::setw(20) << *(move_group.getCurrentState()->getJointPositions(*it)) << std::endl;
    }
}

void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface &planning_scene_interface, double x, double y, double z)
{
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(1);

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "object";
    collision_objects[0].header.frame_id = "world";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.03;
    collision_objects[0].primitives[0].dimensions[1] = 0.09;
    collision_objects[0].primitives[0].dimensions[2] = 0.03;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = x;
    collision_objects[0].primitive_poses[0].position.y = y;
    collision_objects[0].primitive_poses[0].position.z = z;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    planning_scene_interface.applyCollisionObjects(collision_objects);
}

bool pick(moveit::planning_interface::MoveGroupInterface &move_group, double x, double y, double z, double heightAbove = 0.05)
{
    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = 0.5;
    target_pose.orientation.y = 0.5;
    target_pose.orientation.z = -0.5;
    target_pose.orientation.w = 0.5;
    target_pose.position.x = x;
    target_pose.position.y = y;
    target_pose.position.z = z + heightAbove;
    move_group.setPoseTarget(target_pose);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO_NAMED("tutorial", "Visualizing plan 1 (target goal) %s", success ? "SUCCESS" : "FAILED");
    move_group.setMaxVelocityScalingFactor(0.08);
    move_group.move();
    return success;
}

void CheckObjectCallback(const std_msgs::String::ConstPtr &msg)
{

    if (msg->data == "success")
    {
        foundobject = true;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ur5e_robot_move_group_interface");
    ros::NodeHandle node_handle;
    tf::TransformListener listener;
    tf::StampedTransform transform;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    foundobject = false;
    static const std::string PLANNING_GROUP = "manipulator";

    // setup the name of the planning group that used to control and plan for
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);

    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //printJointValue(move_group);
    //moveToOrigin(move_group);

    // detect the object
    ros::Subscriber sub = node_handle.subscribe("/foundObject", 1, CheckObjectCallback);
    ros::Rate r(50);
    while (true)
    {
        if (foundobject)
        {
            break;
        }
        r.sleep();
    }

    listener.lookupTransform("world", "object", ros::Time(0), transform);
    std::cout << transform.getOrigin().x() << " " 
              << transform.getOrigin().y() << " "
              << transform.getOrigin().z() << std::endl;

    double objectx = transform.getOrigin().x();
    double objecty = transform.getOrigin().y();
    double objectz = transform.getOrigin().z();

    //addCollisionObjects(planning_scene_interface, objectx, objecty, objectz);
    //analyseObject(move_group, objectx, objecty, objectz, 0.5);
    bool success = pick(move_group, objectx, objecty, objectz);

    return 0;
}
