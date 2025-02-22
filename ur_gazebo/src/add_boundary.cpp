#include <string>
#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <tf/tf.h>
#include <cmath> // 用于M_PI和类型转换
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <geometry_msgs/Pose.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/PlanningScene.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "add_boundary");
    ros::NodeHandle nh;
    sleep(5);
    // 初始化场景对象
    moveit::planning_interface::PlanningSceneInterface scene;


    // 定义table
    moveit_msgs::CollisionObject table_collision;
    table_collision.header.frame_id = "base_link"; // 设定障碍物的参考坐标系
    table_collision.id = "table"; // 障碍物的 ID
    // 定义障碍物的形状和大小 (立方体)
    shape_msgs::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[0] = 1.2; // X 方向大小 (m)
    table_primitive.dimensions[1] = 1.3; // Y 方向大小 (m)
    table_primitive.dimensions[2] = 0.01; // Z 方向大小 (m)
    // 定义障碍物的位置和姿态
    geometry_msgs::Pose table_box_pose;
    table_box_pose.orientation.x = 0.0;  // 无旋转
    table_box_pose.orientation.y = 0.0;  // 无旋转
    table_box_pose.orientation.z = 0.0;  // 无旋转
    table_box_pose.orientation.w = 1.0;  // 无旋转
    table_box_pose.position.x = table_primitive.dimensions[0]/2.0-0.2;     // X 位置
    table_box_pose.position.y = -(table_primitive.dimensions[1]/2.0-0.3);     // Y 位置
    table_box_pose.position.z = -table_primitive.dimensions[2]/2.0-0.01;    // Z 位置 (中心高度)
    // 将形状和姿态加入到 collision object 中
    table_collision.primitives.push_back(table_primitive);
    table_collision.primitive_poses.push_back(table_box_pose);
    table_collision.operation = table_collision.ADD;

    // 定义backwall
    moveit_msgs::CollisionObject backwall_collision;
    backwall_collision.header.frame_id = "base_link"; // 设定障碍物的参考坐标系
    backwall_collision.id = "backwall"; // 障碍物的 ID
    // 定义障碍物的形状和大小 (立方体)
    shape_msgs::SolidPrimitive backwall_primitive;
    backwall_primitive.type = backwall_primitive.BOX;
    backwall_primitive.dimensions.resize(3);
    backwall_primitive.dimensions[0] = 0.01; // X 方向大小 (m)
    backwall_primitive.dimensions[1] = 1.3; // Y 方向大小 (m)
    backwall_primitive.dimensions[2] = 1.0; // Z 方向大小 (m)
    // 定义障碍物的位置和姿态
    geometry_msgs::Pose backwall_box_pose;
    backwall_box_pose.orientation.x = 0.0;  // 无旋转
    backwall_box_pose.orientation.y = 0.0;  // 无旋转
    backwall_box_pose.orientation.z = 0.0;  // 无旋转
    backwall_box_pose.orientation.w = 1.0;  // 无旋转
    backwall_box_pose.position.x = -0.20;     // X 位置
    backwall_box_pose.position.y = -backwall_primitive.dimensions[1]/2.0+0.3;     // Y 位置
    backwall_box_pose.position.z = backwall_primitive.dimensions[2]/2.0-0.01;    // Z 位置 (中心高度)
    // 将形状和姿态加入到 collision object 中
    backwall_collision.primitives.push_back(backwall_primitive);
    backwall_collision.primitive_poses.push_back(backwall_box_pose);
    backwall_collision.operation = backwall_collision.ADD;

    // 定义leftwall
    moveit_msgs::CollisionObject leftwall_collision;
    leftwall_collision.header.frame_id = "base_link"; // 设定障碍物的参考坐标系
    leftwall_collision.id = "leftwall"; // 障碍物的 ID
    // 定义障碍物的形状和大小 (立方体)
    shape_msgs::SolidPrimitive leftwall_primitive;
    leftwall_primitive.type = leftwall_primitive.BOX;
    leftwall_primitive.dimensions.resize(3);
    leftwall_primitive.dimensions[0] = 1.2; // X 方向大小 (m)
    leftwall_primitive.dimensions[1] = 0.01; // Y 方向大小 (m)
    leftwall_primitive.dimensions[2] = 1.0; // Z 方向大小 (m)
    // 定义障碍物的位置和姿态
    geometry_msgs::Pose leftwall_box_pose;
    leftwall_box_pose.orientation.x = 0.0;  // 无旋转
    leftwall_box_pose.orientation.y = 0.0;  // 无旋转
    leftwall_box_pose.orientation.z = 0.0;  // 无旋转
    leftwall_box_pose.orientation.w = 1.0;  // 无旋转
    leftwall_box_pose.position.x = leftwall_primitive.dimensions[0]/2.0-0.2;     // X 位置
    leftwall_box_pose.position.y = 0.30;     // Y 位置
    leftwall_box_pose.position.z = leftwall_primitive.dimensions[2]/2.0-0.01;    // Z 位置 (中心高度)
    // 将形状和姿态加入到 collision object 中
    leftwall_collision.primitives.push_back(leftwall_primitive);
    leftwall_collision.primitive_poses.push_back(leftwall_box_pose);
    leftwall_collision.operation = leftwall_collision.ADD;

    // 将 collision object 添加到场景中
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(table_collision);
    collision_objects.push_back(backwall_collision);
    collision_objects.push_back(leftwall_collision);

    scene.addCollisionObjects(collision_objects);

    ROS_INFO("ADD BOUNDARY!");

    ros::shutdown(); 

    return 0;
}