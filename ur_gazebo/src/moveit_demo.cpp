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
    ros::init(argc, argv, "moveit_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化场景对象
    moveit::planning_interface::PlanningSceneInterface scene;

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    //获取终端link的名称
    std::string end_effector_link = arm.getEndEffectorLink();
    // ROS_INFO("end_effector_link:%s",end_effector_link.c_str());

    //设置目标位置所使用的参考坐标系
    std::string reference_frame = "base_link";
    arm.setPoseReferenceFrame(reference_frame);

    //当运动规划失败后，允许重新规划
    arm.allowReplanning(true);

    //设置位置(单位：米)和姿态（单位：弧度）的允许误差
    arm.setGoalPositionTolerance(0.001);
    arm.setGoalOrientationTolerance(0.01);

    //设置允许的最大速度和加速度
    arm.setMaxAccelerationScalingFactor(0.5);
    arm.setMaxVelocityScalingFactor(0.5);

    //增加轨迹规划时间
    arm.setPlanningTime(10.0); // 增加到 10 秒


    
    // // 移除场景中之前运行残留的物体
    // // 创建一个包含对象ID的字符串向量
    // std::vector<std::string> object_ids;
    // // object_ids.push_back("tool");
    // object_ids.push_back("table");
    // object_ids.push_back("backwall");
    // object_ids.push_back("leftwall");
    // // 根据ID移除碰撞对象
    // scene.removeCollisionObjects(object_ids);

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

    // 定义障碍物的颜色和透明度
    moveit_msgs::ObjectColor table_color;
    table_color.id = "table"; // 颜色对应的障碍物 ID
    table_color.color.r = 0.0; // 红色分量
    table_color.color.g = 0.0; // 绿色分量
    table_color.color.b = 1.0; // 蓝色分量
    table_color.color.a = 0.2; // 设置透明度 (0.5 表示半透明)

    // 定义障碍物的颜色和透明度
    moveit_msgs::ObjectColor backwall_color;
    backwall_color.id = "backwall"; // 颜色对应的障碍物 ID
    backwall_color.color.r = 1.0; // 红色分量
    backwall_color.color.g = 0.0; // 绿色分量
    backwall_color.color.b = 1.0; // 蓝色分量
    backwall_color.color.a = 0.2; // 设置透明度 (0.5 表示半透明)

    // 定义障碍物的颜色和透明度
    moveit_msgs::ObjectColor leftwall_color;
    leftwall_color.id = "leftwall"; // 颜色对应的障碍物 ID
    leftwall_color.color.r = 1.0; // 红色分量
    leftwall_color.color.g = 0.0; // 绿色分量
    leftwall_color.color.b = 1.0; // 蓝色分量
    leftwall_color.color.a = 0.2; // 设置透明度 (0.5 表示半透明)

    // 发布颜色和透明度信息
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.object_colors.push_back(table_color);
    planning_scene.object_colors.push_back(backwall_color);
    planning_scene.object_colors.push_back(leftwall_color);

    ros::Publisher planning_scene_diff_publisher =
        nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::Rate rate(10);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1) {
        rate.sleep();
    }
    planning_scene_diff_publisher.publish(planning_scene);

    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    
    // // 逆运动学解算
    // // 获取当前位姿数据最为机械臂运动的起始位姿    end_effector_link    "tool0"
    // geometry_msgs::Pose start_pose = arm.getCurrentPose(end_effector_link).pose;
    // // // 打印位置信息
    // // ROS_INFO("Position - x: %f, y: %f, z: %f", start_pose.position.x, start_pose.position.y, start_pose.position.z);
    // // // 打印方向信息（四元数）
    // // ROS_INFO("Orientation - x: %f, y: %f, z: %f, w: %f", 
    // //           start_pose.orientation.x, start_pose.orientation.y, start_pose.orientation.z, start_pose.orientation.w);

    // // 设置机器人终端的目标位置
    // geometry_msgs::Pose target_pose;
    // target_pose.orientation.x = start_pose.orientation.x;//
    // target_pose.orientation.y = start_pose.orientation.y;//
    // target_pose.orientation.z = start_pose.orientation.z;//
    // target_pose.orientation.w = start_pose.orientation.w;//

    // target_pose.position.x = start_pose.position.x;//
    // target_pose.position.y = start_pose.position.y;//
    // target_pose.position.z = start_pose.position.z;//

    // // 设置机器臂当前的状态作为运动初始状态
    // arm.setStartStateToCurrentState();

    // arm.setPoseTarget(target_pose);
    // // arm.setPoseTarget(start_pose);

    // // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    // moveit::planning_interface::MoveGroupInterface::Plan plan;
    // moveit::core::MoveItErrorCode success = arm.plan(plan);

    // ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");   

    // //让机械臂按照规划的轨迹开始运动。
    // if(success)
    //   arm.execute(plan);
    // sleep(1);


    //笛卡尔空间下的路径规划  走直线
    geometry_msgs::Pose start_pose = arm.getCurrentPose(end_effector_link).pose;
    std::vector<geometry_msgs::Pose> waypoints;
    //将初始位姿加入路点列表
	waypoints.push_back(start_pose);
    start_pose.position.z -= 0.3;
	waypoints.push_back(start_pose);
    start_pose.position.x -= 0.2;
	waypoints.push_back(start_pose);
    start_pose.position.y -= 0.2;
	waypoints.push_back(start_pose);
    start_pose.position.x += 0.2;
    // start_pose.position.y += 0.2;
	waypoints.push_back(start_pose);
    start_pose.position.y += 0.2;
	waypoints.push_back(start_pose);
    start_pose.position.z += 0.3;
	waypoints.push_back(start_pose);
    
    moveit_msgs::RobotTrajectory trajectory;
	const double jump_threshold = 0.0;
	const double eef_step = 0.01;
    double fraction = 0.0;
    int maxtries = 30;   //最大尝试规划次数
    int attempts = 0;     //已经尝试规划次数
    

    while(fraction < 1.0 && attempts < maxtries)
    {
        fraction = arm.computeCartesianPath(waypoints, eef_step, trajectory);
        attempts++;
        
        if(attempts % 10 == 0)
            ROS_INFO("Still trying after %d attempts...", attempts);
    }
    
    if(fraction == 1)
    {   
        ROS_INFO("Path computed successfully. Moving the arm.");

	    // 生成机械臂的运动规划数据
	    moveit::planning_interface::MoveGroupInterface::Plan plan;
	    plan.trajectory_ = trajectory;

        // //打印时间戳检查是否严格递增
        // for (const auto& point : plan.trajectory_.joint_trajectory.points)
        // {
        //     ROS_INFO("Waypoint time_from_start: %f", point.time_from_start.toSec());
        // }
        // 检查并修正第一个和第二个时间戳
        if (plan.trajectory_.joint_trajectory.points.size() > 2 &&
            plan.trajectory_.joint_trajectory.points[0].time_from_start == plan.trajectory_.joint_trajectory.points[1].time_from_start)
        {
            // ROS_INFO("Change Waypoint time_from_start: %f", plan.trajectory_.joint_trajectory.points[2].time_from_start.toSec()/2.0);
            // plan.trajectory_.joint_trajectory.points[1].time_from_start = ros::Duration(0.03);
            plan.trajectory_.joint_trajectory.points[1].time_from_start = ros::Duration(plan.trajectory_.joint_trajectory.points[2].time_from_start.toSec()/2.0);
            // ROS_INFO("Change Waypoint time_from_start: %f", plan.trajectory_.joint_trajectory.points[1].time_from_start.toSec());
            
        }


	    // 执行运动
	    arm.execute(plan);
        sleep(1);
    }
    else
    {
        ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    }

    // fraction = arm.computeCartesianPath(waypoints, eef_step, trajectory);
    // // 执行路径
    // if (fraction > 0.0)
    // {
    //     ROS_INFO("Executing Cartesian path...fraction:%f",fraction);
    //     arm.execute(trajectory);
    //     sleep(1);
    // }
    // else
    // {
    //     ROS_WARN("Failed to compute Cartesian path");
    // }


    // 控制机械臂先回到初始化位置
    arm.setNamedTarget("home");
    arm.move();
    sleep(1);

    ros::shutdown(); 

    return 0;
}