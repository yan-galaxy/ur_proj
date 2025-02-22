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

// roslaunch ur_gazebo ur5_bringup.launch
// roslaunch ur5_moveit_config moveit_planning_execution.launch sim:=true
// roslaunch ur5_moveit_config moveit_rviz.launch

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_demo");
    ros::NodeHandle nh;
    ros::AsyncSpinner spinner(1);
    spinner.start();

    // 初始化场景对象
    moveit::planning_interface::PlanningSceneInterface scene;

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    // // 设置规划器为 EST
    // arm.setPlannerId("ESTConfigDefault");  // 使用 EST 规划器
    // ROS_INFO("Planner set to ESTConfigDefault");

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
    arm.setMaxVelocityScalingFactor(0.03);

    //增加轨迹规划时间
    arm.setPlanningTime(10.0); // 增加到 10 秒


    // 1**限制运动空间，防止伤人碰撞
    // 移除场景中之前运行残留的物体
    // 创建一个包含对象ID的字符串向量
    std::vector<std::string> object_ids;
    // object_ids.push_back("tool");
    object_ids.push_back("table");
    object_ids.push_back("backwall");
    object_ids.push_back("leftwall");
    object_ids.push_back("rightwall");
    // 根据ID移除碰撞对象
    scene.removeCollisionObjects(object_ids);

    // 定义table
    moveit_msgs::CollisionObject table_collision;
    table_collision.header.frame_id = "base_link"; // 设定障碍物的参考坐标系
    table_collision.id = "table"; // 障碍物的 ID
    // 定义障碍物的形状和大小 (立方体)
    shape_msgs::SolidPrimitive table_primitive;
    table_primitive.type = table_primitive.BOX;
    table_primitive.dimensions.resize(3);
    table_primitive.dimensions[0] = (1.2+0.3) * std::sqrt(2); // X 方向大小 (m)
    table_primitive.dimensions[1] = 1.2 * std::sqrt(2); // Y 方向大小 (m)
    table_primitive.dimensions[2] = 0.01 ; // Z 方向大小 (m)
    // 定义障碍物的位置和姿态
    geometry_msgs::Pose table_box_pose;
    // 设置绕Z轴旋转45度
    double table_roll = 0.0;
    double table_pitch = 0.0;
    double table_yaw = M_PI / 4.0; // 45度，单位为弧度   M_PI / 4.0
    // 使用tf::Quaternion计算四元数
    tf::Quaternion table_quaternion;
    table_quaternion.setRPY(table_roll, table_pitch, table_yaw);
    // 将tf::Quaternion转换为geometry_msgs::Quaternion
    table_box_pose.orientation.x = table_quaternion.x();//0.0
    table_box_pose.orientation.y = table_quaternion.y();//0.0
    table_box_pose.orientation.z = table_quaternion.z();//0.0
    table_box_pose.orientation.w = table_quaternion.w();//1.0
    table_box_pose.position.x = 0.6;     // X 位置  1.2/2.0=0.6 table_primitive.dimensions[0]/2.0-0.2
    table_box_pose.position.y = 0.6;     // Y 位置  1.2/2.0=0.6 -(table_primitive.dimensions[1]/2.0-0.3)
    table_box_pose.position.z = -table_primitive.dimensions[2]/2.0-0.01;    // Z 位置 (中心高度)  -table_primitive.dimensions[2]/2.0-0.01
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
    // 设置绕Z轴旋转45度
    double backwall_roll = 0.0;
    double backwall_pitch = 0.0;
    double backwall_yaw = M_PI / 4.0; // 45度，单位为弧度   M_PI / 4.0
    // 使用tf::Quaternion计算四元数
    tf::Quaternion backwall_quaternion;
    backwall_quaternion.setRPY(backwall_roll, backwall_pitch, backwall_yaw);
    // 将tf::Quaternion转换为geometry_msgs::Quaternion
    backwall_box_pose.orientation.x = backwall_quaternion.x();//0.0
    backwall_box_pose.orientation.y = backwall_quaternion.y();//0.0
    backwall_box_pose.orientation.z = backwall_quaternion.z();//0.0
    backwall_box_pose.orientation.w = backwall_quaternion.w();//1.0
    backwall_box_pose.position.x = -0.15/1.41421356;     // X 位置
    backwall_box_pose.position.y = -0.15/1.41421356;     // Y 位置
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
    leftwall_primitive.dimensions[0] = 1.8; // X 方向大小 (m)
    leftwall_primitive.dimensions[1] = 0.01; // Y 方向大小 (m)
    leftwall_primitive.dimensions[2] = 1.0; // Z 方向大小 (m)
    // 定义障碍物的位置和姿态
    geometry_msgs::Pose leftwall_box_pose;
    // 设置绕Z轴旋转45度
    double leftwall_roll = 0.0;
    double leftwall_pitch = 0.0;
    double leftwall_yaw = M_PI / 4.0; // 45度，单位为弧度   M_PI / 4.0
    // 使用tf::Quaternion计算四元数
    tf::Quaternion leftwall_quaternion;
    leftwall_quaternion.setRPY(leftwall_roll, leftwall_pitch, leftwall_yaw);
    // 将tf::Quaternion转换为geometry_msgs::Quaternion
    leftwall_box_pose.orientation.x = leftwall_quaternion.x();//0.0
    leftwall_box_pose.orientation.y = leftwall_quaternion.y();//0.0
    leftwall_box_pose.orientation.z = leftwall_quaternion.z();//0.0
    leftwall_box_pose.orientation.w = leftwall_quaternion.w();//1.0
    double leftwall_axis_distance = 0.25;//左边墙到机械臂真正x轴的距离
    leftwall_box_pose.position.x = (std::sqrt(2)/2.0-leftwall_axis_distance)/std::sqrt(2);     // X 位置    0.287868
    leftwall_box_pose.position.y = 1-leftwall_box_pose.position.x;     // Y 位置  0.712132
    leftwall_box_pose.position.z = leftwall_primitive.dimensions[2]/2.0-0.01;    // Z 位置 (中心高度)
    // 将形状和姿态加入到 collision object 中
    leftwall_collision.primitives.push_back(leftwall_primitive);
    leftwall_collision.primitive_poses.push_back(leftwall_box_pose);
    leftwall_collision.operation = leftwall_collision.ADD;


    // 定义 rightwall
    moveit_msgs::CollisionObject rightwall_collision;
    rightwall_collision.header.frame_id = "base_link"; // 设定障碍物的参考坐标系
    rightwall_collision.id = "rightwall"; // 障碍物的 ID
    // 定义障碍物的形状和大小 (立方体)
    shape_msgs::SolidPrimitive rightwall_primitive;
    rightwall_primitive.type = rightwall_primitive.BOX;
    rightwall_primitive.dimensions.resize(3);
    rightwall_primitive.dimensions[0] = 1.8; // X 方向大小 (m)
    rightwall_primitive.dimensions[1] = 0.01; // Y 方向大小 (m)
    rightwall_primitive.dimensions[2] = 1.0; // Z 方向大小 (m)
    // 定义障碍物的位置和姿态
    geometry_msgs::Pose rightwall_box_pose;
    // 设置绕Z轴旋转45度
    double rightwall_roll = 0.0;
    double rightwall_pitch = 0.0;
    double rightwall_yaw = M_PI / 4.0; // 45度，单位为弧度   M_PI / 4.0
    // 使用tf::Quaternion计算四元数
    tf::Quaternion rightwall_quaternion;
    rightwall_quaternion.setRPY(rightwall_roll, rightwall_pitch, rightwall_yaw);
    // 将tf::Quaternion转换为geometry_msgs::Quaternion
    rightwall_box_pose.orientation.x = rightwall_quaternion.x();//0.0
    rightwall_box_pose.orientation.y = rightwall_quaternion.y();//0.0
    rightwall_box_pose.orientation.z = rightwall_quaternion.z();//0.0
    rightwall_box_pose.orientation.w = rightwall_quaternion.w();//1.0
    double rightwall_axis_distance = 0.3;//右边墙到机械臂真正x轴的距离
    rightwall_box_pose.position.y = (std::sqrt(2)/2.0-rightwall_axis_distance)/std::sqrt(2);     // Y 位置   0.287868
    rightwall_box_pose.position.x = 1-rightwall_box_pose.position.y;     // X 位置    0.712132
    rightwall_box_pose.position.z = rightwall_primitive.dimensions[2]/2.0-0.01;    // Z 位置 (中心高度)
    // 将形状和姿态加入到 collision object 中
    rightwall_collision.primitives.push_back(rightwall_primitive);
    rightwall_collision.primitive_poses.push_back(rightwall_box_pose);
    rightwall_collision.operation = rightwall_collision.ADD;

    // 将 collision object 添加到场景中
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.push_back(table_collision);
    collision_objects.push_back(backwall_collision);
    collision_objects.push_back(leftwall_collision);
    collision_objects.push_back(rightwall_collision);

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

    // 定义障碍物的颜色和透明度
    moveit_msgs::ObjectColor rightwall_color;
    rightwall_color.id = "rightwall"; // 颜色对应的障碍物 ID
    rightwall_color.color.r = 1.0; // 红色分量
    rightwall_color.color.g = 0.0; // 绿色分量
    rightwall_color.color.b = 1.0; // 蓝色分量
    rightwall_color.color.a = 0.2; // 设置透明度 (0.5 表示半透明)

    // 发布颜色和透明度信息
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.is_diff = true;
    planning_scene.object_colors.push_back(table_color);
    planning_scene.object_colors.push_back(backwall_color);
    planning_scene.object_colors.push_back(leftwall_color);
    planning_scene.object_colors.push_back(rightwall_color);

    ros::Publisher planning_scene_diff_publisher =
        nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    ros::Rate rate(10);
    while (planning_scene_diff_publisher.getNumSubscribers() < 1) {
        rate.sleep();
    }
    planning_scene_diff_publisher.publish(planning_scene);



    // 2**获取当前末端位姿
    geometry_msgs::PoseStamped current_pose = arm.getCurrentPose("tool0"); // tool0是标准UR5的末端link名称
    ROS_INFO("Current end-effector pose:");
    ROS_INFO("Position(x,y,z): [%.4f, %.4f, %.4f]", 
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z);

    // 将四元数转换为欧拉角显示
    tf::Quaternion q(
        current_pose.pose.orientation.x,
        current_pose.pose.orientation.y,
        current_pose.pose.orientation.z,
        current_pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    ROS_INFO("Orientation(rpy in radians): [roll:%.4f, pitch:%.4f, yaw:%.4f]", roll, pitch, yaw);
    ROS_INFO("Orientation(rpy in degrees): [roll:%.4f, pitch:%.4f, yaw:%.4f]", 
            roll * 180/M_PI, 
            pitch * 180/M_PI, 
            yaw * 180/M_PI);

    // 3** 设置机械臂的目标位姿     
    //测试得理想夹取位姿为 位置(x=0.4123,y=0.2556,z=0.03)欧拉角姿态(roll=-90,pitch=0.0,yaw=-45)
    // 对应关节数据为
    //  - elbow_joint           -2.148289982472555
    //  - shoulder_lift_joint   -2.053070370350973
    //  - shoulder_pan_joint    -2.3539419809924524
    //  - wrist_1_joint         -2.0865190664874476
    //  - wrist_2_joint         -1.553669277821676
    //  - wrist_3_joint          3.150042772293091
    double target_roll_deg = -90;    // 目标roll角（角度）
    double target_pitch_deg = 0.0; // 目标pitch角（角度）
    double target_yaw_deg = -45;   // 目标yaw角（角度）
    double target_roll_rad = target_roll_deg * M_PI / 180.0;
    double target_pitch_rad = target_pitch_deg * M_PI / 180.0;
    double target_yaw_rad = target_yaw_deg * M_PI / 180.0;
    tf::Quaternion target_quaternion;
    target_quaternion.setRPY(target_roll_rad, target_pitch_rad, target_yaw_rad);

    geometry_msgs::Pose target_pose;
    target_pose.orientation.x = target_quaternion.x();
    target_pose.orientation.y = target_quaternion.y();
    target_pose.orientation.z = target_quaternion.z();
    target_pose.orientation.w = target_quaternion.w();

    // 指定前进距离
    double distance = 0.0; // 前进0.1米
    // 计算方向向量 (1, 1, 0)  沿着程序空间x=y的方向 对应实验室实际空间的x轴正方向
    Eigen::Vector3d direction(1.0, 1.0, 0.0);
    // 归一化方向向量
    direction.normalize();
    // 计算新的目标位置
    Eigen::Vector3d current_position(0.4123, 0.2556, current_pose.pose.position.z);
    Eigen::Vector3d target_position = current_position + direction * distance;
    // 设置目标位置
    target_pose.position.x = target_position.x();//current_pose.pose.position.x  0.4123
    target_pose.position.y = target_position.y();//current_pose.pose.position.y  0.2556
    target_pose.position.z = 0.03; // 保持z坐标不变

    // 设置机器臂当前的状态作为运动初始状态
    arm.setStartStateToCurrentState();

    arm.setPoseTarget(target_pose);
    // arm.setPoseTarget(start_pose);

    // 进行运动规划，计算机器人移动到目标的运动轨迹，此时只是计算出轨迹，并不会控制机械臂运动
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::MoveItErrorCode success = arm.plan(plan);

    ROS_INFO("Plan (pose goal) %s",success?"":"FAILED");   

    //让机械臂按照规划的轨迹开始运动。
    if(success)
      arm.execute(plan);
    sleep(1);


    // //笛卡尔空间下的路径规划  走直线
    // geometry_msgs::Pose start_pose = arm.getCurrentPose(end_effector_link).pose;
    // std::vector<geometry_msgs::Pose> waypoints;
    // //将初始位姿加入路点列表
	// waypoints.push_back(start_pose);
    // start_pose.position.z -= 0.2;
	// waypoints.push_back(start_pose);
    // start_pose.position.x -= 0.2;
	// waypoints.push_back(start_pose);
    // start_pose.position.y -= 0.2;
	// waypoints.push_back(start_pose);
    // start_pose.position.x += 0.2;
    // // start_pose.position.y += 0.2;
	// waypoints.push_back(start_pose);
    // start_pose.position.y += 0.2;
	// waypoints.push_back(start_pose);
    // start_pose.position.z += 0.2;
	// waypoints.push_back(start_pose);
    
    // moveit_msgs::RobotTrajectory trajectory;
	// const double jump_threshold = 0.0;
	// const double eef_step = 0.01;
    // double fraction = 0.0;
    // int maxtries = 30;   //最大尝试规划次数
    // int attempts = 0;     //已经尝试规划次数
    

    // while(fraction < 1.0 && attempts < maxtries)
    // {
    //     fraction = arm.computeCartesianPath(waypoints, eef_step, trajectory);
    //     attempts++;
        
    //     if(attempts % 10 == 0)
    //         ROS_INFO("Still trying after %d attempts...", attempts);
    // }
    
    // if(fraction == 1)
    // {   
    //     ROS_INFO("Path computed successfully. Moving the arm.");

	//     // 生成机械臂的运动规划数据
	//     moveit::planning_interface::MoveGroupInterface::Plan plan;
	//     plan.trajectory_ = trajectory;

    //     // //打印时间戳检查是否严格递增
    //     // for (const auto& point : plan.trajectory_.joint_trajectory.points)
    //     // {
    //     //     ROS_INFO("Waypoint time_from_start: %f", point.time_from_start.toSec());
    //     // }
    //     // 检查并修正第一个和第二个时间戳
    //     if (plan.trajectory_.joint_trajectory.points.size() > 2 &&
    //         plan.trajectory_.joint_trajectory.points[0].time_from_start == plan.trajectory_.joint_trajectory.points[1].time_from_start)
    //     {
            
    //         // ROS_INFO("Change Waypoint time_from_start: %f", plan.trajectory_.joint_trajectory.points[2].time_from_start.toSec()/2.0);
    //         // plan.trajectory_.joint_trajectory.points[1].time_from_start = ros::Duration(0.03);
    //         plan.trajectory_.joint_trajectory.points[1].time_from_start = ros::Duration(plan.trajectory_.joint_trajectory.points[2].time_from_start.toSec()/2.0);
    //         // ROS_INFO("Change Waypoint time_from_start: %f", plan.trajectory_.joint_trajectory.points[1].time_from_start.toSec());
            
    //     }


	//     // 执行运动
	//     arm.execute(plan);
    //     sleep(1);
    // }
    // else
    // {
    //     ROS_INFO("Path planning failed with only %0.6f success after %d attempts.", fraction, maxtries);
    // }



    ros::shutdown(); 

    return 0;
}

    // // 控制机械臂先回到初始化位置
    // arm.setNamedTarget("home");
    // arm.move();
    // sleep(1);



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


    // // 控制机械臂先回到初始化位置
    // arm.setNamedTarget("home");
    // arm.move();
    // sleep(1);