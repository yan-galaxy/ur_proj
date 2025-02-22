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

#define DEG2RAD(x) ((x) * M_PI / 180.0)

int main(int argc, char **argv)
{
    ros::init(argc, argv, "moveit_fk_demo");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    moveit::planning_interface::MoveGroupInterface arm("manipulator");

    arm.setGoalJointTolerance(0.001);

    arm.setMaxAccelerationScalingFactor(0.2);
    arm.setMaxVelocityScalingFactor(0.02);

    // // 控制机械臂先回到初始化位置
    // arm.setNamedTarget("home");
    // arm.move();
    // sleep(1);
    

    // double targetPose[6] = {-2.356194, -2.09843, -1.2573, -2.9914, -1.4557, 3.1415};
    double targetPose[6] = {
        DEG2RAD(-135),  // shoulder_pan_joint    -135
        DEG2RAD(-105),  // shoulder_lift_joint   -90
        DEG2RAD(-120),  // elbow_joint           -90
        DEG2RAD(-135),  // wrist_1_joint         -180
        DEG2RAD(-90 ),  // wrist_2_joint         -90
        DEG2RAD(180  )  // wrist_3_joint         180
    };
    std::vector<double> joint_group_positions(6);
    joint_group_positions[0] = targetPose[0];
    joint_group_positions[1] = targetPose[1];
    joint_group_positions[2] = targetPose[2];
    joint_group_positions[3] = targetPose[3];
    joint_group_positions[4] = targetPose[4];
    joint_group_positions[5] = targetPose[5];

    arm.setJointValueTarget(joint_group_positions);
    arm.move();
    
    
    // 方法1：直接获取当前末端位姿
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

    ROS_INFO("Orientation(rpy in radians): [%.4f, %.4f, %.4f]", roll, pitch, yaw);
    ROS_INFO("Orientation(rpy in degrees): [%.4f, %.4f, %.4f]", 
            roll * 180/M_PI, 
            pitch * 180/M_PI, 
            yaw * 180/M_PI);

    // // 控制机械臂先回到初始化位置
    // arm.setNamedTarget("home");
    // arm.move();
    // sleep(1);

    ros::shutdown(); 

    return 0;
}

// 获取关节名称顺序
    // auto joint_names = arm.getJointNames();
    // ROS_INFO("Joint order:");
    // for(size_t i=0; i<joint_names.size(); ++i){
    //     ROS_INFO_STREAM(i << ": " << joint_names[i]);
    // }
    // 输出结果（基于标准UR5）：
    // 0: shoulder_pan_joint
    // 1: shoulder_lift_joint
    // 2: elbow_joint
    // 3: wrist_1_joint
    // 4: wrist_2_joint
    // 5: wrist_3_joint