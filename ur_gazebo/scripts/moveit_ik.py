#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Copyright 2019 Wuhan PS-Micro Technology Co., Itd.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rospy, sys
import moveit_commander
from geometry_msgs.msg import PoseStamped, Pose


class MoveItIkDemo:
    def __init__(self):
        # 初始化move_group的API
        moveit_commander.roscpp_initialize(sys.argv)
        
        # 初始化ROS节点
        rospy.init_node('moveit_ik_demo')
                
        # 初始化需要使用move group控制的机械臂中的arm group
        arm = moveit_commander.MoveGroupCommander('manipulator')
                
        # 获取终端link的名称
        end_effector_link = arm.get_end_effector_link()
                        
        # 设置目标位置所使用的参考坐标系
        reference_frame = 'base_link'
        arm.set_pose_reference_frame(reference_frame)
                
        # 当运动规划失败后，允许重新规划
        arm.allow_replanning(True)
        
        # 设置位置(单位：米)和姿态（单位：弧度）的允许误差
        arm.set_goal_position_tolerance(0.001)
        arm.set_goal_orientation_tolerance(0.01)
       
        # 设置允许的最大速度和加速度
        arm.set_max_acceleration_scaling_factor(0.5)
        arm.set_max_velocity_scaling_factor(0.5)

        # 控制机械臂先回到初始化位置
        arm.set_named_target('home')
        arm.go()
        rospy.sleep(1)





        # # 获取当前状态
        # current_state = arm.get_current_state()
        # print("current_state:",current_state)
        
        # 获取末端执行器的位姿
        end_effector_pose = arm.get_current_pose().pose
        
        # 打印末端执行器位姿
        # print("末端执行器位姿：",end_effector_pose)
        print("位置：", end_effector_pose.position)
        print("方向（四元数）：", end_effector_pose.orientation)


        # 设置机械臂的目标位置，使用六轴的位置数据进行描述（单位：弧度）
        joint_positions = [0.5, -0.5, 0.0, 0.0, 0.0, 0.0]
        arm.set_joint_value_target(joint_positions)
                 
        # 控制机械臂完成运动
        arm.go()
        rospy.sleep(1)

               
        # 设置机械臂工作空间中的目标位姿，位置使用x、y、z坐标描述，
        # 姿态使用四元数描述，基于base_link坐标系
        target_pose = PoseStamped()
        target_pose.header.frame_id = reference_frame
        target_pose.header.stamp = rospy.Time.now() 

        target_pose.pose.position.x = 0.8160727912063483
        target_pose.pose.position.y = 0.19150088321493414
        target_pose.pose.position.z = -0.01239730748779357
        target_pose.pose.orientation.x = -0.004188096880984646
        target_pose.pose.orientation.y = -0.7070941758507405
        target_pose.pose.orientation.z = -0.7070941139071663
        target_pose.pose.orientation.w = 0.004266192719153937
        # target_pose.pose.position.x = 0.2593
        # target_pose.pose.position.y = 0.0636
        # target_pose.pose.position.z = 0.1787
        # target_pose.pose.orientation.x = 0.70692
        # target_pose.pose.orientation.y = 0.0
        # target_pose.pose.orientation.z = 0.0
        # target_pose.pose.orientation.w = 0.70729
        
        # 设置机器臂当前的状态作为运动初始状态
        arm.set_start_state_to_current_state()
        
        # 设置机械臂终端运动的目标位姿
        arm.set_pose_target(target_pose, end_effector_link)
        
        # arm.set_planning_time(10.0)  # 设置规划时间为10秒

        # 规划运动路径
        traj = arm.plan()

        
        print(type(traj))  # 打印traj的类型
        print("打印traj数据")
        print(traj)        # 打印traj的内容
        
        # 按照规划的运动路径控制机械臂运动
        # arm.execute(traj)
        if traj is None:
            rospy.logerr("No plan found. Check if the request is valid and if the robot is properly configured.")
        else:
            arm.execute(traj)


        rospy.sleep(1)

        # 控制机械臂回到初始化位置
        arm.set_named_target('home')
        arm.go()

        # 关闭并退出moveit
        moveit_commander.roscpp_shutdown()
        moveit_commander.os._exit(0)

if __name__ == "__main__":
    MoveItIkDemo()

    
    
