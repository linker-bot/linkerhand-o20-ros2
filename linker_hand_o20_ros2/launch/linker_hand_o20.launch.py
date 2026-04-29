#!/usr/bin/env python3 
# -*- coding: utf-8 -*-
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='linker_hand_o20_ros2',
            executable='linker_hand_o20',
            name='linker_hand_o20',
            output='screen',
            parameters=[{
                'hand_type': 'left', # 配置Linker Hand灵巧手类型 left | right 字母为小写
                'hand_joint': "O20", # O20 字母为大写
                'canfd_device': 0, # 配置CANFD设备编号 0 | 1 先插入的设备为0，后插入的设备为1。单CANFD设置为0即可
                'is_touch': True, # 配置Linker Hand灵巧手是否有压力传感器 True | False
            }],
        ),
    ])
