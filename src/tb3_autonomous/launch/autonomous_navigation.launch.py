#!/usr/bin/env python3
"""
autonomous_navigation.launch.py  [v2 - BUG FIX]

[수정 내역]
  v1 BUG FIX  namespace 인자를 모든 Node 에 전달 안 하던 문제 수정
  v2 BUG FIX  goal_manager / global_planner / local_controller 가
              params_file(nav_params.yaml)을 전혀 로드하지 않아
              nav_params.yaml 수정이 무시되던 문제 수정.
              - 인라인 파라미터 딕셔너리를 params_file 로 교체
              - use_sim_time 만 딕셔너리로 명시적 오버라이드 (params_file 우선 적용 후)
              - 결과: v3 가속도 리미터 / PD 게인 / dyn_replan_interval 등이
                      실제로 적용됨

사용 예:
  ros2 launch tb3_autonomous autonomous_navigation.launch.py use_sim:=true
  ros2 launch tb3_autonomous autonomous_navigation.launch.py use_sim:=false
  ros2 launch tb3_autonomous autonomous_navigation.launch.py namespace:=robot1
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('tb3_autonomous')

    # ── Launch Arguments ──────────────────────────────────────────────────────
    use_sim     = LaunchConfiguration('use_sim')
    map_file    = LaunchConfiguration('map_file')
    params_file = LaunchConfiguration('params_file')
    namespace   = LaunchConfiguration('namespace')

    declare_use_sim = DeclareLaunchArgument(
        'use_sim', default_value='false',
        description='가제보 시뮬레이션 사용 여부 (true/false)')
    declare_map = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(pkg_dir, 'maps', 'my_map.yaml'),
        description='맵 파일 경로 (.yaml)')
    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg_dir, 'config', 'nav_params.yaml'),
        description='파라미터 파일 경로')
    declare_ns = DeclareLaunchArgument(
        'namespace', default_value='',
        description='로봇 네임스페이스 (멀티로봇 시 robot1, robot2 등)')

    # ── ① Nav2 map_server ─────────────────────────────────────────────────────
    map_server_node = Node(
        package    = 'nav2_map_server',
        executable = 'map_server',
        name       = 'map_server',
        namespace  = namespace,
        output     = 'screen',
        parameters = [
            params_file,
            {'yaml_filename': map_file},
            {'use_sim_time':  use_sim},
        ],
    )

    # ── ② Nav2 AMCL ──────────────────────────────────────────────────────────
    amcl_node = Node(
        package    = 'nav2_amcl',
        executable = 'amcl',
        name       = 'amcl',
        namespace  = namespace,
        output     = 'screen',
        parameters = [
            params_file,
            {'use_sim_time': use_sim},
        ],
    )

    # ── ③ Nav2 Lifecycle Manager ──────────────────────────────────────────────
    lifecycle_manager_node = Node(
        package    = 'nav2_lifecycle_manager',
        executable = 'lifecycle_manager',
        name       = 'lifecycle_manager_localization',
        namespace  = namespace,
        output     = 'screen',
        parameters = [
            params_file,
            {
                'use_sim_time': use_sim,
                'autostart':    True,
                'node_names':   ['map_server', 'amcl'],
                'bond_timeout': 4.0,
            },
        ],
    )

    # ── ④ Goal Manager ────────────────────────────────────────────────────────
    # [FIX v2] params_file 을 첫 번째 파라미터로 전달.
    #          use_sim_time 은 두 번째 딕셔너리로 오버라이드.
    goal_manager_node = Node(
        package    = 'tb3_autonomous',
        executable = 'goal_manager',
        name       = 'goal_manager',
        namespace  = namespace,
        output     = 'screen',
        parameters = [
            params_file,
            {'use_sim_time': use_sim},
        ],
    )

    # ── ⑤ Global Planner ─────────────────────────────────────────────────────
    # [FIX v2] params_file 을 첫 번째 파라미터로 전달.
    #          dyn_replan_interval 등 nav_params.yaml 값이 실제 적용됨.
    global_planner_node = Node(
        package    = 'tb3_autonomous',
        executable = 'global_planner',
        name       = 'global_planner',
        namespace  = namespace,
        output     = 'screen',
        parameters = [
            params_file,
            {'use_sim_time': use_sim},
        ],
    )

    # ── ⑥ Local Controller ────────────────────────────────────────────────────
    # [FIX v2] params_file 을 첫 번째 파라미터로 전달.
    #          max_linear_acc / kp_angular / kd_angular 등 v3 파라미터가
    #          실제로 적용됨.
    local_controller_node = Node(
        package    = 'tb3_autonomous',
        executable = 'local_controller',
        name       = 'local_controller',
        namespace  = namespace,
        output     = 'screen',
        parameters = [
            params_file,
            {'use_sim_time': use_sim},
        ],
    )

    # ── RViz2 ─────────────────────────────────────────────────────────────────
    rviz_config = os.path.join(pkg_dir, 'config', 'navigation.rviz')
    rviz_node = Node(
        package    = 'rviz2',
        executable = 'rviz2',
        name       = 'rviz2',
        arguments  = ['-d', rviz_config] if os.path.exists(rviz_config) else [],
        output     = 'screen',
        condition  = IfCondition(use_sim),
    )

    return LaunchDescription([
        declare_use_sim,
        declare_map,
        declare_params,
        declare_ns,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
        goal_manager_node,
        global_planner_node,
        local_controller_node,
        rviz_node,
    ])
