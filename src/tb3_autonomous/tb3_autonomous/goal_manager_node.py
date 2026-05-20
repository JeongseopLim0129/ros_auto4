#!/usr/bin/env python3
"""
goal_manager_node.py  [개선본 v2]

[개선 목록]
  BUG FIX  /goal_pose → /goal_pose_forwarded (무한 피드백 루프 제거) [기존 유지]
  BUG FIX  직접 goal 수신 시 waypoints 가 초기화되는 문제 수정
           → wp_nav 모드와 단일 goal 모드 분리 (_single_goal_mode 플래그)
  IMPROVE  /goal_reached (Bool) 구독으로 상태 감지
           기존 문자열 파싱('/controller_status 에서 목표 도달 검사)은 취약성 보조 수단으로 유지
  IMPROVE  _monitor 에서 pose 기반 goal 도달 체크 추가 (controller 장애 시 백업)
"""
import json, math, os
from typing import List, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger


class GoalManagerNode(Node):
    def __init__(self):
        super().__init__('goal_manager')
        self.declare_parameter('goal_timeout',      60.0)
        self.declare_parameter('waypoint_file',     '')
        self.declare_parameter('auto_start_wp',     False)
        self.declare_parameter('loop_waypoints',    False)
        self.declare_parameter('reached_tolerance', 0.20)

        self.goal_timeout = self.get_parameter('goal_timeout').value
        self.wp_file      = self.get_parameter('waypoint_file').value
        self.auto_start   = self.get_parameter('auto_start_wp').value
        self.loop_wp      = self.get_parameter('loop_waypoints').value
        self.reached_tol  = self.get_parameter('reached_tolerance').value

        self.current_pose:    Optional[PoseStamped] = None
        self.active_goal:     Optional[PoseStamped] = None
        self.waypoints:       List[PoseStamped]     = []
        self.wp_index:        int   = 0
        self.navigating:      bool  = False
        self._goal_start_t:   float = 0.0
        # [NEW] 단일 goal 모드 플래그: True 이면 waypoints 를 건드리지 않음
        self._single_goal_mode: bool = False

        # ── Subscriptions ──────────────────────────────────────────────────
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self._goal_cb, 10)

        amcl_qos = QoSProfile(depth=5,
                               durability=DurabilityPolicy.VOLATILE,
                               reliability=ReliabilityPolicy.BEST_EFFORT)
        self.amcl_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._amcl_cb, amcl_qos)

        # [NEW] Bool 전용 토픽 (primary) — 문자열 파싱 취약성 제거
        self.reached_sub = self.create_subscription(
            Bool, '/goal_reached', self._reached_bool_cb, 10)

        # 기존 문자열 상태 구독은 보조 수단으로 유지
        self.status_sub = self.create_subscription(
            String, '/controller_status', self._ctrl_status_cb, 10)

        # ── Publishers / Services ──────────────────────────────────────────
        self.pose_pub   = self.create_publisher(PoseStamped, '/current_pose',        10)
        self.fwd_pub    = self.create_publisher(PoseStamped, '/goal_pose_forwarded', 10)
        self.status_pub = self.create_publisher(String,      '/manager_status',      10)

        self.cancel_srv   = self.create_service(
            Trigger, '/cancel_navigation',  self._cancel_cb)
        self.start_wp_srv = self.create_service(
            Trigger, '/start_waypoint_nav', self._start_wp_cb)
        self.monitor_timer = self.create_timer(1.0, self._monitor)

        if self.wp_file:
            self._load_waypoints(self.wp_file)
        if self.auto_start and self.waypoints:
            self._send_next_waypoint()

        self.get_logger().info('GoalManagerNode ready')

    # ── AMCL / Pose ───────────────────────────────────────────────────────────

    def _amcl_cb(self, msg: PoseWithCovarianceStamped):
        ps           = PoseStamped()
        ps.header    = msg.header
        ps.pose      = msg.pose.pose
        self.current_pose = ps
        self.pose_pub.publish(ps)

    # ── Goal callbacks ────────────────────────────────────────────────────────

    def _goal_cb(self, msg: PoseStamped):
        """RViz 등에서 단일 목표점 수신.
        [FIX] 기존 코드는 여기서 self.waypoints = [] 로 초기화해
              waypoint_nav 중에 RViz 클릭 한 번으로 전체 순서가 사라지는 문제가 있었음.
              단일 goal 모드로 분기하여 waypoints 를 보호."""
        self.get_logger().info(
            f'Single goal received: '
            f'({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self._single_goal_mode = True
        self.navigating        = True
        self.active_goal       = msg
        self._goal_start_t     = self.get_clock().now().nanoseconds * 1e-9
        self.fwd_pub.publish(msg)
        self._publish_status(
            f'Goal set: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def _reached_bool_cb(self, msg: Bool):
        """[NEW] /goal_reached Bool 토픽 수신 (primary 도달 감지)."""
        if msg.data and self.navigating:
            self._on_goal_reached()

    def _ctrl_status_cb(self, msg: String):
        """기존 문자열 파싱 — 보조 수단으로 유지."""
        if '목표 도달' in msg.data and self.navigating:
            self._on_goal_reached()

    def _on_goal_reached(self):
        """goal 도달 공통 처리."""
        if not self.navigating:
            return
        self.get_logger().info('Goal reached confirmed')
        self.navigating  = False
        self.active_goal = None
        self._publish_status('Goal reached – idle')

        if self._single_goal_mode:
            # 단일 goal 모드: waypoint 시퀀스 건드리지 않음
            self._single_goal_mode = False
            return

        # waypoint 시퀀스 모드: 다음 waypoint 전송
        if self.wp_index < len(self.waypoints):
            self._send_next_waypoint()
        elif self.loop_wp and self.waypoints:
            self.wp_index = 0
            self._send_next_waypoint()

    # ── Services ──────────────────────────────────────────────────────────────

    def _cancel_cb(self, request, response):
        self.navigating        = False
        self.active_goal       = None
        self.waypoints         = []
        self.wp_index          = 0
        self._single_goal_mode = False
        self._publish_status('Navigation cancelled')
        response.success = True
        response.message = 'Cancelled'
        return response

    def _start_wp_cb(self, request, response):
        if not self.waypoints:
            response.success = False
            response.message = 'No waypoints loaded'
            return response
        self.wp_index          = 0
        self._single_goal_mode = False
        self._send_next_waypoint()
        response.success = True
        response.message = f'{len(self.waypoints)} waypoints started'
        return response

    # ── Waypoint helpers ──────────────────────────────────────────────────────

    def _send_next_waypoint(self):
        if self.wp_index >= len(self.waypoints):
            self.get_logger().info('All waypoints done')
            self._publish_status('Waypoint navigation complete')
            return
        goal = self.waypoints[self.wp_index]
        self.wp_index += 1
        self.get_logger().info(
            f'Waypoint {self.wp_index}/{len(self.waypoints)}')
        self.navigating        = True
        self.active_goal       = goal
        self._single_goal_mode = False
        self._goal_start_t     = self.get_clock().now().nanoseconds * 1e-9
        goal.header.stamp      = self.get_clock().now().to_msg()
        self.fwd_pub.publish(goal)

    def _load_waypoints(self, filepath: str):
        if not os.path.exists(filepath):
            self.get_logger().warn(f'Waypoint file not found: {filepath}')
            return
        try:
            with open(filepath, 'r') as f:
                data = json.load(f)
            self.waypoints = []
            for item in data:
                ps                     = PoseStamped()
                ps.header.frame_id     = 'map'
                ps.pose.position.x     = float(item['x'])
                ps.pose.position.y     = float(item['y'])
                yaw                    = float(item.get('yaw', 0.0))
                ps.pose.orientation.z  = math.sin(yaw / 2)
                ps.pose.orientation.w  = math.cos(yaw / 2)
                self.waypoints.append(ps)
            self.get_logger().info(
                f'Loaded {len(self.waypoints)} waypoints: {filepath}')
        except Exception as e:
            self.get_logger().error(f'Failed to load waypoints: {e}')

    # ── Monitor ───────────────────────────────────────────────────────────────

    def _monitor(self):
        """타임아웃 감시 + [NEW] pose 기반 goal 도달 백업 체크."""
        if not self.navigating or self.active_goal is None:
            return

        now     = self.get_clock().now().nanoseconds * 1e-9
        elapsed = now - self._goal_start_t

        # 타임아웃
        if elapsed > self.goal_timeout:
            self.get_logger().warn(f'Goal timeout ({elapsed:.0f}s)')
            self.navigating  = False
            self.active_goal = None
            self._publish_status(f'Timeout ({elapsed:.0f}s)')
            return

        # [NEW] pose 기반 도달 백업: controller 장애 시에도 다음 waypoint 진행
        if self.current_pose is not None:
            gx = self.active_goal.pose.position.x
            gy = self.active_goal.pose.position.y
            cx = self.current_pose.pose.position.x
            cy = self.current_pose.pose.position.y
            if math.hypot(gx - cx, gy - cy) < self.reached_tol:
                self.get_logger().info('Goal reached (monitor pose check)')
                self._on_goal_reached()

    def _publish_status(self, msg: str):
        self.status_pub.publish(String(data=msg))


def main(args=None):
    rclpy.init(args=args)
    node = GoalManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
