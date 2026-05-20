#!/usr/bin/env python3
"""
local_controller_node.py  [주행 품질 개선본 v4]

[v3 → v4 수정 내역]
  BUG FIX 1  _send_cmd 에서 _prev_v / _prev_w 갱신 누락 수정
             - 기존: DANGER 정지 / recovery 직접 publish 후 _prev_v 가 이전 값 유지
                     → 정지 직후 다음 틱에서 가속도 리미터가 이전 속도 기준으로 계산
                     → 갑작스러운 재출발 / 잘못된 역방향 클램프 발생
             - 수정: _send_cmd 말단에서 항상 _prev_v, _prev_w 를 실제 publish 값으로 동기화

  BUG FIX 2  Recovery 회전 시 각속도 가속도 리미터 우회 수정
             - 기존: _run_recovery(RECOVERY_ROTATE) 에서 즉시 ±ROTATE_SPEED 명령
             - 수정: _apply_angular_acc 를 통해 부드러운 회전 시작

  IMPROVE    _find_closest 후방 탐색 창 확장 (5 → 20)
             - 좁은 공간 후진 / recovery 후 로봇이 경로 뒤로 밀렸을 때
               잘못된 closest 포인트 선택 방지
"""
import math
from typing import Optional, List, Tuple
import numpy as np
import rclpy
import rclpy.time
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
import tf2_ros

# ── 기본 파라미터 상수 ───────────────────────────────────────────────────────
MAX_LINEAR_VEL    = 0.22
MAX_ANGULAR_VEL   = 1.80
MIN_LINEAR_VEL    = 0.05
LOOKAHEAD_BASE    = 0.35
LOOKAHEAD_MIN     = 0.20
LOOKAHEAD_MAX     = 0.70
GOAL_TOLERANCE    = 0.20
DANGER_DIST       = 0.18
SLOW_DIST         = 0.25
MAX_LINEAR_ACC    = 0.5
MAX_ANGULAR_ACC   = 3.0
GOAL_DECEL_DIST   = 0.60
ROTATE_IN_PLACE_THR = math.radians(55)
KP_ANG            = 2.0
KD_ANG            = 0.3
VFH_SECTORS       = 72
VFH_THRESHOLD     = 0.28
STUCK_TIMEOUT     = 5.0
STUCK_DIST_THR    = 0.05
RECOVERY_NONE     = 0
RECOVERY_BACKUP   = 1
RECOVERY_ROTATE   = 2
BACKUP_SPEED      = -0.10
BACKUP_DURATION   = 1.5
ROTATE_SPEED      = 1.0
ROTATE_TOLERANCE  = 0.15
BACK_CLEAR_DIST   = 0.15
ROTATE_ONLY_DIST  = 0.50
STATUS_PUB_HZ     = 1.0


# ── VFH (Vector Field Histogram) ─────────────────────────────────────────────
class VFHAvoidance:
    """가장 넓은 연속 빈 섹터(밸리)에서 목표에 가장 가까운 방향 선택."""

    def __init__(self, n_sectors: int = VFH_SECTORS,
                 threshold: float = VFH_THRESHOLD):
        self.n            = n_sectors
        self.thr          = threshold
        self.sector_angle = 2 * math.pi / n_sectors

    @staticmethod
    def _circular_dist(a: int, b: int, n: int) -> int:
        d = abs(a - b) % n
        return min(d, n - d)

    def _find_valleys(self, free_mask: List[bool]) -> List[List[int]]:
        n = len(free_mask)
        if not any(free_mask):
            return []
        doubled = free_mask + free_mask
        valleys: List[List[int]] = []
        current: List[int] = []
        for i, f in enumerate(doubled):
            if f:
                current.append(i % n)
            else:
                if current:
                    valleys.append(current)
                    current = []
        if current:
            valleys.append(current)
        if (len(valleys) >= 2
                and free_mask[0]
                and free_mask[-1]):
            merged = valleys[-1] + valleys[0]
            valleys = valleys[1:-1] + [merged]
        seen: set = set()
        unique: List[List[int]] = []
        for v in valleys:
            key = tuple(sorted(set(v)))
            if key not in seen:
                seen.add(key)
                unique.append(list(dict.fromkeys(v)))
        return unique

    def compute(self, scan: LaserScan,
                goal_angle: float) -> Tuple[float, bool]:
        histogram = np.full(self.n, float('inf'))
        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and scan.range_min <= r <= scan.range_max:
                sector = int((angle % (2 * math.pi)) / self.sector_angle)
                sector = max(0, min(self.n - 1, sector))
                if r < histogram[sector]:
                    histogram[sector] = r
            angle += scan.angle_increment

        free_mask = [histogram[i] > self.thr for i in range(self.n)]
        if not any(free_mask):
            return goal_angle, True

        valleys = self._find_valleys(free_mask)
        if not valleys:
            return goal_angle, True

        goal_sector = int((goal_angle % (2 * math.pi)) / self.sector_angle)

        def valley_score(v: List[int]) -> Tuple[int, int]:
            center = v[len(v) // 2]
            return (-len(v), self._circular_dist(center, goal_sector, self.n))

        best_valley = min(valleys, key=valley_score)
        best_sector = min(best_valley,
                          key=lambda s: self._circular_dist(s, goal_sector, self.n))
        best_angle  = best_sector * self.sector_angle
        if best_angle > math.pi:
            best_angle -= 2 * math.pi
        return best_angle, False


# ── Pure Pursuit ─────────────────────────────────────────────────────────────
class PurePursuit:
    """lookahead circle 과 경로 선분의 교점을 직접 계산."""

    def __init__(self, lookahead: float = LOOKAHEAD_BASE):
        self.ld = lookahead

    def find_lookahead(self, path: List[Tuple[float, float]],
                       rx: float, ry: float,
                       closest_idx: int) -> Tuple[Optional[Tuple[float, float]], int]:
        best_point = None
        best_idx   = closest_idx

        for i in range(closest_idx, len(path) - 1):
            ax, ay = path[i]
            bx, by = path[i + 1]
            pt = self._circle_line_intersect(rx, ry, self.ld, ax, ay, bx, by)
            if pt is not None:
                best_point = pt
                best_idx   = i + 1

        if best_point is None:
            best_point = path[-1]
            best_idx   = len(path) - 1
        return best_point, best_idx

    @staticmethod
    def _circle_line_intersect(cx: float, cy: float, r: float,
                               ax: float, ay: float,
                               bx: float, by: float
                               ) -> Optional[Tuple[float, float]]:
        dx, dy = bx - ax, by - ay
        fx, fy = ax - cx, ay - cy
        a = dx * dx + dy * dy
        if a < 1e-10:
            return None
        b    = 2 * (fx * dx + fy * dy)
        c    = fx * fx + fy * fy - r * r
        disc = b * b - 4 * a * c
        if disc < 0:
            return None
        disc_sqrt = math.sqrt(disc)
        t1 = (-b - disc_sqrt) / (2 * a)
        t2 = (-b + disc_sqrt) / (2 * a)
        for t in (t2, t1):
            if 0.0 <= t <= 1.0:
                return ax + t * dx, ay + t * dy
        return None


# ── Local Controller ─────────────────────────────────────────────────────────
class LocalControllerNode(Node):
    def __init__(self):
        super().__init__('local_controller')
        self.declare_parameter('max_linear_vel',       MAX_LINEAR_VEL)
        self.declare_parameter('max_angular_vel',      MAX_ANGULAR_VEL)
        self.declare_parameter('lookahead_dist',       LOOKAHEAD_BASE)
        self.declare_parameter('goal_tolerance',       GOAL_TOLERANCE)
        self.declare_parameter('danger_dist',          DANGER_DIST)
        self.declare_parameter('slow_dist',            SLOW_DIST)
        self.declare_parameter('control_freq',         20.0)
        self.declare_parameter('robot_frame',          'base_footprint')
        self.declare_parameter('map_frame',            'map')
        self.declare_parameter('max_linear_acc',       MAX_LINEAR_ACC)
        self.declare_parameter('max_angular_acc',      MAX_ANGULAR_ACC)
        self.declare_parameter('goal_decel_dist',      GOAL_DECEL_DIST)
        self.declare_parameter('rotate_in_place_deg',  math.degrees(ROTATE_IN_PLACE_THR))
        self.declare_parameter('kp_angular',           KP_ANG)
        self.declare_parameter('kd_angular',           KD_ANG)

        self.max_v           = self.get_parameter('max_linear_vel').value
        self.max_w           = self.get_parameter('max_angular_vel').value
        self.ld              = self.get_parameter('lookahead_dist').value
        self.goal_tol        = self.get_parameter('goal_tolerance').value
        self.d_danger        = self.get_parameter('danger_dist').value
        self.d_slow          = self.get_parameter('slow_dist').value
        self.freq            = self.get_parameter('control_freq').value
        self.robot_frm       = self.get_parameter('robot_frame').value
        self.map_frm         = self.get_parameter('map_frame').value
        self.max_lin_acc     = self.get_parameter('max_linear_acc').value
        self.max_ang_acc     = self.get_parameter('max_angular_acc').value
        self.goal_decel_dist = self.get_parameter('goal_decel_dist').value
        self.rip_thr         = math.radians(
                                   self.get_parameter('rotate_in_place_deg').value)
        self.kp_ang          = self.get_parameter('kp_angular').value
        self.kd_ang          = self.get_parameter('kd_angular').value
        self.dt              = 1.0 / self.freq

        self.path: List[Tuple[float, float]] = []
        self.path_msg:    Optional[Path]       = None
        self.scan:        Optional[LaserScan]  = None
        self.robot_pose:  Optional[PoseStamped]= None
        self.closest_idx: int  = 0
        self.active:      bool = False
        self.goal_reached:bool = False

        self._prev_v: float = 0.0
        self._prev_w: float = 0.0
        self._prev_heading_err: float = 0.0

        self._prev_pose:           Optional[Tuple[float,float]] = None
        self._stuck_timer:         float = 0.0
        self._last_pos_chk:        float = self.get_clock().now().nanoseconds * 1e-9
        self._recovery_state:      int   = RECOVERY_NONE
        self._recovery_start:      float = 0.0
        self._recovery_target_yaw: float = 0.0

        self._scan_ranges: Optional[np.ndarray] = None
        self._scan_angles: Optional[np.ndarray] = None
        self._last_status_t:   float = 0.0
        self._status_interval: float = 1.0 / STATUS_PUB_HZ

        self.pp  = PurePursuit(lookahead=self.ld)
        self.vfh = VFHAvoidance()
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.path_sub = self.create_subscription(
            Path,        '/global_path',  self._path_cb, 10)
        self.scan_sub = self.create_subscription(
            LaserScan,   '/scan',         self._scan_cb, sensor_qos)
        self.pose_sub = self.create_subscription(
            PoseStamped, '/current_pose', self._pose_cb, 10)

        self.cmd_pub     = self.create_publisher(Twist,  '/cmd_vel',           10)
        self.status_pub  = self.create_publisher(String, '/controller_status', 10)
        self.reached_pub = self.create_publisher(Bool,   '/goal_reached',      10)
        self.replan_cli  = self.create_client(Trigger, '/replan')
        self.timer       = self.create_timer(self.dt, self._control_loop)
        self.get_logger().info(
            f'LocalControllerNode ready  '
            f'[acc={self.max_lin_acc}m/s² {self.max_ang_acc}rad/s²  '
            f'kp={self.kp_ang} kd={self.kd_ang}  '
            f'rip_thr={math.degrees(self.rip_thr):.0f}°]')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _path_cb(self, msg: Path):
        self.path_msg          = msg
        self.path              = [(p.pose.position.x, p.pose.position.y)
                                  for p in msg.poses]
        self.closest_idx       = 0
        self.active            = True
        self.goal_reached      = False
        self._stuck_timer      = 0.0
        self._recovery_state   = RECOVERY_NONE
        self._prev_v           = 0.0
        self._prev_w           = 0.0
        self._prev_heading_err = 0.0
        self.get_logger().info(f'New path: {len(self.path)} points')

    def _scan_cb(self, msg: LaserScan):
        self.scan         = msg
        self._scan_ranges = np.array(msg.ranges, dtype=np.float32)
        self._scan_angles = (np.arange(len(msg.ranges), dtype=np.float32)
                             * msg.angle_increment + msg.angle_min)

    def _pose_cb(self, msg: PoseStamped):
        self.robot_pose = msg

    # ── Control loop ──────────────────────────────────────────────────────────

   # local_controller_node.py 의 _control_loop 함수 내 수정

    def _control_loop(self):
        if not self.active or not self.path:
            return
        pose = self._get_robot_pose_tf()
        if pose is None: return
        rx, ry, rtheta = pose

        if self._recovery_state != RECOVERY_NONE:
            self._run_recovery(rx, ry, rtheta)
            return

        # 목표 지점(마지막 포인트) 정보
        gx, gy = self.path[-1]
        dist_to_goal = math.hypot(gx - rx, gy - ry)
        
        # 1. 위치(Distance) 도달 체크
        if dist_to_goal < self.goal_tol:
            # 최종 목표 방향(Orientation) 확인
            if self.path_msg and len(self.path_msg.poses) > 0:
                goal_q = self.path_msg.poses[-1].pose.orientation
                # 쿼터니언 -> Yaw 변환
                goal_yaw = math.atan2(2 * (goal_q.w * goal_q.z + goal_q.x * goal_q.y),
                                    1 - 2 * (goal_q.y**2 + goal_q.z**2))
                
                yaw_err = self._normalize_angle(goal_yaw - rtheta)
                
                # 2. 방향(Yaw) 오차 체크 (0.05 rad ≈ 2.8도 이내면 정지)
                if abs(yaw_err) > 0.05: 
                    # 선속도는 0, 각속도만 사용하여 제자리 회전
                    # 부드러운 회전을 위해 가속도 리미터 거치기
                    desired_w = float(np.clip(self.kp_ang * yaw_err, -0.8, 0.8))
                    smooth_w  = self._apply_angular_acc(desired_w)
                    self._send_cmd(0.0, smooth_w)
                    self._publish_status(f'정렬 중 | 남은각도: {math.degrees(yaw_err):.1f}°')
                    return
            
            # 위치와 방향 모두 만족하면 완전 정지
            self._send_cmd(0.0, 0.0)
            self.active = False
            self.goal_reached = True
            self.reached_pub.publish(Bool(data=True))
            self._publish_status('임무 완료', immediate=True)
            self.get_logger().info('Goal reached and orientation aligned!')
            return

        # (이하 Stuck 감지 및 일반 주행 로직 동일...)

        # Stuck 감지
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._prev_pose is not None:
            moved   = math.hypot(rx - self._prev_pose[0], ry - self._prev_pose[1])
            elapsed = now - self._last_pos_chk
            if elapsed >= 1.0:
                self._stuck_timer = (self._stuck_timer + elapsed
                                     if moved < STUCK_DIST_THR else 0.0)
                self._prev_pose    = (rx, ry)
                self._last_pos_chk = now
        else:
            self._prev_pose    = (rx, ry)
            self._last_pos_chk = now

        if self._stuck_timer >= STUCK_TIMEOUT:
            self.get_logger().warn('Stuck – starting recovery')
            self._stuck_timer = 0.0
            self._start_recovery(rx, ry, rtheta)
            return

        # 장애물 거리
        min_dist = self._min_front_dist()
        if min_dist < self.d_danger:
            self._send_cmd(0.0, 0.0)
            self._publish_status(f'DANGER {min_dist:.2f}m', immediate=True)
            return

        # 속도 스케일
        obs_scale = (
            max(0.1, min(1.0, (min_dist - self.d_danger) / (self.d_slow - self.d_danger)))
            if min_dist < self.d_slow else 1.0)
        goal_scale  = max(0.15, min(1.0, dist_to_goal / max(self.goal_decel_dist, 1e-3)))
        speed_scale = min(obs_scale, goal_scale)

        # Lookahead 포인트
        self.pp.ld   = float(np.clip(LOOKAHEAD_BASE * obs_scale,
                                     LOOKAHEAD_MIN, LOOKAHEAD_MAX))
        self.closest_idx = self._find_closest(rx, ry)
        lh, _ = self.pp.find_lookahead(self.path, rx, ry, self.closest_idx)
        if lh is None:
            self._send_cmd(0.0, 0.0)
            return

        goal_angle_world = math.atan2(lh[1] - ry, lh[0] - rx)
        goal_angle_robot = self._normalize_angle(goal_angle_world - rtheta)

        # VFH 장애물 회피 방향
        if self.scan is not None:
            avoidance_angle, blocked = self.vfh.compute(self.scan, goal_angle_robot)
            if blocked:
                self.get_logger().warn('VFH blocked – recovery')
                self._start_recovery(rx, ry, rtheta)
                return
        else:
            avoidance_angle = goal_angle_robot

        pp_angle = self._normalize_angle(goal_angle_world - rtheta)
        blend    = (1.0 - obs_scale) if min_dist < self.d_slow else 0.0
        heading_error = self._blend_angles(pp_angle, avoidance_angle, blend)

        # 큰 각도 오차: 제자리 회전 모드
        if abs(heading_error) > self.rip_thr:
            w_rip = float(np.clip(self.kp_ang * heading_error,
                                  -self.max_w, self.max_w))
            w_rip = self._apply_angular_acc(w_rip)
            self._send_cmd(0.0, w_rip)
            self._prev_heading_err = heading_error
            self._publish_status(
                f'Rotating | err:{math.degrees(heading_error):.1f}°')
            return

        # PD 각속도 제어
        d_err       = self._normalize_angle(heading_error - self._prev_heading_err) / self.dt
        angular_raw = self.kp_ang * heading_error + self.kd_ang * d_err
        angular_vel = float(np.clip(angular_raw, -self.max_w, self.max_w))
        self._prev_heading_err = heading_error

        # 선속도 계산
        curve_factor = 1.0 - min(1.0, abs(heading_error) / math.pi)
        linear_vel   = max(MIN_LINEAR_VEL, self.max_v * speed_scale * curve_factor)

        # 가속도 리미터 적용
        linear_vel  = self._apply_linear_acc(linear_vel)
        angular_vel = self._apply_angular_acc(angular_vel)

        self._send_cmd(linear_vel, angular_vel)
        self._publish_status(
            f'Running | goal:{dist_to_goal:.2f}m | '
            f'v:{linear_vel:.2f} w:{angular_vel:.2f} '
            f'scale:{speed_scale:.2f}')

    # ── 가속도 리미터 ─────────────────────────────────────────────────────────

    def _apply_linear_acc(self, desired_v: float) -> float:
        max_dv  = self.max_lin_acc * self.dt
        clamped = float(np.clip(desired_v,
                                self._prev_v - max_dv,
                                self._prev_v + max_dv))
        return clamped   # _prev_v 는 _send_cmd 에서 갱신 (BUG FIX 1)

    def _apply_angular_acc(self, desired_w: float) -> float:
        max_dw  = self.max_ang_acc * self.dt
        clamped = float(np.clip(desired_w,
                                self._prev_w - max_dw,
                                self._prev_w + max_dw))
        return clamped   # _prev_w 는 _send_cmd 에서 갱신 (BUG FIX 1)

    # ── Recovery ──────────────────────────────────────────────────────────────

    def _start_recovery(self, rx: float, ry: float, rtheta: float):
        front      = self._min_front_dist()
        back       = self._min_back_dist()
        target_yaw = self._path_start_yaw(rx, ry)
        if front >= ROTATE_ONLY_DIST:
            self._recovery_state = RECOVERY_ROTATE
        elif back < BACK_CLEAR_DIST:
            self._publish_status('Both sides blocked – replanning', immediate=True)
            self._request_replan()
            return
        else:
            self._recovery_state = RECOVERY_BACKUP
            self._publish_status('Recovery: backing up...', immediate=True)
        self._recovery_start      = self.get_clock().now().nanoseconds * 1e-9
        self._recovery_target_yaw = target_yaw
        self._prev_v = 0.0
        self._prev_w = 0.0

    def _run_recovery(self, rx: float, ry: float, rtheta: float):
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._recovery_state == RECOVERY_BACKUP:
            elapsed = now - self._recovery_start
            if elapsed >= BACKUP_DURATION or self._min_back_dist() < BACK_CLEAR_DIST:
                self._send_cmd(0.0, 0.0)
                self._recovery_state = RECOVERY_ROTATE
                self._recovery_start = now
                self._publish_status('Recovery: rotating...', immediate=True)
                return
        
            self._send_cmd(BACKUP_SPEED, 0.0)

        elif self._recovery_state == RECOVERY_ROTATE:
            yaw_err = self._normalize_angle(self._recovery_target_yaw - rtheta)
            timeout = now - self._recovery_start > 10.0
            if abs(yaw_err) < ROTATE_TOLERANCE or timeout:
                self._send_cmd(0.0, 0.0)
                self._recovery_state = RECOVERY_NONE
                self._stuck_timer    = 0.0
                self.active          = False
                self._request_replan()
                return
            # [BUG FIX 2] 가속도 리미터를 통해 부드럽게 회전 시작
            desired_w = ROTATE_SPEED if yaw_err > 0 else -ROTATE_SPEED
            smooth_w  = self._apply_angular_acc(desired_w)
            self._send_cmd(0.0, smooth_w)

    # ── Helpers ───────────────────────────────────────────────────────────────

    def _get_robot_pose_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frm, self.robot_frm, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1))
            q   = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y ** 2 + q.z ** 2))
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception:
            pass
        if self.robot_pose is not None:
            p   = self.robot_pose.pose.position
            o   = self.robot_pose.pose.orientation
            yaw = math.atan2(2 * (o.w * o.z + o.x * o.y),
                             1 - 2 * (o.y ** 2 + o.z ** 2))
            return p.x, p.y, yaw
        return None

    def _find_closest(self, rx: float, ry: float) -> int:
        # [IMPROVE] 후방 탐색 창 5 → 20: recovery 후 로봇이 경로 뒤로 밀린 경우에도
        #           올바른 closest 포인트 탐색 가능
        start = max(0, self.closest_idx - 20)
        end   = min(len(self.path), self.closest_idx + 30)
        best_idx, best_dist = self.closest_idx, float('inf')
        for i in range(start, end):
            d = math.hypot(self.path[i][0] - rx, self.path[i][1] - ry)
            if d < best_dist:
                best_dist, best_idx = d, i
        return best_idx

    def _min_front_dist(self) -> float:
        return self._min_dist_in_range(-60, 60)

    def _min_back_dist(self) -> float:
        return self._min_dist_in_range(150, 180, also_neg=True)

    def _min_dist_in_range(self, deg_min: float, deg_max: float,
                            also_neg: bool = False) -> float:
        if self._scan_ranges is None or self._scan_angles is None:
            return float('inf')
        scan   = self.scan
        ranges = self._scan_ranges
        angles = self._scan_angles
        mask   = ((angles >= math.radians(deg_min))
                  & (angles <= math.radians(deg_max)))
        if also_neg:
            mask |= ((angles >= math.radians(-deg_max))
                     & (angles <= math.radians(-deg_min)))
        valid = ranges[mask]
        valid = valid[(valid >= scan.range_min)
                      & (valid <= scan.range_max)
                      & np.isfinite(valid)]
        return float(np.min(valid)) if len(valid) > 0 else float('inf')

    def _path_start_yaw(self, rx: float, ry: float) -> float:
        pose = self._get_robot_pose_tf()
        if not self.path:
            return pose[2] if pose else 0.0
        for px, py in self.path:
            if math.hypot(px - rx, py - ry) >= LOOKAHEAD_BASE * 0.5:
                return math.atan2(py - ry, px - rx)
        return math.atan2(self.path[-1][1] - ry, self.path[-1][0] - rx)

    def _blend_angles(self, a1: float, a2: float, w: float) -> float:
        return self._normalize_angle(a1 + w * self._normalize_angle(a2 - a1))

    def _normalize_angle(self, a: float) -> float:
        return (a + math.pi) % (2 * math.pi) - math.pi

    def _send_cmd(self, v: float, w: float):
        cmd           = Twist()
        cmd.linear.x  = float(np.clip(v, -self.max_v, self.max_v))
        cmd.angular.z = float(np.clip(w, -self.max_w, self.max_w))
        self.cmd_pub.publish(cmd)
        self._prev_v = cmd.linear.x
        self._prev_w = cmd.angular.z

    def _request_replan(self):
        if not self.replan_cli.service_is_ready():
            self.get_logger().warn('Replan service not ready')
            return
        future = self.replan_cli.call_async(Trigger.Request())
        future.add_done_callback(
            lambda f: self.get_logger().info('Replan response received'))

    def _publish_status(self, msg: str, immediate: bool = False):
        now = self.get_clock().now().nanoseconds * 1e-9
        if immediate or (now - self._last_status_t) >= self._status_interval:
            self.status_pub.publish(String(data=msg))
            self._last_status_t = now


def main(args=None):
    rclpy.init(args=args)
    node = LocalControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
