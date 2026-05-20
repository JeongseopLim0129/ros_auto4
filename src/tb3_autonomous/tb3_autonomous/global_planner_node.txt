#!/usr/bin/env python3
"""
global_planner_node.py  [개선본 v3]

[v2 → v3 수정 내역]
  BUG FIX   _smooth_path 충돌 복원이 lethal_cost 이상 셀만 체크하던 문제 수정
            - 기존: is_free(ngx, ngy)  → cost < lethal_cost 만 복원
                    스무딩으로 inflation zone(50~64)을 통과해도 복원 안 됨
                    → 벽 가까이 지나가는 경로 계획 위험
            - 수정: cell_cost > smooth_cost_threshold (기본: lethal_cost × 0.6)
                    이상이면 원위치 복원. 파라미터로 조정 가능.

  IMPROVE   _periodic_replan 에서 경로 유효성 먼저 검사 후 필요 시만 재계획
            - 기존: 5초마다 무조건 A* 재수행 → LaserScan 노이즈만으로도
                    경로가 미세하게 달라져 local_controller의 closest_idx
                    가 0으로 리셋되는 현상 발생
            - 수정: _path_has_collision() 으로 현재 경로가 동적 장애물과
                    교차하는지 먼저 확인. 충돌 없으면 재계획 건너뜀.

  IMPROVE   TF 실패 시 명확한 WARN 로그 추가
            - _get_robot_tf 실패 후 current_pose 폴백 시 로그 없이 조용히
              넘어가던 문제 수정. 디버깅 편의성 향상.
"""
import heapq, math, time
from typing import Optional, List, Tuple
import numpy as np
import rclpy
import rclpy.time
import rclpy.duration
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
import tf2_ros

try:
    from scipy.ndimage import distance_transform_edt
    _SCIPY = True
except ImportError:
    _SCIPY = False

LETHAL_COST          = 90
INFLATION_RADIUS     = 0.35
DYN_INFLATION_RADIUS = 0.35
DYN_LETHAL_COST      = 100
SCAN_MAX_RANGE       = 3.5
SMOOTH_MAX_ITER      = 500


class AStarPlanner:
    def __init__(self, grid: np.ndarray, width: int, height: int,
                 resolution: float, origin_x: float, origin_y: float,
                 lethal_cost: int = LETHAL_COST,
                 smooth_cost_threshold: int = None):
        self.grid     = grid
        self.width    = width
        self.height   = height
        self.res      = resolution
        self.origin_x = origin_x
        self.origin_y = origin_y
        self.lethal   = lethal_cost
        # [BUG FIX] 스무딩 복원 임계값: 기본값은 lethal_cost × 0.6
        self.smooth_thr = (smooth_cost_threshold
                           if smooth_cost_threshold is not None
                           else int(lethal_cost * 0.6))

    def world_to_grid(self, wx: float, wy: float) -> Tuple[int, int]:
        return (int((wx - self.origin_x) / self.res),
                int((wy - self.origin_y) / self.res))

    def grid_to_world(self, gx: int, gy: int) -> Tuple[float, float]:
        return (gx * self.res + self.origin_x + self.res / 2.0,
                gy * self.res + self.origin_y + self.res / 2.0)

    def in_bounds(self, gx: int, gy: int) -> bool:
        return 0 <= gx < self.width and 0 <= gy < self.height

    def cell_cost(self, gx: int, gy: int) -> int:
        return int(self.grid[gy, gx])

    def is_free(self, gx: int, gy: int) -> bool:
        return self.in_bounds(gx, gy) and self.cell_cost(gx, gy) < self.lethal

    def plan(self, start_world: Tuple[float, float],
             goal_world: Tuple[float, float]) -> Optional[List[Tuple[float, float]]]:
        sx, sy = self.world_to_grid(*start_world)
        gx, gy = self.world_to_grid(*goal_world)

        if not self.is_free(sx, sy):
            sx, sy = self._find_nearest_free(sx, sy)
            if sx is None:
                return None
        if not self.is_free(gx, gy):
            gx, gy = self._find_nearest_free(gx, gy)
            if gx is None:
                return None

        neighbors = [
            ( 1,  0, 1.0),   (-1,  0, 1.0),
            ( 0,  1, 1.0),   ( 0, -1, 1.0),
            ( 1,  1, 1.414), ( 1, -1, 1.414),
            (-1,  1, 1.414), (-1, -1, 1.414),
        ]
        open_set: list = []
        heapq.heappush(open_set, (0.0, (sx, sy)))
        came_from: dict = {}
        g_score: dict   = {(sx, sy): 0.0}
        visited: set    = set()

        while open_set:
            _, current = heapq.heappop(open_set)
            if current in visited:
                continue
            visited.add(current)
            cx, cy = current
            if (cx, cy) == (gx, gy):
                return self._reconstruct(came_from, (gx, gy))
            for dx, dy, mc in neighbors:
                nx, ny = cx + dx, cy + dy
                if not self.is_free(nx, ny) or (nx, ny) in visited:
                    continue
                extra = max(0, self.cell_cost(nx, ny)) * 0.20
                new_g = g_score[current] + mc + extra
                if new_g < g_score.get((nx, ny), float('inf')):
                    came_from[(nx, ny)] = current
                    g_score[(nx, ny)]   = new_g
                    f = new_g + self._heuristic(nx, ny, gx, gy)
                    heapq.heappush(open_set, (f, (nx, ny)))
        return None

    def _heuristic(self, x1: int, y1: int, x2: int, y2: int) -> float:
        dx, dy = abs(x1 - x2), abs(y1 - y2)
        return max(dx, dy) + (1.414 - 1.0) * min(dx, dy)

    def _reconstruct(self, came_from: dict,
                     current: Tuple[int, int]) -> List[Tuple[float, float]]:
        path = []
        while current in came_from:
            path.append(self.grid_to_world(*current))
            current = came_from[current]
        path.append(self.grid_to_world(*current))
        path.reverse()
        return self._smooth_path(path)

    def _smooth_path(self, path: list,
                     weight_data: float = 0.5,
                     weight_smooth: float = 0.3,
                     tolerance: float = 0.001) -> List[Tuple[float, float]]:
        """경로 스무딩. SMOOTH_MAX_ITER 상한 + inflation zone 복원 포함."""
        if len(path) <= 2:
            return path
        new_path  = [list(p) for p in path]
        change    = tolerance + 1
        iteration = 0
        while change > tolerance and iteration < SMOOTH_MAX_ITER:
            change     = 0.0
            iteration += 1
            for i in range(1, len(new_path) - 1):
                for j in range(2):
                    orig = new_path[i][j]
                    new_path[i][j] += (
                        weight_data   * (path[i][j] - new_path[i][j]) +
                        weight_smooth * (new_path[i - 1][j] + new_path[i + 1][j]
                                         - 2.0 * new_path[i][j]))
                    change += abs(orig - new_path[i][j])

        # [BUG FIX] 스무딩 후 충돌 복원:
        #   기존: is_free (lethal_cost 이상만 체크) → inflation zone 통과해도 복원 안 됨
        #   수정: cell_cost > smooth_thr (= lethal_cost × 0.6 ≈ 39) 이면 원위치 복원
        #         → 벽에 너무 가까운 경로점도 안전하게 원위치로 되돌림
        for i in range(1, len(new_path) - 1):
            ngx = int((new_path[i][0] - self.origin_x) / self.res)
            ngy = int((new_path[i][1] - self.origin_y) / self.res)
            if not self.in_bounds(ngx, ngy):
                new_path[i] = list(path[i])
                continue
            if self.cell_cost(ngx, ngy) > self.smooth_thr:
                new_path[i] = list(path[i])

        return [tuple(p) for p in new_path]

    def _find_nearest_free(self, gx: int, gy: int,
                           max_radius: int = 10) -> Tuple[Optional[int], Optional[int]]:
        for r in range(1, max_radius):
            for dx in range(-r, r + 1):
                for dy in range(-r, r + 1):
                    if abs(dx) == r or abs(dy) == r:
                        nx, ny = gx + dx, gy + dy
                        if self.is_free(nx, ny):
                            return nx, ny
        return None, None


class GlobalPlannerNode(Node):
    def __init__(self):
        super().__init__('global_planner')
        self.declare_parameter('inflation_radius',        INFLATION_RADIUS)
        self.declare_parameter('dyn_inflation_radius',    DYN_INFLATION_RADIUS)
        self.declare_parameter('lethal_cost',             LETHAL_COST)
        self.declare_parameter('path_frame_id',           'map')
        self.declare_parameter('waypoint_spacing',        0.1)
        self.declare_parameter('scan_topic',              '/scan')
        self.declare_parameter('scan_max_range',          SCAN_MAX_RANGE)
        self.declare_parameter('use_dynamic_obstacles',   True)
        self.declare_parameter('dyn_replan_interval',     5.0)
        # [BUG FIX] 스무딩 복원 임계값 파라미터 (lethal_cost 의 비율, 0.0~1.0)
        self.declare_parameter('smooth_cost_ratio',       0.6)

        self.inflation_radius  = self.get_parameter('inflation_radius').value
        self.dyn_inflation_r   = self.get_parameter('dyn_inflation_radius').value
        self.lethal_cost       = self.get_parameter('lethal_cost').value
        self.path_frame_id     = self.get_parameter('path_frame_id').value
        self.waypoint_spacing  = self.get_parameter('waypoint_spacing').value
        self.scan_max_range    = self.get_parameter('scan_max_range').value
        self.use_dynamic       = self.get_parameter('use_dynamic_obstacles').value
        self.dyn_replan_ivl    = self.get_parameter('dyn_replan_interval').value
        smooth_ratio           = self.get_parameter('smooth_cost_ratio').value
        self.smooth_cost_thr   = int(self.lethal_cost * smooth_ratio)

        self.map_msg:             Optional[OccupancyGrid] = None
        self.current_goal:        Optional[PoseStamped]   = None
        self.current_pose:        Optional[PoseStamped]   = None
        self.static_inflated_arr: Optional[np.ndarray]    = None
        self.latest_scan:         Optional[LaserScan]     = None
        self._navigating:         bool                    = False
        # [IMPROVE] 마지막으로 publish 한 경로 (유효성 검사용)
        self._last_path:          Optional[List[Tuple[float, float]]] = None

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        map_qos    = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        sensor_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.map_sub  = self.create_subscription(OccupancyGrid, '/map',
                            self._map_cb, map_qos)
        self.goal_sub = self.create_subscription(PoseStamped,
                            '/goal_pose_forwarded', self._goal_cb, 10)
        self.pose_sub = self.create_subscription(PoseStamped,
                            '/current_pose', self._pose_cb, 10)
        self.scan_sub = self.create_subscription(
                            LaserScan,
                            self.get_parameter('scan_topic').value,
                            self._scan_cb, sensor_qos)

        self.path_pub    = self.create_publisher(Path,          '/global_path',     10)
        self.status_pub  = self.create_publisher(String,        '/planning_status', 10)
        self.costmap_pub = self.create_publisher(OccupancyGrid, '/dynamic_costmap', 10)
        self.replan_srv  = self.create_service(Trigger, '/replan', self._replan_cb)

        if self.dyn_replan_ivl > 0.0:
            self.create_timer(self.dyn_replan_ivl, self._periodic_replan)

        if not _SCIPY:
            self.get_logger().warn(
                'scipy not found – dynamic inflation uses slow Python loop. '
                'pip install scipy')
        self.get_logger().info(
            f'GlobalPlannerNode ready '
            f'(dynamic: {"ON" if self.use_dynamic else "OFF"}, '
            f'replan_interval: {self.dyn_replan_ivl:.1f}s, '
            f'smooth_cost_thr: {self.smooth_cost_thr})')

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _map_cb(self, msg: OccupancyGrid):
        self.map_msg             = msg
        self.static_inflated_arr = self._inflate_static(msg)
        self.get_logger().info(
            f'Map received: {msg.info.width}x{msg.info.height} '
            f'res={msg.info.resolution:.3f}m')
        if self.current_goal is not None:
            self._do_plan()

    def _goal_cb(self, msg: PoseStamped):
        self.get_logger().info(
            f'Goal received: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')
        self.current_goal = msg
        self._navigating  = True
        self._last_path   = None
        self._do_plan()

    def _pose_cb(self, msg: PoseStamped):
        self.current_pose = msg

    def _scan_cb(self, msg: LaserScan):
        self.latest_scan = msg

    def _replan_cb(self, request, response):
        if self.current_goal is None:
            response.success = False
            response.message = 'No goal'
        else:
            ok = self._do_plan()
            response.success = ok
            response.message = 'Replan success' if ok else 'Replan failed'
        return response

    def _periodic_replan(self):
        """[IMPROVE] 주기적 재경로 계획 — 경로 유효성 먼저 확인.
        현재 경로가 동적 장애물과 충돌하지 않으면 재계획을 건너뜀.
        충돌 감지 시에만 A* 재수행 → LaserScan 노이즈로 인한
        불필요한 경로 교체 및 closest_idx 리셋 방지."""
        if not self._navigating or self.current_goal is None:
            return

        if self._last_path is not None:
            combined = self._build_combined_costmap()
            if not self._path_has_collision(self._last_path, combined):
                self.get_logger().debug(
                    'Periodic replan skipped – current path is clear')
                return
            self.get_logger().info(
                'Periodic replan triggered – collision detected on current path')
        else:
            self.get_logger().debug('Periodic replan: no existing path, planning now')

        self._do_plan()

    # ── Planning ──────────────────────────────────────────────────────────────

    def _do_plan(self) -> bool:
        if self.map_msg is None or self.static_inflated_arr is None:
            self._publish_status('No map – waiting')
            return False
        if self.current_goal is None:
            return False
        start = self._get_robot_pose()
        if start is None:
            self._publish_status('Unknown robot pose')
            return False

        combined_arr = self._build_combined_costmap()
        info = self.map_msg.info
        planner = AStarPlanner(
            grid                  = combined_arr,
            width                 = info.width,
            height                = info.height,
            resolution            = info.resolution,
            origin_x              = info.origin.position.x,
            origin_y              = info.origin.position.y,
            lethal_cost           = self.lethal_cost,
            smooth_cost_threshold = self.smooth_cost_thr,
        )
        t0 = time.time()
        wp = planner.plan(
            (start[0], start[1]),
            (self.current_goal.pose.position.x,
             self.current_goal.pose.position.y),
        )
        elapsed = time.time() - t0

        if wp is None:
            self.get_logger().warn('No path found!')
            self._publish_status('No path')
            return False

        self._last_path = wp
        self.path_pub.publish(self._waypoints_to_path(wp))
        self._publish_status(
            f'Path published ({len(wp)}pts, {elapsed * 1000:.1f}ms)'
            f' [dyn: {"on" if self.use_dynamic else "off"}]')
        self.get_logger().info(f'Path planned: {len(wp)}pts / {elapsed * 1000:.1f}ms')
        return True

    # ── Path collision check ──────────────────────────────────────────────────

    def _path_has_collision(self, path: List[Tuple[float, float]],
                            costmap: np.ndarray) -> bool:
        """현재 경로 포인트 중 lethal_cost 이상인 셀을 지나는지 검사.
        샘플링(5포인트마다)으로 계산 부하 경감."""
        if self.map_msg is None:
            return False
        info = self.map_msg.info
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y
        h, w = costmap.shape
        for i, (px, py) in enumerate(path):
            if i % 5 != 0:   # 5포인트마다 샘플링
                continue
            gx = int((px - ox) / res)
            gy = int((py - oy) / res)
            if not (0 <= gx < w and 0 <= gy < h):
                continue
            if int(costmap[gy, gx]) >= self.smooth_cost_thr:
                return True
        return False

    # ── Costmap ───────────────────────────────────────────────────────────────

    def _build_combined_costmap(self) -> np.ndarray:
        combined = self.static_inflated_arr.copy()

        if not self.use_dynamic or self.latest_scan is None:
            return combined

        robot_tf = self._get_robot_tf()
        if robot_tf is None:
            return combined

        rx, ry, ryaw = robot_tf
        scan = self.latest_scan
        info = self.map_msg.info
        w, h = info.width, info.height
        res  = info.resolution
        ox   = info.origin.position.x
        oy   = info.origin.position.y

        angles = (np.arange(len(scan.ranges), dtype=np.float32)
                  * scan.angle_increment + scan.angle_min)
        ranges = np.array(scan.ranges, dtype=np.float32)
        valid  = (np.isfinite(ranges)
                  & (ranges >= scan.range_min)
                  & (ranges <= min(scan.range_max, self.scan_max_range)))

        if not np.any(valid):
            return combined

        r_v = ranges[valid]
        a_v = angles[valid]
        lx  = r_v * np.cos(a_v)
        ly  = r_v * np.sin(a_v)
        cos_r, sin_r = math.cos(ryaw), math.sin(ryaw)
        wx  = rx + lx * cos_r - ly * sin_r
        wy  = ry + lx * sin_r + ly * cos_r
        gx  = ((wx - ox) / res).astype(int)
        gy  = ((wy - oy) / res).astype(int)

        in_map = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
        gx, gy = gx[in_map], gy[in_map]

        if len(gx) == 0:
            return combined

        if _SCIPY:
            dyn_mask = np.zeros((h, w), dtype=bool)
            dyn_mask[gy, gx] = True
            dist = distance_transform_edt(~dyn_mask) * res
            cost_arr = np.where(
                dyn_mask,
                DYN_LETHAL_COST,
                np.where(dist < self.dyn_inflation_r,
                         DYN_LETHAL_COST * (1.0 - dist / self.dyn_inflation_r),
                         0),
            ).astype(np.int16)
            combined = np.maximum(combined, cost_arr)
        else:
            dyn_r = int(math.ceil(self.dyn_inflation_r / res))
            for ogx, ogy in zip(gx.tolist(), gy.tolist()):
                for dy in range(-dyn_r, dyn_r + 1):
                    for dx in range(-dyn_r, dyn_r + 1):
                        nx, ny = ogx + dx, ogy + dy
                        if not (0 <= nx < w and 0 <= ny < h):
                            continue
                        dist_m = math.hypot(dx, dy) * res
                        if dist_m > self.dyn_inflation_r:
                            continue
                        cost = int(DYN_LETHAL_COST * (1.0 - dist_m / self.dyn_inflation_r))
                        if combined[ny, nx] < cost:
                            combined[ny, nx] = cost

        self._publish_dynamic_costmap(combined)
        self.get_logger().debug(
            f'Dynamic obstacles: {len(gx)} cells (inflate {self.dyn_inflation_r}m)')
        return combined

    def _inflate_static(self, map_msg: OccupancyGrid) -> np.ndarray:
        info = map_msg.info
        w, h = info.width, info.height
        res  = info.resolution
        data = np.array(map_msg.data, dtype=np.int16).reshape(h, w)

        if _SCIPY:
            obstacle_mask = (data >= self.lethal_cost) | (data == -1)
            dist_m        = distance_transform_edt(~obstacle_mask) * res
            ratio         = np.where(
                obstacle_mask, 1.0,
                np.where(dist_m < self.inflation_radius,
                         1.0 - dist_m / self.inflation_radius,
                         0.0))
            return np.where(obstacle_mask, 100,
                            (100 * ratio)).astype(np.int16)

        radius   = int(math.ceil(self.inflation_radius / res))
        inflated = data.copy()
        for gy in range(h):
            for gx in range(w):
                v = int(data[gy, gx])
                if v >= self.lethal_cost or v == -1:
                    for dy in range(-radius, radius + 1):
                        for dx in range(-radius, radius + 1):
                            nx, ny = gx + dx, gy + dy
                            if not (0 <= nx < w and 0 <= ny < h):
                                continue
                            dist = math.hypot(dx, dy) * res
                            if dist <= self.inflation_radius:
                                cost = int(100 * (1.0 - dist / self.inflation_radius))
                                if inflated[ny, nx] < cost:
                                    inflated[ny, nx] = cost
        return inflated.astype(np.int16)

    # ── TF / Pose ─────────────────────────────────────────────────────────────

    def _get_robot_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map', 'base_footprint', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.3))
            q   = t.transform.rotation
            yaw = math.atan2(2 * (q.w * q.z + q.x * q.y),
                             1 - 2 * (q.y ** 2 + q.z ** 2))
            return t.transform.translation.x, t.transform.translation.y, yaw
        except Exception as e:
            # [IMPROVE] TF 실패 시 명확한 WARN 로그
            self.get_logger().warn(
                f'TF lookup failed (map→base_footprint): {e}', throttle_duration_sec=2.0)
            return None

    def _get_robot_pose(self):
        tf = self._get_robot_tf()
        if tf:
            return tf[0], tf[1]
        if self.current_pose:
            self.get_logger().warn(
                'Falling back to /current_pose (AMCL) for robot position',
                throttle_duration_sec=5.0)
            p = self.current_pose.pose.position
            return p.x, p.y
        return None

    # ── Path helpers ──────────────────────────────────────────────────────────

    def _waypoints_to_path(self, waypoints: List[Tuple[float, float]]) -> Path:
        path                 = Path()
        path.header.stamp    = self.get_clock().now().to_msg()
        path.header.frame_id = self.path_frame_id

        filtered: List[Tuple[float, float]] = []
        prev = None
        for wx, wy in waypoints:
            if prev and math.hypot(wx - prev[0], wy - prev[1]) < self.waypoint_spacing:
                continue
            filtered.append((wx, wy))
            prev = (wx, wy)

        prev_yaw = 0.0
        for i, (wx, wy) in enumerate(filtered):
            ps                    = PoseStamped()
            ps.header             = path.header
            ps.pose.position.x    = wx
            ps.pose.position.y    = wy
            ps.pose.position.z    = 0.0

            # --- 이 부분이 핵심 수정 사항입니다 ---
            if i == len(filtered) - 1 and self.current_goal is not None:
                # 마지막 점은 사용자가 설정한 목표 방향을 그대로 사용
                ps.pose.orientation = self.current_goal.pose.orientation
            else:
                # 나머지 점들은 다음 점을 바라보는 주행 방향 계산
                if i + 1 < len(filtered):
                    nx, ny   = filtered[i + 1]
                    prev_yaw = math.atan2(ny - wy, nx - wx)
                yaw = prev_yaw
                ps.pose.orientation.z = math.sin(yaw / 2.0)
                ps.pose.orientation.w = math.cos(yaw / 2.0)
            # -----------------------------------
            
            path.poses.append(ps)

        return path

    def _publish_dynamic_costmap(self, grid: np.ndarray):
        if self.map_msg is None:
            return
        msg                 = OccupancyGrid()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info            = self.map_msg.info
        flat     = grid.flatten().tolist()
        msg.data = [int(min(100, max(-1, v))) for v in flat]
        self.costmap_pub.publish(msg)

    def _publish_status(self, text: str):
        self.status_pub.publish(String(data=text))


def main(args=None):
    rclpy.init(args=args)
    node = GlobalPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
