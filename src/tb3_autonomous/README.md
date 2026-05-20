# tb3_autonomous 🤖

TurtleBot3 Waffle Pi 자율주행 ROS2 패키지  
**Gazebo 시뮬레이션 → 실물 로봇** 양쪽에서 동작한다.

---

## 시스템 구조

```
┌─────────────────────────────────────────────────────────┐
│  Remote PC (이 패키지 실행)                               │
│                                                          │
│  ┌── Nav2 (기존 패키지 사용) ──────────────────────────┐  │
│  │  nav2_map_server  →  /map 발행                       │  │
│  │  nav2_amcl        →  /amcl_pose 발행 (위치 추정)     │  │
│  │  nav2_lifecycle_manager  (위 둘의 수명 관리)          │  │
│  └─────────────────────────────────────────────────────┘  │
│                                                          │
│  ┌── 직접 구현 노드 ─────────────────────────────────┐   │
│  │                                                   │   │
│  │  goal_manager_node                                │   │
│  │   • RViz2 /goal_pose 수신                         │   │
│  │   • /amcl_pose → /current_pose 중계               │   │
│  │   • 웨이포인트 순차 주행 지원                       │   │
│  │             │                                    │   │
│  │             ▼                                    │   │
│  │  global_planner_node  (A* 알고리즘)               │   │
│  │   • /map + /goal_pose → /global_path 발행         │   │
│  │   • 장애물 팽창(inflation) 적용                    │   │
│  │   • 경로 평활화(gradient descent)                 │   │
│  │             │                                    │   │
│  │             ▼                                    │   │
│  │  local_controller_node                           │   │
│  │   • Pure Pursuit 경로 추종                        │   │
│  │   • VFH 장애물 회피 (LiDAR /scan)                 │   │
│  │   • Stuck 감지 → 자동 재경로                      │   │
│  │   • /cmd_vel 발행                                 │   │
│  └───────────────────────────────────────────────────┘   │
└─────────────────────────────────────────────────────────┘
         │  /cmd_vel, /scan, TF
         ▼
  TurtleBot3 Waffle Pi (또는 Gazebo)
```

---

## 설치 및 빌드

```bash
# 워크스페이스 준비
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# 이 패키지 디렉터리를 여기에 복사

# Nav2 설치 (없을 경우)
sudo apt install ros-humble-nav2-map-server \
                 ros-humble-nav2-amcl \
                 ros-humble-nav2-lifecycle-manager

# 빌드
cd ~/ros2_ws
colcon build --packages-select tb3_autonomous
source install/setup.bash
```

---

## 맵 파일 설정

`maps/` 디렉터리에 본인의 `my_map.pgm`, `my_map.yaml` 파일을 복사한다.

```bash
cp /path/to/my_map.pgm ~/ros2_ws/src/tb3_autonomous/maps/
cp /path/to/my_map.yaml ~/ros2_ws/src/tb3_autonomous/maps/
```

---

## 실행 방법

### 가제보 시뮬레이션

터미널 1 – Gazebo 실행:
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

터미널 2 – 자율주행 실행:
```bash
ros2 launch tb3_autonomous autonomous_navigation.launch.py use_sim:=true
```

### 실물 TurtleBot3

터미널 1 – 로봇에서 (SSH):
```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch turtlebot3_bringup robot.launch.py
```

터미널 2 – Remote PC:
```bash
ros2 launch tb3_autonomous autonomous_navigation.launch.py \
  use_sim:=false \
  map_file:=/path/to/art_gallery3.yaml
```

---

## 목표 설정 방법

### RViz2 로 목표 설정
1. RViz2 실행 후 "2D Goal Pose" 버튼 클릭
2. 지도 위 목표 위치 클릭 → 자동으로 경로 계획 및 주행 시작

### 커맨드라인으로 목표 전송
```bash
ros2 topic pub --once /goal_pose geometry_msgs/PoseStamped \
  "{header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}"
```

### 웨이포인트 파일 사용
```bash
# config/waypoints_example.json 편집 후:
ros2 launch tb3_autonomous autonomous_navigation.launch.py \
  use_sim:=true \
  params_file:=... 
# 또는 파라미터로 직접:
ros2 param set /goal_manager waypoint_file /path/to/waypoints.json
ros2 service call /start_waypoint_nav std_srvs/srv/Trigger {}
```

### 내비게이션 취소
```bash
ros2 service call /cancel_navigation std_srvs/srv/Trigger {}
```

### 강제 재경로
```bash
ros2 service call /replan std_srvs/srv/Trigger {}
```

---

## 상태 모니터링

```bash
# 플래너 상태
ros2 topic echo /planning_status

# 컨트롤러 상태
ros2 topic echo /controller_status

# 목표 관리 상태
ros2 topic echo /manager_status

# 로봇 현재 속도
ros2 topic echo /cmd_vel

# 전역 경로 확인 (RViz2 에서 /global_path 추가)
ros2 topic echo /global_path --no-arr
```

---

## 파라미터 튜닝 가이드

| 파라미터 | 기본값 | 설명 | 조정 방향 |
|---|---|---|---|
| `inflation_radius` | 0.25 m | 장애물 팽창 반경 | 좁은 공간이면 줄임 |
| `max_linear_vel` | 0.20 m/s | 최대 전진 속도 | 안전 우선이면 줄임 |
| `lookahead_dist` | 0.40 m | Pure Pursuit lookahead | 크면 부드럽지만 커브 단축 |
| `danger_dist` | 0.35 m | 즉시 정지 거리 | 환경에 따라 조정 |
| `slow_dist` | 0.60 m | 감속 시작 거리 | danger_dist 보다 크게 |
| `goal_tolerance` | 0.15 m | 목표 도달 판정 | 정밀도 요구 시 줄임 |

---

## 알고리즘 설명

### Global Planner – A*
- OccupancyGrid 위에서 A* 탐색
- 8-방향 이동, Octile 거리 휴리스틱
- 장애물 팽창(inflation) 후 탐색 → 로봇 몸체 고려
- Gradient Descent 경로 평활화

### Local Controller – Pure Pursuit + VFH
- **Pure Pursuit**: lookahead point 를 향해 곡률 계산 → 속도 명령
- **VFH(Vector Field Histogram)**: LiDAR 를 72개 섹터로 나누어  
  장애물이 없는 방향(valley) 중 목표 방향에 가장 가까운 방향 선택
- 두 알고리즘을 거리에 따라 혼합 (멀면 PP, 가까우면 VFH 비중↑)
- Stuck 감지: 5초 이상 이동 없으면 자동 재경로 요청

---

## 트러블슈팅

| 증상 | 확인사항 |
|---|---|
| 경로가 계획되지 않음 | `/map` 토픽 수신 여부, AMCL 초기화 완료 여부 |
| 로봇이 움직이지 않음 | `/global_path` 발행 여부, TF(map→base_footprint) 확인 |
| 위치추정이 틀림 | RViz2 에서 "2D Pose Estimate" 로 초기 위치 지정 |
| 장애물에 충돌 | `danger_dist`, `slow_dist` 값 증가 |
| 경로가 너무 직선적 | `inflation_radius` 감소 |
