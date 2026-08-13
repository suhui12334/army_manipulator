# army_manipulator

재난구조로봇의 4DoF + 1 Gripper 로봇팔 상위 제어 패키지 모음.
ROS2 Humble + ros2_control + MoveIt2 기반.

## 하드웨어 스펙

| 조인트 | 액추에이터 | 통신 | 회전축 | 비고 |
|---|---|---|---|---|
| joint_base (`base_rotate_joint`) | XH540-W270-T | TTL (Dynamixel) | Z | 베이스 선회 |
| joint_shoulder (`shoulder_lift_joint`) | RMD-X8-120 | CAN | Y | 오프셋 -37.7mm |
| joint_elbow (`elbow_joint`) | RMD-X6-60 | CAN | Y | 오프셋 +18.8mm |
| joint_wrist (`wrist_joint`) | RMD-X4-36 | CAN | Y | 오프셋 -31.3mm |
| joint_gripper (`gripper_joint`) | MX-106T | TTL (Dynamixel) | Z | 랙-피니언 |

링크: 20x20x2T CFRP 사각 파이프 / 하우징: PETG 3D프린팅
링크 길이: shoulder 180mm, elbow 220mm, wrist+gripper 220mm

## 패키지 구성

- `army_manipulator_description` — URDF(XACRO), ros2_control 하드웨어 인터페이스 정의, 단독 RViz 확인용 launch
- `army_manipulator_moveit_config` — SRDF, kinematics/joint_limits/controllers 설정, MoveIt Setup Assistant 실행 launch, move_group/RViz launch
- `army_manipulator_bringup` — mock_hardware 기반 RViz + MoveIt2 통합 bringup launch, 실제 ros2_control 컨트롤러 설정

## 빌드

```bash
cd ~/ros2_ws/src
git clone <this-repo-url> army_manipulator
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

## 실행

### 1) URDF 단독 확인 (조인트 리밋/TF 트리)

```bash
ros2 launch army_manipulator_description display.launch.py
```

### 2) MoveIt Setup Assistant (SRDF/컨트롤러 편집)

```bash
ros2 launch army_manipulator_moveit_config setup_assistant.launch.py
```

GUI에서 **Edit Existing MoveIt Configuration Package** 선택 후
`army_manipulator_moveit_config` 패키지 경로를 지정하면
이미 작성된 `config/army_manipulator.srdf` 등을 이어서 편집할 수 있다.
특히 Self-Collisions 탭에서 자동 충돌 행렬 재계산을 한 번 돌려서
`disable_collisions` 목록을 검증/보강할 것을 권장한다.

### 3) mock_hardware로 RViz + MoveIt2 확인

```bash
ros2 launch army_manipulator_bringup mock_bringup.launch.py
```

### 4) 실제 하드웨어 전환

```bash
ros2 launch army_manipulator_bringup mock_bringup.launch.py use_mock_hardware:=false
```

`army_manipulator_description/urdf/army_manipulator_ros2_control.xacro` 의
`usb_port` / `ifname` / `actuator_id` 를 실제 배선에 맞게 수정해야 한다.

### 5) 실제 mesh 형상으로 확인 (STL 통합 완료)

```bash
ros2 launch army_manipulator_description display.launch.py use_mesh:=true
```

`army_manipulator_description/meshes/{visual,collision}/*.stl` 에 실제 STL이
들어있다. 파일명은 링크 이름과 1:1이 아니라 실측 치수로 대조해서 매핑했다:

| STL 파일 | 매핑된 링크 |
|---|---|
| `base_link.stl` | `base_link` |
| `base_actuator.stl` | `base_actuator` (XH540 바디) |
| `shoulder_link.stl` | `shoulder_link` (RMD-X8-120 플랜지+L1 튜브+RMD-X6-60 케이스) |
| `elbow_link.stl` | `elbow_link` (RMD-X6-60 플랜지+L2 튜브+RMD-X4-36 케이스) |
| `wrist_link.stl` | `wrist_link` |
| `cam_link.stl` | `cam_link` (wrist_link에 fixed) |
| `pinion_gear.stl` | `pinion_gear` (구 gripper_base_link 대체, gripper_joint로 구동) |
| `rack_left.stl` / `rack_right.stl` | `rack_left_link` / `rack_right_link` (prismatic mimic) |

기본값은 `false`라서 STL 매핑에 문제가 있어도 `use_mesh:=false`(기본)로 돌아가면
기존 primitive geometry 데모는 그대로 동작한다.

TODO(mesh-origin 검증): 위 매핑은 STL 바운딩박스 실측으로 정한 1차 확정이며,
cam_link의 정확한 부착 위치(origin)와 rack 슬라이딩 축 방향은 RViz 육안 확인
후 필요시 보정할 것 (`army_manipulator_macro.xacro`의 TODO 주석 참고).

MoveIt2 쪽(`move_group.launch.py`, `moveit_rviz.launch.py`)에서 mesh를 켜려면
launch 인자 대신 환경변수를 쓴다 (MoveItConfigsBuilder 제약):

```bash
ARMY_MANIPULATOR_USE_MESH=true ros2 launch army_manipulator_bringup mock_bringup.launch.py use_mesh:=true
```

### 6) 뎁스카메라 + 타겟 검출 파이프라인

```bash
ros2 launch army_manipulator_bringup realsense_bringup.launch.py
```

카메라 마운트 오프셋(수평 70mm, 높이 100mm, 15도 하향 틸트)은 이제
`army_manipulator_macro.xacro`의 `wrist_link -> cam_link` fixed joint가
담당한다 — cam_link가 wrist_link에 붙어있어 팔이 움직이면 robot_state_publisher가
자동으로 TF를 갱신한다. (이전 버전은 이 오프셋을 `base_link -> camera_link`
static TF로 고정 publish했는데, 팔이 홈 자세를 벗어나면 실제 카메라 위치와
어긋나는 버그였다 — 수정됨.)

`realsense_bringup.launch.py`는 `cam_link`(URDF 마운트 프레임)와
`camera_link`(realsense2_camera 드라이버 루트 프레임)를 연결한다.
`target_detector_node.py`는 D455의 aligned color/depth에서 YOLO 클래스
`supplybox`의 중심 3D 점을 `/arm/target_point`로 publish한다.

`maru_ik_node`는 이 점을 반드시 TF로 `base_link`에 변환한 후 MoveIt의
`move_group` action에 pre-grasp, 하강, 파지, 리프트 목표를 보낸다. 생성된
trajectory는 `arm_controller`/`gripper_controller`와 ros2_control을 거쳐
RMD CAN 및 Dynamixel 명령으로 전송된다. 기본값은 물자를 쥔 채 리프트 자세에
유지하므로 이동 플랫폼이 운반할 수 있다.

실물 전에는 다음 항목을 보정해야 한다.

- D455 렌즈 중심과 `cam_link` 사이의 6D 오프셋
- `grasp_offset_{x,y,z}`: 기본 z=-47.5 mm는 95 mm 상면 검출 기준의 초기값
- `pregrasp_offset_z`, `approach_pitch`, 그리퍼 닫힘 위치 및 RMD 한계값

### 7) D455 → MoveIt → CAN 전체 실행

실물 CAN 인터페이스를 실제 배선 bitrate에 맞춰 먼저 활성화한다.

```bash
sudo ip link set can_arm down
sudo ip link set can_arm up type can bitrate 1000000
```

빌드 결과를 반영한 뒤 전체 파이프라인을 하나의 launch로 실행한다.

```bash
cd ~/army_manipulator
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source ~/army_manipulator/install/setup.bash
ros2 launch army_manipulator_bringup depth_camera_ik_bringup.launch.py \
  use_mock_hardware:=false sim_target:=false
```

실행 데이터 경로는 다음과 같다.

```text
realsense2_camera → target_detector_node (/arm/target_point)
  → TF(base_link) → maru_ik_node → move_group
  → arm_controller/gripper_controller → ros2_control hardware plugin → CAN
```

위 launch가 기동하는 노드 순서와 역할은 다음과 같다.

1. `realsense2_camera`: D455 color, aligned depth, camera TF를 발행한다.
2. `target_detector_node`: YOLO `supplybox` 검출과 depth deprojection으로
   `/arm/target_point`를 발행한다.
3. `robot_state_publisher` + `joint_state_broadcaster`: 실제 엔코더 상태를
   `/joint_states`와 `base_link → camera_color_optical_frame` TF 체인에 반영한다.
4. `move_group`: IK, 관절 리밋, 충돌 검사, 경로 계획을 수행한다.
5. `maru_ik_node`: 목표점을 `base_link`로 변환해 pre-grasp → 하강 → 3초 정지
   → 그리퍼 닫기 → 리프트 시퀀스를 MoveIt에 요청한다.
6. `arm_controller`/`gripper_controller`: MoveIt trajectory를 ros2_control
   hardware interface에 넘기고, RMD(CAN)·Dynamixel(TTL) 명령이 송신된다.
7. `planned_encoder_trajectory`: 계획 waypoint를 raw encoder-radian으로 변환해
   기록 토픽에 발행한다.

다른 터미널에서 아래 상태를 확인한다.

```bash
source /opt/ros/humble/setup.bash
source ~/army_manipulator/install/setup.bash

ros2 control list_controllers
ros2 topic echo /arm/target_point
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
ros2 topic echo --once /arm/planned_encoder_trajectory
```

`joint_state_broadcaster`, `arm_controller`, `gripper_controller`는 모두
`active`여야 한다. 첫 실물 시험은 반드시 `use_mock_hardware:=true`로 같은
launch와 목표점 변환을 먼저 검증한 뒤, 팔 주변을 비운 상태에서 실제 CAN으로
전환한다.

이 토픽은 기록·검증용이다. CAN 프레임은 이 토픽에서 직접 보내지 않고,
`arm_controller`의 단일 명령 경로만 RMD hardware interface를 통해 송신한다.
true encoder tick은 모터별 encoder CPR·감속비·CAN 프로토콜이 확정된 뒤
hardware plugin에서 radian 명령을 변환해야 한다.

## 진행 상황

- [x] URDF -> XACRO 변환 (mock/real ros2_control 분기)
- [x] SRDF/MoveIt2 설정 초안
- [x] mock_hardware bringup launch
- [x] mesh 참조 구조 스캐폴딩 (`use_mesh` 인자, STL은 추후 추가)
- [x] D455 aligned depth + YOLO `supplybox` 3D 좌표 추출
- [x] TF 기반 목표 변환, MoveIt 계획, ros2_control CAN/TTL 실행 경로
- [ ] 실제 STL mesh 파일 반영 후 `use_mesh:=true`로 전환, Self-Collision 재계산
- [ ] base pan(θ0) 조인트 리밋 / 전체 홈 포지션 확정 (CATIA 확인 후)
- [ ] D455 hand-eye 및 grasp offset 실측 보정
- [ ] RMD 모터 실측 후 acceleration limit / torque_constant 보정
- [ ] 팀원 Hardware Interface 완성 후 실제 하드웨어 전환

## 참고

- [MoveIt2 튜토리얼](https://github.com/moveit/moveit2_tutorials)
- [구조 참고: ros2-igus-rebel](https://github.com/AIRLab-POLIMI/ros2-igus-rebel)
- [RMD Hardware Interface](https://github.com/2b-t/myactuator_rmd_ros)
- [Dynamixel Hardware Interface](https://github.com/dynamixel-community/dynamixel_hardware)
