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
| `base_actuator.stl` | `shoulder_pan_link` (XH540 바디) |
| `shoulder_link.stl` | `upper_arm_link` (RMD-X8-120 플랜지+L1 튜브+RMD-X6-60 케이스) |
| `elbow_link.stl` | `forearm_link` (RMD-X6-60 플랜지+L2 튜브+RMD-X4-36 케이스) |
| `wrist_link.stl` | `wrist_link` |
| `cam_link.stl` | `cam_link` (wrist_link에 fixed) |
| `pinion_gear.stl` | `pinion_link` (구 gripper_base_link 대체, gripper_joint로 구동) |
| `lack_left.stl` / `lack_right.stl` | `rack_left_link` / `rack_right_link` (prismatic mimic) |

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
