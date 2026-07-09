# army_manipulator

MARU(Modular Autonomous Rescue Unit) 재난구조로봇의 4DoF + 1 Gripper 로봇팔 상위 제어 패키지 모음.
ROS2 Humble + ros2_control + MoveIt2 기반.

## 하드웨어 스펙

| 조인트 | 액추에이터 | 통신 | 회전축 | 비고 |
|---|---|---|---|---|
| joint_base (`base_rotate_joint`) | XH540-W270-T | RS-485 (Dynamixel) | Z | 베이스 선회 |
| joint_shoulder (`shoulder_lift_joint`) | RMD-X8-120 | CAN | Y | 오프셋 -55mm |
| joint_elbow (`elbow_joint`) | RMD-X6-60 | CAN | Y | 오프셋 +35mm |
| joint_wrist (`wrist_joint`) | RMD-X4-36 | CAN | Y | 오프셋 -25mm |
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

### 4) 실제 하드웨어 전환 (팀원 Hardware Interface 완성 후)

```bash
ros2 launch army_manipulator_bringup mock_bringup.launch.py use_mock_hardware:=false
```

`army_manipulator_description/urdf/army_manipulator_ros2_control.xacro` 의
`usb_port` / `ifname` / `actuator_id` 를 실제 배선에 맞게 수정해야 한다.

## 진행 상황

- [x] URDF -> XACRO 변환 (mock/real ros2_control 분기)
- [x] SRDF/MoveIt2 설정 초안
- [x] mock_hardware bringup launch
- [ ] MoveIt Setup Assistant로 Self-Collision 행렬 재계산
- [ ] RMD 모터 실측 후 acceleration limit / torque_constant 보정
- [ ] 팀원 Hardware Interface 완성 후 실제 하드웨어 전환
- [ ] RealSense D435i + ArUco 비전 좌표 연동

## 참고

- [MoveIt2 튜토리얼](https://github.com/moveit/moveit2_tutorials)
- [구조 참고: ros2-igus-rebel](https://github.com/AIRLab-POLIMI/ros2-igus-rebel)
- [RMD Hardware Interface](https://github.com/2b-t/myactuator_rmd_ros)
- [Dynamixel Hardware Interface](https://github.com/dynamixel-community/dynamixel_hardware)
