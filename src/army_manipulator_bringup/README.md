# army_manipulator_bringup

army_manipulator 로봇 팔 기동 launch/노드 모음. 주 실행 경로는
`launch/mock_bringup.launch.py`(팔+MoveIt 확인용)와
`launch/depth_camera_ik_bringup.launch.py`(카메라 검출 -> IK -> 실행 전체 파이프라인)이다.

## 수동 디버그 도구

아래 노드는 자동 bringup(위 launch 파일들)에 포함되어 있지 않다. 필요할 때 직접 띄운다.

### tcp_trail_publisher.py

TCP(`tcp_link`)의 이동 궤적을 `/maru/tcp_path` (`nav_msgs/Path`)로 publish한다.
팔이 실제로 어떤 경로로 움직였는지 눈으로 확인하고 싶을 때만 사용.

```bash
ros2 run army_manipulator_bringup tcp_trail_publisher.py
```

다른 bringup 노드들(robot_state_publisher, ros2_control_node 등)이 이미 떠 있는
상태에서 실행해야 TF를 조회할 수 있다. 이후 RViz에서 Path display를 추가하고
Topic을 `/maru/tcp_path`로 지정하면 궤적이 그려진다 (Fixed Frame은 `base_frame`
파라미터 값과 일치해야 함, 기본 `base_link`).

파라미터 (전부 선택, 기본값 있음):

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `base_frame` | `base_link` | 궤적 기준 프레임 |
| `tcp_frame` | `tcp_link` | 추적할 TCP 프레임 |
| `publish_hz` | `20.0` | publish 주기(Hz) |
| `min_dist_m` | `0.002` | 이 거리(m) 이상 움직였을 때만 점 추가 |
| `max_points` | `2000` | 누적 포인트 상한 |

예시 (주기/최소거리 조정):

```bash
ros2 run army_manipulator_bringup tcp_trail_publisher.py --ros-args \
  -p publish_hz:=10.0 -p min_dist_m:=0.005
```
