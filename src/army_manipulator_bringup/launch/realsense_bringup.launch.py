"""RealSense D435i 뎁스카메라 기동 + TF 연결.

카메라 마운트 오프셋(70mm 수평 / 100mm 높이 / 15도 하향 틸트)은 URDF의
wrist_link -> cam_link fixed joint(army_manipulator_macro.xacro)에서 관리한다.
cam_link는 wrist_link에 붙어 팔과 함께 움직이므로, robot_state_publisher가
자동으로 이 오프셋을 TF로 반영한다.

TODO(fixed-bug): 예전에는 이 launch 파일이 base_link -> camera_link 로
70/100mm + 15도 오프셋을 "고정"으로 publish했다. 하지만 카메라는 wrist_link에
달려 팔 모션에 따라 움직이므로, base_link 기준 고정 TF는 팔이 홈 자세를 벗어나는
순간부터 실제 카메라 위치와 어긋나는 버그였다. 이제는 URDF가 그 오프셋을
담당하고, 여기서는 URDF의 cam_link(마운트 브라켓 원점)와 realsense2_camera
드라이버의 루트 프레임(camera_link)만 연결한다.

[수정, 2026-09-04] 이 로봇 URDF의 cam_link 축은 카메라가 보는 방향을
로컬 +Z로 정의한 optical 계열 축이다. 반면 realsense2_camera의 camera_link는
ROS body 축이고, 드라이버가 camera_link -> *_optical_frame에 약
RPY=(-pi/2, 0, -pi/2)의 축 변환을 다시 적용한다. 둘을 identity로 연결하면
픽셀 depth(+Z optical)가 cam_link +X로 잘못 해석되어, 수직으로 내려다보는
카메라의 0.44m 깊이가 base_actuator의 수평 X에 더해지는 버그가 생긴다.
따라서 cam_link -> camera_link에는 그 역회전
RPY=(+pi/2, -pi/2, 0)을 넣는다. 공식 D435 URDF의 외형 중심 -> 왼쪽
IR/depth 원점 +17.5mm(body +Y)도 cam_link optical 축으로 표현하면
-17.5mm(X)이므로 translation 기본값을 함께 바꾼다. camera_link 이후의
color/depth 센서 간 미세 extrinsic은 RealSense 드라이버가 그대로 발행한다.

RealSense 노드는 aligned_depth 스트림(depth를 color 프레임에 정렬)을 켜서
인식 노드(summer_supply)가 컬러 픽셀 좌표를 그대로 depth 조회에 쓸 수 있게 한다.

serial_no / camera_namespace / camera_name: [2026-08-30] JECS에 RealSense가
여러 대(주행용 + 팔용) 물릴 수 있는 구성이라, 기본값(serial_no 빈 문자열)
으로 띄우면 열거 순서상 아무 RealSense나 잡혀 엉뚱한 카메라를 구독할
위험이 있었다 - 시리얼을 명시적으로 고정해서 원천 차단한다. 아래
serial_no 기본값은 `rs-enumerate-devices | grep Serial`로 JECS에서 직접
확인한 현재 연결된 카메라의 실제 시리얼이다(카메라 교체/추가 시 다시
확인해서 갱신할 것). **맨 앞 언더스코어(`_243322074693`)는 실수가 아니라
필수다** - 순수 숫자 문자열을 그대로 넣으면 ROS2 launch 파라미터 타입
추론에서 정수로 캐스팅돼버려 RealSense 드라이버가 매칭을 못 하는 문제가
있어(실기에서 실측 확인: 언더스코어 없이 넣으니 토픽이 아예 안 나왔고,
언더스코어 붙이니 정상 동작), 문자열임을 강제하려고 붙인다.
camera_namespace=arm은 실기에서 쓰던 실행 커맨드를 그대로 반영한 값:
  ros2 launch realsense2_camera rs_launch.py camera_namespace:=arm \
    camera_name:=camera serial_no:=_243322074693 ...
이 기본값을 쓰면 카메라 토픽이 /camera/camera/... 가 아니라
/arm/camera/...로 뜨므로, perception_bringup.launch.py/
depth_camera_ik_bringup.launch.py의 color_topic/depth_topic/
camera_info_topic 기본값도 같이 맞춰뒀다(별도 커밋).

[갱신, 2026-09-05] TF 프레임 이름 충돌 해결 - rs_launch.py 포함 대신 드라이버
노드를 직접 띄운다. 구동부 D455도 camera_name=camera로 떠서 두 카메라가
camera_link / camera_color_optical_frame 등 **똑같은 프레임 이름**을 발행했고,
구동부의 reduced_odom_bringup이 base_link -> camera_link static TF까지 내기
때문에 camera_link의 부모가 cam_link(팔)와 base_link(구동부) 사이를 오가며
summer_supply의 camera -> base_actuator 변환이 간헐적으로 "unconnected trees"
로 실패하거나 엉뚱한 기하로 성공했다. realsense2_camera 4.56은 프레임 이름을
`camera_name` **파라미터**(<camera_name>_link, <camera_name>_color_optical_frame
...)로 만들고, 토픽은 노드 이름 기준(~/color/image_raw)으로 만든다. 그래서
노드 이름은 camera(토픽 /arm/camera/... 유지)로 두고 camera_name 파라미터만
arm_camera로 줘서 프레임만 arm_camera_*로 바꾼다 - 구독자 쪽 토픽 변경 없음.
(rs_launch.py는 노드 이름과 camera_name 파라미터를 같은 값으로 묶어서 이
분리가 불가능하고, tf_prefix 인자는 4.56.4 노드 코드에서 실제로 쓰이지 않는다.)
summer_supply는 이미지 header.frame_id를 그대로 쓰므로 자동으로 따라온다.
같은 이유로 팔 URDF의 base_link도 arm_base_link로 바꿨다(구동부 odom ->
base_link와 충돌) - army_manipulator_macro.xacro 참고.
"""
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _load_qos_parameters() -> dict:
    """config/realsense_qos.yaml(최상위가 바로 파라미터명인 flat yaml)을 dict로."""
    path = Path(get_package_share_directory("army_manipulator_bringup")) / "config" / "realsense_qos.yaml"
    with path.open(encoding="utf-8") as stream:
        return yaml.safe_load(stream) or {}


def generate_launch_description():
    declared_arguments = [
        DeclareLaunchArgument(
            "serial_no",
            default_value="_243322074693",
            description=(
                "'rs-enumerate-devices | grep Serial'로 확인. "
            ),
        ),
        DeclareLaunchArgument(
            "camera_namespace",
            default_value="arm",
            description="realsense2_camera 네임스페이스. 실기 확인값(arm) - 토픽이 /arm/camera/...로 뜬다.",
        ),
        DeclareLaunchArgument(
            "camera_name",
            default_value="camera",
            description="realsense2_camera 노드 이름. 토픽이 /<namespace>/<이 값>/... 로 뜬다.",
        ),
        DeclareLaunchArgument(
            "tf_frame_prefix",
            default_value="arm_camera",
            description=(
                "realsense2_camera의 camera_name 파라미터 = TF 프레임 접두사. "
                "<값>_link, <값>_color_optical_frame ... 로 발행된다. 구동부 D455의 "
                "camera_* 프레임과 충돌하지 않도록 기본값 arm_camera."
            ),
        ),
        DeclareLaunchArgument(
            "color_profile",
            default_value="640x480x15",
            description="rgb_camera.color_profile. 실기 확인값과 동일.",
        ),
        DeclareLaunchArgument(
            "depth_profile",
            default_value="640x480x15",
            description="depth_module.depth_profile. 실기 확인값과 동일.",
        ),
        DeclareLaunchArgument(
            "initial_reset",
            default_value="false",
            description=(
                "[추가, 2026-09-05] true면 스트리밍 전에 장치를 하드웨어 리셋한다. "
                "USB 링크가 불안정해 'Depth stream start failure'/'resource busy'가 "
                "뜰 때 시도. 리셋에 수 초가 걸리므로 평소엔 false."
            ),
        ),
        DeclareLaunchArgument(
            "camera_link_x", default_value="-0.0175",
            description=(
                "D435i 외형 중심(cam_link optical 축) -> 왼쪽 IR 원점, "
                "X=-17.5mm."
            ),
        ),
        DeclareLaunchArgument(
            "camera_link_y", default_value="0.0",
            description="cam_link optical 축 기준 왼쪽 IR 원점 Y[m].",
        ),
        DeclareLaunchArgument(
            "camera_link_z", default_value="0.0",
            description="cam_link -> camera_link ROS Z[m].",
        ),
    ]
    serial_no = LaunchConfiguration("serial_no")
    camera_namespace = LaunchConfiguration("camera_namespace")
    camera_name = LaunchConfiguration("camera_name")
    tf_frame_prefix = LaunchConfiguration("tf_frame_prefix")
    color_profile = LaunchConfiguration("color_profile")
    depth_profile = LaunchConfiguration("depth_profile")
    initial_reset = LaunchConfiguration("initial_reset")
    camera_link_x = LaunchConfiguration("camera_link_x")
    camera_link_y = LaunchConfiguration("camera_link_y")
    camera_link_z = LaunchConfiguration("camera_link_z")

    realsense_node = Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace=camera_namespace,
        name=camera_name,
        output="screen",
        emulate_tty=True,
        arguments=["--ros-args", "--log-level", "info"],
        parameters=[
            {
                # 프레임 접두사만 바꾼다(토픽은 노드 이름 기준이라 그대로).
                "camera_name": ParameterValue(tf_frame_prefix, value_type=str),
                # 드라이버가 맨 앞 '_'를 떼고 비교하므로 언더스코어 유무 모두 OK.
                "serial_no": ParameterValue(serial_no, value_type=str),
                "initial_reset": ParameterValue(initial_reset, value_type=bool),
                "align_depth.enable": True,
                "enable_color": True,
                "enable_depth": True,
                "enable_infra1": False,
                "enable_infra2": False,
                "enable_gyro": False,
                "enable_accel": False,
                "pointcloud.enable": False,
                "rgb_camera.color_profile": ParameterValue(color_profile, value_type=str),
                "depth_module.depth_profile": ParameterValue(depth_profile, value_type=str),
            },
            # [추가, 2026-08-31] color/depth 발행 QoS를 SENSOR_DATA(BEST_EFFORT)로
            # 강제(config/realsense_qos.yaml, 상단 주석 참고). summer_supply.py/
            # drive_supply_detector.py는 이미 qos_profile_sensor_data로 구독 중.
            _load_qos_parameters(),
        ],
    )

    # cam_link는 외형 박스 중심이면서 +Z가 실제 시선 방향인 optical 계열
    # 프레임이고, <tf_frame_prefix>_link(기본 arm_camera_link)는 RealSense의
    # body 계열 왼쪽 IR/depth 원점이다.
    # translation은 body +Y=17.5mm를 optical 축으로 바꾼 X=-17.5mm이며,
    # rotation은 드라이버가 뒤에서 적용하는 body -> optical 회전의 역이다.
    # camera_link -> color/depth optical frame은 드라이버의 장치별 calibrated
    # extrinsic을 그대로 사용하므로 여기서 컬러 렌즈 오프셋을 중복 적용하지 않는다.
    camera_mount_to_driver_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="cam_link_to_camera_link_tf",
        output="screen",
        arguments=[
            "--x", camera_link_x, "--y", camera_link_y, "--z", camera_link_z,
            "--roll", "1.5707963267948966",
            "--pitch", "-1.5707963267948966",
            "--yaw", "0",
            "--frame-id", "cam_link",
            "--child-frame-id", [tf_frame_prefix, "_link"],
        ],
    )

    return LaunchDescription(declared_arguments + [realsense_node, camera_mount_to_driver_tf])
