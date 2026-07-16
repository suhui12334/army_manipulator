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
드라이버의 루트 프레임(camera_link)만 identity(또는 D435i 렌즈-브라켓 간
실측 미세 오프셋)로 연결한다.

TODO(lens-offset): 아래 static TF는 identity placeholder다. cam_link.stl
원점과 D435i 렌즈 중심이 정확히 일치하지 않으면, 실측 후 x/y/z, roll/pitch/yaw
값을 채울 것.

RealSense 노드는 aligned_depth 스트림(depth를 color 프레임에 정렬)을 켜서
target_detector_node가 컬러 픽셀 좌표를 그대로 depth 조회에 쓸 수 있게 한다.
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("realsense2_camera"), "launch", "rs_launch.py"]
            )
        ),
        launch_arguments={
            "align_depth.enable": "true",
            "enable_color": "true",
            "enable_depth": "true",
            "pointcloud.enable": "false",
        }.items(),
    )

    # cam_link(URDF, wrist_link에 고정 부착) -> camera_link(realsense2_camera 루트 프레임)
    # identity placeholder. 렌즈-브라켓 실측 오프셋 확정되면 갱신할 것(TODO(lens-offset)).
    camera_mount_to_driver_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="cam_link_to_camera_link_tf",
        output="screen",
        arguments=[
            "--x", "0", "--y", "0", "--z", "0",
            "--roll", "0", "--pitch", "0", "--yaw", "0",
            "--frame-id", "cam_link",
            "--child-frame-id", "camera_link",
        ],
    )

    return LaunchDescription([realsense_launch, camera_mount_to_driver_tf])
