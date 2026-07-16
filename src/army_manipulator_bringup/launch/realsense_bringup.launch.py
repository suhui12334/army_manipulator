"""RealSense D435i 뎁스카메라 + camera_link static TF 기동.

camera_link 장착 위치(확정 스펙, base_link 기준):
  - 수평 오프셋 70mm (x), 높이 100mm (z)
  - 15도 하향 틸트 (pitch = +0.2618 rad; REP-103 기준 +pitch가 카메라 광축을
    아래로 기울인다)

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

    camera_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="camera_link_static_tf",
        output="screen",
        arguments=[
            "--x", "0.070",
            "--y", "0",
            "--z", "0.100",
            "--roll", "0",
            "--pitch", "0.2618",
            "--yaw", "0",
            "--frame-id", "base_link",
            "--child-frame-id", "camera_link",
        ],
    )

    return LaunchDescription([realsense_launch, camera_static_tf])
