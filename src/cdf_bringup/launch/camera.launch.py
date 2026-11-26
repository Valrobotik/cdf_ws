import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_camera_info_url = 'file://' + os.path.join(
        get_package_share_directory('cdf_bringup'),
        'config', 'camera_info', 'zdc_1920x1080.yaml')

    namespace = LaunchConfiguration("namespace")
    video_device = LaunchConfiguration("video_device")
    pixel_format = LaunchConfiguration("pixel_format")
    camera_info_url = LaunchConfiguration("camera_info_url")
    frame_id = LaunchConfiguration("frame_id")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "namespace",
                default_value="mcc/camera",
                description="Namespace of the image and camera_info topics",
            ),
            DeclareLaunchArgument(
                "video_device",
                default_value="/dev/video0",
                description="V4L2 device of the ZDC camera",
            ),
            DeclareLaunchArgument(
                "pixel_format",
                default_value="YUYV",
                description="V4L2 pixel format (YUYV, UYVY or GREY)",
            ),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value=default_camera_info_url,
                description="ZDC 1920x1080 calibration YAML",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="camera_link_optical",
                description="Image and camera_info frame, matches the ArUco node",
            ),
            Node(
                package="v4l2_camera",
                executable="v4l2_camera_node",
                name="v4l2_camera",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "video_device": video_device,
                        "image_size": [1920, 1080],
                        "pixel_format": pixel_format,
                        "output_encoding": "rgb8",
                        "camera_info_url": camera_info_url,
                        "camera_frame_id": frame_id,
                    }
                ],
            ),
        ]
    )
