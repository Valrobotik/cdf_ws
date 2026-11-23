from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    namespace = LaunchConfiguration("namespace")
    video_device = LaunchConfiguration("video_device")
    image_width = LaunchConfiguration("image_width")
    image_height = LaunchConfiguration("image_height")
    framerate = LaunchConfiguration("framerate")
    pixel_format = LaunchConfiguration("pixel_format")
    camera_name = LaunchConfiguration("camera_name")
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
                "image_width", default_value="1920", description="Image width"
            ),
            DeclareLaunchArgument(
                "image_height", default_value="1080", description="Image height"
            ),
            DeclareLaunchArgument(
                "framerate", default_value="10.0", description="Camera framerate"
            ),
            DeclareLaunchArgument(
                "pixel_format",
                default_value="yuyv2rgb",
                description="usb_cam pixel format",
            ),
            DeclareLaunchArgument(
                "camera_name",
                default_value="zdc",
                description="Camera name used by camera_info_manager",
            ),
            DeclareLaunchArgument(
                "camera_info_url",
                default_value="",
                description="Optional file:// URL to the ZDC calibration yaml",
            ),
            DeclareLaunchArgument(
                "frame_id",
                default_value="camera_link_optical",
                description="Image and camera_info frame, matches the ArUco node",
            ),
            Node(
                package="usb_cam",
                executable="usb_cam_node_exe",
                name="usb_cam",
                namespace=namespace,
                output="screen",
                parameters=[
                    {
                        "video_device": video_device,
                        "image_width": ParameterValue(image_width, value_type=int),
                        "image_height": ParameterValue(image_height, value_type=int),
                        "framerate": ParameterValue(framerate, value_type=float),
                        "pixel_format": pixel_format,
                        "camera_name": camera_name,
                        "camera_info_url": camera_info_url,
                        "frame_id": frame_id,
                    }
                ],
            ),
        ]
    )
