# Copyright 2023 Ar-Ray-code
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


import launch
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            'model_path_cam0',
            default_value='./src/hailort_yolo_common/weights/yolov8s.hef',
            description='HEF model path for cam0.'
        ),
        DeclareLaunchArgument(
            'model_path_cam1',
            default_value='./src/hailort_yolo_common/weights/yolov8s.hef',
            description='HEF model path for cam1.'
        ),
        DeclareLaunchArgument(
            'class_labels_path',
            default_value='',
            description='if use custom model, set class name labels. '
        ),
        DeclareLaunchArgument(
            'conf',
            default_value='0.30',
            description='yolo confidence threshold.'
        ),
        DeclareLaunchArgument(
            'nms',
            default_value='0.45',
            description='yolo nms threshold'
        ),
        DeclareLaunchArgument(
            'imshow_isshow',
            default_value='true',
            description=''
        ),
        DeclareLaunchArgument(
            'nms_output_name',
            default_value='yolov8s/yolov8_nms_postprocess',
            description='Exact NMS output tensor name inside the HEF.'
        ),
    ]

    container = ComposableNodeContainer(
        name='yolo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='yolo_ros_hailort_cpp',
                plugin='yolo_ros_hailort_cpp::YoloNode',
                name='yolo_cam0',
                parameters=[{
                    'model_path': LaunchConfiguration('model_path_cam0'),
                    'class_labels_path': LaunchConfiguration('class_labels_path'),
                    'conf': LaunchConfiguration('conf'),
                    'nms': LaunchConfiguration('nms'),
                    'imshow_isshow': LaunchConfiguration('imshow_isshow'),
                    'src_image_topic_name': '/cam0Funai/image_raw',
                    'publish_image_topic_name': '/cam0/yolo/image_raw',
                    'publish_boundingbox_topic_name': '/cam0/yolo/bounding_boxes',
                    'publish_resized_image': False,
                    'nms_output_name': LaunchConfiguration('nms_output_name'),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='yolo_ros_hailort_cpp',
                plugin='yolo_ros_hailort_cpp::YoloNode',
                name='yolo_cam1',
                parameters=[{
                    'model_path': LaunchConfiguration('model_path_cam1'),
                    'class_labels_path': LaunchConfiguration('class_labels_path'),
                    'conf': LaunchConfiguration('conf'),
                    'nms': LaunchConfiguration('nms'),
                    'imshow_isshow': LaunchConfiguration('imshow_isshow'),
                    'src_image_topic_name': '/cam1Funai/image_raw',
                    'publish_image_topic_name': '/cam1/yolo/image_raw',
                    'publish_boundingbox_topic_name': '/cam1/yolo/bounding_boxes',
                    'publish_resized_image': False,
                    'nms_output_name': LaunchConfiguration('nms_output_name'),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return launch.LaunchDescription(
        launch_args +
        [
            container
        ]
    )