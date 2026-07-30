#!/usr/bin/env python3
"""
dual_camera_system.launch.py

Dual IMX477 camera system for NVIDIA Jetson Orin Nano.

Camera capture is available by default.  The legacy Hailo inference stack is
optional because this Jetson does not have a Hailo device/runtime installed.

Both cameras run in parallel, no switching required.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    cam0_model = LaunchConfiguration('cam0_model')
    cam1_model = LaunchConfiguration('cam1_model')
    enable_inference = LaunchConfiguration('enable_inference')
    
    # ================================================================
    # 1. DUAL CSI CAMERA NODE (NVIDIA Argus + nvvidconv)
    # ================================================================
    # Match the known-good V3Link configuration in camera-2x.desktop:
    # IMX477 sensor-mode 2, 1920x1080@30, edge enhancement disabled and ISP
    # digital gain locked to 1.0. nvvidconv resizes to 640x480 before host copy.
    # Publishes:
    #   - /cam0HP/image_raw (Camera 0 - Input Tray)
    #   - /cam1HP/image_raw (Camera 1 - Output Tray)
    
    common_camera_parameters = {
        'sensor_mode': 2,
        'capture_width': 1920,
        'capture_height': 1080,
        'capture_fps': 30,
        'publish_fps': 10,
        'output_width': 640,
        'output_height': 480,
        'ee_mode': 0,
        'isp_digital_gain_min': 1.0,
        'isp_digital_gain_max': 1.0,
        'reconnect_delay_ms': 2000,
    }

    # Each camera is a separate OS process. Start cam1 first (same order as the
    # known-good camera-2x.desktop), then open cam0 after Argus has completed the
    # first session setup. Both remain active in parallel after startup.
    cam1_camera_node = Node(
        package='csi_camera',
        executable='csi_camera_node',
        name='cam1_csi_camera',
        output='screen',
        parameters=[common_camera_parameters, {
            'sensor_id': 1,
            'image_topic': '/cam1HP/image_raw',
            'health_topic': '/camera/cam1/health',
            'frame_id': 'cam1_optical_frame',
        }],
        respawn=True,
        respawn_delay=5.0,
    )

    cam0_camera_node = Node(
        package='csi_camera',
        executable='csi_camera_node',
        name='cam0_csi_camera',
        output='screen',
        parameters=[common_camera_parameters, {
            'sensor_id': 0,
            'image_topic': '/cam0HP/image_raw',
            'health_topic': '/camera/cam0/health',
            'frame_id': 'cam0_optical_frame',
        }],
        respawn=True,
        respawn_delay=5.0,
    )
    
    # ================================================================
    # 2. YOLO CONTAINER WITH 2 MODELS
    # ================================================================
    # Both YOLO nodes run simultaneously, processing their respective camera feeds
    
    yolo_container = ComposableNodeContainer(
        name='yolo_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',  # multi-threaded: allows cam0/cam1 inference overlap
        composable_node_descriptions=[
            # YOLO for Camera 0 (Input Tray Detection)
            ComposableNode(
                package='yolo_ros_hailort_cpp',
                plugin='yolo_ros_hailort_cpp::YoloNode',
                name='yolo_cam0',
                parameters=[{
                    'model_path': cam0_model,
                    'nms_output_name': 'yolov8s/yolov8_nms_postprocess',
                    'src_image_topic_name': '/cam0HP/image_raw',
                    'publish_boundingbox_topic_name': '/cam0HP/yolo/bounding_boxes',
                    'publish_image_topic_name': '/cam0HP/yolo/image_raw',
                    'conf': 0.60,
                    'publish_resized_image': False,
                    # Camera 640x480 (4:3), HEF 640x640. Letterbox giu nguyen
                    # hinh dang cartridge va mapping bbox ve anh goc.
                    'letterbox': True,
                }]
            ),
            # YOLO for Camera 1 (Output Tray Detection)
            ComposableNode(
                package='yolo_ros_hailort_cpp',
                plugin='yolo_ros_hailort_cpp::YoloNode',
                name='yolo_cam1',
                parameters=[{
                    'model_path': cam1_model,
                    'nms_output_name': 'yolov8s/yolov8_nms_postprocess',
                    'src_image_topic_name': '/cam1HP/image_raw',
                    'publish_boundingbox_topic_name': '/cam1HP/yolo/bounding_boxes',
                    'publish_image_topic_name': '/cam1HP/yolo/image_raw',
                    'conf': 0.30,
                    'publish_resized_image': False,
                    # Camera is 640x480 (4:3) but this HEF takes 640x640. Stretching
                    # squashes cartridges enough to drop them below conf: measured
                    # 0.25 stretched vs 0.82 letterboxed on the same frame.
                    'letterbox': True,
                }]
            ),
        ],
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        condition=IfCondition(enable_inference),
    )
    
    # ================================================================
    # 3. BBOX DRAWER (Visualization)
    # ================================================================
    # Overlays bounding boxes on both camera feeds for debugging
    
    bbox_drawer_node = Node(
        package='bbox_drawer_cpp',
        executable='overlay_bboxes_node',
        name='overlay_dual_cam',
        output='screen',
        parameters=[{
            # Camera 0 (Input Tray)
            'cam0.image_topic': '/cam0HP/image_raw',
            'cam0.boxes_topic': '/cam0HP/yolo/bounding_boxes',
            'cam0.output_topic': '/cam0HP/image_overlay',
            'cam0.output_width': 640,
            'cam0.output_height': 480,
            
            # Camera 1 (Output Tray)
            'cam1.image_topic': '/cam1HP/image_raw',
            'cam1.boxes_topic': '/cam1HP/yolo/bounding_boxes',
            'cam1.output_topic': '/cam1HP/image_overlay',
            'cam1.output_width': 640,
            'cam1.output_height': 480,
        }],
        respawn=True,
        respawn_delay=2.0,
        condition=IfCondition(enable_inference),
    )
    
    # ================================================================
    # 4. VISION DECISION NODE (Row/Slot Selection from YOLO)
    # ================================================================
    # Converts YOLO bounding boxes → robot control decisions
    # Publishes:
    #   - /vision/input_tray/selected_row  (Int32)
    #   - /vision/input_tray/row_status    (Int32MultiArray)
    #   - /vision/input_tray/empty         (Bool)
    #   - /vision/output_tray/selected_slot (Int32)
    #   - /vision/output_tray/slot_status  (Int32MultiArray)
    #   - /vision/output_tray/full         (Bool)
    #   - /vision/heartbeat                (Header)

    vision_decision_node = Node(
        package='robot_control_main',
        executable='vision_decision_node',
        name='vision_decision_node',
        output='screen',
        parameters=[{
            # He toa do bbox = anh camera publish (output_width/height o tren).
            # ROI trong vision_roi.yaml duoc scale tu ref_* cua no sang day;
            # doi output size camera thi PHAI doi cap so nay theo.
            'image_width': 640,
            'image_height': 480,
            # Cam0 chi hop le khi truc InX da ve vi tri nhan khay cua robot.
            'inx_camera_position_mm': -60.0,
            'inx_camera_tolerance_mm': 2.0,
            # 'roi_config': mac dinh <share>/robot_control_main/config/vision_roi.yaml
        }],
        respawn=True,
        respawn_delay=3.0,
        condition=IfCondition(enable_inference),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'enable_inference',
            default_value='false',
            description='Start the optional Hailo YOLO/overlay/vision stack',
        ),
        DeclareLaunchArgument(
            'cam0_model',
            default_value='/home/nhan/models/DataTrayInputHP_2.hef',
            description='HEF model used by camera 0 input-tray inference',
        ),
        DeclareLaunchArgument(
            'cam1_model',
            default_value='/home/nhan/models/DataTrayProdutionHP_1.hef',
            description='HEF model used by camera 1 output-tray inference',
        ),
        cam1_camera_node,
        TimerAction(period=3.0, actions=[cam0_camera_node]),
        yolo_container,
        bbox_drawer_node,
        vision_decision_node,
    ])

"""
================================================================================
DUAL IMX477 CAMERA SYSTEM
================================================================================

Hardware:
  ┌──────────────────────────────────────────────┐
  │  NVIDIA Jetson Orin Nano                      │
  │                                               │
  │  Argus sensor-id 0 → IMX477 → Input Tray     │
  │  Argus sensor-id 1 → IMX477 → Output Tray    │
  └──────────────────────────────────────────────┘

Topic Flow:
  Camera 0 Thread → /cam0HP/image_raw → YOLO cam0 → /cam0HP/yolo/bounding_boxes
                                                   ↓
                                              bbox_drawer → /cam0HP/image_overlay
  
  Camera 1 Thread → /cam1HP/image_raw → YOLO cam1 → /cam1HP/yolo/bounding_boxes
                                                   ↓
                                              bbox_drawer → /cam1HP/image_overlay

Launch:
  ros2 launch csi_camera dual_camera_system.launch.py

Verify:
  # Check both cameras publishing
  ros2 topic hz /cam0HP/image_raw
  ros2 topic hz /cam1HP/image_raw
  
  # Check YOLO detections (only with enable_inference:=true)
  ros2 topic echo /cam0HP/yolo/bounding_boxes
  ros2 topic echo /cam1HP/yolo/bounding_boxes
  
  # View live feeds
  ros2 run rqt_image_view rqt_image_view
    → Select /cam0HP/image_overlay
    → Select /cam1HP/image_overlay

Camera-only launch is the default on Jetson:
  ros2 launch csi_camera dual_camera_system.launch.py

Hailo inference requires Hailo hardware/runtime and built YOLO packages:
  ros2 launch csi_camera dual_camera_system.launch.py enable_inference:=true

================================================================================
"""
