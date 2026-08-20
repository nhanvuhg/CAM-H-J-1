#!/usr/bin/env python3
"""Production dual-camera launch: direct V4L2 + CUDA tone/debayer + YOLO."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, Shutdown
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    cam0_model = LaunchConfiguration('cam0_model')
    cam1_model = LaunchConfiguration('cam1_model')
    enable_inference = LaunchConfiguration('enable_inference')
    use_cuda_camera = LaunchConfiguration('use_cuda_camera')
    capture_fps = LaunchConfiguration('capture_fps')
    publish_fps = LaunchConfiguration('publish_fps')
    max_inference_fps = LaunchConfiguration('max_inference_fps')
    cam0_device = LaunchConfiguration('cam0_device')
    cam1_device = LaunchConfiguration('cam1_device')
    roi_anchor_mode = LaunchConfiguration('roi_anchor_mode')
    # The camera node owns both V4L2 devices and performs the validated CUDA
    # debayer/tone path. It publishes BGR8 640x360 with depth=1 semantics.
    camera_topic_parameters = {
        'cam0_topic': '/cam0HP/image_raw',
        'cam1_topic': '/cam1HP/image_raw',
        'cam0_health_topic': '/camera/cam0/health',
        'cam1_health_topic': '/camera/cam1/health',
        # Current physical wiring: input camera is video0, output is video1.
        'cam0_device': ParameterValue(cam0_device, value_type=int),
        'cam1_device': ParameterValue(cam1_device, value_type=int),
        # Small extra luma-edge lift after the 1920x1080 -> 640x360
        # reduction; chroma denoise and edge threshold stay unchanged.
        'cam0_clarity': 0.45,
        'cam1_clarity': 0.60,
    }

    cuda_camera_node = Node(
        package='csi_camera',
        executable='v4l2_dual_camera_cuda_node',
        name='v4l2_dual_camera',
        output='screen',
        parameters=[{
            **camera_topic_parameters,
            'capture_fps': ParameterValue(capture_fps, value_type=int),
            'publish_fps': ParameterValue(publish_fps, value_type=int),
            'exposure': 43000,
        }],
        condition=IfCondition(use_cuda_camera),
        respawn=False,
        # A dead capture process must end the whole launch (YOLO/overlay nodes
        # otherwise keep ros2 launch alive with stale topic names forever).
        on_exit=[Shutdown(reason='CUDA dual-camera capture exited')],
    )

    # V4L2 CPU path is retained only as a diagnostic rollback. It uses the
    # same direct sensor access and never changes the production default.
    cpu_camera_node = Node(
        package='csi_camera',
        executable='v4l2_dual_camera_node',
        name='v4l2_dual_camera',
        output='screen',
        parameters=[camera_topic_parameters],
        condition=UnlessCondition(use_cuda_camera),
        respawn=False,
        on_exit=[Shutdown(reason='CPU dual-camera capture exited')],
    )
    
    # ================================================================
    # 2. TWO TENSORRT YOLO NODES
    # ================================================================
    # Separate processes give each camera its own TensorRT execution context and
    # CUDA stream. SensorDataQoS depth=1 drops stale frames instead of building
    # latency when the GPU is temporarily busy.
    yolo_cam0_node = Node(
        package='yolo_tensorrt_ros2',
        executable='yolo_tensorrt_node',
        name='yolo_cam0',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        condition=IfCondition(enable_inference),
        parameters=[{
            'engine_path': cam0_model,
            'image_topic': '/cam0HP/image_raw',
            'detections_topic': '/cam0HP/yolo/bounding_boxes',
            'health_topic': '/camera/cam0/ai_health',
            'class_names': ['tray', 'cartridge'],
            'confidence_threshold': 0.60,
            'nms_threshold': 0.45,
            'max_inference_fps': ParameterValue(max_inference_fps, value_type=float),
        }],
    )

    yolo_cam1_node = Node(
        package='yolo_tensorrt_ros2',
        executable='yolo_tensorrt_node',
        name='yolo_cam1',
        output='screen',
        respawn=True,
        respawn_delay=3.0,
        condition=IfCondition(enable_inference),
        parameters=[{
            'engine_path': cam1_model,
            'image_topic': '/cam1HP/image_raw',
            'detections_topic': '/cam1HP/yolo/bounding_boxes',
            'health_topic': '/camera/cam1/ai_health',
            'class_names': ['tray', 'cartridge', 'cartridgefall'],
            # Khay thanh pham: loai thang detection duoi 0.60 cho cartridge va
            # cartridgefall. vision_decision_node von da bo qua chung o nguong
            # DETECTION_SCORE_THRESH = 0.60, nhung nguong 0.30 o day van cho
            # chung duoc publish va ve len overlay cam1 — operator thay bbox ma
            # he thong khong tinh. Nang len 0.60 cho khop cam0 va khop dung logic
            # quyet dinh. Lop tray khong anh huong: no can >= 0.86
            # (OUTPUT_TRAY_SCORE_THRESH) nen van qua duoc cong 0.60 nay.
            'confidence_threshold': 0.60,
            'nms_threshold': 0.45,
            'max_inference_fps': ParameterValue(max_inference_fps, value_type=float),
        }],
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
            'image_height': 360,
            # 'tray' = ROI bam theo bbox class-0 cua khay, bu camera xe dich.
            # Bat rieng tung khay: khay nao co 'anchor' trong vision_roi.yaml
            # thi dung, khay chua co thi tu chay ROI tinh. Quay lai hoan toan
            # hanh vi cu bang roi_anchor_mode:=static.
            'roi_anchor_mode': roi_anchor_mode,
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
            default_value='true',
            description='Start the Jetson TensorRT YOLO/overlay/vision stack',
        ),
        DeclareLaunchArgument(
            'use_cuda_camera',
            default_value='true',
            description='Use the production V4L2 CUDA camera; false selects V4L2 CPU diagnostics',
        ),
        DeclareLaunchArgument(
            'max_inference_fps',
            # Khay cartridge gan nhu tinh, 20 luot suy luan/giay/camera la thua
            # xa nhu cau. Gioi han 8 Hz de giu GPU headroom cho CUDA camera/GUI.
            # Overlay phat theo MOI khung anh vao va dung latest_boxes_ da
            # luu, nen FPS hien thi KHONG giam theo — chi bbox cap nhat thua hon.
            default_value='8.0',
            description='Maximum TensorRT inference rate per camera; 0 disables limiting',
        ),
        DeclareLaunchArgument(
            'capture_fps',
            default_value='15',
            description='Rate used to drain both V4L2 camera channels',
        ),
        DeclareLaunchArgument(
            'publish_fps',
            default_value='8',
            description='CUDA process/publish rate per camera; must be <= capture_fps',
        ),
        DeclareLaunchArgument(
            'cam0_device',
            default_value='0',
            description='V4L2 index for logical CAM0 input-tray camera',
        ),
        DeclareLaunchArgument(
            'cam1_device',
            default_value='1',
            description='V4L2 index for logical CAM1 output-tray camera',
        ),
        DeclareLaunchArgument(
            'roi_anchor_mode',
            default_value='tray',
            description="'tray' anchors ROIs to the detected tray box; "
                        "'static' restores fixed image-space ROIs",
        ),
        DeclareLaunchArgument(
            'cam0_model',
            # Retrained on 640x360 frames from this camera; _2 is the same
            # dataset relabelled. The original data_input_hp1.engine predated
            # the Jetson image path and found 30 of 40 cartridges on a full
            # tray, missing the two far rows and the glare-blown right column.
            default_value='/home/nhan/models/data_input_hp_final_2_fp16.engine',
            description='TensorRT engine used by camera 0 input-tray inference',
        ),
        DeclareLaunchArgument(
            'cam1_model',
            default_value='/home/nhan/models/data_output_hp_final_fp16.engine',
            description='TensorRT engine used by camera 1 output-tray inference',
        ),
        cuda_camera_node,
        cpu_camera_node,
        yolo_cam0_node,
        yolo_cam1_node,
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
  │  /dev/video0 → logical CAM0 → rack/input      │
  │  /dev/video1 → logical CAM1 → green output    │
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
  
  # Check YOLO detections
  ros2 topic echo /cam0HP/yolo/bounding_boxes
  ros2 topic echo /cam1HP/yolo/bounding_boxes
  
  # View live feeds
  ros2 run rqt_image_view rqt_image_view
    → Select /cam0HP/image_overlay
    → Select /cam1HP/image_overlay

TensorRT inference is enabled by default:
  ros2 launch csi_camera dual_camera_system.launch.py

Camera-only diagnostic launch:
  ros2 launch csi_camera dual_camera_system.launch.py enable_inference:=false

================================================================================
"""
