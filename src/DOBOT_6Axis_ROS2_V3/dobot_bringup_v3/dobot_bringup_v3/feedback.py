#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from dobot_msgs_v3.msg import ToolVectorActual
from sensor_msgs.msg import JointState
import socket
import numpy as np
import os
import threading
import time
MyType = np.dtype([('len',np.int64,), ('digital_input_bits',np.uint64,), ('digital_output_bits',
    np.uint64,), ('robot_mode',np.uint64,), ('time_stamp',np.uint64,), ( 'time_stamp_reserve_bit', np.uint64,),
    ('test_value',np.uint64,), ('test_value_keep_bit', np.float64,), ('speed_scaling',np.float64,), ('linear_momentum_norm',np.float64,),
    ( 'v_main',np.float64,), ('v_robot',np.float64,), ('i_robot',np.float64,), ('i_robot_keep_bit1',np.float64,), ( 'i_robot_keep_bit2',np.float64,),
    ('tool_accelerometer_values', np.float64, (3, )),
    ('elbow_position', np.float64, (3, )),
    ('elbow_velocity', np.float64, (3, )),
    ('q_target', np.float64, (6, )),
    ('qd_target', np.float64, (6, )),
    ('qdd_target', np.float64, (6, )),
    ('i_target', np.float64, (6, )),
    ('m_target', np.float64, (6, )),
    ('q_actual', np.float64, (6, )),
    ('qd_actual', np.float64, (6, )),
    ('i_actual', np.float64, (6, )),
    ('actual_TCP_force', np.float64, (6, )),
    ('tool_vector_actual', np.float64, (6, )),
    ('TCP_speed_actual', np.float64, (6, )),
    ('TCP_force', np.float64, (6, )),
    ('Tool_vector_target', np.float64, (6, )),
    ('TCP_speed_target', np.float64, (6, )),
    ('motor_temperatures', np.float64, (6, )),
    ('joint_modes', np.float64, (6, )),
    ('v_actual', np.float64, (6, )),
    ('hand_type', np.byte, (4,)),
    ('user', np.byte,),
    ('tool', np.byte,),
    ('run_queued_cmd', np.byte,),
    ('pause_cmd_flag', np.byte,),
    ('velocity_ratio', np.int8,),
    ('acceleration_ratio', np.int8,),
    ('jerk_ratio', np.int8,),
    ('xyz_velocity_ratio', np.int8,),
    ('r_velocity_ratio', np.int8,),
    ('xyz_acceleration_ratio', np.int8,),
    ('r_acceleration_ratio', np.int8,),
    ('xyz_jerk_ratio', np.int8,),
    ('r_jerk_ratio', np.int8,),
    ('brake_status', np.int8,),
    ('enable_status', np.int8,),
    ('drag_status', np.int8,),
    ('running_status', np.int8,),
    ('error_status',np.int8,),
    ('jog_status', np.int8,),
    ('robot_type', np.int8,),
    ('drag_button_signal', np.int8,),
    ('enable_button_signal', np.int8,),
    ('record_button_signal', np.int8,),
    ('reappear_button_signal', np.int8,),
    ('jaw_button_signal', np.int8,),
    ('six_force_online', np.int8,),
    ('reserve2', np.int8, (82,)),
    ('m_actual', np.float64, (6,)),
    ('load', np.float64,),
    ('center_x', np.float64,),
    ('center_y', np.float64,),
    ('center_z', np.float64,),
    ('user1', np.float64, (6,)),
    ('Tool1', np.float64, (6,)),
    ('trace_index', np.float64,),
    ('six_force_value', np.float64, (6,)),
    ('target_quaternion', np.float64, (4,)),
    ('actual_quaternion', np.float64, (4,)),
    ('reserve3',np.int8, (24,))
     ])




class fankuis():
    def __init__(self, ip, port):
        self.ip = ip
        self.port = port
        self.socket_feedback = 0

        if self.port == 30005 or self.port == 30004:
                self.socket_feedback = socket.socket()
                self.socket_feedback.settimeout(1)
                self.socket_feedback.connect((self.ip, self.port))
        else:
            print("Connect to feedback server needs port 30004 or 30005!")
    
    # def feed(self):
    #     try:
    #         self.socket_feedback.setblocking(True)  
    #         self.all = self.socket_feedback.recv(10240)
    #         data = self.all[0:1440]
    #         # print(data)
    #         a = np.frombuffer(data, dtype=MyType)
    #         if hex((a['test_value'][0])) == '0x123456789abcdef':
    #             tool_v = a['tool_vector_actual'][0]
    #             tool_j = a['q_actual'][0]
    #         return [tool_v,tool_j]
    #     except:
    #         return ["NG"]
    
    def feed(self):
        try:
            self.socket_feedback.setblocking(True)
            self.all = self.socket_feedback.recv(10240)
            if len(self.all) < 1440:
                print("Received packet too short:", len(self.all))
                return ["NG"]
            data = self.all[0:1440]
            a = np.frombuffer(data, dtype=MyType)
            #print("Test value:", hex((a['test_value'][0])))
            if hex((a['test_value'][0])) == '0x123456789abcdef':
                tool_v = a['tool_vector_actual'][0]
                tool_j = a['q_actual'][0]
                return [tool_v, tool_j]
            else:
                print("Invalid test_value format")
                return ["NG"]
        except Exception as e:
            print("Feedback exception:", e)
            return ["NG"]

class PublisherNode(Node):
    
    def __init__(self, name):
        super().__init__(name)                                    
        # self.declare_parameter('IP', '192.168.9.1')   
        # self.IP = self.get_parameter('IP').get_parameter_value().string_value
        self.declare_parameter('robot_ip', '172.16.11.34')  # Giá trị mặc định
        self.IP = self.get_parameter('robot_ip').get_parameter_value().string_value
        # feed_v = None khi chưa connect; timer skip gracefully (xem timer_callback).
        # Background thread tự retry mỗi 5s cho tới khi Dobot online.
        self.feed_v = None
        self.pub = self.create_publisher(ToolVectorActual, "dobot_msgs_v3/msg/ToolVectorActual", 10)
        self.pub2 = self.create_publisher(JointState, "joint_states_robot", 10)
        from std_msgs.msg import Int32
        self._Int32 = Int32
        self.pub_hw_speed = self.create_publisher(Int32, '/robot/hw_speed_factor', 10)
        self._last_hw_speed = -1
        self.timer = self.create_timer(0.01, self.timer_callback)
        threading.Thread(target=self._connect_loop, daemon=True, name="feedback_connect").start()

    def _connect_loop(self):
        """Connect port 30004 in background; retry mỗi 5s khi offline.
        Khi feedback socket fail trong timer_callback → set feed_v=None,
        loop này sẽ tự reconnect."""
        while rclpy.ok():
            if self.feed_v is None:
                try:
                    self.get_logger().info(f"⏳ Feedback connect: {self.IP}:30004")
                    self.feed_v = fankuis(self.IP, 30004)
                    self.get_logger().info(f"✅ Feedback {self.IP} connected (30004)")
                except Exception as e:
                    self.get_logger().warn(f"⏳ Feedback {self.IP} offline, retry 5s: {e}")
                    time.sleep(5)
                    continue
            time.sleep(5)
    # def timer_callback(self):                                     
    #     msg = ToolVectorActual()                                           
    #     actual = self.feed_v.feed()
    #     msg2 = JointState()
    #     if actual[0]!= "NG" :                                     
    #        msg2.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
    #        q_target = actual[1]
    #        joint_a = []
    #        for ii in q_target:
    #            joint_a.append(float(ii*3.14159/180))
    #        print(joint_a)
    #        msg2.position = joint_a     
    #        msg.x = actual[0][0]                             
    #        msg.y = actual[0][1]
    #        msg.z = actual[0][2]
    #        msg.rx = actual[0][3]
    #        msg.ry = actual[0][4]
    #        msg.rz =actual[0][5]
    #        self.pub.publish(msg)                                     
    #        self.pub2.publish(msg2)
    def timer_callback(self):
        # Chưa connect (offline hoặc đang reconnect) → skip silently.
        if self.feed_v is None:
            return
        msg = ToolVectorActual()
        actual = self.feed_v.feed()

        if actual == ["NG"]:
            # Feed fail liên tục → reset socket để _connect_loop reconnect.
            if not hasattr(self, '_feed_fail_count'):
                self._feed_fail_count = 0
            self._feed_fail_count += 1
            if self._feed_fail_count >= 10:
                self.get_logger().warn(f"⚠️ Feedback 10 fail liên tiếp — reset socket để reconnect")
                try: self.feed_v.socket_feedback.close()
                except Exception: pass
                self.feed_v = None
                self._feed_fail_count = 0
            return
        self._feed_fail_count = 0

        msg2 = JointState()
        msg2.name = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
        q_target = actual[1]
        joint_a = [float(ii * 3.14159 / 180) for ii in q_target]
        #print(joint_a)
        msg2.position = joint_a     
        msg.x, msg.y, msg.z, msg.rx, msg.ry, msg.rz = actual[0]
        self.pub.publish(msg)
        self.pub2.publish(msg2)

        # Extract and publish hardware speed factor from feedback packet (~every 1s)
        try:
            data = self.feed_v.all[0:1440]
            a = np.frombuffer(data, dtype=MyType)
            # speed_scaling contains the actual global SpeedFactor % (e.g. 12.0 = 12%)
            hw_speed = int(round(float(a['speed_scaling'][0])))
            if 0 <= hw_speed <= 100:
                # Publish every ~1 second (timer runs at 100Hz, so every 100 ticks)
                if not hasattr(self, '_speed_tick'):
                    self._speed_tick = 0
                self._speed_tick += 1
                if hw_speed != self._last_hw_speed or self._speed_tick >= 100:
                    self._last_hw_speed = hw_speed
                    self._speed_tick = 0
                    speed_msg = self._Int32()
                    speed_msg.data = hw_speed
                    self.pub_hw_speed.publish(speed_msg)
        except Exception as e:
            self.get_logger().warn(f"Speed readback error: {e}", throttle_duration_sec=5.0)
        
def main(args=None):                                 
    rclpy.init(args=args)                            
    node = PublisherNode("topic_helloworld_pub")     
    rclpy.spin(node)                                 
    node.destroy_node()                              
    rclpy.shutdown()                                 
