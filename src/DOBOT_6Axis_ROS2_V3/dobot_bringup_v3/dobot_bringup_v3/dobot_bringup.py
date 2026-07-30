#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from dobot_msgs_v3.srv import *
from .dobot_api import *
import os
import time
import threading

# Explicitly import the SRV class to shadow the dobot_api function of the same name
from dobot_msgs_v3.srv import RelMovLUser


# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
# ⚠️  CRITICAL ZONE — đọc memory feedback_critical_code_zones.md trước khi sửa.
# INVARIANT: __init__ KHÔNG block connect. self.dashboard = _NotConnected() stub
# + spawn _connect_loop daemon thread. Service handlers dùng self.dashboard.XXX()
# luôn an toàn (stub trả error string parse được).
# Đừng "đơn giản hóa" thành self.dashboard = DobotApiDashboard(...) trong __init__
# — sẽ crash node với AttributeError 'no attribute dashboard' khi Dobot offline.
# Bug history: trước khi có stub, Dobot offline → process die → restart toàn stack.
# ━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
class _NotConnected:
    """Stub dùng cho self.dashboard / self.move khi Dobot chưa kết nối.
    Mọi method gọi vào đây trả về string error code -1, parse OK qua
    _parse_error_code → service không crash node. Background _connect_loop
    sẽ thay self.dashboard bằng instance thật ngay khi Dobot online lại."""
    def __getattr__(self, name):
        def _stub(*args, **kwargs):
            return f"-1,{{}},{name}();"
        return _stub


class adderServer(Node):
    def __init__(self, name):
        super().__init__(name)   
        # self.declare_parameter('IP', '192.168.27.1')  
        # self.IP = self.get_parameter('IP').get_parameter_value().string_value  
        self.declare_parameter('robot_ip', '172.16.11.34')  # Giá trị mặc định
        self.IP = self.get_parameter('robot_ip').get_parameter_value().string_value
        # self.IP = str(os.getenv("IP_address"))
        self.get_logger().info(self.IP)                                                  
        # self.srv = self.create_service(AO,  self.get_name() + '/AO', self.AO)
        self.srv = self.create_service(AccJ, self.get_name() + '/AccJ',self.AccJ)
        self.srv = self.create_service(AccL, self.get_name() + '/AccL',self.AccL)
        self.srv = self.create_service(Arch, self.get_name() + '/Arch',self.Arch)
        # self.srv = self.create_service(BrakeControl, self.get_name() + '/BrakeControl',self.BrakeControl)
        self.srv = self.create_service(CP, self.get_name() + '/CP',self.CP)
        self.srv = self.create_service(ClearError, self.get_name() + '/ClearError',self.ClearError)
        self.srv = self.create_service(Continues, self.get_name() + '/Continue', self.Continue)
        # self.srv = self.create_service(ContinueScript, self.get_name() + '/ContinueScript',self.ContinueScript)
        self.srv = self.create_service(DI, self.get_name() + '/DI',self.DI)
        self.srv = self.create_service(DO, self.get_name() + '/DO',self.DO)
        self.srv = self.create_service(DOExecute, self.get_name() + '/DOExecute',self.DOExecute)
        self.srv = self.create_service(DOGroup, self.get_name() + '/DOGroup',self.DOGroup)
        self.srv = self.create_service(DisableRobot, self.get_name() + '/DisableRobot',self.DisableRobot)
        self.srv = self.create_service(EmergencyStop, self.get_name() + '/EmergencyStop',self.EmergencyStop)
        self.srv = self.create_service(EnableRobot, self.get_name() + '/EnableRobot',self.EnableRobot)
        self.srv = self.create_service(GetAngle, self.get_name() + '/GetAngle',self.GetAngle)
        self.srv = self.create_service(GetCoils, self.get_name() + '/GetCoils',self.GetCoils)
        self.srv = self.create_service(GetErrorID, self.get_name() + '/GetErrorID',self.GetErrorID)
        self.srv = self.create_service(GetHoldRegs, self.get_name() + '/GetHoldRegs',self.GetHoldRegs)
        self.srv = self.create_service(GetInBits, self.get_name() + '/GetInBits',self.GetInBits)
        self.srv = self.create_service(GetInRegs, self.get_name() + '/GetInRegs',self.GetInRegs)
        self.srv = self.create_service(GetPose, self.get_name() + '/GetPose',self.GetPose)
        # self.srv = self.create_service(InverseSolution, self.get_name() + '/InverseSolution',self.InverseSolution)
        # self.srv = self.create_service(LimZ, self.get_name() + '/LimZ',self.LimZ)
        # self.srv = self.create_service(LoadSwitch, self.get_name() + '/LoadSwitch',self.LoadSwitch)
        self.srv = self.create_service(ModbusClose, self.get_name() + '/ModbusClose',self.ModbusClose)
        self.srv = self.create_service(ModbusCreate, self.get_name() + '/ModbusCreate',self.ModbusCreate)
        # self.srv = self.create_service(PauseScript, self.get_name() + '/PauseScript',self.PauseScript)
        self.srv = self.create_service(PayLoad, self.get_name() + '/PayLoad',self.PayLoad)
        # self.srv = self.create_service(PositiveSolution, self.get_name() + '/PositiveSolution',self.PositiveSolution)
        self.srv = self.create_service(ResetRobot, self.get_name() + '/ResetRobot',self.ResetRobot)
        self.srv = self.create_service(RobotMode, self.get_name() + '/RobotMode',self.RobotMode)
        # self.srv = self.create_service(RunScript, self.get_name() + '/RunScript',self.RunScript)
        # self.srv = self.create_service(SetArmOrientation, self.get_name() + '/SetArmOrientation',self.SetArmOrientation)
        self.srv = self.create_service(SetCoils, self.get_name() + '/SetCoils',self.SetCoils)
        # self.srv = self.create_service(SetCollisionLevel, self.get_name() + '/SetCollisionLevel',self.SetCollisionLevel)
        self.srv = self.create_service(SetHoldRegs, self.get_name() + '/SetHoldRegs',self.SetHoldRegs)
        self.srv = self.create_service(SetPayload, self.get_name() + '/SetPayload',self.SetPayload)
        self.srv = self.create_service(SpeedFactor, self.get_name() + '/SpeedFactor',self.SpeedFactor)
        self.srv = self.create_service(SpeedJ, self.get_name() + '/SpeedJ',self.SpeedJ)
        self.srv = self.create_service(SpeedL, self.get_name() + '/SpeedL',self.SpeedL)
        # self.srv = self.create_service(StartDrag, self.get_name() + '/StartDrag',self.StartDrag)
        # self.srv = self.create_service(StopDrag, self.get_name() + '/StopDrag',self.StopDrag)
        self.srv = self.create_service(StopScript, self.get_name() + '/StopScript',self.StopScript)
        self.srv = self.create_service(Tool, self.get_name() + '/Tool',self.Tool)
        self.srv = self.create_service(ToolDI, self.get_name() + '/ToolDI',self.ToolDI)
        self.srv = self.create_service(ToolDO, self.get_name() + '/ToolDO',self.ToolDO)
        self.srv = self.create_service(ToolDOExecute, self.get_name() + '/ToolDOExecute',self.ToolDOExecute)
        self.srv = self.create_service(User, self.get_name() + '/User',self.User)
        # self.srv = self.create_service(Arc, self.get_name() + '/Arc',self.Arc)
        # self.srv = self.create_service(Circle, self.get_name() + '/Circle',self.Circle)
        self.srv = self.create_service(JointMovJ, self.get_name() + '/JointMovJ',self.JointMovJ)
        # self.srv = self.create_service(Jump, self.get_name() + '/Jump',self.Jump)
        self.srv = self.create_service(MovJ, self.get_name() + '/MovJ',self.MovJ)
        # self.srv = self.create_service(MovJExt, self.get_name() + '/MovJExt',self.MovJExt)
        self.srv = self.create_service(MovJIO, self.get_name() + '/MovJIO',self.MovJIO)
        self.srv = self.create_service(MovL, self.get_name() + '/MovL',self.MovL)
        self.srv = self.create_service(ServoJ, self.get_name() + '/ServoJ',self.ServoJ)
        self.srv = self.create_service(ServoP, self.get_name() + '/ServoP',self.ServoP)
        self.srv = self.create_service(MovLIO, self.get_name() + '/MovLIO',self.MovLIO)
        self.srv = self.create_service(MoveJog, self.get_name() + '/MoveJog',self.MoveJog)
        # self.srv = self.create_service(RelJointMovJ, self.get_name() + '/RelJointMovJ',self.RelJointMovJ)
        self.srv = self.create_service(RelMovJ, self.get_name() + '/RelMovJ',self.RelMovJ)
        # self.srv = self.create_service(RelMovJUser, self.get_name() + '/RelMovJUser',self.RelMovJUser)
        self.srv = self.create_service(RelMovL, self.get_name() + '/RelMovL',self.RelMovL)
        self.srv = self.create_service(RelMovLUser, self.get_name() + '/RelMovLUser',self.RelMovLUser)
        self.srv = self.create_service(Sync, self.get_name() + '/Sync',self.Sync)
        # self.srv = self.create_service(SyncAll, self.get_name() + '/SyncAll',self.SyncAll)
        self.srv = self.create_service(Pause, self.get_name() + '/Pause',self.Pause)
        # self.srv = self.create_service(Wait, self.get_name() + '/',self.Wait)
        # Init với stub — services ready ngay, không crash khi Dobot offline.
        self.dashboard = _NotConnected()
        self.move = _NotConnected()
        # Background reconnect: thử mỗi 5s tới khi connect được.
        threading.Thread(target=self._connect_loop, daemon=True, name="dobot_reconnect").start()

    def _connect_loop(self):
        """Background loop — connect Dobot khi online, retry mỗi 5s khi offline.
        Khi connect thành công, swap self.dashboard + self.move từ stub sang
        instance thật → tất cả service handler tự động hoạt động không cần sửa."""
        while rclpy.ok():
            if not isinstance(self.dashboard, _NotConnected):
                # Đã connect → idle, để service handler tự dùng. Chu kỳ này
                # không ping vì DobotApi không có lightweight heartbeat;
                # disconnect sẽ được phát hiện khi service call fail.
                time.sleep(10)
                continue
            try:
                self.get_logger().info(f"⏳ Dobot connect attempt: {self.IP}:29999")
                d = DobotApiDashboard(self.IP, 29999)
            except Exception as e:
                self.get_logger().warn(f"⏳ Dobot {self.IP} offline, retry 5s: {e}")
                time.sleep(5)
                continue

            # Connected — Stop → ClearError → EnableRobot.
            for label, fn in (
                ("Stop", lambda: d.sendRecvMsg("Stop()")),
                ("ClearError", d.ClearError),
                ("EnableRobot", d.EnableRobot),
                ("RobotMode", d.RobotMode),
            ):
                try:
                    self.get_logger().info(f"{label}: {fn()}")
                    if label in ("Stop", "EnableRobot"):
                        time.sleep(1 if label == "Stop" else 2)
                except Exception as e:
                    self.get_logger().warn(f"⚠️ {label} failed: {e}")

            self.dashboard = d
            self.move = d  # new firmware: motion shares dashboard port 29999
            self.get_logger().info(f"✅ Dobot {self.IP} connected (29999)")

    def _parse_error_code(self, return_t):
        """Safely parse error code from robot response.
        Expected format: 'ErrorID,{...},CommandName();'
        Returns error code as int, or -99999 if parsing fails.
        """
        try:
            idx = return_t.find("{")
            if idx > 0:
                return int(return_t[:idx-1].strip())
            else:
                # Non-standard response (e.g. 'Control Mode Is Not Tcp')
                self.get_logger().error(f"⚠️ Non-standard response: {return_t}")
                return -99999
        except (ValueError, IndexError) as e:
            self.get_logger().error(f"⚠️ Failed to parse response '{return_t}': {e}")
            return -99999

    def EnableRobot(self, request, response):                                           
        return_t = self.dashboard.EnableRobot([request.load])
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                        
        return response 
    
    def ClearError(self, request, response):                                          
        return_t = self.dashboard.ClearError()
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                        
        return response 
    
    def ResetRobot(self, request, response):                                          
        return_t = self.dashboard.ResetRobot()
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                        
        return response 

    def EmergencyStop(self, request, response):
        return_t = self.dashboard.EmergencyStop()
        response.res = self._parse_error_code(return_t)
        self.get_logger().warn(return_t)
        return response

    def StopScript(self, request, response):
        return_t = self.dashboard.StopScript()
        response.res = self._parse_error_code(return_t)
        self.get_logger().warn(return_t)
        return response

    def Pause(self, request, response):
        return_t = self.dashboard.sendRecvMsg("Pause()")
        response.res = self._parse_error_code(return_t)
        self.get_logger().warn(return_t)
        return response

    def Continue(self, request, response):
        return_t = self.dashboard.Continue()
        response.res = self._parse_error_code(return_t)
        self.get_logger().info(return_t)
        return response
    
    def PayLoad(self, request, response):                                          
        return_t = self.dashboard.PayLoad(request.weight,request.inertia)
        response.res = self._parse_error_code(return_t)                                        
        self.get_logger().info(return_t)                                        
        return response 
    
    def SetPayload(self, request, response):                                          
        return_t = self.dashboard.SetPayload(request.weight,request.inertia)
        response.res = self._parse_error_code(return_t)                                         
        self.get_logger().info(return_t)                                        
        return response 
    
    def GetPose(self, request, response):                                          
        return_t = self.dashboard.GetPose(request.user,request.tool)
        response.res = self._parse_error_code(return_t)   
        response.pose = return_t[return_t.find("{"):return_t.find("}")+1]                                         
        self.get_logger().info(return_t)                                        
        return response 
    
    def GetAngle(self, request, response):                                           
        return_t = self.dashboard.GetAngle()
        response.res = self._parse_error_code(return_t)
        response.angle = return_t[return_t.find("{"):return_t.find("}")+1]                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def RobotMode(self, request, response):                                           
        return_t = self.dashboard.RobotMode()
        response.res = self._parse_error_code(return_t)
        response.mode = return_t[return_t.find("{")+1:return_t.find("}")]                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def ModbusCreate(self, request, response):                                           
        return_t = self.dashboard.ModbusCreate(request.ip,request.port,request.slave_id,request.is_rtu)
        response.res = self._parse_error_code(return_t)
        response.index = return_t[return_t.find("{")+1:return_t.find("}")]                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def GetInBits(self, request, response):                                           
        return_t = self.dashboard.GetInBits(request.index,request.addr,request.count)
        response.res = self._parse_error_code(return_t)
        response.value = return_t[return_t.find("{")+1:return_t.find("}")]                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def GetInRegs(self, request, response):                                           
        return_t = self.dashboard.GetInRegs(request.index,request.addr,request.count,request.val_type)
        response.res = self._parse_error_code(return_t)
        response.value = return_t[return_t.find("{")+1:return_t.find("}")]                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def GetHoldRegs(self, request, response):                                           
        return_t = self.dashboard.GetHoldRegs(request.index,request.addr,request.count,request.val_type)
        response.res = self._parse_error_code(return_t)
        response.value = return_t[return_t.find("{")+1:return_t.find("}")]                                           
        self.get_logger().info(return_t)                                     
        return response
    
    def GetCoils(self, request, response):                                           
        return_t = self.dashboard.GetCoils(request.index,request.addr,request.count)
        response.res = self._parse_error_code(return_t)
        response.value = return_t[return_t.find("{")+1:return_t.find("}")]                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def SetCoils(self, request, response):                                          
        return_t = self.dashboard.SetCoils(request.index,request.addr,request.count,request.val_tab)
        response.res = self._parse_error_code(return_t)                                        
        self.get_logger().info(return_t)                                     
        return response 
    
    def SetHoldRegs(self, request, response):                                           
        return_t = self.dashboard.SetHoldRegs(request.index,request.addr,request.count,request.val_tab,request.val_type)
        response.res = self._parse_error_code(return_t)                                        
        self.get_logger().info(return_t)                                     
        return response 
    
    def ModbusClose(self, request, response):                                           
        return_t = self.dashboard.ModbusClose(request.index)
        response.res = self._parse_error_code(return_t)                                         
        self.get_logger().info(return_t)                                     
        return response 
    
    def GetErrorID(self, request, response):                                           
        return_t = self.dashboard.GetErrorID()
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def DisableRobot(self, request, response):                                           
        return_t = self.dashboard.DisableRobot()
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def DOExecute(self, request, response):                                       
        return_t = self.dashboard.DOExecute(request.index,request.status)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def SpeedFactor(self, request, response):                                       
        return_t = self.dashboard.SpeedFactor(request.ratio)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def CP(self, request, response):                                       
        return_t = self.dashboard.CP(request.r)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def SpeedJ(self, request, response):                                       
        return_t = self.dashboard.SpeedJ(request.r)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def SpeedL(self, request, response):                                       
        return_t = self.dashboard.SpeedL(request.r)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 

    def Tool(self, request, response):                                       
        return_t = self.dashboard.Tool(request.index)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def User(self, request, response):                                       
        return_t = self.dashboard.User(request.index)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def DOGroup(self, request, response):                                       
        return_t = self.dashboard.DOGroup(request.args)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def DO(self, request, response):                                       
        return_t = self.dashboard.DO(request.index,request.status)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def DI(self, request, response):                                       
        return_t = self.dashboard.ToolDO(request.index)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def ToolDO(self, request, response):                                       
        return_t = self.dashboard.ToolDO(request.index,request.status)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def ToolDOExecute(self, request, response):                                       
        return_t = self.dashboard.ToolDOExecute(request.index,request.status)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def ToolDI(self, request, response):                                       
        return_t = self.dashboard.ToolDI(request.index)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 

    def AccJ(self, request, response):                                     
        return_t = self.dashboard.AccJ(request.r)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def AccL(self, request, response):                                      
        return_t = self.dashboard.AccL(request.r)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def Arch(self, request, response):                                        
        return_t = self.dashboard.Arch(request.index)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def MovJ(self, request, response):                                
        return_t = self.move.MovJ(request.x,request.y,request.z,request.rx,request.ry,request.rz,request.param_value)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def ServoP(self, request, response):                                
        return_t = self.move.ServoP(request.x,request.y,request.z,request.rx,request.ry,request.rz)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    
    def ServoJ(self, request, response):                                
        return_t = self.move.ServoJ(request.j1,request.j2,request.j3,request.j4,request.j5,request.j6,request.t,request.param_value)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 

    def MovL(self, request, response):                                
        return_t = self.move.MovL(request.x,request.y,request.z,request.rx,request.ry,request.rz,request.param_value)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    def MovJIO(self, request, response):                                
        return_t = self.move.MovJIO(request.x,request.y,request.z,request.rx,request.ry,request.rz,request.param_value)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    def MovLIO(self, request, response):                                
        return_t = self.move.MovLIO(request.x,request.y,request.z,request.rx,request.ry,request.rz,request.param_value)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    def JointMovJ(self, request, response):                                
        return_t = self.move.JointMovJ(request.j1,request.j2,request.j3,request.j4,request.j5,request.j6,request.param_value)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    def RelMovJ(self, request, response):                                
        return_t = self.move.RelMovJ(request.offset1,request.offset2,request.offset3,request.offset4,request.offset5,request.offset6,request.param_value)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    def RelMovL(self, request, response):                               
        return_t = self.move.RelMovL(request.offset1,request.offset2,request.offset3,request.offset4,request.offset5,request.offset6,request.param_value)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    def RelMovLUser(self, request, response):                               
        return_t = self.move.RelMovLUser(request.offset1,request.offset2,request.offset3,request.offset4,request.offset5,request.offset6,request.user,request.param_value)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    def Sync(self, request, response):                                
        return_t = self.move.Sync()
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 
    def MoveJog(self, request, response):                                
        return_t = self.move.MoveJog(request.axis_id,request.param_value)
        response.res = self._parse_error_code(return_t)                                           
        self.get_logger().info(return_t)                                     
        return response 


def main(args=None):                                 
    rclpy.init(args=args)                           
    node = adderServer("dobot_bringup_v3")      
    rclpy.spin(node)                                
    node.destroy_node()                              
    rclpy.shutdown()                                
