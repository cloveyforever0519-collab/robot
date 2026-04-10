# ==============================================================================
# 🚀 物理隔离路线：仿真机动与 UDP 发射端 (彻底根除段错误)
# ==============================================================================
import socket
import json
import numpy as np

from isaacsim import SimulationApp
# 彻底禁用 ROS 2 相关插件，掐断段错误的源头！
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import open_stage, is_stage_loading
from omni.isaac.core.utils.types import ArticulationAction
import omni.timeline
import omni.usd

# 建立 UDP 本地发射器
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_IP = "127.0.0.1"
UDP_PORT = 9999

USD_PATH = "/home/z/imeta_workspace/2_my_brain/custom_urdf/isaac_y1.usd"
open_stage(usd_path=USD_PATH)
while is_stage_loading():
    simulation_app.update()

world = World(stage_units_in_meters=1.0)
robot_path = "/y1/root_joint"
y1_robot = world.scene.add(Articulation(robot_path, name="y1_robot"))

world.reset()
omni.timeline.get_timeline_interface().play()

print(f"🚀 仿真引擎纯净启动！正在通过 UDP 向端口 {UDP_PORT} 疯狂发射数据...")

step_count = 0
while simulation_app.is_running():
    step_count += 1
    current_positions = y1_robot.get_joint_positions()
    
    if current_positions is not None and len(current_positions) == 8:
        # 生成动态测试数据
        current_positions[0] = np.sin(step_count * 0.02) * 0.2 
        
        # 强制夹爪镜像控制
        gripper_target = 0.04 
        current_positions[6] = gripper_target
        current_positions[7] = -gripper_target 
        
        # 物理执行
        action = ArticulationAction(joint_positions=current_positions)
        y1_robot.apply_action(action)
        
        # 核心：通过 UDP 打包发送数据给系统
        payload = {"positions": current_positions.tolist()}
        sock.sendto(json.dumps(payload).encode('utf-8'), (UDP_IP, UDP_PORT))
        
    world.step(render=True)

sock.close()
simulation_app.close()
