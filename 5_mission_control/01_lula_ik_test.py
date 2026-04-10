# ==============================================================================
# 🚀 Y1 路线 C - 代码 01: 纯净物理与小脑基准测试 (5.1.0 不坏金身版)
# ==============================================================================
from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

import numpy as np
import omni.usd
from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.utils.stage import open_stage, is_stage_loading
from omni.isaac.core.utils.numpy.rotations import euler_angles_to_quats
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.motion_generation.lula import LulaKinematicsSolver
from omni.isaac.motion_generation import ArticulationKinematicsSolver

USD_PATH   = "/home/z/imeta_workspace/2_my_brain/custom_urdf/isaac_y1.usd"
URDF_PATH  = "/home/z/imeta_workspace/2_my_brain/custom_urdf/isaac_y1.urdf"
LULA_YAML  = "/home/z/imeta_workspace/2_my_brain/y1_lula.yaml"

open_stage(usd_path=USD_PATH)
while is_stage_loading(): simulation_app.update()

world = World(stage_units_in_meters=1.0)
stage = omni.usd.get_context().get_stage()
# 智能探测路径，防止 /World 报错
robot_path = "/y1" if stage.GetPrimAtPath("/y1") else "/World/y1"
y1_robot = world.scene.add(Articulation(robot_path, name="y1_robot"))
world.reset()

ik_solver = LulaKinematicsSolver(robot_description_path=LULA_YAML, urdf_path=URDF_PATH)
art_ik_solver = ArticulationKinematicsSolver(y1_robot, ik_solver, end_effector_frame_name="link8")
print("🧠 Lula 小脑已接入！")

def safe_move_to(pos, euler_rot):
    target_pos = np.array(pos, dtype=np.float64)
    target_quat = euler_angles_to_quats(np.array(euler_rot, dtype=np.float64))
    ik_result, success = art_ik_solver.compute_inverse_kinematics(target_pos, target_quat)
    
    if not success: return False
    target_arm = ik_result.joint_positions
    start_arm = y1_robot.get_joint_positions()[:6]
    
    for t in range(120):
        if not simulation_app.is_running(): break
        curr = start_arm + (t / 120.0) * (target_arm - start_arm)
        # ⚠️ 绝对核心：5.1.0 必须使用 ArticulationAction
        action = ArticulationAction(joint_positions=np.concatenate([curr, [0.04, 0.04]]))
        y1_robot.apply_action(action)
        world.step(render=True)
    return True

safe_move_to([0.35, 0.0, 0.30], [0, np.pi*0.75, 0])
while simulation_app.is_running(): world.step(render=True)
simulation_app.close()
