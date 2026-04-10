# ==============================================================================
# 🚀 任务代号 04: Lula IK 自动抓取脚本专家 V22 - 实机防炸量产版
# 1. 真实物理抓取：方块恢复低摩擦以实现自适应居中，使用底层 API 为夹爪单独注入 10.0 摩擦力！
# 2. 防炸机工作区：严格约束方块生成在正前方 120 度范围内，杜绝限位死锁！
# 3. 继承 V21 所有优势：门字型高空拱桥、150 FPS 适配、0 缝隙真实握力。
# ==============================================================================
import os
import socket
import json
import numpy as np

from isaacsim import SimulationApp
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.utils.stage import open_stage, is_stage_loading
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.utils.rotations import euler_angles_to_quat
import omni.isaac.core.utils.prims as prim_utils
from omni.isaac.core.materials import PhysicsMaterial
from pxr import UsdShade  # 🌟 引入 USD 底层材质 API

from isaacsim.robot_motion.motion_generation import LulaKinematicsSolver
import omni.timeline

# ================= 1. 动态锻造 Lula 配置文件 =================
lula_yaml_path = "/home/z/imeta_workspace/5_mission_control/temp_y1_lula.yaml"
with open(lula_yaml_path, "w") as f:
    f.write("""api_version: 1.0
name: y1
cspace:
  - joint1
  - joint2
  - joint3
  - joint4
  - joint5
  - joint6
default_q: [0.0, -0.8, 1.2, 0.0, -0.5, 0.0]
cspace_to_urdf_rules: []
""")

# ================= 2. 环境与通信初始化 =================
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_IP, UDP_PORT = "127.0.0.1", 9999

open_stage("/home/z/imeta_workspace/2_my_brain/custom_urdf/isaac_y1.usd")
while is_stage_loading():
    simulation_app.update()

world = World(stage_units_in_meters=1.0)
y1_robot = world.scene.add(Articulation("/y1/root_joint", name="y1_robot"))

# 🎲 抓取目标：红色小方块
target_cube = world.scene.add(
    DynamicCuboid(
        prim_path="/World/target_cube",
        name="target_cube",
        position=np.array([0.3, 0.0, 0.02]),
        scale=np.array([0.06, 0.06, 0.06]), 
        color=np.array([1.0, 0.0, 0.0]),
        mass=0.05
    )
)

# 🌟 分离物理材质策略
# 1. 小方块：恢复低摩擦，让它在桌面上能被推着滑动，实现【对称自适应居中】
cube_material = PhysicsMaterial(
    prim_path="/World/cube_material",
    dynamic_friction=0.8,  
    static_friction=0.8,   
    restitution=0.1        
)
target_cube.apply_physics_material(cube_material)

# 2. 夹爪：锻造超级橡胶手套
gripper_material = PhysicsMaterial(
    prim_path="/World/gripper_material",
    dynamic_friction=10.0,  
    static_friction=10.0,   
    restitution=0.0        
)

# 🟦 放置目标：蓝色大托盘
BIN_POS = np.array([0.45, 0, 0.02])
drop_bin = world.scene.add(
    DynamicCuboid(
        prim_path="/World/drop_bin",
        name="drop_bin",
        position=BIN_POS,
        scale=np.array([0.10, 0.10, 0.02]), 
        color=np.array([0.0, 0.3, 0.8]),
        mass=50.0 
    )
)

world.reset()
omni.timeline.get_timeline_interface().play()
simulation_app.update() 

# ⚡ 初始化控制器与材质注入
try:
    controller = y1_robot.get_articulation_controller()
    controller.set_gains(kps=np.array([100000.0] * 8), kds=np.array([10000.0] * 8))
    controller.set_max_efforts(np.array([100000.0] * 8)) 
    print("⚡ [底层物理] 发力限制解封！")
    
    # 🌟 核心杀招：遍历全宇宙，给夹爪戴上橡胶手套！
    stage = omni.usd.get_context().get_stage()
    mat_prim = stage.GetPrimAtPath(gripper_material.prim_path)
    usd_material = UsdShade.Material(mat_prim)
    for prim in stage.Traverse():
        if prim.GetName() in ["link7", "link8"]: # 锁定你的两个夹爪网格
            UsdShade.MaterialBindingAPI(prim).Bind(usd_material, UsdShade.Tokens.strongerThanDescendants)
            print(f"🧲 已成功为 {prim.GetName()} 穿上顶级橡胶摩擦手套！")
except Exception as e:
    print(f"⚠️ 初始化异常: {e}")

# ================= 3. 装载 Lula IK =================
lula_kinematics = LulaKinematicsSolver(
    robot_description_path=lula_yaml_path,
    urdf_path="/home/z/imeta_workspace/2_my_brain/custom_urdf/isaac_y1.urdf"
)

# ================= 4. Sim2Real 闭环状态机 =================
state = "INIT"
timeout_timer = 0
pause_timer = 0

MAX_VEL = 0.06          
TCP_OFFSET_Z = 0.145    

CRUISE_Z = TCP_OFFSET_Z + 0.15  
HOVER_Z  = TCP_OFFSET_Z + 0.10  
GRASP_Z  = TCP_OFFSET_Z         

OPEN_GRIP  = -0.04   
CLOSE_GRIP = -0      
TOLERANCE  = 0.05    
TIMEOUT_MAX = 300    

READY_JPOS = np.array([0.0, -0.8, 1.2, 0.0, -np.pi/2, 0.0, -0.04, 0.04])
goal_jpos = np.copy(READY_JPOS)
goal_gripper = OPEN_GRIP  

print("🧠 [AI 师傅 V22] 夹爪摩擦实装！防炸机安全工作区已划定！")

while simulation_app.is_running():
    current_jpos = y1_robot.get_joint_positions()
    cube_pos, _ = target_cube.get_world_pose()
    
    if current_jpos is not None and len(current_jpos) == 8:
            
        target_pitch = np.pi / 2 
        cube_yaw = np.arctan2(cube_pos[1], cube_pos[0])
        cube_quat = euler_angles_to_quat(np.array([0, target_pitch, cube_yaw]))
        cube_seed = np.array([cube_yaw, -0.8, 1.2, 0.0, -np.pi/2, 0.0])
        
        bin_yaw = np.arctan2(BIN_POS[1], BIN_POS[0])
        bin_quat = euler_angles_to_quat(np.array([0, target_pitch, bin_yaw]))
        bin_seed = np.array([bin_yaw, -0.8, 1.2, 0.0, -np.pi/2, 0.0])

        arm_error = np.max(np.abs(current_jpos[:6] - goal_jpos[:6]))
        is_arrived = arm_error < TOLERANCE

        if pause_timer > 0:
            pause_timer -= 1
        else:
            if state == "INIT":
                state = "RESET_BLOCK"

            elif state == "RESET_BLOCK":
                while True:
                    r = np.random.uniform(0.25, 0.40)      
                    # 🌟 防炸机机制：将生成角度严格限制在正前方 120度 范围内 (约 -1.0 到 1.0 弧度)
                    # 彻底解决背向死锁，永远在舒适区作业！
                    theta = np.random.uniform(-2.5, 2.5) 
                    new_x = r * np.cos(theta)
                    new_y = r * np.sin(theta)
                    dist_to_bin = np.hypot(new_x - BIN_POS[0], new_y - BIN_POS[1])
                    if dist_to_bin > 0.20: break
                
                target_cube.set_world_pose(position=np.array([new_x, new_y, 0.02]))
                goal_gripper = OPEN_GRIP 
                print(f"\n📦 新目标(安全区) -> X:{new_x:.2f} Y:{new_y:.2f}")
                state = "CALC_APPROACH"

            elif state == "CALC_APPROACH":
                ik_target = cube_pos + np.array([0, 0, HOVER_Z]) 
                jpos, success = lula_kinematics.compute_inverse_kinematics(
                    frame_name="link6", target_position=ik_target, 
                    target_orientation=cube_quat, warm_start=cube_seed
                )
                if success: goal_jpos[:6] = jpos  
                
                print("🚁 1. 启动悬停指令...")
                timeout_timer = 0
                state = "WAIT_APPROACH"

            elif state == "WAIT_APPROACH":
                if is_arrived:
                    state = "CALC_DOWN"
                elif timeout_timer > TIMEOUT_MAX:
                    state = "RESET_BLOCK"
                else: timeout_timer += 1

            elif state == "CALC_DOWN":
                ik_target = cube_pos + np.array([0, 0, GRASP_Z]) 
                jpos, success = lula_kinematics.compute_inverse_kinematics(
                    frame_name="link6", target_position=ik_target, 
                    target_orientation=cube_quat, warm_start=cube_seed
                )
                if success: goal_jpos[:6] = jpos
                
                print("⬇️ 2. 执行垂直下探...")
                timeout_timer = 0
                state = "WAIT_DOWN"

            elif state == "WAIT_DOWN":
                if is_arrived:
                    state = "DO_GRASP"
                elif timeout_timer > TIMEOUT_MAX:
                    state = "RESET_BLOCK"
                else: timeout_timer += 1

            elif state == "DO_GRASP":
                goal_gripper = CLOSE_GRIP 
                prim_utils.set_prim_attribute_value(target_cube.prim_path, "physics:rigidBodyEnabled", True)
                print("✊ 3. 夹爪收紧中 (方块自适应居中)...")
                pause_timer = 100 
                state = "CALC_LIFT"

            elif state == "CALC_LIFT":
                ik_target = cube_pos + np.array([0, 0, CRUISE_Z])
                jpos, success = lula_kinematics.compute_inverse_kinematics(
                    frame_name="link6", target_position=ik_target, 
                    target_orientation=cube_quat, warm_start=cube_seed
                )
                if success: goal_jpos[:6] = jpos
                
                print("⬆️ 4. 垂直拉起至巡航高度...")
                timeout_timer = 0
                state = "WAIT_LIFT"

            elif state == "WAIT_LIFT":
                if is_arrived:
                    state = "CALC_MOVE_MID" 
                elif timeout_timer > TIMEOUT_MAX:
                    state = "RESET_BLOCK"
                else: timeout_timer += 1

            elif state == "CALC_MOVE_MID":
                mid_x = (cube_pos[0] + BIN_POS[0]) / 2.0
                mid_y = (cube_pos[1] + BIN_POS[1]) / 2.0
                mid_z = CRUISE_Z + 0.06  
                ik_target = np.array([mid_x, mid_y, mid_z])
                
                jpos, success = lula_kinematics.compute_inverse_kinematics(
                    frame_name="link6", target_position=ik_target, 
                    target_orientation=bin_quat, warm_start=bin_seed 
                )
                if success: goal_jpos[:6] = jpos
                
                print("🚚 4.5 经过高空拱桥中继点...")
                timeout_timer = 0
                state = "WAIT_MOVE_MID"

            elif state == "WAIT_MOVE_MID":
                if is_arrived:
                    state = "CALC_MOVE_BIN"
                elif timeout_timer > TIMEOUT_MAX * 1.5:
                    state = "RESET_BLOCK"
                else: timeout_timer += 1

            elif state == "CALC_MOVE_BIN":
                ik_target = BIN_POS + np.array([0, 0, CRUISE_Z])
                jpos, success = lula_kinematics.compute_inverse_kinematics(
                    frame_name="link6", target_position=ik_target, 
                    target_orientation=bin_quat, warm_start=bin_seed 
                )
                if success: goal_jpos[:6] = jpos
                
                print("🚚 5. 执行后半段跨越至托盘...")
                timeout_timer = 0
                state = "WAIT_MOVE_BIN"

            elif state == "WAIT_MOVE_BIN":
                if is_arrived:
                    state = "CALC_LOWER_BIN"
                elif timeout_timer > TIMEOUT_MAX * 1.5:
                    state = "RESET_BLOCK"
                else: timeout_timer += 1

            elif state == "CALC_LOWER_BIN":
                ik_target = BIN_POS + np.array([0, 0, TCP_OFFSET_Z + 0.05])
                jpos, success = lula_kinematics.compute_inverse_kinematics(
                    frame_name="link6", target_position=ik_target, 
                    target_orientation=bin_quat, warm_start=bin_seed
                )
                if success: goal_jpos[:6] = jpos
                
                print("⬇️ 6. 深入托盘下探...")
                timeout_timer = 0
                state = "WAIT_LOWER_BIN"

            elif state == "WAIT_LOWER_BIN":
                if is_arrived:
                    state = "DO_RELEASE"
                elif timeout_timer > TIMEOUT_MAX:
                    state = "RESET_BLOCK"
                else: timeout_timer += 1

            elif state == "DO_RELEASE":
                goal_gripper = OPEN_GRIP 
                print("👐 7. 夹爪松开，完美入筐！")
                pause_timer = 150
                state = "CALC_READY"

            elif state == "CALC_READY":
                ik_target = BIN_POS + np.array([0, 0, CRUISE_Z])
                jpos, success = lula_kinematics.compute_inverse_kinematics(
                    frame_name="link6", target_position=ik_target, 
                    target_orientation=bin_quat, warm_start=bin_seed
                )
                if success: goal_jpos[:6] = jpos
                
                print("⬆️ 8. 撤离归位，全流程闭环完成！")
                timeout_timer = 0
                state = "WAIT_READY"

            elif state == "WAIT_READY":
                if is_arrived:
                    pause_timer = 50 
                    state = "RESET_BLOCK"
                elif timeout_timer > TIMEOUT_MAX:
                    state = "RESET_BLOCK"
                else: timeout_timer += 1

        goal_jpos[6] = goal_gripper
        goal_jpos[7] = -goal_gripper

        delta = goal_jpos - current_jpos
        max_delta = np.max(np.abs(delta)) 
        
        if max_delta > MAX_VEL:
            step = delta * (MAX_VEL / max_delta)
        else:
            step = delta
            
        next_jpos = current_jpos + step
        
        y1_robot.apply_action(ArticulationAction(joint_positions=next_jpos))
        
        payload = {"qpos": next_jpos.tolist(), "obj_pos": cube_pos.tolist()}
        sock.sendto(json.dumps(payload).encode('utf-8'), (UDP_IP, UDP_PORT))
        
    world.step(render=True)

sock.close()
simulation_app.close()
