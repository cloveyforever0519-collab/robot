# ==============================================================================
# 🧠 任务代号 09: AI 夺权行动 V13 (原生 PyTorch 终极破壁版)
# 1. 彻底剔除 Numpy/Pickle/JSON，全程使用原生 PyTorch 加载归一化特征！
# 2. 完美继承原版 ALOHA 的时序集成 (Temporal Aggregation)。
# ==============================================================================
import torch
import torch.nn as nn
import numpy as np
from collections import deque

from omni.isaac.kit import SimulationApp
app = SimulationApp({"headless": False}) 

from omni.isaac.core import World
from omni.isaac.core.articulations import Articulation
from omni.isaac.core.objects import DynamicCuboid, FixedCuboid
from omni.isaac.core.utils.stage import open_stage
from omni.isaac.core.utils.types import ArticulationAction
from omni.isaac.core.materials import PhysicsMaterial
from pxr import UsdShade

class ACTPolicy(nn.Module):
    def __init__(self, state_dim=11, action_dim=8, chunk_size=100, hidden_dim=256):
        super().__init__()
        self.state_proj = nn.Linear(state_dim, hidden_dim)
        self.query_embed = nn.Embedding(chunk_size, hidden_dim)
        decoder_layer = nn.TransformerDecoderLayer(d_model=hidden_dim, nhead=8, dim_feedforward=1024, dropout=0.1, batch_first=True)
        self.transformer = nn.TransformerDecoder(decoder_layer, num_layers=4)
        self.action_head = nn.Linear(hidden_dim, action_dim)

    def forward(self, qpos, obj_pos):
        B = qpos.shape[0]
        state = torch.cat([qpos, obj_pos], dim=1) 
        memory = self.state_proj(state).unsqueeze(1) 
        tgt = self.query_embed.weight.unsqueeze(0).repeat(B, 1, 1)
        out = self.transformer(tgt, memory)
        return self.action_head(out)

def get_random_safe_pose(bin_pos):
    while True:
        r = np.random.uniform(0.25, 0.40)
        theta = np.random.uniform(-2.5, 2.5) 
        new_x = r * np.cos(theta)
        new_y = r * np.sin(theta)
        dist_to_bin = np.hypot(new_x - bin_pos[0], new_y - bin_pos[1])
        if dist_to_bin > 0.20: return np.array([new_x, new_y, 0.05])

def main():
    open_stage("/home/z/imeta_workspace/2_my_brain/custom_urdf/isaac_y1.usd")
    world = World(stage_units_in_meters=1.0)
    robot = world.scene.add(Articulation(prim_path="/y1", name="y1_arm"))
    BIN_POS = np.array([0.45, 0.0, 0.02])
    
    drop_bin = world.scene.add(DynamicCuboid(prim_path="/World/drop_bin", name="drop_bin", position=BIN_POS, scale=np.array([0.10, 0.10, 0.02]), color=np.array([0.0, 0.3, 0.8]), mass=50.0))
    cube = world.scene.add(DynamicCuboid(prim_path="/World/target_cube", name="target_cube", position=get_random_safe_pose(BIN_POS), scale=np.array([0.06, 0.06, 0.06]), color=np.array([1.0, 0.0, 0.0]), mass=0.05))
    
    cube.apply_physics_material(PhysicsMaterial(prim_path="/World/cube_material", dynamic_friction=0.8, static_friction=0.8, restitution=0.1))
    gripper_material = PhysicsMaterial(prim_path="/World/gripper_material", dynamic_friction=10.0, static_friction=10.0, restitution=0.0)
    
    world.reset()
    READY_JPOS = np.array([0.0, -0.8, 1.2, 0.0, -np.pi/2, 0.0, -0.04, 0.04])
    robot.set_joint_positions(READY_JPOS)

    import omni.usd
    stage = omni.usd.get_context().get_stage()
    mat_prim = stage.GetPrimAtPath(gripper_material.prim_path)
    for prim in stage.Traverse():
        if prim.GetName() in ["link7", "link8"]: UsdShade.MaterialBindingAPI(prim).Bind(UsdShade.Material(mat_prim), UsdShade.Tokens.strongerThanDescendants)

    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    # 🌟 核心破壁：直接加载 PyTorch 格式的统计数据！
    STATS_PATH = '/home/z/imeta_workspace/3_datasets/dataset_stats.pt'
    stats = torch.load(STATS_PATH, map_location=device)

    model = ACTPolicy().to(device)
    WEIGHT_PATH = '/home/z/imeta_workspace/3_datasets/weights/act_policy_exact_epoch_6.pt'
    print(f"🔌 正在将灵魂注入机械臂: {WEIGHT_PATH}")
    model.load_state_dict(torch.load(WEIGHT_PATH, map_location=device))
    model.eval() 

    chunk_size = 100
    action_queue = deque(maxlen=chunk_size)
    query_frequency = 1

    episode_timer = 0
    success_timer = 0

    print("✅ 夺权完成！纯净 PyTorch 测试版已启动！")

    while app.is_running():
        qpos = robot.get_joint_positions()
        obj_pos, _ = cube.get_world_pose()

        if qpos is None:
            world.step(render=True); continue

        # 全程使用 PyTorch Tensor 运算
        qpos_tensor = torch.from_numpy(qpos).float().unsqueeze(0).to(device)
        obj_pos_tensor = torch.from_numpy(obj_pos).float().unsqueeze(0).to(device)

        # 归一化
        norm_qpos = (qpos_tensor - stats['qpos_mean']) / stats['qpos_std']
        norm_obj_pos = (obj_pos_tensor - stats['obj_pos_mean']) / stats['obj_pos_std']

        if episode_timer % query_frequency == 0:
            with torch.no_grad():
                all_actions = model(norm_qpos, norm_obj_pos)
            action_queue.append(all_actions)

        # 时序集成 (Temporal Aggregation) - 纯 PyTorch 实现
        actions_for_curr_step = []
        for i, past_actions in enumerate(action_queue):
            time_offset = len(action_queue) - 1 - i
            if time_offset < past_actions.shape[1]:
                actions_for_curr_step.append(past_actions[0, time_offset])
        
        actions_for_curr_step = torch.stack(actions_for_curr_step) # [N, 8]
        k = 0.05
        exp_weights = torch.exp(-k * torch.arange(len(actions_for_curr_step), device=device))
        exp_weights = exp_weights / exp_weights.sum()
        exp_weights = exp_weights.unsqueeze(1) # [N, 1]
        
        raw_action = (actions_for_curr_step * exp_weights).sum(dim=0) # [8]

        # 反归一化
        target_qpos = raw_action * stats['action_std'] + stats['action_mean']
        target_qpos_np = target_qpos.cpu().numpy()
        
        robot.apply_action(ArticulationAction(joint_positions=target_qpos_np))
        episode_timer += 1

        trigger_reset = False
        dist_to_bin = np.hypot(obj_pos[0] - BIN_POS[0], obj_pos[1] - BIN_POS[1])
        if dist_to_bin < 0.10 and obj_pos[2] < 0.12:
            success_timer += 1
            if success_timer > 60: 
                print(f"🎉 完美入筐！重置..."); trigger_reset = True
        else: success_timer = 0

        if obj_pos[2] < 0.0 or obj_pos[0] < 0.1 or obj_pos[0] > 0.7 or abs(obj_pos[1]) > 0.5: trigger_reset = True
        elif episode_timer > 550 and obj_pos[2] < 0.04 and success_timer == 0: trigger_reset = True
        elif episode_timer > 800: trigger_reset = True

        if trigger_reset:
            cube.set_world_pose(position=get_random_safe_pose(BIN_POS))
            cube.set_linear_velocity(np.array([0.0, 0.0, 0.0]))
            cube.set_angular_velocity(np.array([0.0, 0.0, 0.0]))
            robot.set_joint_positions(READY_JPOS)
            action_queue.clear() 
            episode_timer = 0; success_timer = 0

        world.step(render=True)
    app.close()

if __name__ == '__main__':
    main()
