# ==============================================================================
# 🧠 任务代号 06: ACT 数据喂食器 (PyTorch DataLoader)
# 1. 自动扫描 10000 个 HDF5 文件。
# 2. 实现核心逻辑：Action Chunking (预测未来 chunk_size 步的动作序列)。
# 3. 极速读取与 PyTorch 张量转换。
# ==============================================================================
import os
import h5py
import numpy as np
import torch
from torch.utils.data import Dataset, DataLoader
import glob

class ALOHADataset(Dataset):
    def __init__(self, data_dir, chunk_size=100):
        super().__init__()
        self.data_dir = data_dir
        self.chunk_size = chunk_size
        
        # 扫描所有 hdf5 文件
        self.file_paths = glob.glob(os.path.join(data_dir, '*.hdf5'))
        print(f"📦 发现数据集: 共 {len(self.file_paths)} 个文件")
        
        # 为了极致速度，我们不在 __init__ 里全读进内存，而是在 __getitem__ 里动态读

    def __len__(self):
        return len(self.file_paths)

    def __getitem__(self, idx):
        file_path = self.file_paths[idx]
        
        with h5py.File(file_path, 'r') as root:
            qpos = root['/observations/qpos'][()]
            obj_pos = root['/observations/obj_pos'][()]
            action = root['action'][()]
            
        original_len = qpos.shape[0]
        
        # 🌟 随机选择一个起始时间点 t (保证至少有 1 帧可以用)
        # 为了防止越界，t 最大只能是 original_len - 1
        t = np.random.randint(0, original_len)
        
        # 提取状态输入 (State): 当前时刻 t 的关节角度和方块位置
        state_qpos = torch.from_numpy(qpos[t]).float()
        state_obj  = torch.from_numpy(obj_pos[t]).float()
        
        # 🌟 核心：Action Chunking
        # 获取从 t 开始，未来 chunk_size 步的动作
        # 如果 t 离结尾太近，长度不够 chunk_size，就在末尾补齐(Padding)最后一帧的动作
        end_t = min(t + self.chunk_size, original_len)
        action_chunk = action[t:end_t]
        
        if len(action_chunk) < self.chunk_size:
            pad_len = self.chunk_size - len(action_chunk)
            # 用最后一帧的动作去填补空白 (表示到达目标后保持不动)
            pad_action = np.tile(action_chunk[-1], (pad_len, 1))
            action_chunk = np.concatenate([action_chunk, pad_action], axis=0)
            
        action_tensor = torch.from_numpy(action_chunk).float()
        
        # 还要生成一个 is_pad 掩码，告诉模型哪些是补齐的假数据
        is_pad = torch.zeros(self.chunk_size, dtype=torch.bool)
        if end_t - t < self.chunk_size:
            is_pad[end_t - t:] = True
            
        return state_qpos, state_obj, action_tensor, is_pad

# ================= 测试流水线 =================
if __name__ == "__main__":
    DATA_DIR = '/home/z/imeta_workspace/3_datasets/raw_hdf5'
    
    # 实例化 Dataset
    dataset = ALOHADataset(DATA_DIR, chunk_size=100)
    
    # 放入 DataLoader (设置 batch_size=32，模拟一次训练送入 32 个数据)
    dataloader = DataLoader(dataset, batch_size=32, shuffle=True, num_workers=4)
    
    print("🚀 开始模拟数据喂食...")
    for batch_idx, (b_qpos, b_obj, b_action, b_is_pad) in enumerate(dataloader):
        print(f"\n--- Batch {batch_idx} ---")
        print(f"输入状态 (关节 qpos): {b_qpos.shape} --> [Batch, 8关节]")
        print(f"输入状态 (方块 obj): {b_obj.shape} --> [Batch, 3坐标]")
        print(f"输出预测 (Action Chunk): {b_action.shape} --> [Batch, 100步预测, 8关节]")
        print(f"有效性掩码 (is_pad): {b_is_pad.shape}")
        
        # 只测一个 Batch 就停下，证明管线通了就行
        print("\n✅ 数据喂食器跑通！张量维度完美契合 ACT 框架！")
        break
