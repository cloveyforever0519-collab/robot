# ==============================================================================
# 🔥 任务代号 07: ACT 炼丹炉 V32 - [5090D + U9 专属核爆版]
# 1. 显存狂暴解禁：Batch Size 2048，完美填满 RTX 5090D 的 32GB VRAM！
# 2. U9 多核调度：num_workers=8 + pin_memory 建立 PCIe 5.0 高速直达通道。
# 3. 硬件级加速：开启 PyTorch AMP (自动混合精度)，激活第四代 Tensor Cores！
# ==============================================================================
import os
# 🌟 顶级护身符：在开启多线程前，彻底关闭 HDF5 底层锁，杜绝假死！
os.environ["HDF5_USE_FILE_LOCKING"] = "FALSE"

import h5py
import torch
import torch.nn as nn
from torch.optim import AdamW
from torch.utils.data import Dataset, DataLoader
import numpy as np
import glob
import pickle
import time

# ================= 1. 终极数据喂食器 =================
class ALOHAExactDataset(Dataset):
    def __init__(self, data_dir, stats_path, chunk_size=100):
        self.file_paths = glob.glob(os.path.join(data_dir, '*.hdf5'))
        self.chunk_size = chunk_size
        
        with open(stats_path, 'rb') as f:
            self.stats = pickle.load(f)
            
        cache_path = os.path.join(os.path.dirname(stats_path), 'frame_index_cache.pkl')
        
        if os.path.exists(cache_path):
            print(f"\n⚡ [闪电启动] 检测到全局帧索引缓存，1秒极速拉起...", flush=True)
            with open(cache_path, 'rb') as f:
                self.index_map = pickle.load(f)
            print(f"📦 完美继承真实样本量: {len(self.index_map)} 帧！", flush=True)
        else:
            raise FileNotFoundError("缓存文件丢失，请先用老版本生成缓存！")

    def __len__(self):
        return len(self.index_map)

    def __getitem__(self, idx):
        file_path, t, original_len = self.index_map[idx]
        
        with h5py.File(file_path, 'r') as root:
            qpos_t = root['/observations/qpos'][t]
            obj_pos_t = root['/observations/obj_pos'][t]
            
            end_t = min(t + self.chunk_size, original_len)
            action_chunk = root['action'][t:end_t]
            
        qpos_t = (qpos_t - self.stats['qpos_mean']) / self.stats['qpos_std']
        obj_pos_t = (obj_pos_t - self.stats['obj_pos_mean']) / self.stats['obj_pos_std']
        action_chunk = (action_chunk - self.stats['action_mean']) / self.stats['action_std']
        
        state_qpos = torch.from_numpy(qpos_t).float()
        state_obj  = torch.from_numpy(obj_pos_t).float()
        
        if len(action_chunk) < self.chunk_size:
            pad_len = self.chunk_size - len(action_chunk)
            pad_action = np.tile(action_chunk[-1], (pad_len, 1))
            action_chunk = np.concatenate([action_chunk, pad_action], axis=0)
            
        action_tensor = torch.from_numpy(action_chunk).float()
        is_pad = torch.zeros(self.chunk_size, dtype=torch.bool)
        if end_t - t < self.chunk_size:
            is_pad[end_t - t:] = True
            
        return state_qpos, state_obj, action_tensor, is_pad

# ================= 2. Transformer 大脑 =================
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

# ================= 3. 炼丹主循环 =================
def train():
    print("🔥 检测到 RTX 5090D 级算力平台，底层引擎进入[核爆模式]...", flush=True)
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    
    DATA_DIR = '/home/z/imeta_workspace/3_datasets/raw_hdf5'
    STATS_PATH = '/home/z/imeta_workspace/3_datasets/dataset_stats.pkl'
    
    dataset = ALOHAExactDataset(DATA_DIR, STATS_PATH, chunk_size=100)
    
    # 🌟 5090D + U9 专属：2048 巨型吞吐量 + 8 线程并发搬运 + PCIe 锁页内存直达！
    dataloader = DataLoader(dataset, batch_size=2048, shuffle=True, 
                            num_workers=8, pin_memory=True, prefetch_factor=2)

    model = ACTPolicy().to(device)
    
    # 🌟 由于 Batch Size 扩大了 16 倍，学习率相应拉升，保证学习效率！
    optimizer = AdamW(model.parameters(), lr=5e-4, weight_decay=1e-4)
    
    # 🌟 5090D 专属：混合精度缩放器
    scaler = torch.amp.GradScaler('cuda')

    num_epochs = 15 
    save_dir = '/home/z/imeta_workspace/3_datasets/weights'
    os.makedirs(save_dir, exist_ok=True)

    print(f"\n🚀 [核爆启动] 批次总量已被暴力压缩至 3700 步左右！开始清空显存！", flush=True)
    for epoch in range(1, num_epochs + 1):
        model.train()
        epoch_loss = 0.0
        total_batches = len(dataloader)
        
        for batch_idx, (b_qpos, b_obj, b_action, b_is_pad) in enumerate(dataloader):
            b_qpos, b_obj, b_action, b_is_pad = b_qpos.to(device), b_obj.to(device), b_action.to(device), b_is_pad.to(device)

            optimizer.zero_grad()

            # 🌟 5090D 专属：开启 Tensor Cores 的 AMP 狂暴计算！
            with torch.autocast(device_type='cuda', dtype=torch.float16):
                pred_action = model(b_qpos, b_obj)
                loss_all = torch.nn.functional.l1_loss(pred_action, b_action, reduction='none')
                valid_mask = (~b_is_pad).unsqueeze(-1).expand_as(loss_all)
                loss = loss_all[valid_mask].mean()

            # 混合精度反向传播
            scaler.scale(loss).backward()
            scaler.step(optimizer)
            scaler.update()
            
            epoch_loss += loss.item()
            
            # 因为总步数变少了，我们每 200 步就打印一次
            if batch_idx % 200 == 0 and batch_idx > 0:
                print(f"   ⚡ [Epoch {epoch}] 进度: {batch_idx}/{total_batches} | 当前批次 Loss: {loss.item():.5f}", flush=True)

        avg_loss = epoch_loss / total_batches
        print(f"📈 [Epoch {epoch:03d}/{num_epochs:03d}] 完整通关 Loss: {avg_loss:.5f}", flush=True)

        weight_path = os.path.join(save_dir, f'act_policy_exact_epoch_{epoch}.pt')
        torch.save(model.state_dict(), weight_path)
        print(f"💾 [高价值存档] 已保存至: {weight_path}", flush=True)

if __name__ == '__main__':
    train()
