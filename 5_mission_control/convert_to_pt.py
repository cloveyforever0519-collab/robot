import pickle
import torch
import numpy as np

# 1. 读取危险的 pkl 文件
with open('/home/z/imeta_workspace/3_datasets/dataset_stats.pkl', 'rb') as f:
    stats = pickle.load(f)

# 2. 将 Numpy 数组彻底转化为安全的 PyTorch 张量
pt_stats = {k: torch.from_numpy(np.array(v)).float() for k, v in stats.items()}

# 3. 保存为绝对跨环境兼容的 .pt 文件
torch.save(pt_stats, '/home/z/imeta_workspace/3_datasets/dataset_stats.pt')
print("✅ 拔刺成功！已生成绝对兼容的 dataset_stats.pt")
