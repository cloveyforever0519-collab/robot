# 06_compute_stats.py
import os
import glob
import h5py
import numpy as np
import pickle

DATA_DIR = '/home/z/imeta_workspace/3_datasets/raw_hdf5'
STATS_PATH = '/home/z/imeta_workspace/3_datasets/dataset_stats.pkl'

def main():
    file_paths = glob.glob(os.path.join(DATA_DIR, '*.hdf5'))
    print(f"🔍 开始计算 {len(file_paths)} 个文件的归一化特征，这可能需要几分钟...")
    
    all_qpos, all_obj_pos, all_action = [], [], []
    
    # 为了防止内存爆炸，我们抽取前 1000 个文件计算统计特征（足够精确）
    sample_files = file_paths[:1000] if len(file_paths) > 1000 else file_paths
    
    for fpath in sample_files:
        with h5py.File(fpath, 'r') as f:
            all_qpos.append(f['/observations/qpos'][()])
            all_obj_pos.append(f['/observations/obj_pos'][()])
            all_action.append(f['action'][()])
            
    all_qpos = np.concatenate(all_qpos, axis=0)
    all_obj_pos = np.concatenate(all_obj_pos, axis=0)
    all_action = np.concatenate(all_action, axis=0)
    
    stats = {
        'qpos_mean': np.mean(all_qpos, axis=0),
        'qpos_std': np.std(all_qpos, axis=0) + 1e-5, # 防止除零
        'obj_pos_mean': np.mean(all_obj_pos, axis=0),
        'obj_pos_std': np.std(all_obj_pos, axis=0) + 1e-5,
        'action_mean': np.mean(all_action, axis=0),
        'action_std': np.std(all_action, axis=0) + 1e-5
    }
    
    with open(STATS_PATH, 'wb') as f:
        pickle.dump(stats, f)
    
    print(f"✅ 归一化特征计算完成！已保存至 {STATS_PATH}")
    print(f"👉 qpos_mean: {stats['qpos_mean']}")

if __name__ == '__main__':
    main()
