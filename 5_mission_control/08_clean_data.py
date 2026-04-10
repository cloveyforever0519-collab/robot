# ==============================================================================
# 🧹 任务代号 08: 数据集终极净化器
# 1. 极速扫描目录下所有的 HDF5 文件。
# 2. 精准定位缺乏 'obj_pos' 的早期废弃数据或损坏数据。
# 3. 物理销毁，确保喂给大模型的血液 100% 纯净！
# ==============================================================================
import os
import glob
import h5py

DATA_DIR = '/home/z/imeta_workspace/3_datasets/raw_hdf5'
all_files = glob.glob(os.path.join(DATA_DIR, '*.hdf5'))

print(f"🔍 正在启动净化扫描，共发现 {len(all_files)} 个文件...")

bad_files = []

for file_path in all_files:
    try:
        with h5py.File(file_path, 'r') as root:
            # 检查核心命脉 obj_pos 是否存在
            if 'observations/obj_pos' not in root:
                bad_files.append(file_path)
    except Exception as e:
        # 如果文件直接坏了打不开，也一并视为毒药
        bad_files.append(file_path)

if not bad_files:
    print("✅ 扫描完成！你的数据集纯洁无瑕，没有任何毒药数据！")
else:
    print(f"⚠️ 发现 {len(bad_files)} 个残缺/损坏文件，正在执行物理销毁...")
    for f in bad_files:
        os.remove(f)
        print(f" 🗑️ 已销毁: {os.path.basename(f)}")
    print(f"\n🎉 清洗大功告成！残余的 {len(all_files) - len(bad_files)} 个文件现已达到 100% 工业级纯度！")
