import h5py
import numpy as np
import os
import glob

# 指向你的数据集目录
DATA_DIR = '/home/z/imeta_workspace/3_datasets/raw_hdf5'

# 🌟 智能寻找最新生成的 HDF5 文件
list_of_files = glob.glob(os.path.join(DATA_DIR, '*.hdf5'))
if not list_of_files:
    print("❌ 文件夹里空空如也，找不到任何 HDF5 文件！")
    exit()

# 获取最新修改的文件
TEST_FILE = max(list_of_files, key=os.path.getctime)

print(f"🔍 正在质检最新鲜的数据文件: {TEST_FILE}\n")

with h5py.File(TEST_FILE, 'r') as root:
    # 1. 检查根目录下的内容
    print("📂 HDF5 文件结构:")
    for key in root.keys():
        print(f" ├── {key}")
        if isinstance(root[key], h5py.Group):
            for sub_key in root[key].keys():
                print(f" │    └── {sub_key}")
    print("-" * 40)

    # 2. 读取数据并打印形状
    qpos = root['/observations/qpos'][()]
    obj_pos = root['/observations/obj_pos'][()]
    action = root['action'][()]

    print("📊 张量维度 (Shape) 检查:")
    print(f" 🎯 机械臂关节 (qpos): {qpos.shape}  --> (代表: 帧数, 8个关节)")
    print(f" 🎯 目标物坐标 (obj_pos): {obj_pos.shape} --> (代表: 帧数, X/Y/Z坐标)")
    print(f" 🎯 专家动作 (action): {action.shape}  --> (代表: 帧数, 8个关节目标位置)")
    print("-" * 40)

    # 3. 抽查第一帧和最后一帧的数据合法性
    print("🕵️ 数据合理性抽查:")
    print(f" [第一帧] 夹爪状态: {qpos[0, -2:]} (应该是张开的负数)")
    print(f" [最后一帧] 夹爪状态: {qpos[-1, -2:]} (闭合/张开 取决于你切片的位置)")
    
    # 算一下方块在头尾移动的距离
    start_obj = obj_pos[0]
    end_obj = obj_pos[-1]
    dist = np.linalg.norm(end_obj - start_obj)
    print(f" 📦 方块移动总距离: {dist:.3f} 米")
    
    if dist > 0.1:
        print(" ✅ 质检结论：极其健康！方块发生了显著的物理位移！")
    else:
        print(" ❌ 质检结论：警告！方块移动距离太短，可能是抓取失败的回合！")

print("\n总指挥，报告完毕！")
