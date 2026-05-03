import carla
import time

# 1. 连接到当前正在运行的 Carla 世界
client = carla.Client('127.0.0.1', 2000)
client.set_timeout(5.0)
world = client.get_world()

# 2. 获取上帝视角相机（Spectator）
spectator = world.get_spectator()

print("==================================================")
print(" 🛰️ Carla GPS 定位仪已启动！")
print(" 🎮 请切回 Carla 渲染窗口，用鼠标和 WASD 键自由飞行。")
print(" 📍 飞到你想要发车的地点，直接看这里的坐标！")
print("==================================================\n")

try:
    while True:
        # 实时获取当前镜头的坐标和朝向
        t = spectator.get_transform()
        loc = t.location
        rot = t.rotation
        
        # 打印在同一行，方便查看
        print(f"📌 完美发车点 -> X: {loc.x:8.1f} | Y: {loc.y:8.1f} | Z: {loc.z:8.1f} | Yaw: {rot.yaw:8.1f}", end='\r')
        time.sleep(0.1)
except KeyboardInterrupt:
    print("\n\n✅ 定位结束！")
