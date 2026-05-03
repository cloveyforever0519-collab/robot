import carla
import json

def apply_truck_physics():
    # 1. 连接 CARLA
    client = carla.Client('localhost', 2000)
    client.set_timeout(10.0)
    world = client.get_world()

    # 2. 找到生成的 ego_vehicle
    truck = None
    for actor in world.get_actors().filter('vehicle.*'):
        if actor.attributes.get('role_name') == 'ego_vehicle':
            truck = actor
            break

    if not truck:
        print("错误：未找到 ego_vehicle，请确保 ros2 launch 已经运行！")
        return

    # 3. 获取物理控制对象
    physics_control = truck.get_physics_control()

    # 4. 应用核心参数
    # 设置质量和重心
    physics_control.mass = 8200.0
    physics_control.center_of_gravity = carla.Location(x=1.8, y=0.0, z=1.15)
    
    # 设置扭矩曲线 (Torque Curve)
    physics_control.torque_curve = [
        carla.Vector2D(x=600, y=1500),
        carla.Vector2D(x=1100, y=2800),
        carla.Vector2D(x=1500, y=2800),
        carla.Vector2D(x=2500, y=1200)
    ]
    physics_control.max_rpm = 2500.0
    physics_control.drag_coefficient = 0.65

    # 5. 设置变速箱齿轮 (修正参数名)
    forward_gears = []
    gear_ratios = [14.4, 12.3, 9.6, 7.4, 5.4, 4.0, 2.8, 1.9, 1.4, 1.0, 0.7]
    
    for r in gear_ratios:
        # 关键修正：使用 down_ratio 和 up_ratio
        gear = carla.GearPhysicsControl(
            ratio=r, 
            down_ratio=0.5, 
            up_ratio=0.8
        )
        forward_gears.append(gear)
    
    physics_control.forward_gears = forward_gears
    physics_control.use_gear_autobox = True 

    # 6. 应用设置
    truck.apply_physics_control(physics_control)
    print("--- 物理参数注入成功 ---")
    print(f"成功加载 {len(forward_gears)} 个前进档位")
    print(f"当前车辆质量: {truck.get_physics_control().mass} kg")

if __name__ == "__main__":
    apply_truck_physics()
