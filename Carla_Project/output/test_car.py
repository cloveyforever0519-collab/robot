import carla
import time

def main():
    try:
        # 1. 连接 CARLA
        client = carla.Client('127.0.0.1', 2000)
        client.set_timeout(10.0)
        world = client.get_world()

        # 2. 找到你的车辆 (假设场景中已经有一辆车了)
        blueprint_library = world.get_blueprint_library()
        vehicles = world.get_actors().filter('vehicle.*')
        
        if not vehicles:
            print("错误：场景中没有车辆，请先启动仿真并生成车辆！")
            return
        
        vehicle = vehicles[0] # 获取第一辆车
        print(f"已锁定车辆: {vehicle.type_id}")

        # 3. 强制释放手刹并给满油门
        print("正在尝试强制起步（持续3秒）...")
        control = carla.VehicleControl()
        control.throttle = 1.0    # 满油门
        control.brake = 0.0       # 无刹车
        control.hand_brake = False # 关键：强制关手刹
        control.manual_gear_shift = False # 自动挡
        
        # 持续发送控制指令
        end_time = time.time() + 3.0
        while time.time() < end_time:
            vehicle.apply_control(control)
            time.sleep(0.1)

        print("测试完成。请观察 CARLA 画面中的车动了没。")

    except Exception as e:
        print(f"发生错误: {e}")

if __name__ == '__main__':
    main()
