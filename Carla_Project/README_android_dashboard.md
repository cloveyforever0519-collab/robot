# 安卓大屏前端运行说明

## 部署结构

- Ubuntu 22.04 电脑运行后端：`main_server.py`
- 安卓大屏不用安装后端，只用浏览器访问：`http://Ubuntu电脑IP:8080/`
- Carla / Streamlit 主控继续运行 `main_gui_new.py`
- 真值链路：`main_gui_new.py -> UDP 5003 -> main_server.py -> WebSocket 8765 -> 安卓大屏`
- 控制链路：`安卓大屏 -> HTTP /api/control -> UDP 5001 -> main_gui_new.py`

## Ubuntu 22.04 启动

```bash
cd Carla_Project
python3 -m pip install -r requirements_dashboard.txt
python3 main_server.py
```

浏览器打开：

```text
http://Ubuntu电脑IP:8080/
```

安卓大屏建议使用横屏、浏览器全屏或添加到桌面后全屏打开。

默认控制指令目标为正式 Ubuntu 仿真机 `10.32.127.216:5001`。如果临时联调目标变化，可以指定控制指令目标：

```bash
CARLA_CONTROL_HOST=Carla主控IP python3 main_server.py
```

## Windows 虚拟 VCU

正式台架 Windows 发送端：

- IP：`10.32.127.110`
- 环境：32 位 Anaconda `can32`
- CAN：CANalyst-II，仅打开通道 0

运行：

```bat
conda activate can32
python virtual_vcu.py
```

如需只验证 UDP，不碰 CAN 盒：

```bat
python virtual_vcu.py --udp-only
```

## Carla 主控

```bash
streamlit run main_gui_new.py
```

主控会把实时真值发到 `5000/5002/5003`。大屏后端监听 `5003`，避免和 VCU 网关、云端桥占用同一个 `5000` 端口。

## 已接入功能

- 车型 JSON 下发：`/api/spawn`
- 手驾触控上行：转向、油门、刹车、驻车
- 天气切换：晴天、多云、小雨、暴雨、大雾、深夜
- 交通参与者加载/清空
- 场景要素验收：显示 10 类验收项的达标数量，结果来自 `main_gui_new.py` 真值字段
- 实时显示：车速、档位、方向盘角度、踏板开度、限速、红绿灯、雷达目标、碰撞状态
- 车辆动力学：四轮 RPM、估算垂向载荷、位姿真值
- 传感器阵列：双目相机、激光雷达、毫米波、GNSS/IMU、超声波状态

## 场景验收显示

点击“加载交通参与者”后，大屏右侧会显示 `场景验收 x/10`。如果某一类 CARLA 蓝图库不足或生成失败，会显示该类的实际数量，方便现场直接定位缺口。
