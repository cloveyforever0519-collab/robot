# Carla L4 HIL 正式台架交付说明

## 双机拓扑

- Windows 虚拟 VCU：`10.32.127.110`
- Ubuntu Carla 仿真机：`10.32.127.216`
- Ubuntu 控制接收：UDP `5001`
- Carla 真值输出：UDP `5000/5002/5003`
- 安卓大屏后端监听：UDP `5003`
- 安卓大屏网页：HTTP `8080`，WebSocket `8765`

统一配置文件为 `bench_config.json`。临时联调可以继续用命令行参数或环境变量覆盖，但正式交付默认以该文件为准。

## Windows 端红线

- 必须使用 32 位 Anaconda `can32` 环境。
- CANalyst-II 通过 WinUSB/libusb 接管。
- 只允许打开 CANalyst-II 通道 `0`。
- 禁止任何双通道、通道 `1`、恢复官方 `ControlCAN.dll` 的尝试。
- 当前稳定架构：单通道盲发 CAN + UDP 直连 Ubuntu。

## Cockpit_Control 0x116

官方 DBC：`can/智能座舱CAN协议-NJ0423.dbc`，文件编码为 GBK。代码保留人工位运算组包，避免 DBC 编码或 Motorola 位序解析差异影响台架。

8 字节固定格式：

```text
Byte0 = 0x01              D档
Byte1 = 0~100             Cockpit_ACC 油门
Byte2 = 0~100             Cockpit_Beak 刹车
Byte3 = EPS high byte     Motorola/big-endian signed 16-bit
Byte4 = EPS low byte
Byte5 = 0x02              Cockpit_Key_XbW 线控使能
Byte6 = 0x01              Cockpit_LED_Ready
Byte7 = 0x00              reserved
```

方向盘转换：

```python
physical_angle = int(steer_norm * 500)
angle_hex = physical_angle & 0xFFFF
byte3 = (angle_hex >> 8) & 0xFF
byte4 = angle_hex & 0xFF
```

## 场景要素验收

`main_gui_new.py` 已按 PPT 要求拆成 10 个独立验收项：

```text
车辆模型 10
中国特色交通标准 10
路障 16
覆盖物 4
井盖 1
普通对手车 15
紧急对手车 1
行人 5
自行车 2
动物 1
```

点击 Streamlit 侧边栏“加载交通”后，主界面会显示“场景要素验收汇总”。同一份结果会通过真值字段 `7_场景要素验收 (Scene Compliance)` 下发给安卓大屏。

注意：如果当前 CARLA 0.9.15 蓝图库没有某类真实蓝图，例如井盖或动物，系统会明确显示“未达标”，不会用其他物体假冒通过。

## 启动顺序

Ubuntu 仿真机：

```bash
streamlit run main_gui_new.py
```

在 UI 左侧确认控制模式为：

```text
🛞 硬件在环手动模式 (台架驾驶，无挂载)
```

安卓大屏后端：

```bash
python3 -m pip install -r requirements_dashboard.txt
python3 main_server.py
```

安卓大屏浏览器访问：

```text
http://10.32.127.216:8080/
```

Windows 虚拟 VCU：

```bat
conda activate can32
python virtual_vcu.py
```

仅测试 UDP：

```bat
python virtual_vcu.py --udp-only
```

## 自检

在项目根目录执行：

```bash
python bench_check.py
```

上线前需要探测端口时执行：

```bash
python bench_check.py --online
```
