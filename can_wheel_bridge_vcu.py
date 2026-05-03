import can
import cantools
import socket
import json
import time
import math
import threading
import os
import re

print("=================================================================")
print(" 🚀 L4 硬件在环: 工业级 VCU 全闭环力反馈网关 (外科手术净化版)")
print("=================================================================")

# ==========================================
# 1. 网络与环境配置
# ==========================================
UDP_IP = "127.0.0.1"
UDP_PORT_TX = 5001  # 发给 Carla 的控制端口
UDP_PORT_RX = 5000  # 接收 Carla 的真值端口

sock_tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock_rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
try:
    sock_rx.bind((UDP_IP, UDP_PORT_RX))
    sock_rx.setblocking(False)
except Exception as e:
    print(f"⚠️ 无法绑定接收端口 {UDP_PORT_RX}，请检查是否被占用！({e})")

# ==========================================
# 2. 挂载并暴力净化 DBC 数据库 (工业级黑科技)
# ==========================================
DBC_FILE = 'vcu_protocol.dbc'
if not os.path.exists(DBC_FILE):
    print(f"❌ 致命错误：找不到 {DBC_FILE} 文件！请在同目录下创建该文件。")
    exit(1)

# 【核心修复】：外科手术级剔除干扰项
with open(DBC_FILE, 'r', encoding='utf-8', errors='ignore') as f:
    lines = f.readlines()

clean_lines = []
for line in lines:
    stripped = line.strip()
    # 彻底剔除所有包含中文的元数据行（注释 CM_、值表 VAL_、属性 BA_）
    if stripped.startswith('CM_') or stripped.startswith('VAL_') or stripped.startswith('BA_') or stripped.startswith('SIG_'):
        continue
    
    # 铲除剩余行中的特殊符号（例如单位 "°" 这种非 ASCII 字符）
    safe_line = re.sub(r'[^\x00-\x7F]+', '', line)
    clean_lines.append(safe_line)

clean_dbc_content = ''.join(clean_lines)

try:
    # 从纯粹的二进制结构字符串中加载 DBC
    db = cantools.database.load_string(clean_dbc_content)
    print(f"✅ DBC 数据库外科手术净化成功: 提取出 {len(db.messages)} 条核心报文！")
except Exception as e:
    print(f"❌ 致命异常：DBC 纯净版解析依然失败: {e}")
    exit(1)

# ==========================================
# 3. 挂载物理 CAN 盒
# ==========================================
try:
    # 请根据实际硬件修改, 如 PCAN 就是 bustype='pcan', channel='PCAN_USBBUS1'
    bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=500000)
    print("✅ 物理 CAN 总线连接成功！")
except Exception as e:
    print(f"⚠️ CAN 硬件未连接，进入纯网络测试模式... ({e})")
    bus = None

# ==========================================
# 4. 全局共享真值状态仓
# ==========================================
sim_state = {
    "speed_ms": 0.0,
    "steer_angle_deg": 0.0,
    "gear": 0,
    "eps_torque_nm": 0.0
}

# ==========================================
# 5. [下行] 接收 Carla UDP 并通过 CAN 发给方向盘
# ==========================================
def pack_and_send_vcu_can():
    """
    根据 BO_ 226 Carla_EV_state 进行编码。
    Cantools 会自动帮我们做 /0.001 (车速) 和 /0.1 (扭矩) 的数学运算！
    """
    try:
        msg_def = db.get_message_by_name('Carla_EV_state')
        
        # 组装物理值字典 (无需中文字符串，全数字交互)
        data_dict = {
            'Carla_Gear': sim_state["gear"] & 0x03,
            'Carla_EV_Speed': max(0.0, min(100.0, sim_state["speed_ms"])), 
            'Carla_Steer_Angle': max(-540.0, min(540.0, sim_state["steer_angle_deg"])),
            'Carla_EPS_Torque': max(-12.0, min(12.0, sim_state["eps_torque_nm"]))
        }
        
        # 自动编码为 bytearray
        encoded_data = msg_def.encode(data_dict)
        
        if bus is not None:
            can_msg = can.Message(arbitration_id=msg_def.frame_id, data=encoded_data, is_extended_id=False)
            bus.send(can_msg)
            
    except Exception as e:
        pass # 屏蔽发送频率过高时的杂音报错

def udp_rx_thread():
    last_tx_time = time.time()
    while True:
        try:
            data, _ = sock_rx.recvfrom(4096)
            telemetry = json.loads(data.decode('utf-8'))
            
            # 解析真值
            kinematics = telemetry.get("1_刚体运动学 (Rigid Body Kinematics)", {})
            ctrl_state = telemetry.get("3_驾驶控制反读 (Control State)", {})
            
            v_xyz = kinematics.get("3_线速度矢量_XYZ_米每秒", [0,0,0])
            sim_state["speed_ms"] = math.sqrt(v_xyz[0]**2 + v_xyz[1]**2 + v_xyz[2]**2)
            
            is_reverse = ctrl_state.get("14_倒车挂档状态", False)
            sim_state["gear"] = 2 if is_reverse else 1
            
            carla_steer = ctrl_state.get("11_方向盘转角_负1至1", 0.0)
            sim_state["steer_angle_deg"] = carla_steer * 540.0
            
            # 【力反馈生成器】车速越高，打角越大，阻力越强
            base_torque = carla_steer * 8.0 
            speed_factor = min(1.0, sim_state["speed_ms"] / 10.0)
            sim_state["eps_torque_nm"] = base_torque * speed_factor

            # 以 50Hz (20ms) 向 VCU 射出报文
            current_time = time.time()
            if current_time - last_tx_time >= 0.02:
                pack_and_send_vcu_can()
                last_tx_time = current_time

        except BlockingIOError:
            time.sleep(0.005)
        except Exception:
            pass

# 启动真值监听线程
t_udp = threading.Thread(target=udp_rx_thread, daemon=True)
t_udp.start()

# ==========================================
# 6. [上行] 监听 VCU 的 CAN 报文，解析给 Carla
# ==========================================
print(f"📡 正在监听台架动作，并桥接至 -> {UDP_IP}:{UDP_PORT_TX}")
try:
    while True:
        if bus is not None:
            can_msg = bus.recv(timeout=0.1)
            # 监听 BO_ 278 (0x116)
            if can_msg and can_msg.arbitration_id == 278:
                
                # decode_choices=False 强制只解析出底层数字
                decoded_sig = db.decode_message(can_msg.arbitration_id, can_msg.data, decode_choices=False)
                
                throttle_pct = decoded_sig.get('Cockpit_ACC', 0)
                brake_pct = decoded_sig.get('Cockpit_Beak', 0)
                steer_deg = decoded_sig.get('Cockpit_EPS_Angle', 0)
                gear = decoded_sig.get('Cockpit_Gear', 0)
                e_stop = decoded_sig.get('Cockpit_Key_Stop', 0)
                xbw_switch = decoded_sig.get('Cockpit_Key_XbW', 0)
                
                # 转换至 Carla 需要的区间
                carla_throttle = throttle_pct / 100.0
                carla_brake = brake_pct / 100.0
                carla_steer = steer_deg / 540.0 # 假设方向盘 540度打死
                carla_steer = max(-1.0, min(1.0, carla_steer))
                
                # 如果按下急停开关，强制刹车抱死！
                if e_stop == 1:
                    carla_brake = 1.0
                    carla_throttle = 0.0

                control_cmd = {
                    "steer": carla_steer,
                    "throttle": carla_throttle,
                    "brake": carla_brake,
                    "reverse": (gear == 2), # 2是R档
                    "hand_brake": (e_stop == 1) 
                }
                
                # 射向 UDP 5001 端口
                sock_tx.sendto(json.dumps(control_cmd).encode('utf-8'), (UDP_IP, UDP_PORT_TX))
                
                gear_str = "R (倒档)" if gear == 2 else "D (前进)"
                xbw_str = "🟢 智驾开启" if xbw_switch else "🔘 物理接管"
                stop_str = "🚨 紧急抱死!" if e_stop else "✅ 正常"
                
                print(f"🛞 角度:{steer_deg:>5.1f}° | ⛽ 油门:{throttle_pct:>3.0f}% | 🛑 刹车:{brake_pct:>3.0f}% | 档位:{gear_str} || {xbw_str} | {stop_str}", end='\r')
        else:
            time.sleep(0.1)

except KeyboardInterrupt:
    print("\n🛑 硬件桥接器安全下线！")
finally:
    if bus: bus.shutdown()
    sock_tx.close()
    sock_rx.close()
