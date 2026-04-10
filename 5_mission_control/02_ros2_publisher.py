import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Header, Float64MultiArray
import socket
import json

class UDPToROS2Bridge(Node):
    def __init__(self):
        super().__init__('y1_udp_bridge')
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.obj_pub = self.create_publisher(Float64MultiArray, '/obj_pos', 10)
        
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", 9999))
        self.sock.setblocking(False)
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.get_logger().info("📡 [双通道桥接] 正在监听机械臂与方块数据...")

    def timer_callback(self):
        try:
            data, _ = self.sock.recvfrom(2048)
            parsed_data = json.loads(data.decode('utf-8'))
            
            # 1. 发布关节
            msg = JointState()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = ['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7', 'j8']
            msg.position = parsed_data["qpos"]
            self.joint_pub.publish(msg)
            
            # 2. 发布方块坐标
            obj_msg = Float64MultiArray()
            obj_msg.data = parsed_data["obj_pos"]
            self.obj_pub.publish(obj_msg)
            
        except BlockingIOError:
            pass

def main():
    rclpy.init()
    bridge = UDPToROS2Bridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
