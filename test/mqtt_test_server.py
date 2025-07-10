#!/usr/bin/env python
#coding=utf-8

import paho.mqtt.client as mqtt
import threading
import time
import json
import sys

# Python 2/3 兼容性处理
if sys.version_info[0] == 2:
    input = raw_input

class SimpleMQTTTestServer:
    """简单的MQTT测试服务器 - 用于测试机器人MQTT桥接功能"""
    
    def __init__(self, broker_host='localhost', broker_port=1883):
        self.broker_host = broker_host
        self.broker_port = broker_port
        self.robot_id = 'transbot_001'
        
        # 创建MQTT客户端
        self.client = mqtt.Client(client_id='test_server')
        self.client.on_connect = self.on_connect
        self.client.on_message = self.on_message
        
        self.connected = False
        
        print("MQTT测试服务器启动...")
        print("连接到: {}:{}".format(broker_host, broker_port))
        
    def on_connect(self, client, userdata, flags, rc):
        """连接回调"""
        if rc == 0:
            self.connected = True
            print("✓ 连接成功!")
            
            # 订阅机器人上报的所有主题
            topics = [
                "robot/{}/status".format(self.robot_id),
                "robot/{}/feedback".format(self.robot_id),
                "robot/{}/telemetry".format(self.robot_id),
                "robot/{}/heartbeat".format(self.robot_id),
                "robot/{}/task_status".format(self.robot_id),
                "robot/{}/task_feedback".format(self.robot_id),
            ]
            
            for topic in topics:
                client.subscribe(topic)
                print("订阅主题: {}".format(topic))
                
        else:
            print("✗ 连接失败，错误代码: {}".format(rc))
    
    def on_message(self, client, userdata, msg):
        """消息回调"""
        try:
            topic = msg.topic
            payload = msg.payload.decode('utf-8')
            
            # print("\n=== 收到机器人消息 ===")
            # print("主题: {}".format(topic))
            # print("内容: {}".format(payload))
            
            # 尝试解析JSON
            try:
                data = json.loads(payload)
                # print("JSON解析成功:")
                # for key, value in data.items():
                #     print("  {}: {}".format(key, value))
            except:
                print("非JSON格式消息")
            
            # print("=" * 30)
            
        except Exception as e:
            print("消息处理错误: {}".format(e))
    
    def connect(self):
        """连接到MQTT服务器"""
        try:
            self.client.connect(self.broker_host, self.broker_port, 60)
            self.client.loop_start()
            return True
        except Exception as e:
            print("连接失败: {}".format(e))
            return False
    
    def send_mode_command(self, mode):
        """发送模式切换命令"""
        topic = "robot/{}/command/mode".format(self.robot_id)
        payload = json.dumps({
            "mode": mode,
            "timestamp": time.time()
        })
        
        self.client.publish(topic, payload)
        print("发送模式命令: {} -> {}".format(mode, topic))
    
    def send_velocity_command(self, linear_x, angular_z):
        """发送速度控制命令"""
        topic = "robot/{}/command/velocity".format(self.robot_id)
        payload = json.dumps({
            "linear_x": linear_x,
            "linear_y": 0.0,
            "angular_z": angular_z,
            "timestamp": time.time()
        })
        
        self.client.publish(topic, payload)
        print("发送速度命令: linear_x={}, angular_z={}".format(linear_x, angular_z))
    
    def send_navigation_command(self, x, y, yaw=0.0):
        """发送导航命令"""
        topic = "robot/{}/command/navigation".format(self.robot_id)
        payload = json.dumps({
            "x": x,
            "y": y,
            "z": 0.0,
            "qx": 0.0,
            "qy": 0.0,
            "qz": 0.0,
            "qw": 1.0,
            "timestamp": time.time()
        })
        
        self.client.publish(topic, payload)
        print("发送导航命令: x={}, y={}".format(x, y))
    
    def send_task_command(self, task_data):
        """发送任务命令"""
        topic = "robot/{}/command/task".format(self.robot_id)
        payload = json.dumps(task_data)
        
        self.client.publish(topic, payload)
        print("发送任务命令: {}".format(task_data.get('task_type', 'unknown')))
    
    def send_emergency_stop(self, emergency=True):
        """发送紧急停止命令"""
        topic = "robot/{}/command/emergency".format(self.robot_id)
        payload = json.dumps({
            "emergency": emergency,
            "timestamp": time.time()
        })
        
        self.client.publish(topic, payload)
        print("发送紧急停止命令: {}".format(emergency))
    
    def send_camera_command(self, capture=True):
        """发送相机命令"""
        topic = "robot/{}/command/camera".format(self.robot_id)
        payload = json.dumps({
            "capture": capture,
            "timestamp": time.time()
        })
        
        self.client.publish(topic, payload)
        print("发送相机命令: capture={}".format(capture))
    
    def send_elevator_command(self, position):
        """发送升降杆命令"""
        topic = "robot/{}/command/elevator".format(self.robot_id)
        payload = json.dumps({
            "position": position,
            "timestamp": time.time()
        })
        
        self.client.publish(topic, payload)
        print("发送升降杆命令: position={}".format(position))
    
    def interactive_mode(self):
        """交互模式"""
        print("\n=== MQTT测试服务器交互模式 ===")
        print("可用命令:")
        print("  1. mode <模式>           - 切换模式 (auto/manual/pause)")
        print("  2. vel <linear> <angular> - 速度控制")
        print("  3. nav <x> <y>           - 导航到目标点")
        print("  4. task                  - 发送测试任务")
        print("  5. emergency             - 紧急停止")
        print("  6. camera                - 拍照")
        print("  7. elevator <position>   - 升降杆控制")
        print("  8. quit                  - 退出")
        print()
        
        while True:
            try:
                cmd_input = input("请输入命令: ").strip()
                if not cmd_input:
                    continue
                
                cmd = cmd_input.split()
                
                if cmd[0] == 'quit':
                    break
                elif cmd[0] == 'mode' and len(cmd) == 2:
                    self.send_mode_command(cmd[1])
                elif cmd[0] == 'vel' and len(cmd) == 3:
                    self.send_velocity_command(float(cmd[1]), float(cmd[2]))
                elif cmd[0] == 'nav' and len(cmd) == 3:
                    self.send_navigation_command(float(cmd[1]), float(cmd[2]))
                elif cmd[0] == 'task':
                    task_data = {
                        "task_id": "test_task_{}".format(int(time.time())),
                        "task_type": "patrol",
                        "behaviors": [
                            {
                                "type": "navigate",
                                "parameters": {"x": 1.0, "y": 2.0}
                            },
                            {
                                "type": "capture_image",
                                "parameters": {}
                            }
                        ],
                        "timestamp": time.time()
                    }
                    self.send_task_command(task_data)
                elif cmd[0] == 'emergency':
                    self.send_emergency_stop(True)
                elif cmd[0] == 'camera':
                    self.send_camera_command(True)
                elif cmd[0] == 'elevator' and len(cmd) == 2:
                    self.send_elevator_command(float(cmd[1]))
                else:
                    print("无效命令或参数错误")
                    
            except KeyboardInterrupt:
                break
            except ValueError:
                print("参数格式错误")
            except Exception as e:
                print("命令执行错误: {}".format(e))
    
    def auto_test_mode(self):
        """自动测试模式"""
        print("\n=== 自动测试模式 ===")
        
        tests = [
            ("模式切换测试", lambda: self.send_mode_command("auto")),
            ("速度控制测试", lambda: self.send_velocity_command(0.5, 0.2)),
            ("导航测试", lambda: self.send_navigation_command(1.0, 2.0)),
            ("相机测试", lambda: self.send_camera_command(True)),
            ("升降杆测试", lambda: self.send_elevator_command(0.5)),
            ("任务测试", lambda: self.send_task_command({
                "task_id": "auto_test_task",
                "task_type": "patrol",
                "behaviors": [
                    {"type": "navigate", "parameters": {"x": 1.0, "y": 2.0}},
                    {"type": "capture_image", "parameters": {}}
                ],
                "timestamp": time.time()
            })),
            ("紧急停止测试", lambda: self.send_emergency_stop(True)),
            ("恢复测试", lambda: self.send_emergency_stop(False))
        ]
        
        for test_name, test_func in tests:
            print("\n执行: {}".format(test_name))
            test_func()
            time.sleep(2)  # 等待2秒
        
        print("\n自动测试完成!")
    
    def run(self):
        """运行测试服务器"""
        if not self.connect():
            return
        
        # 等待连接
        timeout = 5
        while not self.connected and timeout > 0:
            time.sleep(1)
            timeout -= 1
        
        if not self.connected:
            print("连接超时")
            return
        
        print("\n选择测试模式:")
        print("1. 交互模式 (手动发送命令)")
        print("2. 自动测试模式")
        
        try:
            choice = input("请选择 (1/2): ").strip()
            if choice == '1':
                self.interactive_mode()
            elif choice == '2':
                self.auto_test_mode()
            else:
                print("无效选择")
        except KeyboardInterrupt:
            pass
        
        print("\n测试服务器关闭")
        self.client.loop_stop()
        self.client.disconnect()

if __name__ == '__main__':
    # 检查是否有broker地址参数
    broker_host = 'localhost'
    if len(sys.argv) > 1:
        broker_host = sys.argv[1]
    
    server = SimpleMQTTTestServer(broker_host)
    server.run()