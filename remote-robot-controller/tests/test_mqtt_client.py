#!/usr/bin/env python
# -*- coding: utf-8 -*-

import unittest
import sys
import os

# 添加src目录到路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from core.mqtt_client import MQTTClient
from core.config_manager import ConfigManager

class TestMQTTClient(unittest.TestCase):
    """MQTT客户端测试"""
    
    def setUp(self):
        """测试设置"""
        self.config_manager = ConfigManager()
        self.config = self.config_manager.get_config()
        
    def test_mqtt_client_creation(self):
        """测试MQTT客户端创建"""
        client = MQTTClient(self.config)
        self.assertIsNotNone(client)
        self.assertEqual(client.robot_id, self.config['robot']['id'])
    
    def test_topic_generation(self):
        """测试主题生成"""
        client = MQTTClient(self.config)
        topics = client.get_robot_topics()
        
        robot_id = self.config['robot']['id']
        expected_cmd_topic = f"robot/{robot_id}/command/mode"
        
        self.assertEqual(topics['commands']['mode'], expected_cmd_topic)

if __name__ == '__main__':
    unittest.main()