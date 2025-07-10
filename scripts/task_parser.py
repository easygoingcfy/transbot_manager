#!/usr/bin/env python
#coding=utf-8
# filepath: /home/jetson/transbot_ws/src/transbot_manager/scripts/task_parser.py

import json
import rospy
from task_composer import TaskComposer, SequenceNode, ParallelNode, SelectorNode, ActionNode

class TaskParser:
    """JSON任务描述解析器"""
    
    @staticmethod
    def parse_task_from_json(json_str):
        """从JSON字符串解析任务"""
        try:
            rospy.loginfo("=== 开始解析任务 ===")
            
            task_data = json.loads(json_str)
            
            rospy.loginfo("解析后的任务数据:")
            rospy.loginfo(json.dumps(task_data, indent=2, ensure_ascii=False))
            
            # 打印任务基本信息
            task_id = task_data.get('task_id', 'Unknown')
            task_name = task_data.get('name', 'Unknown')
            task_type = task_data.get('type', 'Unknown')
            
            rospy.loginfo("任务基本信息:")
            rospy.loginfo("  任务ID: {}".format(task_id))
            rospy.loginfo("  任务名称: {}".format(task_name))
            rospy.loginfo("  任务类型: {}".format(task_type))
            
            # 递归构建任务节点
            task_node = TaskParser._build_task_node(task_data)
            
            if task_node:
                TaskParser._print_task_tree(task_node, 0)
            else:
                rospy.logerr("任务解析失败!")
            
            rospy.loginfo("=== 任务解析完成 ===")
            return task_node
            
        except json.JSONDecodeError as e:
            rospy.logerr("JSON解析错误: {}".format(e))
            rospy.logerr("原始数据: {}".format(json_str))
            return None
        except Exception as e:
            rospy.logerr("任务解析错误: {}".format(e))
            rospy.logerr("原始数据: {}".format(json_str))
            return None
    
    @staticmethod
    def _build_task_node(node_data, depth=0):
        """递归构建任务节点"""
        indent = "  " * depth
        node_type = node_data.get('type')
        name = node_data.get('name', 'unnamed')
        
        
        if node_type == 'sequence':
            node = SequenceNode(name)
        elif node_type == 'parallel':
            node = ParallelNode(name)
        elif node_type == 'selector':
            node = SelectorNode(name)
        elif node_type == 'action':
            # 修复字段名匹配问题 - 支持两种字段名
            behavior_type = node_data.get('behavior') or node_data.get('behavior_type')
            if not behavior_type:
                rospy.logerr("{}动作节点缺少behavior字段: {}".format(indent, name))
                return None
            
            params = node_data.get('params', {})
            
            return ActionNode(name, behavior_type, **params)
        else:
            rospy.logerr("{}未知的节点类型: {}".format(indent, node_type))
            return None
        
        # 添加子节点
        children = node_data.get('children', [])
        
        for i, child_data in enumerate(children):
            child_node = TaskParser._build_task_node(child_data, depth + 1)
            if child_node:  # 只添加成功解析的子节点
                node.add_child(child_node)
            else:
                rospy.logerr("{}第 {} 个子节点构建失败".format(indent, i+1))
        
        return node
    
    @staticmethod
    def _print_task_tree(node, depth=0):
        """打印任务树结构"""
        indent = "  " * depth
        
        if hasattr(node, 'behavior_type'):
            # 动作节点
            rospy.loginfo("{}├─ [动作] {} ({})".format(indent, node.name, node.behavior_type))
            if hasattr(node, 'params'):
                rospy.loginfo("{}   参数: {}".format(indent, node.params))
        else:
            # 复合节点
            node_type = type(node).__name__
            child_count = len(node.children) if hasattr(node, 'children') else 0
            rospy.loginfo("{}├─ [{}] {} ({} 个子节点)".format(indent, node_type, node.name, child_count))
            
            # 递归打印子节点
            if hasattr(node, 'children'):
                for child in node.children:
                    TaskParser._print_task_tree(child, depth + 1)
    
    @staticmethod
    def create_sample_json():
        """创建示例JSON任务描述"""
        sample_task = {
            "task_id": "sample_001",
            "type": "sequence",
            "name": "Sample Patrol Task",
            "children": [
                {
                    "type": "action",
                    "name": "Initialize",
                    "behavior": "speak",
                    "params": {"text": "Starting patrol task"}
                },
                {
                    "type": "sequence",
                    "name": "Patrol Route",
                    "children": [
                        {
                            "type": "action",
                            "name": "Navigate to Point 1",
                            "behavior": "navigate",
                            "params": {"x": 0.2, "y": 0.2, "theta": 0.0}
                        },
                        {
                            "type": "action",
                            "name": "Wait and Check",
                            "behavior": "wait",
                            "params": {"duration": 3.0}
                        },
                        {
                            "type": "action",
                            "name": "Capture Image",
                            "behavior": "capture_image",
                            "params": {"save_path": "/tmp/patrol_1.jpg"}
                        },
                        {
                            "type": "action",
                            "name": "Report Complete",
                            "behavior": "speak",
                            "params": {"text": "Checkpoint 1 completed"}
                        }
                    ]
                }
            ]
        }
        return json.dumps(sample_task, indent=2, ensure_ascii=False)
    
    @staticmethod
    def create_simple_navigation_task(x, y, theta=0.0, name="Navigation Task"):
        """创建简单导航任务的JSON"""
        task = {
            "task_id": "nav_{}".format(int(rospy.Time.now().to_sec())),
            "type": "sequence",
            "name": name,
            "children": [
                {
                    "type": "action",
                    "name": "Start Navigation",
                    "behavior": "speak",
                    "params": {"text": "Starting navigation to target"}
                },
                {
                    "type": "action",
                    "name": "Execute Navigation",
                    "behavior": "navigate",
                    "params": {"x": x, "y": y, "theta": theta}
                },
                {
                    "type": "action",
                    "name": "Navigation Complete",
                    "behavior": "speak",
                    "params": {"text": "Target reached"}
                }
            ]
        }
        return json.dumps(task, ensure_ascii=False)

# 测试函数
def test_parser():
    """测试解析器"""
    rospy.init_node('task_parser_test')
    
    # 测试示例任务
    sample_json = TaskParser.create_sample_json()
    rospy.loginfo("测试示例任务解析:")
    task_node = TaskParser.parse_task_from_json(sample_json)
    
    if task_node:
        rospy.loginfo("解析成功!")
    else:
        rospy.logerr("解析失败!")

if __name__ == '__main__':
    test_parser()