#!/usr/bin/env python
import json
import rospy
from task_composer import TaskComposer, SequenceNode, ParallelNode, SelectorNode, ActionNode

class TaskParser:
    """JSON任务描述解析器"""
    
    @staticmethod
    def parse_task_from_json(json_str):
        """从JSON字符串解析任务"""
        try:
            task_data = json.loads(json_str)
            return TaskParser._build_task_node(task_data)
        except json.JSONDecodeError as e:
            rospy.logerr(f"JSON解析错误: {e}")
            return None
    
    @staticmethod
    def _build_task_node(node_data):
        """递归构建任务节点"""
        node_type = node_data.get('type')
        name = node_data.get('name', 'unnamed')
        
        if node_type == 'sequence':
            node = SequenceNode(name)
        elif node_type == 'parallel':
            node = ParallelNode(name)
        elif node_type == 'selector':
            node = SelectorNode(name)
        elif node_type == 'action':
            behavior_type = node_data.get('behavior_type')
            params = node_data.get('params', {})
            return ActionNode(name, behavior_type, **params)
        else:
            raise ValueError(f"未知的节点类型: {node_type}")
        
        # 添加子节点
        for child_data in node_data.get('children', []):
            child_node = TaskParser._build_task_node(child_data)
            node.add_child(child_node)
        
        return node
    
    @staticmethod
    def create_sample_json():
        """创建示例JSON任务描述"""
        sample_task = {
            "type": "sequence",
            "name": "巡逻检查任务",
            "children": [
                {
                    "type": "action",
                    "name": "初始化",
                    "behavior_type": "speak",
                    "params": {"text": "开始执行巡逻任务"}
                },
                {
                    "type": "sequence",
                    "name": "巡逻路径",
                    "children": [
                        {
                            "type": "action",
                            "name": "导航到点1",
                            "behavior_type": "navigate",
                            "params": {"x": 2.0, "y": 1.0, "theta": 0.0}
                        },
                        {
                            "type": "parallel",
                            "name": "检查点1任务",
                            "children": [
                                {
                                    "type": "action",
                                    "name": "拍照1",
                                    "behavior_type": "capture_image",
                                    "params": {"save_path": "/tmp/patrol_1.jpg"}
                                },
                                {
                                    "type": "action",
                                    "name": "播报1",
                                    "behavior_type": "speak",
                                    "params": {"text": "已到达检查点1"}
                                }
                            ]
                        }
                    ]
                }
            ]
        }
        return json.dumps(sample_task, indent=2, ensure_ascii=False)