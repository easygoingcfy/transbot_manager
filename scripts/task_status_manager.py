#!/usr/bin/env python
# -*- coding: utf-8 -*-
# filepath: /home/jetson/transbot_ws/src/transbot_manager/scripts/task_status_manager.py

import rospy
import json
import threading
import sys
from std_msgs.msg import String
from robot_enums import TaskResult

# Python 2编码修复
if sys.version_info[0] == 2:
    reload(sys)
    sys.setdefaultencoding('utf-8')

class TaskStatusManager:
    """任务状态管理器 - 跟踪和发布任务执行状态"""
    
    def __init__(self):
        # 状态发布器
        self.status_pub = rospy.Publisher('/task_execution_status', String, queue_size=10)
        self.feedback_pub = rospy.Publisher('/task_feedback', String, queue_size=10)
        
        # 当前任务状态
        self.current_task = None
        self.task_history = []
        self.status_lock = threading.Lock()
        
        rospy.loginfo("任务状态管理器已初始化")
    
    def safe_string(self, text):
        """安全字符串处理"""
        if text is None:
            return "None"
        
        if sys.version_info[0] == 2:
            if isinstance(text, unicode):
                return text.encode('utf-8')
            elif isinstance(text, str):
                try:
                    return text.decode('utf-8').encode('utf-8')
                except UnicodeDecodeError:
                    return repr(text)
            else:
                return str(text)
        else:
            return str(text)
    
    def start_task(self, task_id, task_name, task_type="unknown"):
        """开始任务跟踪"""
        with self.status_lock:
            # 安全处理字符串
            safe_task_id = self.safe_string(task_id)
            safe_task_name = self.safe_string(task_name)
            safe_task_type = self.safe_string(task_type)
            
            self.current_task = {
                'task_id': safe_task_id,
                'name': safe_task_name,
                'type': safe_task_type,
                'status': TaskResult.RUNNING,
                'start_time': rospy.Time.now().to_sec(),
                'progress': 0.0,
                'message': '任务开始执行'
            }
            
            rospy.loginfo("开始跟踪任务: {} - {}".format(safe_task_id, safe_task_name))
            self._publish_status()
            self._publish_feedback("任务开始: {}".format(safe_task_name))
    
    def update_progress(self, progress, message=""):
        """更新任务进度"""
        if not self.current_task:
            return
            
        with self.status_lock:
            self.current_task['progress'] = max(0.0, min(100.0, progress))
            if message:
                self.current_task['message'] = self.safe_string(message)
            
            self._publish_status()
            self._publish_feedback("进度更新: {:.1f}% - {}".format(
                progress, self.safe_string(message)))
    
    def update_status(self, status, message=""):
        """更新任务状态"""
        if not self.current_task:
            return
            
        with self.status_lock:
            self.current_task['status'] = self.safe_string(status)
            if message:
                self.current_task['message'] = self.safe_string(message)
            
            self._publish_status()
            self._publish_feedback("状态更新: {} - {}".format(
                self.safe_string(status), self.safe_string(message)))
    
    def complete_task(self, result, message="任务完成"):
        """完成任务"""
        if not self.current_task:
            rospy.logwarn("尝试完成任务，但没有活动任务")
            return
            
        with self.status_lock:
            safe_result = self.safe_string(result)
            safe_message = self.safe_string(message)
            
            self.current_task['status'] = safe_result
            self.current_task['message'] = safe_message
            self.current_task['end_time'] = rospy.Time.now().to_sec()
            self.current_task['duration'] = self.current_task['end_time'] - self.current_task['start_time']
            
            if result == TaskResult.SUCCESS:
                self.current_task['progress'] = 100.0
            
            # 添加到历史记录
            self.task_history.append(self.current_task.copy())
            
            rospy.loginfo("任务完成: {} - 结果: {}".format(
                self.current_task['name'], safe_result))
            
            self._publish_status()
            self._publish_feedback("任务完成: {} - {}".format(safe_result, safe_message))
            
            # 清除当前任务
            self.current_task = None
    
    def cancel_task(self, message="任务被取消"):
        """取消当前任务"""
        self.complete_task(TaskResult.CANCELLED, message)
    
    def get_current_status(self):
        """获取当前任务状态"""
        with self.status_lock:
            return self.current_task.copy() if self.current_task else None
    
    def get_task_history(self, limit=10):
        """获取任务历史"""
        with self.status_lock:
            return self.task_history[-limit:] if self.task_history else []
    
    def _publish_status(self):
        """发布状态消息"""
        if self.current_task:
            status_msg = {
                'timestamp': rospy.Time.now().to_sec(),
                'task_id': self.current_task['task_id'],
                'name': self.current_task['name'],
                'status': self.current_task['status'],
                'progress': self.current_task['progress'],
                'message': self.current_task['message']
            }
            
            try:
                json_str = json.dumps(status_msg, ensure_ascii=False)
                if sys.version_info[0] == 2 and isinstance(json_str, unicode):
                    json_str = json_str.encode('utf-8')
                self.status_pub.publish(String(data=json_str))
            except Exception as e:
                rospy.logerr("状态发布失败: {}".format(repr(e)))
    
    def _publish_feedback(self, message):
        """发布反馈消息"""
        feedback_msg = {
            'timestamp': rospy.Time.now().to_sec(),
            'message': self.safe_string(message),
            'task_id': self.current_task['task_id'] if self.current_task else None
        }
        
        try:
            json_str = json.dumps(feedback_msg, ensure_ascii=False)
            if sys.version_info[0] == 2 and isinstance(json_str, unicode):
                json_str = json_str.encode('utf-8')
            self.feedback_pub.publish(String(data=json_str))
        except Exception as e:
            rospy.logerr("反馈发布失败: {}".format(repr(e)))

class DummyStatusManager:
    """空操作状态管理器 - 用于快速测试"""
    
    def start_task(self, *args, **kwargs):
        rospy.loginfo("DummyStatusManager: start_task called")
        
    def update_progress(self, *args, **kwargs):
        pass
        
    def complete_task(self, *args, **kwargs):
        rospy.loginfo("DummyStatusManager: complete_task called")
    
    def cancel_task(self, *args, **kwargs):
        rospy.loginfo("DummyStatusManager: cancel_task called")

# 全局实例
try:
    status_manager = TaskStatusManager()
except Exception as e:
    rospy.logwarn("TaskStatusManager初始化失败，使用简化版本: {}".format(repr(e)))
    status_manager = DummyStatusManager()

if __name__ == '__main__':
    rospy.init_node('task_status_manager')
    rospy.loginfo("任务状态管理器节点已启动")
    rospy.spin()