#!/usr/bin/env python
# -*- coding: utf-8 -*-
# filepath: /home/jetson/transbot_ws/src/transbot_manager/scripts/cloud_task_receiver.py

import rospy
import json
import uuid
import sys
from std_msgs.msg import String
from task_parser import TaskParser
from robot_enums import TaskResult
from task_status_manager import status_manager

# Python 2编码修复
if sys.version_info[0] == 2:
    reload(sys)
    sys.setdefaultencoding('utf-8')

class CloudTaskReceiver:
    """云端任务接收器"""
    
    def __init__(self):
        rospy.init_node('cloud_task_receiver')
        
        # 订阅云端任务
        self.task_sub = rospy.Subscriber('/cloud_tasks', String, self.task_callback)
        
        # 当前执行的任务
        self.current_task = None
        
        rospy.loginfo("云端任务接收器已启动")
    
    def safe_json_loads(self, json_str):
        """安全的JSON解析，处理编码问题"""
        try:
            # 如果是bytes，先解码为unicode
            if isinstance(json_str, bytes):
                json_str = json_str.decode('utf-8')
            
            # 解析JSON
            return json.loads(json_str, encoding='utf-8')
        except UnicodeDecodeError as e:
            rospy.logerr("JSON字符串解码失败: {}".format(e))
            return None
        except json.JSONDecodeError as e:
            rospy.logerr("JSON解析失败: {}".format(e))
            return None
        except Exception as e:
            rospy.logerr("JSON处理异常: {}".format(e))
            return None
    
    def safe_string(self, text):
        """安全字符串处理，避免编码问题"""
        if text is None:
            return "None"
        
        if isinstance(text, unicode):
            return text.encode('utf-8')
        elif isinstance(text, str):
            try:
                return text.decode('utf-8').encode('utf-8')
            except UnicodeDecodeError:
                return repr(text)  # 如果解码失败，返回repr
        else:
            return str(text)
    
    def task_callback(self, msg):
        """接收云端下发的JSON任务"""
        try:
            # 安全解析JSON
            task_data = self.safe_json_loads(msg.data)
            if not task_data:
                rospy.logerr("任务数据解析失败")
                return
            
            task_id = task_data.get('task_id', str(uuid.uuid4()))
            task_name = task_data.get('name', 'Unknown Task')
            
            # 安全处理字符串
            safe_task_id = self.safe_string(task_id)
            safe_task_name = self.safe_string(task_name)
            
            rospy.loginfo("收到云端任务: {} - {}".format(safe_task_id, safe_task_name))
            
            # 开始任务状态跟踪
            status_manager.start_task(safe_task_id, safe_task_name)
            
            # 解析并执行任务
            task_node = TaskParser.parse_task_from_json(msg.data)
            
            if task_node:
                self.execute_task(task_node)
            else:
                status_manager.complete_task(TaskResult.FAILURE, "任务解析失败")
                
        except UnicodeEncodeError as e:
            error_msg = "字符编码错误: {}".format(repr(e))
            rospy.logerr(error_msg)
            status_manager.complete_task(TaskResult.FAILURE, error_msg)
        except Exception as e:
            error_msg = "任务处理错误: {}".format(repr(e))
            rospy.logerr(error_msg)
            status_manager.complete_task(TaskResult.FAILURE, error_msg)
    
    def execute_task(self, task_node):
        """执行解析后的任务"""
        self.current_task = task_node
        
        try:
            safe_name = self.safe_string(task_node.name)
            rospy.loginfo("开始执行任务: {}".format(safe_name))
            
            result = task_node.execute()
            
            if result == TaskResult.SUCCESS:
                rospy.loginfo("任务执行成功: {}".format(safe_name))
                status_manager.complete_task(TaskResult.SUCCESS)
            else:
                rospy.logwarn("任务执行失败: {} - 结果: {}".format(safe_name, result))
                status_manager.complete_task(result, "任务执行失败: {}".format(result))
                
        except Exception as e:
            error_msg = "任务执行异常: {}".format(repr(e))
            rospy.logerr(error_msg)
            status_manager.complete_task(TaskResult.FAILURE, error_msg)
        finally:
            self.current_task = None
    
    def cancel_current_task(self):
        """取消当前执行的任务"""
        if self.current_task:
            safe_name = self.safe_string(self.current_task.name)
            rospy.loginfo("取消当前任务: {}".format(safe_name))
            if hasattr(self.current_task, 'cancel'):
                self.current_task.cancel()
    
    def run(self):
        """运行接收器"""
        rospy.loginfo("云端任务接收器开始监听...")
        rospy.spin()

if __name__ == '__main__':
    try:
        receiver = CloudTaskReceiver()
        receiver.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("云端任务接收器已停止")
    except Exception as e:
        rospy.logerr("启动失败: {}".format(repr(e)))