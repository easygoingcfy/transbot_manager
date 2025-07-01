#!/usr/bin/env python
import rospy
import json
import uuid
from std_msgs.msg import String
from task_parser import TaskParser
from robot_enums import TaskResult
from task_status_manager import status_manager

class CloudTaskReceiver:
    """云端任务接收器"""
    
    def __init__(self):
        rospy.init_node('cloud_task_receiver')
        
        # 订阅云端任务
        self.task_sub = rospy.Subscriber('/cloud_tasks', String, self.task_callback)
        
        # 当前执行的任务
        self.current_task = None
        
        rospy.loginfo("云端任务接收器已启动")
    
    def task_callback(self, msg):
        """接收云端下发的JSON任务"""
        try:
            task_data = json.loads(msg.data)
            task_id = task_data.get('task_id', str(uuid.uuid4()))
            task_name = task_data.get('name', 'Unknown Task')
            
            rospy.loginfo(f"收到云端任务: {task_id} - {task_name}")
            
            # 开始任务状态跟踪
            status_manager.start_task(task_id, task_name)
            
            # 解析并执行任务
            task_node = TaskParser.parse_task_from_json(msg.data)
            
            if task_node:
                self.execute_task(task_node)
            else:
                status_manager.complete_task(TaskResult.FAILURE, "任务解析失败")
                
        except Exception as e:
            rospy.logerr(f"任务处理错误: {e}")
            status_manager.complete_task(TaskResult.FAILURE, str(e))
    
    def execute_task(self, task_node):
        """执行解析后的任务"""
        self.current_task = task_node
        
        try:
            result = task_node.execute()
            
            if result == TaskResult.SUCCESS:
                status_manager.complete_task(TaskResult.SUCCESS)
            else:
                status_manager.complete_task(result, f"任务执行失败: {result}")
                
        except Exception as e:
            rospy.logerr(f"任务执行异常: {e}")
            status_manager.complete_task(TaskResult.FAILURE, str(e))
        
        self.current_task = None
    
    def run(self):
        """运行接收器"""
        rospy.spin()

if __name__ == '__main__':
    try:
        receiver = CloudTaskReceiver()
        receiver.run()
    except rospy.ROSInterruptException:
        pass