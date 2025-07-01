#!/usr/bin/env python
import rospy
import actionlib
import threading
from robot_enums import TaskResult, TaskState
from task_composer import TaskComposer
from transbot_msgs.msg import TaskExecutionAction, TaskExecutionGoal, TaskExecutionResult, TaskExecutionFeedback

class TaskExecutionServer:
    def __init__(self):
        rospy.init_node('task_execution_server')
        
        self.current_task = None
        self.task_state = TaskState.IDLE
        self.task_lock = threading.Lock()
        
        # 创建Action Server
        self.action_server = actionlib.SimpleActionServer(
            'task_execution',
            TaskExecutionAction,
            execute_cb=self.execute_callback,
            auto_start=False
        )
        
        self.action_server.start()
        rospy.loginfo("任务执行服务器已启动")
        
    def execute_callback(self, goal):
        """Action Server执行回调"""
        rospy.loginfo(f"收到任务执行请求: {goal.task_name}")
        
        with self.task_lock:
            if self.task_state != TaskState.IDLE:
                result = TaskExecutionResult()
                result.result = TaskResult.FAILURE.value
                result.message = "另一个任务正在执行中"
                self.action_server.set_aborted(result)
                return
        
        # 根据任务名称创建任务
        task_creators = {
            'patrol': TaskComposer.create_patrol_task,
            'inspection': TaskComposer.create_inspection_task,
        }
        
        if goal.task_name not in task_creators:
            result = TaskExecutionResult()
            result.result = TaskResult.FAILURE.value
            result.message = f"未知任务类型: {goal.task_name}"
            self.action_server.set_aborted(result)
            return
        
        # 创建并执行任务
        self.current_task = task_creators[goal.task_name]()
        self.task_state = TaskState.NAVIGATING  # 可以根据具体情况更新
        
        try:
            # 发送反馈
            feedback = TaskExecutionFeedback()
            feedback.current_state = self.task_state.value
            feedback.progress = 0.0
            self.action_server.publish_feedback(feedback)
            
            # 执行任务
            task_result = self.current_task.execute()
            
            # 设置结果
            result = TaskExecutionResult()
            result.result = task_result.value
            result.message = f"任务 {goal.task_name} 执行完成"
            
            if task_result == TaskResult.SUCCESS:
                self.action_server.set_succeeded(result)
            else:
                self.action_server.set_aborted(result)
                
        except Exception as e:
            rospy.logerr(f"任务执行异常: {str(e)}")
            result = TaskExecutionResult()
            result.result = TaskResult.FAILURE.value
            result.message = f"任务执行异常: {str(e)}"
            self.action_server.set_aborted(result)
        
        finally:
            with self.task_lock:
                self.current_task = None
                self.task_state = TaskState.IDLE

if __name__ == '__main__':
    server = TaskExecutionServer()
    rospy.spin()