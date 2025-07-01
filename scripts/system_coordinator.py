#!/usr/bin/env python
import rospy
import json
from std_msgs.msg import String, Bool
from robot_enums import ControlMode, SystemStatus

class SystemCoordinator:
    """系统协调器 - 协调任务层和控制层"""
    
    def __init__(self):
        rospy.init_node('system_coordinator')
        
        # 任务层状态发布器
        self.task_status_pub = rospy.Publisher('/task_layer/status', String, queue_size=1)
        
        # 订阅任务执行状态
        rospy.Subscriber('/task_execution_status', String, self._task_execution_callback)
        
        # 订阅控制层状态
        rospy.Subscriber('/robot_status', String, self._control_status_callback)
        
        # 系统状态
        self.task_active = False
        self.control_available = True
        
        rospy.loginfo("系统协调器已启动")
    
    def _task_execution_callback(self, msg):
        """任务执行状态回调"""
        try:
            status_data = json.loads(msg.data)
            task_status = status_data.get('status')
            
            if task_status == 'running':
                self.task_active = True
            elif task_status in ['completed', 'failed', 'cancelled']:
                self.task_active = False
            
            # 通知控制层
            self._publish_task_status()
            
        except json.JSONDecodeError:
            rospy.logwarn("任务状态格式错误")
    
    def _control_status_callback(self, msg):
        """控制层状态回调"""
        # 处理控制层状态变化
        pass
    
    def _publish_task_status(self):
        """发布任务层状态"""
        status = {
            "active": self.task_active,
            "timestamp": rospy.Time.now().to_sec()
        }
        self.task_status_pub.publish(json.dumps(status))

if __name__ == '__main__':
    try:
        coordinator = SystemCoordinator()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass