#!/usr/bin/env python
import rospy
import smach
import smach_ros
import actionlib
from robot_enums import TaskState, TaskResult
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

class NavigateToTarget(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'aborted', 'preempted'],
                           input_keys=['target_pose'],
                           output_keys=['nav_result'])
        self.nav_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.task_state_pub = rospy.Publisher('/task_state_update', String, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("开始导航到目标点")
        self.task_state_pub.publish(TaskState.NAVIGATING.value)
        
        self.nav_client.wait_for_server(rospy.Duration(5.0))
        if not self.nav_client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("导航服务器未响应")
            return 'aborted'
        
        goal = MoveBaseGoal()
        goal.target_pose = userdata.target_pose
        
        self.nav_client.send_goal(goal)
        
        # 等待结果，支持抢占
        finished_within_time = self.nav_client.wait_for_result(rospy.Duration(30.0))
        
        if not finished_within_time:
            self.nav_client.cancel_goal()
            rospy.logwarn("导航超时")
            return 'aborted'
        
        state = self.nav_client.get_state()
        if state == actionlib.GoalStatus.SUCCEEDED:
            userdata.nav_result = TaskResult.SUCCESS
            return 'succeeded'
        else:
            userdata.nav_result = TaskResult.FAILURE
            return 'aborted'

class AdjustPose(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'aborted'],
                           input_keys=['target_orientation'])
        self.task_state_pub = rospy.Publisher('/task_state_update', String, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("调整机器人姿态")
        self.task_state_pub.publish(TaskState.ADJUSTING_POSE.value)
        
        # 这里实现具体的姿态调整逻辑
        # 可以是另一个Action或者发布特定的cmd_vel指令
        rospy.sleep(2.0)  # 临时用sleep，实际应该用Action反馈
        
        return 'succeeded'

class TakePhoto(smach.State):
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'aborted'],
                           output_keys=['photo_result'])
        self.task_state_pub = rospy.Publisher('/task_state_update', String, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("开始拍照")
        self.task_state_pub.publish(TaskState.TAKING_PHOTO.value)
        
        # 这里调用拍照Action
        # photo_client.send_goal(photo_goal)
        # 临时用sleep模拟
        rospy.sleep(1.0)
        
        userdata.photo_result = TaskResult.SUCCESS
        return 'succeeded'

class ReportComplete(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.task_state_pub = rospy.Publisher('/task_state_update', String, queue_size=1)

    def execute(self, userdata):
        rospy.loginfo("任务完成，上报结果")
        self.task_state_pub.publish(TaskState.COMPLETED.value)
        return 'succeeded'

def create_task_state_machine():
    sm = smach.StateMachine(outcomes=['TASK_COMPLETED', 'TASK_FAILED'])
    
    # 设置用户数据
    sm.userdata.target_pose = PoseStamped()  # 从外部输入
    sm.userdata.target_orientation = 0.0     # 目标朝向
    
    with sm:
        smach.StateMachine.add('NAVIGATE_TO_TARGET', 
                             NavigateToTarget(),
                             transitions={'succeeded': 'ADJUST_POSE',
                                        'aborted': 'TASK_FAILED',
                                        'preempted': 'TASK_FAILED'})
        
        smach.StateMachine.add('ADJUST_POSE',
                             AdjustPose(),
                             transitions={'succeeded': 'TAKE_PHOTO',
                                        'aborted': 'TASK_FAILED'})
        
        smach.StateMachine.add('TAKE_PHOTO',
                             TakePhoto(),
                             transitions={'succeeded': 'REPORT_COMPLETE',
                                        'aborted': 'TASK_FAILED'})
        
        smach.StateMachine.add('REPORT_COMPLETE',
                             ReportComplete(),
                             transitions={'succeeded': 'TASK_COMPLETED'})
    
    return sm

if __name__ == '__main__':
    rospy.init_node('task_state_machine')
    
    sm = create_task_state_machine()
    
    # 可选：启动SMACH查看器
    sis = smach_ros.IntrospectionServer('task_server', sm, '/TASK_SM')
    sis.start()
    
    outcome = sm.execute()
    rospy.loginfo("任务执行结果: %s", outcome)
    
    sis.stop()