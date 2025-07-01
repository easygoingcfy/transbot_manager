#!/usr/bin/env python

"""Mapping Manager for Transbot."""

import rospy
from cartographer_ros_msgs.srv import FinishTrajectory, WriteState
from std_srvs.srv import Trigger, TriggerResponse
import subprocess
import os

class MappingManager:
    def __init__(self):
        # 获取地图保存目录和默认地图名参数
        self.map_dir = rospy.get_param("~map_dir", "/home/jetson/transbot_ws/src/transbot_manager/maps")
        self.default_map_name = rospy.get_param("~map_name", "carto_map")
        # 提供服务，支持通过请求参数指定地图名
        rospy.Service('~finish_and_save_map', Trigger, self.handle_finish_and_save)

    def handle_finish_and_save(self, req):
        """
        结束建图、保存pbstream、生成pgm/yaml地图文件。
        地图名可通过参数~map_name动态指定。
        """
        try:
            # 获取当前地图名参数（支持动态修改）
            map_name = rospy.get_param("~map_name", self.default_map_name)
            pbstream_path = os.path.join(self.map_dir, "{}.pbstream".format(map_name))
            map_filestem = os.path.join(self.map_dir, map_name)

            # 1. 结束建图轨迹
            rospy.wait_for_service('/finish_trajectory')
            finish_srv = rospy.ServiceProxy('/finish_trajectory', FinishTrajectory)
            finish_srv(0)
            rospy.loginfo("Trajectory finished.")

            # 2. 保存pbstream文件
            rospy.wait_for_service('/write_state')
            write_srv = rospy.ServiceProxy('/write_state', WriteState)
            write_srv(pbstream_path)
            rospy.loginfo("State written to %s", pbstream_path)

            # 3. pbstream转pgm/yaml地图
            cmd = [
                "rosrun", "cartographer_ros", "cartographer_pbstream_to_ros_map",
                "-pbstream_filename=" + pbstream_path,
                "-map_filestem=" + map_filestem
            ]
            subprocess.check_call(cmd)
            rospy.loginfo("Map generated at %s(.pgm/.yaml)", map_filestem)
            return TriggerResponse(success=True, message="Map saved successfully as '{}'.".format(map_name))
        except Exception as e:
            rospy.logerr(str(e))
            return TriggerResponse(success=False, message=str(e))

if __name__ == "__main__":
    rospy.init_node('mapping_manager')
    MappingManager()
    rospy.spin()