#!/usr/bin/env python3
import rospy
import math
import xml.etree.ElementTree as ET
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from actionlib_msgs.msg import GoalStatus
import tf2_ros
import tf2_geometry_msgs

class DirectWaypointNavigator:
    def __init__(self, waypoint_file):
        self.waypoints = self.load_waypoints(waypoint_file)
        # 限制只跑前5个航点
        if len(self.waypoints) > 5:
            self.waypoints = self.waypoints[:5]
            rospy.loginfo("已限制仅运行前5个航点")

        self.current_index = 0
        self.retry_count = 0
        self.max_retry = 2
        self.timeout_sec = 60
        self.arrive_dist_threshold = 0.35

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("等待move_base导航服务连接...")
        self.client.wait_for_server()
        rospy.loginfo("导航服务已连接，本次运行航点数量：%d", len(self.waypoints))
        rospy.sleep(0.5)
        self.send_next_goal()

    def get_current_pose(self):
        """从TF获取机器人在map坐标系下的当前位置"""
        try:
            transform = self.tf_buffer.lookup_transform(
                "map", "base_link", rospy.Time(0), rospy.Duration(1.0))
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return x, y
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            return 0.0, 0.0

    def load_waypoints(self, file_path):
        waypoints = []
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            for waypoint in root.findall('Waypoint'):
                name = waypoint.find('Name').text
                x = float(waypoint.find('Pos_x').text)
                y = float(waypoint.find('Pos_y').text)
                qx = float(waypoint.find('Ori_x').text)
                qy = float(waypoint.find('Ori_y').text)
                qz = float(waypoint.find('Ori_z').text)
                qw = float(waypoint.find('Ori_w').text)
                rospy.loginfo("加载航点: %s (%.2f, %.2f)", name, x, y)
                waypoints.append((x, y, qx, qy, qz, qw))
        except Exception as e:
            rospy.logerr("加载航点文件失败: %s", e)
            rospy.signal_shutdown("文件加载失败")
        return waypoints

    def get_dist_to_target(self, target_x, target_y):
        cx, cy = self.get_current_pose()
        dx = cx - target_x
        dy = cy - target_y
        return math.hypot(dx, dy)

    def send_next_goal(self):
        # 所有航点已完成 → 直接退出
        if self.current_index >= len(self.waypoints):
            rospy.loginfo("全部指定航点行驶完毕，任务结束！")
            rospy.signal_shutdown("所有航点已完成")
            return

        x, y, qx, qy, qz, qw = self.waypoints[self.current_index]
        
        # 计算到目标的距离
        dist = self.get_dist_to_target(x, y)

        # 如果当前位置已接近目标，直接判定到达
        if dist < self.arrive_dist_threshold:
            rospy.loginfo("航点%d距离过近，直接判定到达", self.current_index+1)
            self.retry_count = 0
            self.current_index += 1
            rospy.sleep(0.8)
            self.send_next_goal()
            return

        rospy.loginfo("正在前往第 %d 个航点: (%.2f, %.2f)", self.current_index+1, x, y)
        self.client.cancel_all_goals()
        rospy.sleep(0.2)

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        goal.target_pose.pose.orientation.x = qx
        goal.target_pose.pose.orientation.y = qy
        goal.target_pose.pose.orientation.z = qz
        goal.target_pose.pose.orientation.w = qw

        self.client.send_goal(goal)
        finish = self.client.wait_for_result(rospy.Duration(self.timeout_sec))
        state = self.client.get_state()

        if finish and state == GoalStatus.SUCCEEDED:
            rospy.loginfo("航点 %d 到达成功", self.current_index+1)
            self.retry_count = 0
            self.current_index += 1
            rospy.sleep(1)
            self.send_next_goal()
        else:
            self.retry_count += 1
            if not finish:
                err_msg = "导航超时"
            elif state == GoalStatus.ABORTED:
                err_msg = "规划失败/障碍物阻挡"
            else:
                err_msg = "未知导航失败"

            rospy.logerr("航点%d %s，重试次数：%d/%d",
                         self.current_index+1, err_msg, self.retry_count, self.max_retry)
            if self.retry_count <= self.max_retry:
                rospy.sleep(1)
                self.send_next_goal()
            else:
                rospy.logwarn("航点%d多次失败，强制跳过", self.current_index+1)
                self.retry_count = 0
                self.current_index += 1
                rospy.sleep(1)
                self.send_next_goal()

if __name__ == '__main__':
    rospy.init_node('direct_nav_waypoints', anonymous=True)
    waypoint_file = "/home/ucar/waypoints.xml"
    try:
        navigator = DirectWaypointNavigator(waypoint_file)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

