#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatus
from location.msg import Location
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header, String, Bool
from geometry_msgs.msg import Point, Quaternion, Pose
from location.srv import *
from sound_system.srv import *
from std_msgs.msg import Float64
from tf.msg import tfMessage
import math
from rviz_marker import RvizMarker


class NavigationHumanDetect:

    def __init__(self):

        self.transform = None

        rospy.init_node('navigation', anonymous=False)
        rospy.Subscriber("/navigation_human_detect/move_command", Float64, self.navigation_callback)
        rospy.Subscriber("/tf", tfMessage, self.subscribe_location_tf)
        self.result_publisher = rospy.Publisher("/navigation_human_detect/goal", Bool, queue_size=10)
        self.rviz = RvizMarker()
        rospy.spin()

    def subscribe_location_tf(self, message):
        # type: (tfMessage) -> None
        """
        現在位置を取得し続ける
        :param message:
        :return:
        """
        for transform in message.transforms:
            if transform.header.frame_id == "odom" and transform.child_frame_id == "base_footprint":
                self.transform = transform.transform

    def calc(self, transform, range):
        range -= 0.1
        translation = transform.translation
        rotation = transform.rotation
        angle = 2 * math.acos(rotation.w)
        if rotation.z < 0:
            angle *= -1

        x = translation.x + range * math.cos(angle) - 0.7
        y = translation.y + range * math.sin(angle)

        pose = Pose()
        pose.position = Point(x, y, translation.z)
        #pose.orientation = Quaternion(rotation.x, rotation.y, rotation.z, rotation.w)
        pose.orientation = Quaternion(0, 0, 0, 1)
        return pose

    def navigation_callback(self, message):
        # type: (Float64) -> None
        """
        移動命令を受け取って実際にmove_baseに移動命令をアクションで送る
        :param message: 場所情報
        :return: なし
        """
        try:
            transform = None
            for i in range(10):
                if self.transform is not None:
                    transform = self.transform
                    break
                rospy.sleep(0.5)

            # 接続要求
            # gmapping か amcl が立ち上げっていれば繋がる
            client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
            print("wait move_base server")
            client.wait_for_server()
            print("callback move_base")
            # データの作成
            # ただしなぜかは不明だが orientation(型はOrientation)の初期値は(0,0,0,0)だがこれだと動かない (原因は不明)
            # 必ず w の値を0以外に設定する

            goal = MoveBaseGoal()
            goal.target_pose.header.stamp = rospy.Time.now()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose = self.calc(transform, message.data)
            self.rviz.register(goal.target_pose.pose)
            # データの送信
            print("send place msg @navigation")
            client.send_goal(goal)

            # データ送信後アクションサーバーからの返答待ち
            # 失敗にしろ成功にしろ結果は返ってくるのでタイムアウトの必要はなし
            client.wait_for_result()
            if client.get_state() == GoalStatus.SUCCEEDED:
                print("SUCCEEDED")
                self.result_publisher.publish(True)
            elif client.get_state() == GoalStatus.ABORTED:
                # orientationのw値が0だとこっちが即返ってくる
                print("ABORTED")
                self.result_publisher.publish(False)

        except rospy.ServiceException as e:
            rospy.ERROR(e)


if __name__ == "__main__":
    NavigationHumanDetect()
