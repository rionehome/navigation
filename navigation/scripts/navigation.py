#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import actionlib
from actionlib_msgs.msg import GoalID
from actionlib_msgs.msg import GoalStatus
from location.msg import Location
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Header, String, Bool
from geometry_msgs.msg import Point, Quaternion
from location.srv import *
from sound_system.srv import *


class Navigation:

	def __init__(self):
		self.speak_topic = "/sound_system/speak"

		rospy.init_node('navigation', anonymous=False)
		rospy.Subscriber("/navigation/move_command", Location, self.navigation_callback)
		self.result_publisher = rospy.Publisher("/navigation/goal", Bool, queue_size=10)
		rospy.spin()

	def navigation_callback(self, message):
		# type: (Location) -> None
		print message, "@navigation_callback"
		"""
		移動命令を受け取って実際にmove_baseに移動命令をアクションで送る
		:param message: 場所情報
		:return: なし
		"""
		try:
			# 接続要求
			# gmapping か amcl が立ち上げっていれば繋がる
			client = actionlib.SimpleActionClient("/move_base", MoveBaseAction)
			print "wait move_base server"
			client.wait_for_server()
			print "callback move_base"
			# データの作成
			# ただしなぜかは不明だが orientation(型はOrientation)の初期値は(0,0,0,0)だがこれだと動かない (原因は不明)
			# 必ず w の値を0以外に設定する
			goal = MoveBaseGoal()
			goal.target_pose.header.stamp = rospy.Time.now()
			goal.target_pose.header.frame_id = "map"
			goal.target_pose.pose.position = Point(message.x, message.y, message.z)
			goal.target_pose.pose.orientation = Quaternion(0, 0, 0, 1)

			# データの送信
			print "send place msg @navigation"
			client.send_goal(goal)

			# データ送信後アクションサーバーからの返答待ち
			# 失敗にしろ成功にしろ結果は返ってくるのでタイムアウトの必要はなし
			client.wait_for_result()
			if client.get_state() == GoalStatus.SUCCEEDED:
				print("SUCCEEDED")
				answer = "I arrived at the target point"
				rospy.wait_for_service(self.speak_topic)
				rospy.ServiceProxy(self.speak_topic, StringService)(answer)
				self.result_publisher.publish(True)
			elif client.get_state() == GoalStatus.ABORTED:
				# orientationのw値が0だとこっちが即返ってくる
				print("ABORTED")
				answer = "Sorry, I can't arrived at the target point"
				rospy.wait_for_service(self.speak_topic)
				rospy.ServiceProxy(self.speak_topic, StringService)(answer)
				self.result_publisher.publish(False)
		except rospy.ServiceException as e:
			rospy.ERROR(e)


if __name__ == "__main__":
	Navigation()
