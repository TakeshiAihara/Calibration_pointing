#!/usr/bin/env python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import math
from std_msgs.msg import UInt32
import sys
import subprocess
from hsrb_interface import geometry
import geometry_msgs.msg
import tf
import shape_msgs.msg
from tf.transformations import quaternion_from_euler, quaternion_multiply
import trajectory_msgs.msg

# Preparation
robot = hsrb_interface.Robot()
omni_base = robot.try_get('omni_base')
tts = robot.try_get('default_tts')
whole_body = robot.get('whole_body')

#Publisher
# rospy.init_node('Python_pub')
pub = rospy.Publisher('flag_line', UInt32 , queue_size=1)
# Main
if __name__=='__main__':
	#関節を検出できる位置に移動
	rospy.sleep(1)#冒頭でsayを実行する場合は、その前にスリープを入れると安定する。
#	tts.say(u'移動します')
	try:
		rospy.loginfo('Start (move_to_front)')
		whole_body.move_to_neutral()#腕を前にする姿勢
		omni_base.go_pose(geometry.pose(ei=3.14, ej=-1.57,ek=3.14), 100.0, ref_frame_id='poses_front')#後ろ向く
		rospy.loginfo('End (move_to_front)')
	except Exception as e:
		#tts.say(u'移動できませんでした')
		rospy.logerr('Fail (move_to_front): {}'.format(e))#ログメッセージからエラー出力


	rospy.sleep(1)
	#tts.say(u'較正を行うための作業を行います')

	#一回目の指差し(左)=================================================================================
	try:
		rospy.loginfo('Start (calibration_1)')
		#一回目の較正(ハンドの姿勢形成)
		# whole_body.move_to_joint_positions({'wrist_flex_joint': 0.07})#手首調整
		whole_body.move_to_joint_positions({'arm_flex_joint': -1.9})#アーム下げる
		whole_body.move_to_joint_positions({'wrist_flex_joint': 0.35})#手首調整
		whole_body.move_to_joint_positions({'head_pan_joint': math.radians(45)})#視線を左に向ける
		whole_body.move_to_joint_positions({'head_tilt_joint': math.radians(-20)})#視線を下に向ける
		rospy.loginfo('End (calibration_1)')
	except Exception as e:
		# tts.say(u'人が見つかりませんでした')
		rospy.logerr('Fail (calibration_1): {}'.format(e))
	#tts.say(u'ハンドを指差してください')
	rospy.sleep(5)

	#flagに2をなげる========================================================================
	try:
		rospy.loginfo('Start (flag_line_2)')
		pub.publish(2)
		rospy.loginfo('End (flag_line_2)')
	except Exception as e:
		rospy.logerr('Fail (flag_line_2): {}'.format(e))
	#tts.say(u'指差しをやめてください')

	#二回目の指差し(正面下)=================================================================================
	try:
		rospy.loginfo('Start (calibration_2)')
		#二回目の較正(ハンドの姿勢形成)
		omni_base.go_rel(0.0,0.0,1.047,100.0)#台車を左に６０度移動
		whole_body.move_to_joint_positions({'head_pan_joint': math.radians(-15)})#右をむく(首)
		whole_body.move_to_joint_positions({'head_tilt_joint': math.radians(-20)})#視線を下に向ける
		rospy.loginfo('End (calibration_2)')
	except Exception as e:
		# tts.say(u'人が見つかりませんでした')
		rospy.logerr('Fail (calibration_2): {}'.format(e))
	#tts.say(u'ハンドをもう一度、指差してください')
	rospy.sleep(5)

	#flagに3をなげる=========================================================================
	try:
		rospy.loginfo('Start (flag_line_3)')
		pub.publish(3)
		rospy.loginfo('End (flag_line_3)')
	except Exception as e:
		rospy.logerr('Fail (flag_line_3): {}'.format(e))
	#tts.say(u'指差しをやめてください')

	#机の指差し===============================================================================
	try:
		rospy.loginfo('Start (neutral_poses)')
		whole_body.move_to_neutral()
		#骨格検出姿勢に移る
		omni_base.go_rel(0.0,0.0,0.785,100.0)#台車を左に45度移動
		whole_body.move_to_joint_positions({'head_pan_joint': math.radians(-45)})#右をむく(首)
		whole_body.move_to_joint_positions({'head_tilt_joint': math.radians(-20)})#視線を下に向ける
		rospy.loginfo('End (neutral_poses)')
	except Exception as e:
		tts.say(u'人が見つかりませんでした')
		rospy.logerr('Fail (neutral_poses): {}'.format(e))
	rospy.sleep(1)

	#tts.say(u'机のゴミを指差してください')
	rospy.sleep(5)
	#tts.say(u'指差しをやめてください')

	#flagに4をなげる
	try:
		rospy.loginfo('Start (flag_line_4)')
		pub.publish(4)
		rospy.loginfo('End (flag_line_4)')
	except Exception as e:
		rospy.logerr('Fail (flag_line_4): {}'.format(e))

	#机の方向を向く===============================================================================
	rospy.sleep(1)
	#tts.say(u'指差し位置を特定します')
	try:
		rospy.loginfo('Start (move_to_plane)')
		whole_body.move_to_neutral()
		omni_base.go_pose(geometry.pose(ei=3.14, ej=-1.57,ek=3.14), 100.0, ref_frame_id='plane_front')
		omni_base.go_rel(0.0,0.0,0.785,100.0)#台車を左に45度移動
		whole_body.move_to_joint_positions({'head_pan_joint': math.radians(-45)})#右をむく(首)
		whole_body.move_to_joint_positions({'head_tilt_joint': math.radians(-45)})#視線を下に向ける
		rospy.loginfo('End (move_to_plane)')
	except Exception as e:
		#tts.say(u'移動に失敗しました')
		rospy.logerr('Fail (move_to_plane): {}'.format(e))
	rospy.sleep(1)

	#初期位置に戻る========================================================================
	# tts.say(u'初期位置に戻ります。')
	# try:
	# 	rospy.loginfo('Start (neutral)')
	# 	res = subprocess.check_output(["python neutral.py"],shell=True)
	# 	rospy.loginfo('End (neutral)')
	# except Exception as e:
	# 	rospy.logerr('Fail (neutral): {}'.format(e))
	# tts.say(u'終了します。')
	sys.exit(1)
