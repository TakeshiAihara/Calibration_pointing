#!/usr/bin/env python
# -*- coding: utf-8 -*-

import hsrb_interface
import rospy
import sys
import math
import tf
from hsrb_interface import geometry

# Preparation
robot = hsrb_interface.Robot()
omni_base = robot.try_get('omni_base')
whole_body = robot.try_get('whole_body')
tts = robot.try_get('default_tts')
gripper = robot.get('gripper')

tl = tf.TransformListener()
waiting_time = 60
step_size = 5

if __name__=='__main__':
	try:
		whole_body.move_to_neutral()
	except Exception as e:
		rospy.logerr('Fail (detect_tube: move_to_neutral): {}'.format(e))
		sys.exit(1)

	try:
		omni_base.go_abs(0,0,0)	#初期位置(mapの位置)に戻る
	except Exception as e:
		rospy.logerr('Fail (detect_tube: move_to_map): {}'.format(e))
		sys.exit(1)

	try:
		whole_body.move_to_neutral()
		gripper.command(0.8)
	except Exception as e:
		rospy.logerr('Fail (detect_tube: move_to_neutral): {}'.format(e))
		sys.exit(1)


	#sys.exit(1)
