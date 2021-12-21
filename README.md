
1,hand_palm_linkを動かす。hand_palm_linkをを指す。

2, $rostopic pub flag_line std_msgs/UInt32 2

3,別の位置にhand_palm_linkを動かす。hand_palm_linkをを指す。

4, $rostopic pub flag_line std_msgs/UInt32 3

5,机のゴミ(印)を指す(指差ししている人をみている状況)。

6, $rostopic pub flag_line std_msgs/UInt32 4

7,机の方向を向く

8,較正後の推定位置(提案手法)
  従来手法
  較正なしの手法
   -目から指先
   -肘から指先
   -第三関節から指先
  を比べる。


command====================================
$ roslaunch openpose_ros calibration_pointing.launch

$ rosrun openpose_ros calibrate_action
