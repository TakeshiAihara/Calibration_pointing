#include <iostream>
#include <stdio.h>
#include <math.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/LU>
#include <string>
#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/common/transforms.h>
#include <time.h>
#include <pcl/features/normal_3d_omp.h>
#include <fstream>
#include <limits>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <tf2_ros/static_transform_broadcaster.h>//tf2関連
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>//tf2::transform使えるようにする

#include <pcl/people/person_cluster.h> //角度について
#include <algorithm>
#include <vector>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "openpose_ros_msgs/PointWithProb.h"

//複数のトピックをサブスクライブするため
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>

//pcl::PointXYZ-位置座標のみ
//pcl::PointXYZRGB-位置座標＋色情報
//pcl::PointXYZRGBA-位置座標＋色情報＋透明度情報
//pcl::PointNormal-位置座標＋法線情報＋曲率情報

using namespace Eigen;
typedef pcl::PointXYZRGBA PointT;

class detect_poses {
private:
	ros::Subscriber sub_poses_Nose;
	ros::Subscriber sub_poses_LEye;
	ros::Subscriber sub_poses_Wrist;
	ros::Subscriber sub_poses_Elbow;
	ros::Subscriber sub_poses_Fingertip;
	ros::Subscriber sub_poses_First_joint;
	ros::Subscriber sub_poses_Second_joint;
	ros::Subscriber sub_poses_Third_joint;
	ros::Subscriber sub_depth;

	ros::Publisher pub_Nose_pose;
	ros::Publisher pub_LEye_pose;
	ros::Publisher pub_Wrist_pose;
	ros::Publisher pub_Elbow_pose;
	ros::Publisher pub_Fingertip_pose;
	ros::Publisher pub_First_joint_pose;
	ros::Publisher pub_Second_joint_pose;
	ros::Publisher pub_Third_joint_pose;

	int x_Nose = 0;
	int y_Nose = 0;
	int x_LEye = 0;
	int y_LEye = 0;
	int x_Wrist = 0;
	int y_Wrist = 0;
	int x_Elbow = 0;
	int y_Elbow = 0;
	int x_Fingertip = 0;
	int y_Fingertip = 0;
	int x_First_joint = 0;
	int y_First_joint = 0;
	int x_Second_joint = 0;
	int y_Second_joint = 0;
	int x_Third_joint = 0;
	int y_Third_joint = 0;

	double depth_Wrist = 0.0;
	double depth_Nose = 0.0;
	double depth_LEye = 0.0;
	double depth_Elbow = 0.0;
	double depth_Fingertip = 0.0;
	double depth_First_joint = 0.0;
	double depth_Second_joint = 0.0;
	double depth_Third_joint = 0.0;

public:
	detect_poses() {
		ros::NodeHandle nh("~");

		sub_poses_Nose = nh.subscribe("/openpose_ros/poses_Nose", 1, &detect_poses::poses_Nose_callback, this);
		sub_poses_LEye = nh.subscribe("/openpose_ros/poses_LEye", 1, &detect_poses::poses_LEye_callback, this);
		sub_poses_Wrist = nh.subscribe("/openpose_ros/poses_RWrist", 1, &detect_poses::poses_RWrist_callback, this);
		sub_poses_Elbow = nh.subscribe("/openpose_ros/poses_RElbow", 1, &detect_poses::poses_RElbow_callback, this);
		sub_poses_Fingertip = nh.subscribe("/openpose_ros/poses_Fingertip", 1, &detect_poses::poses_Fingertip_callback, this);
		sub_poses_First_joint = nh.subscribe("/openpose_ros/poses_First_joint", 1, &detect_poses::poses_First_joint_callback, this);
		sub_poses_Second_joint = nh.subscribe("/openpose_ros/poses_Second_joint", 1, &detect_poses::poses_Second_joint_callback, this);
		sub_poses_Third_joint = nh.subscribe("/openpose_ros/poses_Third_joint", 1, &detect_poses::poses_Third_joint_callback, this);

		//sub_depth = nh.subscribe("/camera/depth/image_rect_raw", 1, &detect_poses::depth_callback, this);
	  sub_depth = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/image_raw", 1, &detect_poses::depth_callback, this);

		pub_Nose_pose = nh.advertise<geometry_msgs::Pose>("Nose_pose", 1);
		pub_LEye_pose = nh.advertise<geometry_msgs::Pose>("LEye_pose", 1);
		pub_Wrist_pose = nh.advertise<geometry_msgs::Pose>("Wrist_pose", 1);
		pub_Elbow_pose = nh.advertise<geometry_msgs::Pose>("Elbow_pose", 1);
		pub_Fingertip_pose = nh.advertise<geometry_msgs::Pose>("Fingertip_pose", 1);
		pub_First_joint_pose = nh.advertise<geometry_msgs::Pose>("First_joint_pose", 1);
		pub_Second_joint_pose = nh.advertise<geometry_msgs::Pose>("Second_joint_pose", 1);
		pub_Third_joint_pose = nh.advertise<geometry_msgs::Pose>("Third_joint_pose", 1);
	}
	tf2::Transform transform_Wrist;
	tf2::Transform transform_Nose;
	tf2::Transform transform_LEye;
	tf2::Transform transform_Elbow;
	tf2::Transform transform_Fingertip;
	tf2::Transform transform_First_joint;
	tf2::Transform transform_Second_joint;
	tf2::Transform transform_Third_joint;

	//========出力関数=========================================================================
	void Output_pub(pcl::PointCloud<PointT>::Ptr cloud_,sensor_msgs::PointCloud2 msgs_,ros::Publisher pub_){
		if(cloud_->size() <= 0){
			ROS_INFO("Size (%lu)",cloud_->size());
		}
		pcl::toROSMsg(*cloud_, msgs_);
		// msgs_.header.frame_id = "camera_depth_optical_frame";
		msgs_.header.frame_id = "head_rgbd_sensor_rgb_frame";
		pub_.publish(msgs_);
	}

	void publish_pose(tf2::Transform input_transform, ros::Publisher pub_container, geometry_msgs::Pose msgs_container)
	{                                                                                         //Pose型：点群の位置、姿勢を表す

		msgs_container.position.x = input_transform.getOrigin().x();//原点ベクトルの平行移動を返す
		msgs_container.position.y = input_transform.getOrigin().y();
		msgs_container.position.z = input_transform.getOrigin().z();

		msgs_container.orientation.x = input_transform.getRotation().x();//回転を表すクォータニオンを返す
		msgs_container.orientation.y = input_transform.getRotation().y();
		msgs_container.orientation.z = input_transform.getRotation().z();
		msgs_container.orientation.w = input_transform.getRotation().w();

		pub_container.publish(msgs_container);

		return;
	}
	//openpose/poses_pub_Noseの処理
	void poses_Nose_callback(const openpose_ros_msgs::PointWithProb msgs_poses_Nose)
	{
		if(msgs_poses_Nose.prob!=0.0){
			x_Nose=msgs_poses_Nose.x;
			y_Nose=msgs_poses_Nose.y;
			  // std::cout << "x_Nose："<< x_Nose  <<" "<<"y_Nose：" <<y_Nose << '\n';
		 }
	}

	//openpose/poses_pub_LEyeの処理
	void poses_LEye_callback(const openpose_ros_msgs::PointWithProb msgs_poses_LEye)
	{
		if(msgs_poses_LEye.prob!=0.0){
			x_LEye=msgs_poses_LEye.x;
			y_LEye=msgs_poses_LEye.y;
			  // std::cout << "x_LEye："<< x_LEye  <<" "<<"y_LEye：" <<y_LEye << '\n';
		 }
	}

	//openpose/poses_pub_RWristの処理
	void poses_RWrist_callback(const openpose_ros_msgs::PointWithProb msgs_poses_RWrist)
	{

		if(msgs_poses_RWrist.prob!=0.0){
			x_Wrist=msgs_poses_RWrist.x;
			y_Wrist=msgs_poses_RWrist.y;
			// std::cout << "x_Wrist："<< x_Wrist  <<" "<<"y_Wrist：" <<y_Wrist << '\n';
		}
	}

	//openpose/poses_pub_RElbowの処理
	void poses_RElbow_callback(const openpose_ros_msgs::PointWithProb msgs_poses_RElbow)
	{
		if(msgs_poses_RElbow.prob!=0.0){
			x_Elbow=msgs_poses_RElbow.x;
			y_Elbow=msgs_poses_RElbow.y;
			// std::cout << "x_Elbow："<< x_Elbow  <<" "<<"y_Elbow：" <<y_Elbow << '\n';
		}
	}

	//openpose/poses_pub_Fingertipの処理
	void poses_Fingertip_callback(const openpose_ros_msgs::PointWithProb msgs_poses_Fingertip)
	{
		if(msgs_poses_Fingertip.prob!=0.0){
			x_Fingertip=msgs_poses_Fingertip.x;
			y_Fingertip=msgs_poses_Fingertip.y;
			// std::cout << "x_Fingertip："<< x_Fingertip  <<" "<<"y_Fingertip：" <<y_Fingertip << '\n';
		}
	}

	//openpose/poses_pub_First_jointの処理
	void poses_First_joint_callback(const openpose_ros_msgs::PointWithProb msgs_poses_First_joint)
	{
		if(msgs_poses_First_joint.prob!=0.0){
			x_First_joint=msgs_poses_First_joint.x;
			y_First_joint=msgs_poses_First_joint.y;
		 // std::cout << "x_First_joint："<< x_First_joint  <<" "<<"y_First_joint：" <<y_First_joint << '\n';
	 }
	}

	//openpose/poses_pub_Second_jointの処理
	void poses_Second_joint_callback(const openpose_ros_msgs::PointWithProb msgs_poses_Second_joint)
	{
		if(msgs_poses_Second_joint.prob!=0.0){
			x_Second_joint=msgs_poses_Second_joint.x;
			y_Second_joint=msgs_poses_Second_joint.y;
		 // std::cout << "x_Second_joint："<< x_Second_joint  <<" "<<"y_Second_joint：" <<y_Second_joint << '\n';
	 }
	}

	//openpose/poses_pub_Third_jointの処理
	void poses_Third_joint_callback(const openpose_ros_msgs::PointWithProb msgs_poses_Third_joint)
	{
		if(msgs_poses_Third_joint.prob!=0.0){
			x_Third_joint=msgs_poses_Third_joint.x;
			y_Third_joint=msgs_poses_Third_joint.y;
		 // std::cout << "x_Third_joint："<< x_Third_joint  <<" "<<"y_Third_joint：" <<y_Third_joint << '\n';
	 }
	}

  //スクリーン座標からワールド座標への計算
	void depth_callback(const sensor_msgs::ImageConstPtr &msg)
	{
		std::cout << "depth_callback_start" << '\n';
  	cv_bridge::CvImagePtr cv_ptr;
	    try{
	        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
	    }catch (cv_bridge::Exception& ex){
	        ROS_ERROR("error");
	        exit(-1);
	    }
			cv::Mat depth(cv_ptr->image.rows, cv_ptr->image.cols, CV_32FC1);
			Matrix3d K;//内部パラメータ
			MatrixXd world_Nose(3,1);
			MatrixXd screen_Nose(3,1);
			MatrixXd world_LEye(3,1);
			MatrixXd screen_LEye(3,1);
			MatrixXd world_Wrist(3,1);
			MatrixXd screen_Wrist(3,1);
			MatrixXd world_Elbow(3,1);
			MatrixXd screen_Elbow(3,1);
			MatrixXd world_Fingertip(3,1);
			MatrixXd screen_Fingertip(3,1);
			MatrixXd world_First_joint(3,1);
			MatrixXd screen_First_joint(3,1);
			MatrixXd world_Second_joint(3,1);
			MatrixXd screen_Second_joint(3,1);
			MatrixXd world_Third_joint(3,1);
			MatrixXd screen_Third_joint(3,1);
			//hsr
			K<< 540.0198,0.0,330.1313,
					0.0,542.5186,226.2484,
					0.0,0.0,1.0;
			//realsense
			// K<< 419.7269,0.0,423.9367,
			// 		0.0,419.7269,240.5581,
			// 		0.0,0.0,1.0;

			//鼻のｚ軸算出
			float* Dimage_Nose = cv_ptr->image.ptr<float>(y_Nose);
			depth_Nose=Dimage_Nose[x_Nose]/1000;
			  // std::cout << "depth_Nose=========================="<< depth_Nose << '\n';
			//左目のｚ軸算出
			float* Dimage_LEye = cv_ptr->image.ptr<float>(y_LEye);
			depth_LEye=Dimage_LEye[x_LEye]/1000;
			//手首のｚ軸算出
			float* Dimage_Wrist = cv_ptr->image.ptr<float>(y_Wrist);
			depth_Wrist=Dimage_Wrist[x_Wrist]/1000;
			//肘のｚ軸算出
			float* Dimage_Elbow = cv_ptr->image.ptr<float>(y_Elbow);
			depth_Elbow=Dimage_Elbow[x_Elbow]/1000;
			//指先のｚ軸算出
			float* Dimage_Fingertip = cv_ptr->image.ptr<float>(y_Fingertip);
			depth_Fingertip=Dimage_Fingertip[x_Fingertip]/1000;
			// std::cout << "depth_Fingertip========================"<< depth_Fingertip << '\n';
			//第一関節のｚ軸算出
			float* Dimage_First_joint = cv_ptr->image.ptr<float>(y_First_joint);
			depth_First_joint=Dimage_First_joint[x_First_joint]/1000;
			 // std::cout << "depth_First_joint========================"<< depth_First_joint << '\n';
			//第二関節のｚ軸算出
			float* Dimage_Second_joint = cv_ptr->image.ptr<float>(y_Second_joint);
			depth_Second_joint=Dimage_Second_joint[x_Second_joint]/1000;
			 // std::cout << "depth_Second_joint========================"<< depth_Second_joint << '\n';
			 // //第三関節のｚ軸算出
 			float* Dimage_Third_joint = cv_ptr->image.ptr<float>(y_Third_joint);
 			depth_Third_joint=Dimage_Third_joint[x_Third_joint]/1000;
 			 // std::cout << "depth_Third_joint========================"<< depth_Third_joint << '\n';

			screen_Wrist<<x_Wrist,
							      y_Wrist,
							      1;
			screen_Nose<<x_Nose,
									 y_Nose,
									 1;
			screen_LEye<<x_LEye,
						 			 y_LEye,
						 			 1;
			screen_Elbow<<x_Elbow,
						 			 y_Elbow,
						 			 1;
			screen_Fingertip<<x_Fingertip,
						 				    y_Fingertip,
						 			      1;
			screen_First_joint<<x_First_joint,
													y_First_joint,
													1;
			screen_Second_joint<<x_Second_joint,
										       y_Second_joint,
										       1;
			screen_Third_joint<<x_Third_joint,
													y_Third_joint,
													1;

			world_Wrist=(K.inverse())*depth_Wrist*screen_Wrist;//座標変換
			world_Nose=(K.inverse())*depth_Nose*screen_Nose;//座標変換
			world_LEye=(K.inverse())*depth_LEye*screen_LEye;//座標変換
			world_Elbow=(K.inverse())*depth_Elbow*screen_Elbow;//座標変換
			world_Fingertip=(K.inverse())*depth_Fingertip*screen_Fingertip;//座標変換
			world_First_joint=(K.inverse())*depth_First_joint*screen_First_joint;//座標変換
			world_Second_joint=(K.inverse())*depth_Second_joint*screen_Second_joint;//座標変換
			world_Third_joint=(K.inverse())*depth_Third_joint*screen_Third_joint;//座標変換


			//set_Wrist
			transform_Wrist.setOrigin( tf2::Vector3(world_Wrist(0,0), world_Wrist(1,0), world_Wrist(2,0)) );//ベクトル
			transform_Wrist.setRotation(tf2::Quaternion(0.0,0.0,0.0,1.0));
			//Publish_Wrist
			geometry_msgs::Pose msgs_Wrist_pose;
			publish_pose(transform_Wrist, pub_Wrist_pose, msgs_Wrist_pose);

			//set_Nose
			transform_Nose.setOrigin( tf2::Vector3(world_Nose(0,0), world_Nose(1,0), world_Nose(2,0)) );//ベクトル
			transform_Nose.setRotation(tf2::Quaternion(0.0,0.0,0.0,1.0));
			//Publish_Nose
			geometry_msgs::Pose msgs_Nose_pose;
			publish_pose(transform_Nose, pub_Nose_pose, msgs_Nose_pose);

			//set_LEye
			transform_LEye.setOrigin( tf2::Vector3(world_LEye(0,0), world_LEye(1,0), world_LEye(2,0)) );//ベクトル
			transform_LEye.setRotation(tf2::Quaternion(0.0,0.0,0.0,1.0));
			//Publish_LEye
			geometry_msgs::Pose msgs_LEye_pose;
			publish_pose(transform_LEye, pub_LEye_pose, msgs_LEye_pose);

			//set_Elbow
			transform_Elbow.setOrigin( tf2::Vector3(world_Elbow(0,0), world_Elbow(1,0), world_Elbow(2,0)) );//ベクトル
			transform_Elbow.setRotation(tf2::Quaternion(0.0,0.0,0.0,1.0));
			//Publish_Elbow
			geometry_msgs::Pose msgs_Elbow_pose;
			publish_pose(transform_Elbow, pub_Elbow_pose, msgs_Elbow_pose);

			//set_Fingertip
			transform_Fingertip.setOrigin( tf2::Vector3(world_Fingertip(0,0), world_Fingertip(1,0), world_Fingertip(2,0)) );//ベクトル
			transform_Fingertip.setRotation(tf2::Quaternion(0.0,0.0,0.0,1.0));
			//Publish_Fingertip
			geometry_msgs::Pose msgs_Fingertip_pose;
			publish_pose(transform_Fingertip, pub_Fingertip_pose, msgs_Fingertip_pose);

			//set_First_joint
			transform_First_joint.setOrigin( tf2::Vector3(world_First_joint(0,0), world_First_joint(1,0), world_First_joint(2,0)) );//ベクトル
			transform_First_joint.setRotation(tf2::Quaternion(0.0,0.0,0.0,1.0));
			//Publish_First_joint
			geometry_msgs::Pose msgs_First_joint_pose;
			publish_pose(transform_First_joint, pub_First_joint_pose, msgs_First_joint_pose);

			//set_Second_joint
			transform_Second_joint.setOrigin( tf2::Vector3(world_Second_joint(0,0), world_Second_joint(1,0), world_Second_joint(2,0)) );//ベクトル
			transform_Second_joint.setRotation(tf2::Quaternion(0.0,0.0,0.0,1.0));
			//Publish_Second_joint
			geometry_msgs::Pose msgs_Second_joint_pose;
			publish_pose(transform_Second_joint, pub_Second_joint_pose, msgs_Second_joint_pose);

			//set_Third_joint
			transform_Third_joint.setOrigin( tf2::Vector3(world_Third_joint(0,0), world_Third_joint(1,0), world_Third_joint(2,0)) );//ベクトル
			transform_Third_joint.setRotation(tf2::Quaternion(0.0,0.0,0.0,1.0));
			//Publish_Third_joint
			geometry_msgs::Pose msgs_Third_joint_pose;
			publish_pose(transform_Third_joint, pub_Third_joint_pose, msgs_Third_joint_pose);
	}
};

int main(int argc, char** argv)
{
	ros::init (argc, argv, "detect_poses");

	detect_poses detect_poses;

	ros::Rate spin_rate(1);
	while(ros::ok())
	{
		ros::spinOnce();
		spin_rate.sleep();
	}

	return 0;
}
