#include <iostream>
#include <stdio.h>
#include <math.h>
#include <bits/stdc++.h>
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
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

#include <tf2_ros/static_transform_broadcaster.h>//tf2関連
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>//tf2::transform使えるようにする
#include <tf2_eigen/tf2_eigen.h>

#include <pcl/people/person_cluster.h> //角度について
#include <algorithm>
#include <vector>

#include "openpose_ros_msgs/PointWithProb.h"

#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/UInt32.h>//UInt16型のトピックを受け取れるようになる

//pcl::PointXYZ-位置座標のみ
//pcl::PointXYZRGB-位置座標＋色情報
//pcl::PointXYZRGBA-位置座標＋色情報＋透明度情報
//pcl::PointNormal-位置座標＋法線情報＋曲率情報

using namespace Eigen;
typedef pcl::PointXYZRGBA PointT;

class tabletop_trash {
private:
	ros::Subscriber sub_;
	ros::Subscriber sub_Nose;
	ros::Subscriber sub_Wrist;
	ros::Subscriber sub_Elbow;
	ros::Subscriber sub_Fingertip;
	ros::Subscriber sub_Second_joint;
	ros::Subscriber sub_Third_joint;
	ros::Subscriber sub_flag_line;

	ros::Publisher pub_plane;
	ros::Publisher pub_rest;
	ros::Publisher pub_table_plane;
	ros::Publisher pub_table_rest;

	ros::Publisher pub_trimming;
	ros::Publisher pub_tube;
	ros::Publisher pub_dwnsmp;
	ros::Publisher pub_cluster_0;
	ros::Publisher pub_cluster_1;
	ros::Publisher pub_cluster_2;
	ros::Publisher pub_cluster_3;
	ros::Publisher pub_tube_cluster;
	ros::Publisher pub_tube_points;

	ros::Publisher pub_tube_ave_point;
	ros::Publisher pub_plane_ave_point;
	ros::Publisher pub_cluster_ave_point;
	ros::Publisher pub_tube_centroid_point;
	ros::Publisher pub_cluster_centroid_point;

	ros::Publisher pub_tube_pose;
	ros::Publisher pub_plane_pose;

	ros::Publisher pub_Nose;
	ros::Publisher pub_Wrist;
	ros::Publisher pub_Elbow;
	ros::Publisher pub_Fingertip;
	ros::Publisher pub_Second_joint;
	ros::Publisher pub_Third_joint;
	ros::Publisher pub_line;
	ros::Publisher pub_trimming_line;
	ros::Publisher pub_cluster_best;

	float threshold_plane = 0.035;//閾値(値が大きいほど除去が厳しくなる)
	float threshold_tube = 0.0804;
	int number_neighbors = 20;
	float radius_min_limits = 0.015;
	float radius_max_limits = 0.04;

	float x_Wrist = 0.0;
	float x_Nose = 0.0;
	float x_Elbow = 0.0;
	float x_Fingertip = 0.0;
	float x_Second_joint = 0.0;
	float x_Third_joint = 0.0;

	float y_Wrist = 0.0;
	float y_Nose = 0.0;
	float y_Elbow = 0.0;
	float y_Fingertip = 0.0;
	float y_Second_joint = 0.0;
	float y_Third_joint = 0.0;

	float z_Wrist = 0.0;
	float z_Nose = 0.0;
	float z_Elbow = 0.0;
	float z_Fingertip = 0.0;
	float z_Second_joint = 0.0;
	float z_Third_joint = 0.0;

	int flag_line=1;
  //保管用の指差し延長線
	pcl::PointCloud<PointT>cloud_line;
	pcl::PointCloud<PointT>cloud_trimming_line;

public:
	tabletop_trash() {
		ros::NodeHandle nh("~");
		nh.param<float>("threshold_plane", threshold_plane, threshold_plane);
		nh.param<float>("threshold_tube", threshold_tube, threshold_tube);
		nh.param<float>("radius_min_limits", radius_min_limits, radius_min_limits);
		nh.param<float>("radius_max_limits", radius_max_limits, radius_max_limits);

		// sub_ = nh.subscribe("/camera/depth/color/points", 1, &tabletop_trash::cloud_callback, this);
		sub_ = nh.subscribe("/hsrb/head_rgbd_sensor/depth_registered/rectified_points", 1, &tabletop_trash::cloud_callback, this);
		sub_Nose = nh.subscribe("/detect_poses_node/Nose_pose", 1, &tabletop_trash::Nose_callback, this);
		sub_Wrist = nh.subscribe("/detect_poses_node/Wrist_pose", 1, &tabletop_trash::Wrist_callback, this);
		sub_Elbow = nh.subscribe("/detect_poses_node/Elbow_pose", 1, &tabletop_trash::Elbow_callback, this);
		sub_Fingertip = nh.subscribe("/detect_poses_node/Fingertip_pose", 1, &tabletop_trash::Fingertip_callback, this);
		sub_Second_joint = nh.subscribe("/detect_poses_node/Second_joint_pose", 1, &tabletop_trash::Second_joint_callback, this);
		sub_Third_joint = nh.subscribe("/detect_poses_node/Third_joint_pose", 1, &tabletop_trash::Third_joint_callback, this);
		sub_flag_line = nh.subscribe("/flag_line", 1, &tabletop_trash::Flag_line_callback, this);

		pub_plane = nh.advertise<sensor_msgs::PointCloud2>("cloud_plane", 1);
		pub_rest = nh.advertise<sensor_msgs::PointCloud2>("cloud_rest", 1);
		pub_table_plane = nh.advertise<sensor_msgs::PointCloud2>("cloud_table_plane", 1);
		pub_table_rest = nh.advertise<sensor_msgs::PointCloud2>("cloud_table_rest", 1);
		pub_tube = nh.advertise<sensor_msgs::PointCloud2>("tube", 1);
		pub_dwnsmp =  nh.advertise<sensor_msgs::PointCloud2>("dwnsmp", 1);
		pub_cluster_0 =  nh.advertise<sensor_msgs::PointCloud2>("cluster_0", 1);
		pub_cluster_1 =  nh.advertise<sensor_msgs::PointCloud2>("cluster_1", 1);
		pub_cluster_2 =  nh.advertise<sensor_msgs::PointCloud2>("cluster_2", 1);
		pub_cluster_3 =  nh.advertise<sensor_msgs::PointCloud2>("cluster_3", 1);
		pub_tube_cluster =  nh.advertise<sensor_msgs::PointCloud2>("tube_cluster", 1);
		pub_tube_points =  nh.advertise<sensor_msgs::PointCloud2>("tube_points", 1);
		pub_Nose = nh.advertise<sensor_msgs::PointCloud2>("Nose",1);
		pub_Wrist = nh.advertise<sensor_msgs::PointCloud2>("Wrist",1);
		pub_Elbow = nh.advertise<sensor_msgs::PointCloud2>("Elbow",1);
		pub_Fingertip = nh.advertise<sensor_msgs::PointCloud2>("Fingertip",1);
		pub_Second_joint = nh.advertise<sensor_msgs::PointCloud2>("Second_joint",1);
		pub_Third_joint = nh.advertise<sensor_msgs::PointCloud2>("Third_joint",1);
		pub_line = nh.advertise<sensor_msgs::PointCloud2>("line",1);
		pub_trimming_line = nh.advertise<sensor_msgs::PointCloud2>("trimming_line",1);
		pub_cluster_best = nh.advertise<sensor_msgs::PointCloud2>("cluster_best",1);


		pub_tube_ave_point = nh.advertise<geometry_msgs::PointStamped>("tube_ave_point", 1);
		pub_plane_ave_point = nh.advertise<geometry_msgs::PointStamped>("plane_ave_point", 1);
		pub_tube_centroid_point = nh.advertise<geometry_msgs::PointStamped>("tube_centroid_point", 1);
		pub_cluster_ave_point = nh.advertise<geometry_msgs::PointStamped>("cluster_ave_point", 1);
		pub_cluster_centroid_point = nh.advertise<geometry_msgs::PointStamped>("cluster_centroid_point", 1);

		pub_tube_pose = nh.advertise<geometry_msgs::Pose>("tube_pose", 1);
		pub_plane_pose = nh.advertise<geometry_msgs::Pose>("planar_pose", 1);

	}
	tf2_ros::TransformBroadcaster br_plane;
	tf2::Transform transform_mult;//回転や並進などの変換をサポートするクラス

	tf2_ros::TransformBroadcaster br_tube;
	tf2::Transform transform_pub;

	tf2::Transform transform_mult_mult;
	tf2_ros::Buffer tfBuffer;

	//========出力関数=========================================================================
	void Output_pub(pcl::PointCloud<PointT>::Ptr cloud_,sensor_msgs::PointCloud2 msgs_,ros::Publisher pub_){
		if(cloud_->size() <= 0){
			// ROS_INFO("Size (%lu)",cloud_->size());
		}
		pcl::toROSMsg(*cloud_, msgs_);
		 // msgs_.header.frame_id = "camera_depth_optical_frame";
		 msgs_.header.frame_id = "head_rgbd_sensor_rgb_frame";
		pub_.publish(msgs_);
	}

	void Output_pub_MAP(pcl::PointCloud<PointT>::Ptr cloud_,sensor_msgs::PointCloud2 msgs_,ros::Publisher pub_){
		if(cloud_->size() <= 0){
			// ROS_INFO("Size (%lu)",cloud_->size());
		}
		pcl::toROSMsg(*cloud_, msgs_);
		msgs_.header.frame_id = "map";
		pub_.publish(msgs_);
	}

	//========出力関数(map基準の生の点群)=========================================================================
	void Output_pub_map(pcl::PointCloud<PointT> cloud_,sensor_msgs::PointCloud2 msgs_,ros::Publisher pub_){
		if(cloud_.size() <= 0){
			// ROS_INFO("Size (%lu)",cloud_->size());
		}
		pcl::toROSMsg(cloud_, msgs_);
		 msgs_.header.frame_id = "map";
		pub_.publish(msgs_);
	}

//ダウンサンプリング===============================================================================================
	pcl::PointCloud<PointT>::Ptr Down_Sampling(pcl::PointCloud<PointT>::Ptr cloud_input)
	{
		pcl::PointCloud<PointT>::Ptr cloud_vg (new pcl::PointCloud<PointT>);//格納用

		pcl::VoxelGrid<PointT> vg;//空間をボクセルで区切り、点群の重心で近似し、ダウンサンプリングを行う
		vg.setInputCloud (cloud_input);//入力データセットへのポインタを提供する
		vg.setLeafSize (0.005f, 0.005f, 0.005f);//サイズが0.5*0.5*0.5のフィルターを作成
		vg.filter (*cloud_vg);//入力データを渡し、計算、出力
		return cloud_vg;
	}


//サーフェス法線の推定==================================================
	pcl::PointCloud<pcl::Normal>::Ptr Estimated_Normal_OMP(pcl::PointCloud<PointT>::Ptr cloud_input)
	{
		pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);//法線情報を入れる変数
		pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());//KdTree構造をつくる
		pcl::NormalEstimationOMP<PointT, pcl::Normal> ne;//サーフェス曲線と曲率を推定する（Normal型点群）

		//法線の推定================
		ne.setSearchMethod (tree);//推定方法
		ne.setInputCloud (cloud_input);//推定に用いる点群
		ne.useSensorOriginAsViewPoint();//センサの原点を使用するか、ユーザが指定した視点を使用するかの設定
		ne.setNumberOfThreads(0);
		ne.setKSearch (50);//特徴推定に使用するｋ最近傍の数を所得する
		ne.compute (*cloud_normals);//推定するための計算（出力先はclod_normals）
		return cloud_normals;
	}


	pcl::PointCloud<PointT>::Ptr Detected_tube(pcl::PointCloud<PointT>::Ptr cloud_input, pcl::PointCloud<pcl::Normal>::Ptr normal_input)
	{
		//円柱を検出する=============
		pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg;//円柱を検出するクラス
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		seg.setInputCloud(cloud_input);
		seg.setInputNormals (normal_input);
		seg.setModelType(pcl::SACMODEL_CYLINDER);//円柱モデルの指定
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setNormalDistanceWeight (0.01);//点法線と平面法線の間の角距離に与える相対的な重み
		seg.setMaxIterations (10000);//試行回数
		seg.setDistanceThreshold(threshold_tube);//閾値設定
		seg.setRadiusLimits (radius_min_limits, radius_max_limits);//モデルの最小・最大許容半径制限を設定する
		seg.setOptimizeCoefficients(true);//係数の情報必要かどうかの設定
		seg.segment(*inliers, *coefficients);
		//std::cout << *coefficients << std::endl;
		// 円柱以外を除去する。＝＝＝＝＝＝＝＝＝＝
		pcl::PointCloud<PointT>::Ptr cloud_tube(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_outside_tube(new pcl::PointCloud<PointT>);
		pcl::ExtractIndices<PointT> extract;
		extract.setInputCloud(cloud_input);
		extract.setIndices(inliers);
		extract.setNegative(false);
		extract.filter(*cloud_tube);
		//std::cout << *cloud_tube << std::endl;
		return cloud_tube;
	}

	Eigen::Vector4f ave_point_of_PointCloud(pcl::PointCloud<PointT>::Ptr cloud_input, ros::Publisher pub_container, geometry_msgs::PointStamped msgs_container)
	{       //Eigen::Vector4f-int型の四次元ベクトル
					//geometry_msgs-姿勢
		      //平均を計算する
		float  x_sum  = 0.0;
		float  y_sum  = 0.0;
		float  z_sum  = 0.0;
		for(size_t i = 0; i< cloud_input->size(); ++i)
		{
			 x_sum += cloud_input->points[i].x;
			 y_sum += cloud_input->points[i].y;
			 z_sum += cloud_input->points[i].z;
		}
		Eigen::Vector4f ave_point;//格納
		ave_point.x() =  x_sum/static_cast<int>(cloud_input->size());
		ave_point.y() =  y_sum/static_cast<int>(cloud_input->size());
		ave_point.z() =  z_sum/static_cast<int>(cloud_input->size());

		msgs_container.header.stamp = ros::Time::now();//現在時刻
		msgs_container.point.x = ave_point.x();
		msgs_container.point.y = ave_point.y();
		msgs_container.point.z = ave_point.z();

		return ave_point;
	}

	Eigen::Vector4f centroid_point_of_PointCloud(pcl::PointCloud<PointT>::Ptr cloud_input, ros::Publisher pub_container, geometry_msgs::PointStamped msgs_container)
	{
		//中心座標の出力===================================================
		Eigen::Vector4f centroid_point;
		pcl::compute3DCentroid(*cloud_input, centroid_point);//重心を計算
                                                         //3Dベクトルとして返す
		msgs_container.header.stamp = ros::Time::now();
		msgs_container.point.x = centroid_point.x();
		msgs_container.point.y = centroid_point.y();
		msgs_container.point.z = centroid_point.z();

		pub_container.publish(msgs_container);

		return centroid_point;
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

	//点群をしぼる
	pcl::PointCloud<PointT>::Ptr Pass_Through(pcl::PointCloud<PointT>::Ptr cloud_input,float x_min,float x_max,float y_min,float y_max,float z_min,float z_max)
	{
		pcl::PassThrough<PointT>pass;
		pcl::PointCloud<PointT>::Ptr cloud_passthrough_x(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_passthrough_y(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_passthrough_z(new pcl::PointCloud<PointT>);

		pass.setInputCloud(cloud_input);
		pass.setFilterFieldName("x");
		pass.setFilterLimits(x_min,x_max);
		pass.filter(*cloud_passthrough_x);

		pass.setInputCloud(cloud_passthrough_x);
		pass.setFilterFieldName("y");
		pass.setFilterLimits(y_min,y_max);
		pass.filter(*cloud_passthrough_y);

		pass.setInputCloud(cloud_passthrough_y);
		pass.setFilterFieldName("z");
		pass.setFilterLimits(z_min,z_max);
		pass.filter(*cloud_passthrough_z);

		return cloud_passthrough_z;
	}

	//transform========================================================================================
	pcl::PointCloud<PointT>::Ptr Transform_points(pcl::PointCloud<PointT>::Ptr cloud_input, pcl::PointCloud<PointT>::Ptr cloud_output, Eigen::Vector4f plane_ave_point, tf2::Vector3 plane_axis,float plane_angle)
	{
		//座標変換(平行移動→回転移動)================================================================
		pcl::PointCloud<PointT>::Ptr cloud_transform(new pcl::PointCloud<PointT>);
		Eigen::Affine3f affine_translation=Eigen::Affine3f::Identity();
		affine_translation.translation()<<-plane_ave_point.x(),-plane_ave_point.y(),-plane_ave_point.z();
		pcl::transformPointCloud(*cloud_input,*cloud_transform,affine_translation);
		Eigen::Affine3f affine_rotation=Eigen::Affine3f::Identity();
		affine_rotation.rotate(Eigen::AngleAxisf(-plane_angle,Eigen::Vector3f(plane_axis.x(), plane_axis.y(), plane_axis.z())));
		pcl::transformPointCloud(*cloud_transform,*cloud_output,affine_rotation);
		return cloud_output;
	}

	//transform_return=====================================================================================
	pcl::PointCloud<PointT>::Ptr Transform_return_points(pcl::PointCloud<PointT>::Ptr cloud_input, pcl::PointCloud<PointT>::Ptr cloud_output, Eigen::Vector4f plane_ave_point, tf2::Vector3 plane_axis,float plane_angle)
	{
		//元の座標に戻す(回転移動→平行移動)
	 pcl::PointCloud<PointT>::Ptr cloud_transform_return(new pcl::PointCloud<PointT>);
	 Eigen::Affine3f affine_translation_return=Eigen::Affine3f::Identity();
	 affine_translation_return.rotate(Eigen::AngleAxisf(plane_angle,Eigen::Vector3f(plane_axis.x(), plane_axis.y(), plane_axis.z())));
	 pcl::transformPointCloud(*cloud_input,*cloud_transform_return,affine_translation_return);
	 Eigen::Affine3f affine_rotation_return=Eigen::Affine3f::Identity();
	 affine_rotation_return.translation()<<plane_ave_point.x(),plane_ave_point.y(),plane_ave_point.z();
	 pcl::transformPointCloud(*cloud_transform_return,*cloud_output,affine_rotation_return);
	 return cloud_output;
 }

	void Nose_callback(const geometry_msgs::Pose input_pose)
	{
		x_Nose=input_pose.position.x;
		y_Nose=input_pose.position.y;
		z_Nose=input_pose.position.z;
	  // std::cout << "x_Nose："<< x_Nose  <<" "<<"y_Nose：" <<y_Nose << " "<<"z_Nose：" <<z_Nose <<'\n';
	}

	void Wrist_callback(const geometry_msgs::Pose input_pose)
	{
		x_Wrist=input_pose.position.x;
		y_Wrist=input_pose.position.y;
		z_Wrist=input_pose.position.z;
		 // std::cout << "x_Wrist："<< x_Wrist  <<" "<<"y_Wrist：" <<y_Wrist << " "<<"z_Wrist：" <<z_Wrist <<'\n';
	}

	void Elbow_callback(const geometry_msgs::Pose input_pose)
	{
		x_Elbow=input_pose.position.x;
		y_Elbow=input_pose.position.y;
		z_Elbow=input_pose.position.z;
	}

	void Fingertip_callback(const geometry_msgs::Pose input_pose)
	{
		x_Fingertip=input_pose.position.x;
		y_Fingertip=input_pose.position.y;
		z_Fingertip=input_pose.position.z;
		 // std::cout << "x_Fingertip："<< x_Fingertip  <<" "<<"y_Fingertip：" <<y_Fingertip << " "<<"z_Fingertip：" <<z_Fingertip <<'\n';
	}

	void Second_joint_callback(const geometry_msgs::Pose input_pose)
	{
		x_Second_joint=input_pose.position.x;
		y_Second_joint=input_pose.position.y;
		z_Second_joint=input_pose.position.z;
		 // std::cout << "x_Second_joint："<< x_Second_joint  <<" "<<"y_Second_joint：" <<y_Second_joint << " "<<"z_Second_joint：" <<z_Second_joint <<'\n';
	}

	void Third_joint_callback(const geometry_msgs::Pose input_pose)
	{
		x_Third_joint=input_pose.position.x;
		y_Third_joint=input_pose.position.y;
		z_Third_joint=input_pose.position.z;
		 // std::cout << "x_Third_joint："<< x_Third_joint  <<" "<<"y_Third_joint：" <<y_Third_joint << " "<<"z_Third_joint：" <<z_Third_joint <<'\n';
	}

	void Flag_line_callback(const std_msgs::UInt32 input_data)
	{
		if(input_data.data==0){
			flag_line=0;
	  	std::cout <<"sub_flag_line=0"<<'\n';
		}else if(input_data.data==1){
			flag_line=1;
	  	std::cout <<"sub_flag_line=1"<<'\n';
		}
	}

//コールバック関数============================================================
	void cloud_callback(const sensor_msgs::PointCloud2ConstPtr &cloud_msg)
	{
		std::cout << "tabletop_trash_start" << '\n';

		// 読み込む。＝＝＝＝＝＝＝＝＝＝
		pcl::PointCloud<PointT>::Ptr cloud_input(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_trimming(new pcl::PointCloud<PointT>);
		std::vector<int> index_all;
		pcl::fromROSMsg(*cloud_msg, *cloud_input);
		pcl::removeNaNFromPointCloud(*cloud_input,*cloud_trimming,index_all);


		//鼻と手首,指先の点群を切り取る======================================================================================================================
		pcl::PointCloud<PointT>::Ptr cloud_Nose(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_Wrist(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_Elbow(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_Fingertip (new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_Second_joint(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_Third_joint(new pcl::PointCloud<PointT>);
		if(z_Nose!=0.0){
			cloud_Nose=Pass_Through(cloud_trimming,x_Nose-0.02,x_Nose+0.02,y_Nose-0.02,y_Nose+0.02,z_Nose-0.02,z_Nose+0.02);
		}
		if(z_Wrist!=0.0){
			cloud_Wrist=Pass_Through(cloud_trimming,x_Wrist-0.02,x_Wrist+0.02,y_Wrist-0.02,y_Wrist+0.02,z_Wrist-0.02,z_Wrist+0.02);
		}
		if(z_Elbow!=0.0){
			cloud_Elbow=Pass_Through(cloud_trimming,x_Elbow-0.02,x_Elbow+0.02,y_Elbow-0.02,y_Elbow+0.02,z_Elbow-0.02,z_Elbow+0.02);
		}
		if(z_Fingertip!=0.0){
			cloud_Fingertip=Pass_Through(cloud_trimming,x_Fingertip-0.02,x_Fingertip+0.02,y_Fingertip-0.02,y_Fingertip+0.02,z_Fingertip-0.02,z_Fingertip+0.02);
		}
		if(z_Second_joint!=0.0){
			cloud_Second_joint=Pass_Through(cloud_trimming,x_Second_joint-0.02,x_Second_joint+0.02,y_Second_joint-0.02,y_Second_joint+0.02,z_Second_joint-0.02,z_Second_joint+0.02);
		}
		if(z_Third_joint!=0.0){
			cloud_Third_joint=Pass_Through(cloud_trimming,x_Third_joint-0.02,x_Third_joint+0.02,y_Third_joint-0.02,y_Third_joint+0.02,z_Third_joint-0.02,z_Third_joint+0.02);
		}

		//指先の検出
		//指差し直線の算出===========================================================================
		float x,y,z;
		float t;


		//指のなす角をもとめる
		double vec=0.0;
		double rad=0.0;
		double deg=0.0;
		double ax=x_Fingertip-x_Second_joint;
		double ay=y_Fingertip-y_Second_joint;
		double az=z_Fingertip-z_Second_joint;
		double bx=x_Third_joint-x_Second_joint;
		double by=y_Third_joint-y_Second_joint;
		double bz=z_Third_joint-z_Second_joint;

		vec=ax*bx+ay*by+az*bz/sqrt(((pow(ax,2.0)+pow(ay,2.0)+pow(az,2.0))*(pow(bx,2.0)+pow(by,2.0)+pow(bz,2.0))));
		rad=acos(vec);
		deg=rad*180.0/M_PI;
		std::cout << "degree=" <<deg<<'\n';

		//手首からどれだけ離れているか====================================
		double dis_Fingertip=DBL_MAX;
		double dis_Second_joint=DBL_MAX;
		dis_Fingertip=sqrt(pow(x_Fingertip-x_Wrist,2.0)+pow(y_Fingertip-y_Wrist,2.0)+pow(z_Fingertip-z_Wrist,2.0));
		dis_Second_joint=sqrt(pow(x_Second_joint-x_Wrist,2.0)+pow(y_Second_joint-y_Wrist,2.0)+pow(z_Second_joint-z_Wrist,2.0));
		std::cout << "dis_Fingertip" <<dis_Fingertip<< '\n';
		std::cout << "dis_Second_joint" <<dis_Second_joint<< '\n';

	if(flag_line==1){
		if(dis_Fingertip<=0.2&&dis_Second_joint<=0.2&&z_Nose!=0.0){
			if(150.0<=deg&&deg<=180.0){
				std::cout << "Strate_finger" << '\n';
			 cloud_line.width = 1000;
       cloud_line.height = 1;
       cloud_line.points.resize(cloud_line.width * cloud_line.height);

			for(int t=0;t<cloud_line.points.size();t++){      //指が伸び切っている時
				cloud_line.points[t].x=x_Nose+(x_Fingertip-x_Nose)*t/100;
				cloud_line.points[t].y=y_Nose+(y_Fingertip-y_Nose)*t/100;
				cloud_line.points[t].z=z_Nose+(z_Fingertip-z_Nose)*t/100;
			}
		}else
		if(0.0<=deg&&deg<=150.0){
			std::cout << "Little_strate_finger" << '\n';
			cloud_line.width = 1000;
			cloud_line.height = 1;
			cloud_line.points.resize(cloud_line.width * cloud_line.height);
			for(int t=0;t<cloud_line.points.size();t++){     //指が曲がっている時
				cloud_line.points[t].x=x_Nose+(x_Second_joint-x_Nose)*t/100;
				cloud_line.points[t].y=y_Nose+(y_Second_joint-y_Nose)*t/100;
				cloud_line.points[t].z=z_Nose+(z_Second_joint-z_Nose)*t/100;
			}
			}else{ //指差しを正確にしていない時
				std::cout << "Cannot_estimate_line" << '\n';
				return;
			}
		}else{
			std::cout << "Cannot_estimate_line" << '\n';
			return;
		}
	}
		pcl::PointCloud<PointT>::Ptr cloud_dwnsmp(new pcl::PointCloud<PointT>);
    //ダウンサンプリングを行う
		cloud_dwnsmp = Down_Sampling(cloud_trimming);

		pcl::PointCloud<PointT>::Ptr cloud_rest(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_plane(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_table_rest(new pcl::PointCloud<PointT>);
		pcl::PointCloud<PointT>::Ptr cloud_table_plane(new pcl::PointCloud<PointT>);

		if(flag_line==0){
			//卓上認識========================
			// 床平面を認識する。================
			pcl::SACSegmentation<PointT> seg;
			pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
			pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			seg.setOptimizeCoefficients(true);
			seg.setModelType(pcl::SACMODEL_PLANE);
			seg.setMethodType(pcl::SAC_RANSAC);
			seg.setMaxIterations (100);  //試行回数
			seg.setDistanceThreshold(threshold_plane);//閾値設定
			seg.setInputCloud(cloud_dwnsmp);
			seg.segment(*inliers, *coefficients);

			// 床平面を除去する==================
			pcl::ExtractIndices<PointT> extract;
			extract.setInputCloud(cloud_dwnsmp);//入力データ
			extract.setIndices(inliers);//インデクッスの入力
			extract.setNegative(false);//平面以外を除去
			extract.filter(*cloud_plane);
			extract.setNegative(true);//平面を除去
			extract.filter(*cloud_rest);

			// 机平面を認識する。===================================================================================
			pcl::SACSegmentation<PointT> seg2;
			pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);
			pcl::ModelCoefficients::Ptr coefficients2 (new pcl::ModelCoefficients);
			seg2.setOptimizeCoefficients(true);
			seg2.setModelType(pcl::SACMODEL_PLANE);
			seg2.setMethodType(pcl::SAC_RANSAC);
			seg2.setMaxIterations (100);  //試行回数
			seg2.setDistanceThreshold(threshold_plane);//閾値設定
			seg2.setInputCloud(cloud_rest);
			seg2.segment(*inliers2, *coefficients2);

			// 机平面を除去する==================
			pcl::ExtractIndices<PointT> extract2;
			extract2.setInputCloud(cloud_rest);//入力データ
			extract2.setIndices(inliers2);//インデクッスの入力
			extract2.setNegative(false);//平面以外を除去
			extract2.filter(*cloud_table_plane);
			extract2.setNegative(true);//平面を除去
			extract2.filter(*cloud_table_rest);

			//机平面にtfをはる=======================================================================
			geometry_msgs::PointStamped msgs_plane_ave_point;
			Eigen::Vector4f plane_ave_point = ave_point_of_PointCloud(cloud_table_plane,pub_plane_ave_point,msgs_plane_ave_point);
			tf2::Transform transform_plane;
			tf2::Vector3 plane_axis;//ベクトルの生成
			tf2::Vector3 z_axis(0, 0, 1);//z軸
			float a_plane = coefficients2->values[0];//平面のクラスタの点群の情報
			float b_plane = coefficients2->values[1];
			float c_plane = coefficients2->values[2];
			if(a_plane <= 0)//床平面の軸が上側にあるとき
			{
				plane_axis.setValue(-1*a_plane, -1*b_plane, -1*c_plane);
			}
			else//床平面の軸が上側にあるとき
			{
				plane_axis.setValue(a_plane, b_plane, c_plane);
			}
		//	plane_axis.setValue(a_plane, b_plane, c_plane);
			plane_axis = plane_axis.normalized(); //ベクトルの正規化（ベクトルの方向を維持しつつ長さを１にする）

			double plane_angle = plane_axis.angle(z_axis);
			//回転軸
			tf2::Vector3 plane2_axis(z_axis.y()*plane_axis.z() - z_axis.z()*plane_axis.y(),z_axis.z()*plane_axis.x() - z_axis.x()*plane_axis.z() , z_axis.x()*plane_axis.y() - z_axis.y()*plane_axis.x());
			//外積：法線だす
			plane2_axis = plane2_axis.normalized();

			transform_plane.setOrigin( tf2::Vector3(plane_ave_point.x(), plane_ave_point.y(), plane_ave_point.z()) );//位置
			transform_plane.setRotation( tf2::Quaternion(plane2_axis, plane_angle) );//姿勢
			geometry_msgs::TransformStamped plane_transformStamped;
			plane_transformStamped.header.stamp = ros::Time::now();
			plane_transformStamped.header.frame_id = "head_rgbd_sensor_rgb_frame";
			plane_transformStamped.child_frame_id = "apu/plane";

			plane_transformStamped.transform.translation.x = transform_plane.getOrigin().x();
			plane_transformStamped.transform.translation.y = transform_plane.getOrigin().y();
			plane_transformStamped.transform.translation.z = transform_plane.getOrigin().z();

			plane_transformStamped.transform.rotation.x = transform_plane.getRotation().x();
			plane_transformStamped.transform.rotation.y = transform_plane.getRotation().y();
			plane_transformStamped.transform.rotation.z = transform_plane.getRotation().z();
			plane_transformStamped.transform.rotation.w = transform_plane.getRotation().w();
			br_plane.sendTransform(plane_transformStamped);
		}


		pcl::PointCloud<PointT>::Ptr cloud_transform_line(new pcl::PointCloud<PointT>);
		sensor_msgs::PointCloud2 cloud_msg_line;
		sensor_msgs::PointCloud2 conv_cloud_msg;
		pcl::PointCloud<PointT>::Ptr cloud_input_line(new pcl::PointCloud<PointT>);
		std::vector<int> index_all_line;
		pcl::toROSMsg(cloud_line, cloud_msg_line);
		cloud_msg_line.header.frame_id = "head_rgbd_sensor_rgb_frame";

		if(cloud_line.size()>0){
		//姿勢をmapに合わせる===================================================================
		//map→rgbd_camera
	   if(flag_line==1){
			tf2_ros::TransformListener tf_listener(tfBuffer);
			geometry_msgs::TransformStamped transform_line_after;
			if(tfBuffer.canTransform("map", "head_rgbd_sensor_rgb_frame", ros::Time::now(), ros::Duration(10.0))){//tfにおけるwaitForTransformの代わり,変換できていれば１を返す
				try{
				 transform_line_after=tfBuffer.lookupTransform("map", "head_rgbd_sensor_rgb_frame", ros::Time(0));
					}
				catch (tf2::TransformException& ex){
						ROS_ERROR("TF Exception: %s", ex.what());
						ros::Duration(1.0).sleep();
					}
				}

			//座標変換
			Eigen::Matrix4f mat = tf2::transformToEigen(transform_line_after.transform).matrix().cast<float>();
			pcl_ros::transformPointCloud(mat,cloud_msg_line, conv_cloud_msg);
			// std::cout << "Transform pointcloud" << '\n';
			// std::cout << " mat" <<  mat<<'\n';
			// std::cout << "transform_line_after" <<transform_line_after<<'\n';

			//msg→pointcloud
			pcl::fromROSMsg(conv_cloud_msg, *cloud_input_line);
			pcl::removeNaNFromPointCloud(*cloud_input_line,cloud_trimming_line,index_all_line);
		}else{
			std::cout << "Complite line !" << '\n';
		}
	 }

	 //estimate position prepare==================================================================================
	 //cloud_table_plane
	 pcl::PointCloud<PointT>::Ptr cloud_transform_table_plane(new pcl::PointCloud<PointT>);
	 sensor_msgs::PointCloud2 cloud_msg_table_plane;
	 sensor_msgs::PointCloud2 conv2_cloud_msg;
	 pcl::PointCloud<PointT>::Ptr cloud_input_table_plane(new pcl::PointCloud<PointT>);
	 std::vector<int> index_all_table_plane;
	 pcl::PointCloud<PointT>::Ptr cloud_trimming_table_plane(new pcl::PointCloud<PointT>);
	 pcl::toROSMsg(*cloud_table_plane, cloud_msg_table_plane);
	 cloud_msg_table_plane.header.frame_id = "head_rgbd_sensor_rgb_frame";

	 //cloud_table_rest
	 pcl::PointCloud<PointT>::Ptr cloud_transform_rest(new pcl::PointCloud<PointT>);
	 sensor_msgs::PointCloud2 cloud_msg_rest;
	 sensor_msgs::PointCloud2 conv3_cloud_msg;
	 pcl::PointCloud<PointT>::Ptr cloud_input_rest(new pcl::PointCloud<PointT>);
	 std::vector<int> index_all_rest;
	 pcl::PointCloud<PointT>::Ptr cloud_trimming_table_rest(new pcl::PointCloud<PointT>);
	 pcl::toROSMsg(*cloud_table_rest, cloud_msg_rest);
	 cloud_msg_rest.header.frame_id = "head_rgbd_sensor_rgb_frame";

	 Eigen::Vector4f centroid_point_table_plane;
	 Eigen::Vector4f estimation_point;

	 //クラスタの重心用
	 Eigen::Vector4f centroid_point_cluster_0;
	 Eigen::Vector4f centroid_point_cluster_1;
	 Eigen::Vector4f centroid_point_cluster_2;
	 Eigen::Vector4f centroid_point_cluster_3;

	 std::vector<double> dis_cluster;

	 //クラスタ格納用の点群
	 pcl::PointCloud<PointT>::Ptr cloud_cluster_0 (new pcl::PointCloud<PointT>);
	 pcl::PointCloud<PointT>::Ptr cloud_cluster_1 (new pcl::PointCloud<PointT>);
	 pcl::PointCloud<PointT>::Ptr cloud_cluster_2 (new pcl::PointCloud<PointT>);
	 pcl::PointCloud<PointT>::Ptr cloud_cluster_3 (new pcl::PointCloud<PointT>);
	 //結果格納用
	 pcl::PointCloud<PointT>::Ptr cloud_cluster_best (new pcl::PointCloud<PointT>);


	  //ゴミ特定==================================================================================
	 if(cloud_table_plane->size()>0){
		 //map基準に変換
		 tf2_ros::TransformListener tf_listener(tfBuffer);
			geometry_msgs::TransformStamped transform_table_plane;
			if(tfBuffer.canTransform("map", "head_rgbd_sensor_rgb_frame", ros::Time::now(), ros::Duration(10.0))){//tfにおけるwaitForTransformの代わり,変換できていれば１を返す
				try{
				 transform_table_plane=tfBuffer.lookupTransform("map", "head_rgbd_sensor_rgb_frame", ros::Time(0));
					}
				catch (tf2::TransformException& ex){
						ROS_ERROR("TF Exception: %s", ex.what());
						ros::Duration(1.0).sleep();
					}
				}
			//Transform_plane
			Eigen::Matrix4f mat_table_plane = tf2::transformToEigen(transform_table_plane.transform).matrix().cast<float>();
			pcl_ros::transformPointCloud(mat_table_plane,cloud_msg_table_plane, conv2_cloud_msg);
			pcl::fromROSMsg(conv2_cloud_msg, *cloud_input_table_plane);
			pcl::removeNaNFromPointCloud(*cloud_input_table_plane,*cloud_trimming_table_plane,index_all_table_plane);

			//Transform_rest
			Eigen::Matrix4f mat_rest = tf2::transformToEigen(transform_table_plane.transform).matrix().cast<float>();
			pcl_ros::transformPointCloud(mat_rest,cloud_msg_rest, conv3_cloud_msg);
			pcl::fromROSMsg(conv3_cloud_msg, *cloud_input_rest);
			pcl::removeNaNFromPointCloud(*cloud_input_rest,*cloud_trimming_table_rest,index_all_rest);


			pcl::compute3DCentroid(*cloud_trimming_table_plane, centroid_point_table_plane);//重心を計算
			//机平面と指差しとの交点を算出
			for(int i=0;i<cloud_trimming_line.size();i++){
				if(cloud_trimming_line.points[i].z >=centroid_point_table_plane.z()-0.005&&cloud_trimming_line.points[i].z <=centroid_point_table_plane.z()+0.005){
					estimation_point.x()=cloud_trimming_line.points[i].x;
					estimation_point.y()=cloud_trimming_line.points[i].y;
					estimation_point.z()=cloud_trimming_line.points[i].z;
				}
			}
			std::cout << "estimation_point.x()" << estimation_point.x()<<'\n';
			std::cout << "estimation_point.y()" << estimation_point.y()<<'\n';
			std::cout << "estimation_point.z()" << estimation_point.z()<<'\n';

			//推定位置をもとにゴミであるクラスタの認識
			// クラスタリング
			pcl::search::KdTree<PointT>::Ptr cluster_tree (new pcl::search::KdTree<PointT>);//KdTree構造をつくる
			cluster_tree->setInputCloud(cloud_trimming_table_rest);
			std::vector<pcl::PointIndices> cluster_indices;
			pcl::EuclideanClusterExtraction<PointT> ec;
			ec.setClusterTolerance (0.03);//探索する半径の設定
			ec.setMinClusterSize (70);//最小点の数を設定
			ec.setMaxClusterSize (600);//最大の点の数を設定
			ec.setSearchMethod (cluster_tree);//検索に使用する手法を指定
			ec.setInputCloud (cloud_trimming_table_rest);//点群を入力
			ec.extract (cluster_indices);//クラスター情報を出力

			int cc[4] = {10000,10000,10000,10000};//格納する配列の宣言
			int j = 0;//cc[]を認識する指標
			for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it != cluster_indices.end(); ++it){
				pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
				for(std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++){
					cloud_cluster->points.push_back(cloud_trimming_table_rest->points[*pit]); //ダウンサンプリングした結果を入れていく
				}
				//ダウンサンプリング・クラスタリングしたデータから点群の作成
				cloud_cluster->width = cloud_cluster->points.size();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;//PCLの関数内でのInfやNaNのチェックをスキップする
				//クラスタのサイズが大きい順にcc[]に格納されている
	      //クラスタ作成
				if(j == 0){					//一番大きいクラスタが格納される
					cc[0] = cloud_cluster->points.size();
					pcl::copyPointCloud(*cloud_cluster,*cloud_cluster_0);

				}
				else if(j == 1){
					cc[1] = cloud_cluster->points.size();
					pcl::copyPointCloud(*cloud_cluster,*cloud_cluster_1);

				}

				else if(j == 2){
					cc[2] = cloud_cluster->points.size();
					pcl::copyPointCloud(*cloud_cluster,*cloud_cluster_2);

				}
				else if(j == 3){
					cc[3] = cloud_cluster->points.size();
					pcl::copyPointCloud(*cloud_cluster,*cloud_cluster_3);
				}

				j++;
			}
			//クラスタの重心を計算,ｘ−ｙ平面での距離計算
			if(cloud_cluster_0->points.size()>0){
				pcl::compute3DCentroid(*cloud_cluster_0, centroid_point_cluster_0);
				dis_cluster.push_back(sqrt(pow(centroid_point_cluster_0.x()-estimation_point.x(),2.0)+pow(centroid_point_cluster_0.y()-estimation_point.y(),2.0)));
			}
			if(cloud_cluster_1->points.size()>0){
				pcl::compute3DCentroid(*cloud_cluster_1, centroid_point_cluster_1);
				dis_cluster.push_back(sqrt(pow(centroid_point_cluster_1.x()-estimation_point.x(),2.0)+pow(centroid_point_cluster_1.y()-estimation_point.y(),2.0)));
			}
			if(cloud_cluster_2->points.size()>0){
				pcl::compute3DCentroid(*cloud_cluster_2, centroid_point_cluster_2);
				dis_cluster.push_back(sqrt(pow(centroid_point_cluster_2.x()-estimation_point.x(),2.0)+pow(centroid_point_cluster_2.y()-estimation_point.y(),2.0)));
			}
			if(cloud_cluster_3->points.size()>0){
				pcl::compute3DCentroid(*cloud_cluster_3, centroid_point_cluster_3);
				dis_cluster.push_back(sqrt(pow(centroid_point_cluster_3.x()-estimation_point.x(),2.0)+pow(centroid_point_cluster_3.y()-estimation_point.y(),2.0)));
			}

			//最短距離にあるクラスタを指差し物体とする
			double min_dis=std::numeric_limits<double>::max();
			int min_number=0;
			for(int i=0;i<dis_cluster.size();i++){
				if(dis_cluster[i]<min_dis){
					min_dis=dis_cluster[i];
					min_number=i;
				}
			}
			switch (min_number)
			{
				case 0:
					pcl::copyPointCloud(*cloud_cluster_0,*cloud_cluster_best);
					std::cout << "best cluster is 0"<<'\n';
					break;
				case 1:
					pcl::copyPointCloud(*cloud_cluster_1,*cloud_cluster_best);
					std::cout << "best cluster is 1"<<'\n';
					break;
				case 2:
					pcl::copyPointCloud(*cloud_cluster_2,*cloud_cluster_best);
					std::cout << "best cluster is 2"<<'\n';
					break;
				case 3:
					pcl::copyPointCloud(*cloud_cluster_3,*cloud_cluster_best);
					std::cout << "best cluster is 3"<<'\n';
					break;
				default:
					break;
			}
		}


		// 出力================================================================
		if(cloud_Nose->size() > 0)
		{
			sensor_msgs::PointCloud2 msg_Nose;
			Output_pub(cloud_Nose,msg_Nose,pub_Nose);
		}
		else
		{
			ROS_INFO("Size (cloud_Nose)");
		}
		if(cloud_Wrist->size() > 0)
		{
			sensor_msgs::PointCloud2 msg_Wrist;
			Output_pub(cloud_Wrist,msg_Wrist,pub_Wrist);
		}
		else
		{
			ROS_INFO("Size (cloud_Wrist)");
		}
		// if(cloud_line.size() > 0)
		// {
		// 	sensor_msgs::PointCloud2 msg_line;
		// 	Output_pub(*cloud_line,msg_line,pub_line);
		// }
		// else
		// {
		// 	ROS_INFO("Size (cloud_line)");
		// }
		if(cloud_Fingertip->size() > 0)
		{
			sensor_msgs::PointCloud2 msg_Fingertip;
			Output_pub(cloud_Fingertip,msg_Fingertip,pub_Fingertip);
		}
		else
		{
			ROS_INFO("Size (cloud_Fingertip)");
		}
		if(cloud_Second_joint->size() > 0)
		{
			sensor_msgs::PointCloud2 msg_Second_joint;
			Output_pub(cloud_Second_joint,msg_Second_joint,pub_Second_joint);
		}
		else
		{
			ROS_INFO("Size (cloud_Second_joint)");
		}
		if(cloud_Third_joint->size() > 0)
		{
			sensor_msgs::PointCloud2 msg_Third_joint;
			Output_pub(cloud_Third_joint,msg_Third_joint,pub_Third_joint);
		}
		else
		{
			ROS_INFO("Size (cloud_Third_joint)");
		}
		if(cloud_plane->size() > 0)
		{
			sensor_msgs::PointCloud2 msg_plane;
			Output_pub(cloud_plane,msg_plane,pub_plane);
		}
		else
		{
			ROS_INFO("Size (cloud_plane)");
		}
		if(cloud_table_plane->size() > 0)
		{
			sensor_msgs::PointCloud2 msg_table_plane;
			Output_pub(cloud_table_plane,msg_table_plane,pub_table_plane);
		}
		else
		{
			ROS_INFO("Size (cloud_table_plane)");
		}
		if(cloud_trimming_line.size() > 0)
		{
			sensor_msgs::PointCloud2 msg_trimming_line;
			Output_pub_map(cloud_trimming_line,msg_trimming_line,pub_trimming_line);
		}
		else
		{
			ROS_INFO("Size (cloud_trimming_line)");
		}
		if(cloud_cluster_best->size() > 0)
		{
			sensor_msgs::PointCloud2 msg_cluster_best;
			Output_pub_MAP(cloud_cluster_best,msg_cluster_best,pub_cluster_best);
		}
		else
		{
			ROS_INFO("Size (cloud_cluster_best)");
		}

		std::cout << "tabletop_trash_end" << '\n';
		return;
	}
};

int main(int argc, char** argv)
{
	ros::init (argc, argv, "tabletop_trash");

	tabletop_trash tabletop_trash;

	ros::Rate spin_rate(1);
	while(ros::ok())
	{
		ros::spinOnce();
		spin_rate.sleep();
	}

	return 0;
}
