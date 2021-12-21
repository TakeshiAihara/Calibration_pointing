#include <ros/ros.h>
#include <time.h>
#include <pcl/common/transforms.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_listener.h>

class Tf_publisher_cluster {
private:
	ros::Subscriber sub_Nose_fingertip;

public:
	Tf_publisher_cluster()
	{
		ros::NodeHandle nh("~");

		sub_Nose_fingertip = nh.subscribe("/calibration_pointing_node/Nose_fingertip_pose", 1, &Tf_publisher_cluster::Nose_fingertip_callback, this);

	}


	tf2_ros::TransformBroadcaster br_Nose_fingertip_pub;
  tf2::Transform transform_Nose_fingertip_pub;
	geometry_msgs::TransformStamped Nose_fingertip_transformStamped;




	ros::Time pub_time_Nose_fingertip;

	tf2_ros::Buffer tfBuffer;


	void Nose_fingertip_callback(const geometry_msgs::Pose input_pose)
  {
		pub_time_Nose_fingertip = ros::Time::now();
		Nose_fingertip_transformStamped.header.stamp = ros::Time::now();
		Nose_fingertip_transformStamped.header.frame_id = "head_rgbd_sensor_rgb_frame";
		Nose_fingertip_transformStamped.child_frame_id = "Nose_fingertip";

		Nose_fingertip_transformStamped.transform.translation.x = input_pose.position.x;
		Nose_fingertip_transformStamped.transform.translation.y = input_pose.position.y;
		Nose_fingertip_transformStamped.transform.translation.z = input_pose.position.z;
		Nose_fingertip_transformStamped.transform.rotation.x = input_pose.orientation.x;
		Nose_fingertip_transformStamped.transform.rotation.y = input_pose.orientation.y;
		Nose_fingertip_transformStamped.transform.rotation.z = input_pose.orientation.z;
		Nose_fingertip_transformStamped.transform.rotation.w = input_pose.orientation.w;
		if(std::isnan(Nose_fingertip_transformStamped.transform.translation.x)==0){//nan判定
			br_Nose_fingertip_pub.sendTransform(Nose_fingertip_transformStamped);
	 }
  }
};


int main(int argc, char** argv)
{
	ros::init (argc, argv, "Tf_publisher_cluster");

Tf_publisher_cluster Tf_publisher_cluster;
	ros::Rate spin_rate_pub(1);
	while(ros::ok())
	{
		ros::spinOnce();
		spin_rate_pub.sleep();

}
	return 0;
}
