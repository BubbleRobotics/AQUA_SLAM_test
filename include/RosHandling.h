//
// Created by da on 24/03/2021.
//

#ifndef ROSHANDLING_H
#define ROSHANDLING_H

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <cv_bridge/cv_bridge.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <message_filters/subscriber.hpp>
#include <message_filters/synchronizer.hpp>
#include <message_filters/sync_policies/exact_time.hpp>
#include <message_filters/sync_policies/approximate_time.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.hpp>

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/core/eigen.hpp>

#include <boost/shared_ptr.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <mutex>
#include <thread>

#include "KeyFrame.h"

using namespace std;

namespace ORB_SLAM3
{
class Atlas;
class System;
class LocalMapping;

class RosHandling
{
public:
	RosHandling(System *pSys, LocalMapping *pLocal, rclcpp::Node::SharedPtr pMainNode);
	void PublishLeftImg(const sensor_msgs::msg::Image &img);
	void PublishRightImg(const sensor_msgs::msg::Image &img);
	void PublishImgWithInfo(const sensor_msgs::msg::Image::SharedPtr img);
	void PublishImgMergeCandidate(const cv::Mat &img);
	void PublishIntegration(Atlas *pAtlas);
    void PublishLossKF(set<KeyFrame*,KFComparator> &loss_kfs);
	// void PublishGT(const Eigen::Isometry3d &T_g0_gj_gt, const ros::Time &stamp);
	void PublishOrb(const Eigen::Isometry3d &T_c0_cj_orb, const Eigen::Isometry3d &T_d_c, const rclcpp::Time &stamp);
	void PublishCamera(const Eigen::Isometry3d &T_c0_cj_orb, const rclcpp::Time &stamp);
	// void PublishEkf(const Eigen::Isometry3d &T_e0_ej_ekf, const ros::Time &stamp);
	// void PublishDensePointCloudPose(const Eigen::Isometry3d &T_c0_cmj, const ros::Time &stamp);
	//publish pointcloud and octomap
	void UpdateMap(ORB_SLAM3::Atlas *pAtlas);
	void PublishMap(ORB_SLAM3::Atlas *pAtlas, int state);
    void Run(Atlas* pAtlas);
	void BroadcastTF(const Eigen::Isometry3d &T_c0_cj_orb,
	                 const rclcpp::Time &stamp,
	                 const string &id,
	                 const string &child_id);
    // publish reference integration path
	void GenerateFreePointcloud(const pcl::PointXYZRGB &start,
	                            const pcl::PointXYZRGB &end,
	                            float resolution,
	                            pcl::PointCloud<pcl::PointXYZRGB> &cloud_free);
	double LinearInterpolation(double start_x, double end_x, double start_y, double end_y, double x);
	void PublishLossInteration(const Eigen::Isometry3d &T_e0_er, const Eigen::Isometry3d &T_e0_ec);
	bool SavePose(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                  std::shared_ptr<std_srvs::srv::Empty::Response> res);
	bool LoadMap(const std::shared_ptr<std_srvs::srv::Empty::Request> req, 
				 std::shared_ptr<std_srvs::srv::Empty::Response> res);
	bool CalibrateDVLGyro(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
              			  std::shared_ptr<std_srvs::srv::Empty::Response> res);
    bool FullBA(const std::shared_ptr<std_srvs::srv::Empty::Request> req,
                std::shared_ptr<std_srvs::srv::Empty::Response> res);

protected:
	System *mp_system;
	LocalMapping *mp_LocalMapping;

	image_transport::Publisher mp_img_l_pub;
	image_transport::Publisher mp_img_r_pub;
	image_transport::Publisher mp_img_info_pub;
	image_transport::Publisher mp_img_merge_cond_pub;

	//publish qualisys path
	nav_msgs::msg::Path m_integration_path;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mp_integration_path_pub;;
    // publish reference integration path
    nav_msgs::msg::Path m_ref_integration_path;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mp_ref_integration_path_pub;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr mp_markers_pub;
	//publish qualisys path
	nav_msgs::msg::Path m_gt_path;

	//publish orb pose, odometry and path, in camera frame
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mp_pose_orb_pub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mp_odom_orb_pub;
	nav_msgs::msg::Path m_path_orb;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mp_path_orb_pub;
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr mp_pose_orb_camera_pub;

	//publish ekf pose and path, in EKF frame
	//doesnt seem to ever publish 
	nav_msgs::msg::Path m_path_ekf;
	rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr mp_path_ekf_pub;

	//publish point cloud
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr mp_pointcloud_pub;
	rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr mp_octomap_pub;

	std::mutex m_mutex_map;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mp_cloud_occupied;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mp_cloud_free;
	float m_octomap_resolution;
	boost::shared_ptr<octomap::OcTree> mp_octree;

	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mp_pose_integration_ref_pub;
	rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr mp_pose_integration_cur_pub;

	rclcpp::Service<std_srvs::srv::Empty>::SharedPtr mp_save_srv, m_load_srv, m_calib_srv, m_fullBA_srv;

    // gravity dir of current map
    Eigen::Isometry3d mT_w_c0;

public:
	void setLocalMapping(LocalMapping *mpLocalMapping)
	{
		mp_LocalMapping = mpLocalMapping;
	}

	// Spinning node from main funciton. Needed for ros publishing
	rclcpp::Node::SharedPtr mpMainNode;

	// To avoid having to create a new publisher every time RosHandling::BroadcastTF is called
	std::shared_ptr<tf2_ros::TransformBroadcaster> mp_tf_broadcaster;

protected:


	//	todo unfinished

	//publish orb pose, in orb frame
	// boost::shared_ptr<ros::Publisher> mp_pose_pointcloud_pub;  // currently not used
	// call service to send last N good keyframes once lost feature tracking
	rclcpp::Client<std_srvs::srv::Empty>::SharedPtr mp_lost_srv;

};

}

#endif //ROSHANDLING_H
