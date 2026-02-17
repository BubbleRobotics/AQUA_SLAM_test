//
// Created by da on 24/03/2021.
//

#ifndef ROSHANDLING_H
#define ROSHANDLING_H

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Transform.h>
#include <geometry_msgs/TransformStamped.h>
#include <nav_msgs/Odometry.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Path.h>
#include <tf/transform_broadcaster.h>
#include <octomap/OcTree.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap/ColorOcTree.h>
#include <octomap_msgs/conversions.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>
#include<opencv2/core/eigen.hpp>

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
	RosHandling(System *pSys, LocalMapping *pLocal);
	void PublishLeftImg(const sensor_msgs::ImageConstPtr &img);
	void PublishRightImg(const sensor_msgs::ImageConstPtr &img);
	void PublishImgWithInfo(const sensor_msgs::ImageConstPtr &img);
	void PublishImgMergeCandidate(const cv::Mat &img);
	void PublishIntegration(Atlas *pAtlas);
    void PublishLossKF(set<KeyFrame*,KFComparator> &loss_kfs);
	// void PublishGT(const Eigen::Isometry3d &T_g0_gj_gt, const ros::Time &stamp);
	void PublishOrb(const Eigen::Isometry3d &T_c0_cj_orb, const Eigen::Isometry3d &T_d_c, const ros::Time &stamp);
	void PublishCamera(const Eigen::Isometry3d &T_c0_cj_orb, const ros::Time &stamp);
	// void PublishEkf(const Eigen::Isometry3d &T_e0_ej_ekf, const ros::Time &stamp);
	// void PublishDensePointCloudPose(const Eigen::Isometry3d &T_c0_cmj, const ros::Time &stamp);
	//publish pointcloud and octomap
	void UpdateMap(ORB_SLAM3::Atlas *pAtlas);
	void PublishMap(ORB_SLAM3::Atlas *pAtlas, int state);
    void Run(Atlas* pAtlas);
	void BroadcastTF(const Eigen::Isometry3d &T_c0_cj_orb,
	                 const ros::Time &stamp,
	                 const string &id,
	                 const string &child_id);
    // publish reference integration path
	void GenerateFreePointcloud(const pcl::PointXYZRGB &start,
	                            const pcl::PointXYZRGB &end,
	                            float resolution,
	                            pcl::PointCloud<pcl::PointXYZRGB> &cloud_free);
	double LinearInterpolation(double start_x, double end_x, double start_y, double end_y, double x);
	void PublishLossInteration(const Eigen::Isometry3d &T_e0_er, const Eigen::Isometry3d &T_e0_ec);
	bool SavePose(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
	bool LoadMap(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
	bool CalibrateDVLGyro(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
    bool FullBA(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);
	bool CalibrationBA(std_srvs::EmptyRequest &req, std_srvs::EmptyResponse &res);  // For extrinsic param calibration

protected:
	System *mp_system;
	LocalMapping *mp_LocalMapping;

	boost::shared_ptr<image_transport::ImageTransport> mp_it;
	boost::shared_ptr<image_transport::Publisher> mp_img_l_pub;
	boost::shared_ptr<image_transport::Publisher> mp_img_r_pub;
	boost::shared_ptr<image_transport::Publisher> mp_img_info_pub;
	boost::shared_ptr<image_transport::Publisher> mp_img_merge_cond_pub;

	//publish qualisys path
	nav_msgs::Path m_integration_path;
	boost::shared_ptr<ros::Publisher> mp_integration_path_pub;
    // publish reference integration path
    nav_msgs::Path m_ref_integration_path;
    boost::shared_ptr<ros::Publisher> mp_ref_integration_path_pub;
    boost::shared_ptr<ros::Publisher> mp_markers_pub;
	//publish qulisys pose(if exist),
	boost::shared_ptr<ros::Publisher> mp_gt_pub;
	//publish qualisys path
	nav_msgs::Path m_gt_path;
	boost::shared_ptr<ros::Publisher> mp_gt_path_pub;

	//publish orb pose, odometry and path, in camera frame
	boost::shared_ptr<ros::Publisher> mp_pose_orb_pub;
	boost::shared_ptr<ros::Publisher> mp_odom_orb_pub;
	nav_msgs::Path m_path_orb;
	boost::shared_ptr<ros::Publisher> mp_path_orb_pub;
	boost::shared_ptr<ros::Publisher> mp_pose_orb_camera_pub;

	//publish ekf pose and path, in EKF frame
	boost::shared_ptr<ros::Publisher> mp_pose_ekf_pub;
	nav_msgs::Path m_path_ekf;
	boost::shared_ptr<ros::Publisher> mp_path_ekf_pub;

	//publish point cloud
	boost::shared_ptr<ros::Publisher> mp_pointcloud_pub;
	boost::shared_ptr<ros::Publisher> mp_octomap_pub;
	boost::shared_ptr<ros::Publisher> mp_map_info_pub;

	std::mutex m_mutex_map;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mp_cloud_occupied;
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr mp_cloud_free;
	float m_octomap_resolution;
	boost::shared_ptr<octomap::OcTree> mp_octree;

	boost::shared_ptr<ros::Publisher> mp_pose_integration_ref_pub;
	boost::shared_ptr<ros::Publisher> mp_pose_integration_cur_pub;

	boost::shared_ptr<ros::ServiceServer> mp_save_srv, m_load_srv, m_calib_srv, m_fullBA_srv, m_calibrationBA_srv;

    // gravity dir of current map
    Eigen::Isometry3d mT_w_c0;

public:
	void setLocalMapping(LocalMapping *mpLocalMapping)
	{
		mp_LocalMapping = mpLocalMapping;
	}
protected:


	//	todo unfinished

	//publish orb pose, in orb frame
	boost::shared_ptr<ros::Publisher> mp_pose_pointcloud_pub;
	// call service to send last N good keyframes once lost feature tracking
	boost::shared_ptr<ros::ServiceClient> mp_lost_srv;
	tf::TransformBroadcaster m_tb;

};
}

#endif //ROSHANDLING_H
