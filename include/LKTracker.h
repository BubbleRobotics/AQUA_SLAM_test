#ifndef LKTRACKER_H
#define LKTRACKER_H

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.hpp>

// C++
#include <cstdio>
#include <iostream>
#include <queue>
#include <csignal>
// #include <vector>

// OpenCV
#include <opencv2/opencv.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

// STL
#include <map>
#include <vector>

// ORB-SLAM3 forward declarations
namespace ORB_SLAM3
{

class MapPoint;
class Frame;
class FrameKLT;
class KeyFrame;

class LKTracker
{
public:

    LKTracker(rclcpp::Node::SharedPtr pNode);
    LKTracker(bool bStereo, rclcpp::Node::SharedPtr pNode);
    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                    std::vector<int> &curLeftIds,
                    std::vector<cv::Point2f> &curLeftPts,
                    std::vector<cv::Point2f> &curRightPts,
                    std::map<int, cv::Point2f> &prevLeftPtsMap);
    std::map<int, std::vector<std::pair<int, Eigen::Matrix<double,7,1>>>>
    trackImage(double _cur_time,
               const cv::Mat &_img,
               const cv::Mat &_img1);

    bool trackFrame(Frame &cur_frame, const Frame &prev_frame);
    bool TrackReferenceKeyFrameKLT(KeyFrame *cur_frame, const Frame &prev_frame);

    bool InitializeFrame(Frame &cur_f);

    void drawTrackFrame(Frame &cur_frame,
                        const Frame &prev_frame,
                        cv::Mat &result);

    void DetectORBPoints(FrameKLT &f);
    void DetectFeature(FrameKLT &f, int max_num);
    void ComputeFeatureDescriptor(FrameKLT &f, int max_num);

    void ssc(std::vector<cv::KeyPoint> keyPoints,
             int numRetPoints,
             float tolerance,
             int cols,
             int rows,
             std::vector<cv::KeyPoint> &out);

protected:

    std::vector<cv::Point2f> ptsVelocity(
        std::vector<int> &ids,
        std::vector<cv::Point2f> &pts,
        std::map<int, cv::Point2f> &cur_id_pts,
        std::map<int, cv::Point2f> &prev_id_pts);

    double distance(cv::Point2f &pt1, cv::Point2f &pt2);

    void setMask();

    bool inBorder(const cv::Point2f &pt, int col, int row);

    void reduceVector(std::vector<cv::Point2f> &v,
                      std::vector<uchar> status);

    void reduceVector(std::vector<int> &v,
                      std::vector<uchar> status);

    void cv2Eigen_3d(const cv::Mat &T,
                     Eigen::Isometry3d &T_eigen);

    void eigen2CV_3d(const Eigen::Isometry3d &T_eigen,
                     cv::Mat &T);

public:

    std::vector<int> ids, ids_right;

    std::vector<int> track_cnt;

    bool SHOW_TRACK = true;

    bool stereo_cam = true;

    const int MAX_CNT = 150;

    const int MIN_DIST = 30;

    int row, col;

    cv::Mat mask;

    cv::Mat imTrack;

    std::vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts, predict_pts, n_pts;

    bool FLOW_BACK = true;

    int n_id = 0;

    double cur_time, prev_time;

    cv::Mat prev_img, cur_img;

    std::vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;

    std::map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map, prevLeftPtsMap;

    std::map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;

    std::vector<cv::Point2f> pts_velocity, right_pts_velocity;

private:

    rclcpp::Node::SharedPtr mNode;  // Node handed on for logging etc.

    // Updated ros2 publishers
    image_transport::Publisher mTrack_img_pub;

};

}

#endif