#ifndef LKTRACKER_H
#define LKTRACKER_H

// c++
#include <cstdio>
#include <iostream>
#include <queue>
#include <execinfo.h>
#include <csignal>
#include <memory>

// opencv
#include <opencv2/opencv.hpp>

// ROS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <image_transport/image_transport.hpp>
#include <image_transport/subscriber_filter.hpp>
#include <message_filters/sync_policies/approximate_time.h>
#include <cv_bridge/cv_bridge.h>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>

using namespace std;

namespace ORB_SLAM3
{

class MapPoint;
class Frame;
class FrameKLT;
class KeyFrame;

class LKTracker
{
public:
    LKTracker();
    LKTracker(bool bStereo);

    void drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight,
                   vector<int> &curLeftIds,
                   vector<cv::Point2f> &curLeftPts,
                   vector<cv::Point2f> &curRightPts,
                   map<int, cv::Point2f> &prevLeftPtsMap);

    map<int, vector<pair<int, Eigen::Matrix<double, 7, 1>>>> trackImage(
        double _cur_time,
        const cv::Mat &_img,
        const cv::Mat &_img1);

    bool trackFrame(Frame &cur_frame, const Frame &prev_frame);
    bool TrackReferenceKeyFrameKLT(KeyFrame *cur_frame, const Frame &prev_frame);
    bool InitializeFrame(Frame &cur_f);
    void drawTrackFrame(Frame &cur_frame, const Frame &prev_frame, cv::Mat &result);
    void DetectORBPoints(FrameKLT &f);
    void DetectFeature(FrameKLT &f, int max_num);
    void ComputeFeatureDescriptor(FrameKLT &f, int max_num);

    void ssc(vector<cv::KeyPoint> keyPoints,
             int numRetPoints,
             float tolerance,
             int cols,
             int rows,
             vector<cv::KeyPoint> &out);

protected:

    vector<cv::Point2f> ptsVelocity(vector<int> &ids,
                                    vector<cv::Point2f> &pts,
                                    map<int, cv::Point2f> &cur_id_pts,
                                    map<int, cv::Point2f> &prev_id_pts);

    double distance(cv::Point2f &pt1, cv::Point2f &pt2);

    void setMask();
    bool inBorder(const cv::Point2f &pt, int col, int row);

    void reduceVector(vector<cv::Point2f> &v, vector<uchar> status);
    void reduceVector(vector<int> &v, vector<uchar> status);

    void cv2Eigen_3d(const cv::Mat &T, Eigen::Isometry3d &T_eigen);
    void eigen2CV_3d(const Eigen::Isometry3d &T_eigen, cv::Mat &T);

public:

    vector<int> ids, ids_right;
    vector<int> track_cnt;

    bool SHOW_TRACK = true;
    bool stereo_cam = true;

    const int MAX_CNT = 150;
    const int MIN_DIST = 30;

    int row, col;

    cv::Mat mask;
    cv::Mat imTrack;

    vector<cv::Point2f> prev_pts, cur_pts, cur_right_pts, predict_pts, n_pts;

    bool FLOW_BACK = true;
    int n_id = 0;

    double cur_time, prev_time;

    cv::Mat prev_img, cur_img;

    vector<cv::Point2f> prev_un_pts, cur_un_pts, cur_un_right_pts;

    map<int, cv::Point2f> cur_un_pts_map, prev_un_pts_map, prevLeftPtsMap;
    map<int, cv::Point2f> cur_un_right_pts_map, prev_un_right_pts_map;

    vector<cv::Point2f> pts_velocity, right_pts_velocity;

    // 🔁 ROS2 change: boost -> std
    std::shared_ptr<image_transport::Publisher> pTrack_img_pub;
};

} // namespace ORB_SLAM3

#endif // LKTRACKER_H