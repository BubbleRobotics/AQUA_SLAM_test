#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <vector>
#include <queue>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/image.hpp>  // raw images
#include <sensor_msgs/msg/compressed_image.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <std_msgs/msg/header.hpp>

#include <cv_bridge/cv_bridge.hpp>

#include <dvl_msgs/msg/dvl.hpp>

#include <opencv2/core/core.hpp>

#include <Eigen/Core>

#include <System.h>
#include <ImuTypes.h>

using namespace std;

/* ----------------------------- IMU Grabber ----------------------------- */

class ImuGrabber
{
public:

    std::queue<sensor_msgs::msg::Imu::SharedPtr> imuBuf;
    std::mutex mBufMutex;

    void GrabImu(const sensor_msgs::msg::Imu::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mBufMutex);
        imuBuf.push(msg);
    }
};


/* ----------------------------- DVL Grabber ----------------------------- */

class DVLGrabber
{
public:

    std::queue<dvl_msgs::msg::DVL::SharedPtr> dvlBuf2;
    std::mutex mBufMutex;

    void GrabDVL2(const dvl_msgs::msg::DVL::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mBufMutex);
        dvlBuf2.push(msg);
    }
};


/* ----------------------------- Image Grabber ----------------------------- */

class ImageGrabber
{
public:

    ImageGrabber(
        ORB_SLAM3::System* pSLAM,
        ImuGrabber* pImuGb,
        DVLGrabber* pDvlGb)
        : mpSLAM(pSLAM),
          mpImuGb(pImuGb),
          mpDvlGb(pDvlGb)
    {}

    std::queue<sensor_msgs::msg::CompressedImage::SharedPtr> imgLeftBuf, imgRightBuf;
    std::mutex mBufMutexLeft, mBufMutexRight;

    ORB_SLAM3::System* mpSLAM;
    ImuGrabber* mpImuGb;
    DVLGrabber* mpDvlGb;

    /* ----------------------------- Image callbacks ----------------------------- */

    void GrabImageLeft(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mBufMutexLeft);
        imgLeftBuf.push(msg);
    }

    void GrabImageRight(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(mBufMutexRight);
        imgRightBuf.push(msg);
    }

    /* ----------------------------- Convert ROS image to OpenCV ----------------------------- */

    cv::Mat GetImage(const sensor_msgs::msg::Image::SharedPtr img_msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr;

        try
        {
            cv_ptr = cv_bridge::toCvShare(img_msg, "bgr8");
        }
        catch (cv_bridge::Exception &e)
        {
            std::cerr << "cv_bridge exception: " << e.what() << std::endl;
        }

        return cv_ptr->image.clone();
    }


    /* ------------------------- Convert compressed image to OpenCV ------------------------- */

    cv::Mat GetCompressedImage(const sensor_msgs::msg::CompressedImage::SharedPtr img_msg)
    {
        try
        {
            // Convert ROS compressed image data to cv::Mat
            cv::Mat rawData(1, img_msg->data.size(), CV_8UC1, (void*)img_msg->data.data());
            cv::Mat decoded = cv::imdecode(rawData, cv::IMREAD_COLOR); // decode as BGR
            if (decoded.empty()) {
                std::cerr << "Failed to decode compressed image" << std::endl;
            }
            return decoded;
        }
        catch (const std::exception &e)
        {
            std::cerr << "Exception decoding compressed image: " << e.what() << std::endl;
            return cv::Mat();
        }
    }

    /* ----------------------------- Sync thread ----------------------------- */

    void SyncWithImu2()
    {

        const double maxTimeDiff = 0.1;

        while (rclcpp::ok())
        {

            if(imgLeftBuf.empty() || imgRightBuf.empty() || mpImuGb->imuBuf.empty())
                continue;

            double tImLeft =
                rclcpp::Time(imgLeftBuf.front()->header.stamp).seconds();

            double tImRight =
                rclcpp::Time(imgRightBuf.front()->header.stamp).seconds();


            /* ----- align timestamps ----- */

            {
                std::lock_guard<std::mutex> lock(mBufMutexRight);

                while((tImLeft - tImRight) > maxTimeDiff && imgRightBuf.size()>1)
                {
                    imgRightBuf.pop();
                    tImRight = rclcpp::Time(imgRightBuf.front()->header.stamp).seconds();
                }
            }

            {
                std::lock_guard<std::mutex> lock(mBufMutexLeft);

                while((tImRight - tImLeft) > maxTimeDiff && imgLeftBuf.size()>1)
                {
                    imgLeftBuf.pop();
                    tImLeft = rclcpp::Time(imgLeftBuf.front()->header.stamp).seconds();
                }
            }

            if (fabs(tImLeft - tImRight) > maxTimeDiff)
                continue;


            if (tImLeft > rclcpp::Time(mpImuGb->imuBuf.back()->header.stamp).seconds())
                continue;


            /* ----- get images, convert from compressed to OpenCV ----- */

            cv::Mat imLeft, imRight;

            {
                std::lock_guard<std::mutex> lock(mBufMutexLeft);
                imLeft = GetCompressedImage(imgLeftBuf.front());
                imgLeftBuf.pop();
            }

            {
                std::lock_guard<std::mutex> lock(mBufMutexRight);
                imRight = GetCompressedImage(imgRightBuf.front());
                imgRightBuf.pop();
            }


            vector<ORB_SLAM3::IMU::ImuPoint> vImuMeas;
            vector<ORB_SLAM3::IMU::DvlPoint> vDVLMeas;
            vector<ORB_SLAM3::IMU::GyroDvlPoint> vGyroDVLMeas;


            /* ----------------------------- IMU ----------------------------- */

            {
                std::lock_guard<std::mutex> lock(mpImuGb->mBufMutex);

                while(!mpImuGb->imuBuf.empty() &&
                      rclcpp::Time(mpImuGb->imuBuf.front()->header.stamp).seconds() <= tImLeft)
                {

                    auto msg = mpImuGb->imuBuf.front();

                    double t = rclcpp::Time(msg->header.stamp).seconds();

                    cv::Point3f acc(
                        msg->linear_acceleration.x,
                        msg->linear_acceleration.y,
                        msg->linear_acceleration.z);

                    cv::Point3f gyr(
                        msg->angular_velocity.x,
                        msg->angular_velocity.y,
                        msg->angular_velocity.z);

                    vImuMeas.push_back(
                        ORB_SLAM3::IMU::ImuPoint(acc,gyr,t));

                    vGyroDVLMeas.push_back(
                        ORB_SLAM3::IMU::GyroDvlPoint(
                            acc.x,acc.y,acc.z,
                            gyr.x,gyr.y,gyr.z,
                            0,0,0,0,0,0,0,t));

                    mpImuGb->imuBuf.pop();
                }
            }


            /* ----------------------------- DVL ----------------------------- */

            {
                std::lock_guard<std::mutex> lock(mpDvlGb->mBufMutex);

                while(!mpDvlGb->dvlBuf2.empty() &&
                      rclcpp::Time(mpDvlGb->dvlBuf2.front()->header.stamp).seconds() <= tImLeft)
                {

                    auto msg = mpDvlGb->dvlBuf2.front();

                    double t = rclcpp::Time(msg->header.stamp).seconds();

                    if(!msg->velocity_valid)
                    {
                        mpDvlGb->dvlBuf2.pop();
                        continue;
                    }

                    Eigen::Vector3d v(
                        msg->velocity.x,
                        msg->velocity.y,
                        msg->velocity.z);

                    if(v.norm() > 1.0)
                    {
                        mpDvlGb->dvlBuf2.pop();
                        continue;
                    }

                    vDVLMeas.push_back(
                        ORB_SLAM3::IMU::DvlPoint(
                            msg->velocity.x,
                            msg->velocity.y,
                            msg->velocity.z,
                            msg->beams[0].velocity,
                            msg->beams[1].velocity,
                            msg->beams[2].velocity,
                            msg->beams[3].velocity,
                            t));

                    vGyroDVLMeas.push_back(
                        ORB_SLAM3::IMU::GyroDvlPoint(
                            0,0,0,
                            msg->velocity.x,
                            msg->velocity.y,
                            msg->velocity.z,
                            msg->beams[0].velocity,
                            msg->beams[1].velocity,
                            msg->beams[2].velocity,
                            msg->beams[3].velocity,
                            t));

                    mpDvlGb->dvlBuf2.pop();
                }
            }


            if(vImuMeas.empty())
                continue;


            std::sort(
                vGyroDVLMeas.begin(),
                vGyroDVLMeas.end(),
                [](auto &a, auto &b)
                {
                    return a.t < b.t;
                });


            mpSLAM->TrackStereoGroDVL(
                imLeft,
                imRight,
                tImLeft,
                vGyroDVLMeas,
                !vDVLMeas.empty());
        }
    }
};



/* ----------------------------- MAIN ----------------------------- */

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);

    if(argc < 4)
    {
        std::cerr << "Usage: stereo_inertial_dvl "
                  << "path_to_vocabulary path_to_settings do_rectify"
                  << std::endl;

        return 1;
    }

    auto node = rclcpp::Node::make_shared("stereo_inertial_dvl");


    ORB_SLAM3::System SLAM(
        argv[1],
        argv[2],
        ORB_SLAM3::System::DVL_STEREO,
        node,
        false);


    ImuGrabber imugb;
    DVLGrabber dvlgb;

    ImageGrabber igb(&SLAM,&imugb,&dvlgb);


    cv::FileStorage fsSettings(argv[2], cv::FileStorage::READ);

    string imu_topic = fsSettings["ImuTopic"];
    string dvl_topic = fsSettings["DvlTopic"];
    string img_l_topic = fsSettings["LeftImgTopic"];
    string img_r_topic = fsSettings["RightImgTopic"];


    RCLCPP_INFO_STREAM(node->get_logger(),"ImuTopic: " << imu_topic);
    RCLCPP_INFO_STREAM(node->get_logger(),"DvlTopic: " << dvl_topic);
    RCLCPP_INFO_STREAM(node->get_logger(),"LeftImgTopic: " << img_l_topic);
    RCLCPP_INFO_STREAM(node->get_logger(),"RightImgTopic: " << img_r_topic);


    auto sub_imu =
        node->create_subscription<sensor_msgs::msg::Imu>(
            imu_topic,
            100,
            std::bind(&ImuGrabber::GrabImu,&imugb,std::placeholders::_1));


    auto sub_dvl =
        node->create_subscription<dvl_msgs::msg::DVL>(
            dvl_topic,
            100,
            std::bind(&DVLGrabber::GrabDVL2,&dvlgb,std::placeholders::_1));


    auto sub_left =
        node->create_subscription<sensor_msgs::msg::CompressedImage>(
            img_l_topic,
            100,
            std::bind(&ImageGrabber::GrabImageLeft,&igb,std::placeholders::_1));


    auto sub_right =
        node->create_subscription<sensor_msgs::msg::CompressedImage>(
            img_r_topic,
            100,
            std::bind(&ImageGrabber::GrabImageRight,&igb,std::placeholders::_1));


    std::thread sync_thread(&ImageGrabber::SyncWithImu2,&igb);

    rclcpp::spin(node);

    rclcpp::shutdown();

    sync_thread.join();

    return 0;
}