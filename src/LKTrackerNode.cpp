#include "LKTracker.h"

using std::placeholders::_1;
using std::placeholders::_2;

namespace ORB_SLAM3
{

class LKTrackerNode : public rclcpp::Node
{
public:

    LKTrackerNode() : Node("lk_tracker_node")
    {

        init();

        image_transport::TransportHints hints(this, "compressed");

        left_sub_.subscribe(this, "/camera/left/image_dehazed", hints.getTransport());
        right_sub_.subscribe(this, "/camera/right/image_dehazed", hints.getTransport());

        sync_ = std::make_shared<Synchronizer>(SyncPolicy(50), left_sub_, right_sub_);

        sync_->registerCallback(
            std::bind(&LKTrackerNode::imageCallback, this, _1, _2)
        );

        RCLCPP_INFO(get_logger(), "LKTracker node started");
    }

    void init()
    {
        // Pass a shared pointer of the node to the LKTracker object
        tracker_ = std::make_shared<LKTracker>(shared_from_this());
    }

private:

    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::Image,
        sensor_msgs::msg::Image> SyncPolicy;

    typedef message_filters::Synchronizer<SyncPolicy> Synchronizer;

    void imageCallback(
        const sensor_msgs::msg::Image::ConstSharedPtr &left,
        const sensor_msgs::msg::Image::ConstSharedPtr &right)
    {

        cv_bridge::CvImagePtr cv_ptr_l;
        cv_bridge::CvImagePtr cv_ptr_r;

        try
        {
            cv_ptr_l = cv_bridge::toCvCopy(left, sensor_msgs::image_encodings::BGR8);
            cv_ptr_r = cv_bridge::toCvCopy(right, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception &e)
        {
            RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }

        double cur_img_time = rclcpp::Time(left->header.stamp).seconds();

        cv::Mat gray_l;
        cv::Mat gray_r;

        cv::cvtColor(cv_ptr_l->image, gray_l, cv::COLOR_BGR2GRAY);
        cv::cvtColor(cv_ptr_r->image, gray_r, cv::COLOR_BGR2GRAY);

        tracker_->trackImage(cur_img_time, gray_l, gray_r);
    }

    std::shared_ptr<LKTracker> tracker_;  // LKTracker object as an attribute

    image_transport::SubscriberFilter left_sub_;
    image_transport::SubscriberFilter right_sub_;

    std::shared_ptr<Synchronizer> sync_;
};

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<ORB_SLAM3::LKTrackerNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}