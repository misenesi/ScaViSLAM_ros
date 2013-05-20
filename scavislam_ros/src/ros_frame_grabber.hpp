#ifndef ROS_FRAME_GRABBER_HPP
#define ROS_FRAME_GRABBER_HPP

#include "frame_grabber.hpp"
#include "util.cpp"

#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,
                sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> MySyncPolicy;


template <class Camera>
class RosFrameGrabber : public ScaViSLAM::FrameGrabber<Camera>
{
public:
  RosFrameGrabber (const Camera & cam, const Eigen::Vector4d & cam_distortion_, ScaViSLAM::PerformanceMonitor * per_mon_);

  void processNextFrame();

private:

  void imagesCallback(const sensor_msgs::ImageConstPtr & msg1,
                                      const sensor_msgs::ImageConstPtr & msg2,
                                      const sensor_msgs::CameraInfoConstPtr& info_msg_left,
                                      const sensor_msgs::CameraInfoConstPtr& info_msg_right );

  ros::NodeHandle nh_;
  image_transport::ImageTransport image_transport_;
  image_transport::SubscriberFilter subscriber_left_;
  image_transport::SubscriberFilter subscriber_right_;
  message_filters::Subscriber<sensor_msgs::CameraInfo> lcinfo_sub_; /**< Left camera info msg. */
  message_filters::Subscriber<sensor_msgs::CameraInfo> rcinfo_sub_; /**< Right camera info msg. */
  message_filters::Synchronizer<MySyncPolicy> sync_;

  cv::Mat left_img_, right_img_;
  bool has_data_;
};


const static int sync_queue_size = 10;

template <class Camera>
RosFrameGrabber<Camera>::RosFrameGrabber(const Camera & cam, const Eigen::Vector4d & cam_distortion_, ScaViSLAM::PerformanceMonitor * per_mon):
ScaViSLAM::FrameGrabber<Camera>(cam, cam_distortion_, per_mon),
nh_ ("~"),image_transport_ (nh_),
subscriber_left_( image_transport_, "/stereo/left/image_rect", 1 ),
subscriber_right_( image_transport_, "/stereo/right/image_rect", 1 ),
lcinfo_sub_(nh_,"/stereo/left/camera_info",1),
rcinfo_sub_(nh_,"/stereo/right/camera_info",1),
sync_( MySyncPolicy(sync_queue_size), subscriber_left_, subscriber_right_ ,lcinfo_sub_,rcinfo_sub_),
has_data_(false)
{
  sync_.registerCallback( boost::bind( &RosFrameGrabber::imagesCallback, this, _1, _2,_3,_4 ) );
}


template <class Camera>
void RosFrameGrabber<Camera>::imagesCallback(const sensor_msgs::ImageConstPtr & msg1,
                                    const sensor_msgs::ImageConstPtr & msg2,
                                    const sensor_msgs::CameraInfoConstPtr& info_msg_left,
                                    const sensor_msgs::CameraInfoConstPtr& info_msg_right )
{
  ROS_DEBUG("callback called");
  has_data_ = true;
  left_img_ = convertSensorMsgToCV(msg1);
  right_img_ = convertSensorMsgToCV(msg2);
}

template <class Camera>
void RosFrameGrabber<Camera>::
processNextFrame()
{
  this->frame_data.nextFrame();

  this->per_mon_->start("grab frame");
  if(!has_data_)
    ROS_INFO("Waiting for first image to be published...");
  while(!has_data_)
    ros::spinOnce();

//  if (params_.color_img)
//  {
//      frame_data.cur_left().color_uint8 = //get color left frame
//      cv::cvtColor(frame_data.cur_left().color_uint8,
//                   frame_data.cur_left().uint8,
//                   CV_BGR2GRAY);
//  }
  this->frame_data.cur_left().uint8 = left_img_.clone();
  this->frame_data.right.uint8 = right_img_.clone();
  this->per_mon_->stop("grab frame");
  this->preprocessing();
  this->frame_data.frame_id++;
}


#endif // ROS_FRAME_GRABBER_HPP
