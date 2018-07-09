#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "depth_image_proc/depth_traits.h"
#include "depth_image_proc/depth_conversions.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include <iostream>
#include <vector>
#include <string>
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace darknet_ros_msgs;

class PersonTrack{

public:

  double img_x;
  double img_y;

  PersonTrack(){

    sub_1_.subscribe(nh_,"camera/depth/image_rect_raw", 1);
    sub_2_.subscribe(nh_, "/darknet_ros/bounding_boxes", 1);
    sync_.reset(new Sync(MySyncPolicy(10), sub_1_, sub_2_));
    sync_->registerCallback(boost::bind(&PersonTrack::chatterCallback, this, _1, _2));

  }

  void chatterCallback(const ImageConstPtr& msg,
                       const BoundingBoxesConstPtr& msg1)
  {
   std::cout << "Synchronization successful" << std::endl;
  img_x =  msg1->bounding_boxes[0].xmin;
  img_y =  msg1->bounding_boxes[0].ymin;
  ROS_INFO("Position x is: %f",img_x);
  ROS_INFO("Position y is: %f",img_y);
  cv_bridge::CvImagePtr Dest = cv_bridge::toCvCopy(msg);

  int size = 2;
  int count = 1;
  double center_x = 0;
  double center_y = 0;
  double dist = 0;
  while (count < 70) {
      double dist_sum = 0;
      center_x = img_x;
      center_y = img_y;
      for (int i = -size; i < size; ++i) {
        for (int j = -size; j < size; ++j) {
          auto val = Dest->image.at<uint16_t>(center_x+i,center_y+j);
          auto distance = depth_image_proc::DepthTraits<uint16_t>::toMeters(val);
          if (distance != 0) {
              dist_sum += distance;
              count += 1;
          }
          }
        }
      try {
           dist = dist_sum/count;
      } catch (overflow_error err) {
        dist = 0.5;
      }
      }
  ROS_INFO("distance is: %f",dist);
  }

private:
  ros::NodeHandle nh_;
  message_filters::Subscriber<Image> sub_1_;
  message_filters::Subscriber<BoundingBoxes> sub_2_;

  typedef message_filters::sync_policies::ApproximateTime<Image,
                     BoundingBoxes> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;

};

  



int main(int argc, char **argv)
{

    ros::init(argc, argv, "PersonTrack");

    PersonTrack personTrack;

    ros::spin();

}
