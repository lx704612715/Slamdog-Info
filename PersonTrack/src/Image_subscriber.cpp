#include "ros/ros.h"
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/image_encodings.h"
#include "depth_image_proc/depth_traits.h"
#include "depth_image_proc/depth_conversions.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "darknet_ros_msgs/BoundingBox.h"
#include <sensor_msgs/Image.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "std_msgs/String.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <opencv2/opencv.hpp>
#include "PersonTrack/PersonPosition.h"
#include "opencv2/highgui.hpp"



using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace darknet_ros_msgs;
using namespace cv;
static const string OPENCV_WINDOW = "Origin Image window";
static const string OPENCV_WINDOW2 = "kalman filter";
static const string OPENCV_WINDOW3 = "Origin Image Depth window";
static const string OPENCV_WINDOW4 = "BilateralFilter";

namespace tracking {
class KalmanFilter
{
public:
  KalmanFilter(int x, int y):
    KF_(4, 2)
        {
            measurement_ = Mat::zeros(2, 1, CV_32F);
            KF_.transitionMatrix = (Mat_<float>(4, 4) << 1, 0, 1, 0,
                                                         0, 1, 0, 1,
                                                         0, 0, 1, 0,
                                                         0, 0, 0, 1);
            setIdentity(KF_.measurementMatrix, Scalar::all(1));
            setIdentity(KF_.processNoiseCov, Scalar::all(1e-3));
            setIdentity(KF_.measurementNoiseCov, Scalar::all(1e-1));
            setIdentity(KF_.errorCovPost, Scalar::all(1));

            KF_.statePost = (Mat_<float>(4, 1) << x, y, 0, 0);
        }

  Point2f run(float x, float y)
  {
    Mat prediction = KF_.predict();
    Point2f predict_pt = Point2f(prediction.at<float>(0),prediction.at<float>(1));

    measurement_.at<float>(0, 0) = x;
    measurement_.at<float>(1, 0) = y;

    KF_.correct(measurement_);

    return predict_pt;
  }
private:
  Mat measurement_;
  cv::KalmanFilter KF_;
};

}


class PersonTrackController{

public:
float dist_diff;
float ang_diff;
float angular_max = 0.07 * 1.7;
float angular_min = 0.01;
float linear_max = 0.07 * 2.2;
float angular_real_max = 0.04;
float linear_real_max = 0.05;
float angular_window = 0.4;
float linear_window = 0.01;
float desired_dist = 1.0;
float weight = 0.02;
float last_pub_x;
float last_pub_y;
float last_pub_dist_diff = 0;
float last_pub_ang_diff = 0;
float dist_max = 3.0;
float dist_min = 1.0;
int dist_direction;
int ang_direction;
float momentun;
float max_speed = 0.04;
float max_angular_speed = 0.04;
float min_angular_speed = 0.01;
Point2f currentPoint;
Point2f kalmanPoint;

//PersonTrackController(float x, float y){
//  center_x = x;
//  center_y = y;
//}


void getPosition(float center_x,float center_y, float dist_input){

   tracking::KalmanFilter kf(center_x, center_y);
   currentPoint = Point2f(center_x,center_y);
   kalmanPoint = kf.run(center_x,center_y);
   ROS_INFO("x is: %f",center_x);
   ROS_INFO("y is: %f",center_y);
   ROS_INFO("predict point x is %f", kalmanPoint.x);
   ROS_INFO("predict point y is: %f", kalmanPoint.y);

  dist_input = min(dist_input, dist_max);
//  dist_diff = max(dist_diff, dist_min);
  dist_diff = dist_input - desired_dist;
  ang_diff = ((320 - kalmanPoint.x)/320)*0.06;

  dist_diff = weight * dist_diff;
//  if(dist_diff !=0 && dist_diff < -0.3) {
//    dist_direction = -1;
//  }


  if (ang_diff >= 0) {
    ang_direction = 1;
  }
  else {
    ang_direction = -1;
  }

  if (last_pub_dist_diff == 0 && last_pub_ang_diff == 0) {
    momentun = 0.8;
  }
  else {
    momentun = 0.5;
  }

  ang_diff = momentun * ang_diff + (1 - momentun) * last_pub_ang_diff;
  dist_diff = momentun * dist_diff + (1 - momentun) * last_pub_dist_diff;

  if (dist_diff < linear_window) {
    dist_diff = 0;
  }

 // if (dist_diff != 0 && dist_direction > 0) {
 //   dist_diff = (linear_max * dist_diff) / (1 - linear_window)
  //      - (linear_max * linear_window) / 0.5;
 // }
  //if (dist_direction < 0) {
   // dist_diff = -0.01;
  //}
//  if (ang_diff != 0) {
//    ang_diff = ang_direction * ((angular_max * ang_diff) / (1 - angular_window ) -
//        (angular_max * angular_window) / (1 - angular_window));
//  }

//  if (last_pub_dist_diff < 0 && dist_diff > 0 && last_pub_ang_diff != 0) {
//  dist_diff = 0;
//  }

  if (dist_diff > max_speed) {
    dist_diff = 0.04;
  }
//  if (dist_diff < 0) {
//    dist_diff = 0;
//  }

 if (abs(ang_diff) < 0.018) {
    ang_diff = 0;
 }


  if (abs(ang_diff) > max_angular_speed) {
    ang_diff = max_angular_speed * ang_direction;
  }
  if (abs(ang_diff) < min_angular_speed) {
    ang_diff = 0;
  }
//  if (ang_diff != 0) {
//    dist_diff = 0;
//  }
//  if (dist_direction < 0) {
//    ang_diff = 0;
//  }

  ROS_INFO("dist_diff is: %f",dist_diff);
  ROS_INFO("ang_diff is: %f",ang_diff);
  ROS_INFO("last_pub_dist_diff is: %f",last_pub_dist_diff);
  ROS_INFO("last_pub_ang_diff is: %f",last_pub_ang_diff);

  last_pub_dist_diff = dist_diff;
  last_pub_ang_diff = ang_diff;
}
};

class PersonTracking{

public:
  float img_x;
  float img_y;
  float img_x_min;
  float img_y_min;
  float img_x_max;
  float img_y_max;
  float center_x;
  float center_y;
  const double pi = 3.141592653;
  float dist;
  float ang;

  PersonTracking(){
    sub_1_.subscribe(nh_,"/camera/aligned_depth_to_color/image_raw", 1);
//  sub_1_.subscribe(nh_,"camera/color/image_raw", 1);
    sub_2_.subscribe(nh_, "/darknet_ros/bounding_boxes", 1);
    sync_.reset(new Sync(MySyncPolicy(10), sub_1_, sub_2_));
    sync_->registerCallback(boost::bind(&PersonTracking::chatterCallback, this, _1, _2));

  }

  void chatterCallback(const ImageConstPtr& msg,
                       const BoundingBoxesConstPtr& msg1)
  {
    std::cout << "Synchronization successful" << std::endl;

     img_x_min =  msg1->bounding_boxes[0].xmin;
     img_y_min =  msg1->bounding_boxes[0].ymin;
     img_x_max = msg1->bounding_boxes[0].xmax;
     img_y_max = msg1->bounding_boxes[0].ymax;

     center_x = (img_x_max + img_x_min)/2;
     center_y = (img_y_max + img_y_min)/2;

     try
     {
       cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
     }
     catch (cv_bridge::Exception& e)
     {
       ROS_ERROR("cv_bridge exception: %s", e.what());
       return;
     }

     cv_bridge::CvImagePtr Dest = cv_bridge::toCvCopy(msg);
     Mat depth32f(640, 480, CV_32FC1);
     Dest->image.convertTo(depth32f, CV_32FC1,1/65535.0);
    Mat bilaterimg(640, 480, CV_16UC1);
    Mat bilaterimg32f(640, 480, CV_32FC1);
    bilateralFilter(depth32f, bilaterimg32f, 15, 80, 80);
    bilaterimg32f.convertTo(bilaterimg, CV_16UC1, 65535.0);

     int size = 2;
     int count = 0;
     double dist_sum = 0;
     for (int i = -size; i < size; ++i) {
       for (int j = -size; j < size; ++j) {
          auto val = bilaterimg.at<uint16_t>(center_x+i,center_y+j);
//         auto val = Dest->image.at<uint16_t>(center_x+i,center_y+j);
         auto distance = depth_image_proc::DepthTraits<uint16_t>::toMeters(val);
         distance = distance;
         if (distance != 0 && distance < 5) {
             dist_sum += distance;
             count += 1;
         }
         }
       }
     if (count != 0) {
         dist = dist_sum/count;
     }
     else {
       dist = 0;
     }
     personTrackController.getPosition(center_x,center_y, dist);
     pub_ = nh_.advertise<geometry_msgs::Twist>("/slamdog/slamdog_hardware_controller/cmd_vel", 10);
//      pub_ = nh_.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 10); //发布cmd_vel
     geometry_msgs::Twist output;
     ros::Rate loop_rate(4);
     output.linear.x = personTrackController.dist_diff;
     output.angular.z = personTrackController.ang_diff;
     pub_.publish(output);
     ROS_INFO("distance is: %f",dist);
     ROS_INFO("angular is: %f",ang);
     ROS_INFO("linear_x is: %f",output.linear.x);
     ROS_INFO("angular_z is: %f", output.angular.z);

  }


private:

  ros::NodeHandle nh_;
  message_filters::Subscriber<Image> sub_1_;
  message_filters::Subscriber<BoundingBoxes> sub_2_;
  typedef message_filters::sync_policies::ApproximateTime<Image,
                     BoundingBoxes> MySyncPolicy;
  typedef message_filters::Synchronizer<MySyncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  PersonTrackController personTrackController;
  ros::Publisher pub_;
  cv_bridge::CvImagePtr cv_ptr;

};

  
int main(int argc, char **argv)
{

    ros::init(argc, argv, "PersonTrack");

    PersonTracking personTrack;
    ros::spin();

    return 0;

}
