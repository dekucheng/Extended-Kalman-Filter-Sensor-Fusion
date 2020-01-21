#ifndef _EKF_NODE_
#define _EKF_NODE_

// ros 
#include <ros/ros.h>
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <signal.h>

#include <memory>

// messages
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "nav_msgs/Path.h"

// eigen3
#include <eigen3/Eigen/Dense>

// project include
#include "my_ekf/ekf_estimation.h"
#include "my_ekf/utils.h"

using namespace Eigen;
using namespace std;

typedef nav_msgs::OdometryConstPtr OdomConstPtr;
typedef sensor_msgs::ImuConstPtr ImuConstPtr;
typedef nav_msgs::OdometryConstPtr VoConstPtr;
typedef nav_msgs::OdometryConstPtr GpsConstPtr;
typedef geometry_msgs::TwistConstPtr VelConstPtr;
typedef apriltag_ros::AprilTagDetectionArrayConstPtr LandmarkConstPtr;


class Ekf_Node
{
public:
  /// constructor
  Ekf_Node();

  // destructor
  virtual ~Ekf_Node();


private:
  /// the mail filter loop that will be called periodically
  void spin(const ros::TimerEvent& e);

  /// callback function for odo data
  void odomCallback(const OdomConstPtr& odom);

  /// callback function for imu data
  void imuCallback(const ImuConstPtr& imu);

  /// landmark callback
  void landmarkCallback(const LandmarkConstPtr& mark);

  // noise odom publisher
  void add_pose_to_path(const Vector3d& meas, nav_msgs::Path& path);

  void initialize_path();


  /// get the status of the filter
//   bool getStatus(robot_pose_ekf::GetStatus::Request& req, robot_pose_ekf::GetStatus::Response& resp);

  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Timer timer_;
  ros::Publisher pose_pub_, noise_odom_path_pub_, odom_path_pub_;
  ros::Subscriber odom_sub_, imu_sub_, landmark_sub_;

  // ekf filter
  shared_ptr<Ekf_Estimation>  my_filter;
  // tf listener
  tf::TransformListener robot_state;

  // some required transform matrices
  tf::StampedTransform base_imu_offset;

  // estimated robot pose message to send
  geometry_msgs::PoseWithCovarianceStamped  output_; 

  // measurements_data 1d (theta) for imu, 6d for odom 
  double imu_meas_;
  // tricks for odom measurement, the measurement actually is the 2d transform
  Matrix<double, 6, 1> odom_meas_;
  Matrix<double, 6, 1> last_odom_meas_;
  Matrix<double, 6, 1> noise_odom_;
  nav_msgs::Path odom_path, noise_path;

  // landmark measurements
  vector<landmark_pose> landmark_pose_set;



  ros::Time odom_time_, imu_time_, landmark_time_;
  ros::Time odom_stamp_, imu_stamp_, filter_stamp_, landmark_stamp_;
  ros::Time odom_init_stamp_, imu_init_stamp_, landmark_init_stamp_;
  bool odom_active_, imu_active_, landmark_active_;
  bool odom_used_, imu_used_, landmark_used_;
  bool odom_initializing_, imu_initializing_, landmark_initializing_;
  bool odom_update_, imu_update_, landmark_update_;
  double timeout_;
  int max_path_plot;

  Matrix4d imu_covariance_;
  Matrix<double, 6, 6> odom_covariance_;

  bool imu_transform_received;
  std::string output_frame_, base_footprint_frame_, camera_optical_frame_, tf_prefix_;

  // unsigned int odom_callback_counter_, imu_callback_counter_, vo_callback_counter_,gps_callback_counter_, ekf_sent_counter_;

}; // class


#endif
