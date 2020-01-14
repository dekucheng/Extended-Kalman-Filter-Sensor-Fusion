#ifndef _EKF_ESTIMATION_
#define _EKF_ESTIMATION_

// ros 
#include <ros/ros.h>
#include <tf/tf.h>
#include "tf_conversions/tf_eigen.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <signal.h>

#include <memory>
// #include "ekf_estimation.h"

// messages
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// eigen3
#include <eigen3/Eigen/Dense>

using namespace Eigen;
using namespace std;

class Ekf_Estimation
{
public:
    /// constructor
    Ekf_Estimation();

    ros::Time last_filter_time;
    bool is_Initialized;
    bool imu_Initialized;

    Matrix<double, 6, 1> state;

    // since in gazebo simulation there is no such offset... the measurement could
    // be used directly ...
    Affine3d base_imu_offset;
    Matrix4d imu_pose;

    bool update();

    // destructor
    virtual ~Ekf_Estimation();

private:
    tf::TransformListener    robot_state;

}; // class


#endif
