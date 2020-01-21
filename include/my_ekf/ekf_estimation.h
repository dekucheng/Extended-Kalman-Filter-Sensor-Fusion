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
#include <unordered_map>

// messages
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

// eigen3
#include <eigen3/Eigen/Dense>

#include "my_ekf/utils.h"

using namespace Eigen;
using namespace std;

class Ekf_Estimation
{
public:
    /// constructor
    Ekf_Estimation();

    ros::Time last_filter_time;
    bool is_Initialized, imu_Initialized, landmark_Initialized;
    bool IF_PRINT;


    // Initialization
    void Init(const Vector3d& first_odom, const ros::Time& first_time_stamp);
    void Init_imu(const tf::StampedTransform& t);
    void Init_landmark(const tf::StampedTransform &t);
    bool check_time(const ros::Time& t);
    void addmeasurement(const Matrix<double, 6, 1>& noise_odom);
    void addmeasurement(const double imu_meas);

    Vector3d get_state() const;
    bool update(const bool odom_update, const bool imu_update, const ros::Time this_update_time);
    // destructor
    virtual ~Ekf_Estimation();

private:
    tf::TransformListener    robot_state;

    // filter member variables
    Vector3d state, prior;
    Matrix3d Cov, G, R;
    // odom meas matrices
    Matrix3d H_odom, Q_odom, K_odom;
    Vector3d odom_meas;
    // imu meas matrices
    Matrix<double, 1, 3> H_imu;
    Vector3d K_imu;
    Matrix<double, 1, 1> Q_imu;
    double imu_meas;

    // since in gazebo simulation there is no such offset... the measurement could
    // be used directly ...
    Affine3d base_imu_offset, base_camera_offset;
    Matrix4d imu_pose;

    // apriltag landmark poses
    unordered_map<int, Matrix4d> known_landmark_poses;

    void motion_update();
    void update_with_odom();
    void update_with_imu();
}; // class

#endif
